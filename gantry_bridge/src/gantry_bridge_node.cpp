#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "gantry_bridge/motion_backend.hpp"

using std::placeholders::_1;

using gantry_bridge::AxisState;
using gantry_bridge::Direction;
using gantry_bridge::IMotionBackend;
using gantry_bridge::MotionProfile;
using gantry_bridge::StopMode;

namespace {

// =============================================================================
// Machine-specific constants
// Units: meters, seconds, pulses, pulses/s, pulses/s^2 unless noted otherwise.
// =============================================================================

// --- Axes --------------------------------------------------------------------
// Hardware axis IDs on the motion controller. Hardware order is X, Y, gripper, Z;
// note that Isaac Sim joint order is X, Y, Z, left finger, right finger.
constexpr int kAxisX = 0;
constexpr int kAxisY = 1;
constexpr int kAxisGripper = 2;
constexpr int kAxisZ = 3;
constexpr std::array<int, 4> kAxisIds = {kAxisX, kAxisY, kAxisGripper, kAxisZ};

// Pulses per meter, hardware order: 10000 pulses per motor revolution / screw lead.
//   X:       lead 12 mm -> 10000 / 0.012 = 833,333.33
//   Y:       lead 8 mm  -> nominal 1,250,000; 1,262,500 is a measured calibration value
//   Gripper: lead 8 mm  -> 10000 / 0.008 = 1,250,000
//   Z:       lead 3 mm  -> 10000 / 0.003 = 3,333,333.33
constexpr std::array<double, 4> kPpu = {
  833333.33,   // X
  1262500.00,  // Y (measured)
  1250000.00,  // Gripper
  3333333.33   // Z
};

// --- Motion profile applied once at startup ---------------------------------
constexpr double kInitVelStart = 1.0;
constexpr double kInitVelMax = 10000.0;
constexpr double kInitAccel = 20000.0;   // Must be non-zero.
constexpr double kInitDecel = 20000.0;
// Y axis: higher max velocity, half acceleration and deceleration.
constexpr double kInitVelMaxY = 50000.0;
constexpr double kInitAccelY = 10000.0;
constexpr double kInitDecelY = 10000.0;
// Z axis: higher start velocity so the vertical axis overcomes gravity when starting.
constexpr double kInitVelStartZ = 100.0;
constexpr double kInitVelMaxZ = 15000.0;
// Delay after enabling each servo so it locks before the next axis is enabled.
constexpr std::chrono::milliseconds kServoEnableSettle{50};

// --- Sim-to-hardware offsets (velocity and PVT lanes) -----------------------
constexpr double kSimOffsetX = 0.005;        // Sim X rest is 0.005, hardware rest is 0.0 (subtracted).
constexpr double kSimOffsetGripper = 0.008;  // Sim closed -0.008 -> hardware closed 0.0 (added, velocity lane only).
constexpr double kSimOffsetZ = 0.015;        // Hardware Z is raised 15 mm relative to sim (added).

// --- Position limits ----------------------------------------------------------
struct Range { double min; double max; };
// Velocity lane.
constexpr Range kVelRangeX{0.0, 0.15};
constexpr Range kVelRangeY{0.0, 0.04};
constexpr Range kVelRangeZ{-0.025, 0.0};
constexpr Range kVelRangeGripper{0.0, 0.012};
// PVT lane.
constexpr Range kPvtRangeX{0.0, 0.15};
constexpr Range kPvtRangeY{0.0, 0.04};
constexpr Range kPvtRangeZ{-0.025, -0.002};
constexpr Range kPvtRangeGripper{-0.015, 0.0};  // 0.0 = open, -0.015 = fully closed.

// --- PTP lane -----------------------------------------------------------------
constexpr int64_t kPtpMinIntervalMs = 100;        // Minimum interval between PTP commands.
constexpr double kPtpDeadzoneM = 0.0001;          // Ignore target changes of 0.1 mm or less.
constexpr double kPtpNoiseThresholdPulse = 200.0; // Skip axes within 200 pulses of the target.

// --- Velocity lane --------------------------------------------------------------
// Adaptive proportional gain, selected by absolute position error.
constexpr double kGainFarErrorM = 0.05;  // |error| > 5 cm
constexpr double kGainFar = 1.0;
constexpr double kGainMidErrorM = 0.01;  // 1 cm < |error| <= 5 cm
constexpr double kGainMid = 5.0;
constexpr double kGainNear = 15.0;       // |error| <= 1 cm
// A standstill axis farther than this from its target aligns at a very low gain.
constexpr double kAlignErrorM = 0.02;
constexpr double kAlignGain = 0.5;
// Velocity caps.
constexpr double kSoftStartVelLimit = 5000.0;  // From standstill (about 0.6 cm/s on X).
constexpr double kMovingVelLimit = 200000.0;   // While already moving.
constexpr double kVelLimitY = 25000.0;         // Y axis (short stroke), always.
// Deadzone.
constexpr double kVelDeadzoneM = 0.0001;  // Position error below 0.1 mm.
constexpr double kMinJogVel = 100.0;      // Commanded velocity below 100 pulses/s.
// Jog start profile, velocity change and stop.
constexpr double kJogVelStart = 0.0;
constexpr double kJogAccel = 500000.0;
constexpr double kJogDecel = 5000000.0;
constexpr double kChangeVelAccel = 1000000.0;
constexpr double kChangeVelDecel = 5000000.0;
constexpr double kVelStopDecel = 5000000.0;

// --- PVT lane -----------------------------------------------------------------
constexpr std::size_t kPvtMaxPoints = 100;
constexpr double kPvtVelLimit = 1000000.0;  // About 1.2 m/s on X.
constexpr std::chrono::milliseconds kPvtResetSettle{20};    // After reset_error().
constexpr std::chrono::milliseconds kPvtEnableSettle{100};  // After enable().
constexpr int64_t kPvtSettleMs = 500;  // Ignore axis states for 500 ms after PVT start.

// --- Watchdog and safety guard -------------------------------------------------
constexpr std::chrono::milliseconds kTimerPeriod{100};
constexpr int64_t kWatchdogTimeoutMs = 250;  // Stop jogging if no velocity command for 250 ms.
constexpr double kWatchdogStopDecel = 500000.0;
constexpr double kSafetyMaxErrorM = 0.08;    // Emergency stop beyond 8 cm error.
constexpr double kEmergencyStopDecel = 0.0;

// --- Feedback -------------------------------------------------------------------
constexpr double kFeedbackDeadband = 0.0001;  // Zero out |value| < 0.0001 (m or m/s).
constexpr char kPvtLogPath[] = "/tmp/gantry_pvt_log.csv";

}  // namespace


class GantryBridgeNode : public rclcpp::Node
{
public:
  GantryBridgeNode()
  : Node("gantry_bridge_node"),
    backend_(gantry_bridge::create_motion_backend())
  {
    // Isaac Sim joint names; must match the names and order in the USD asset.
    // There are 4 motors but 5 joints because the gripper is modeled as two fingers.
    m_joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5"};

    last_velocity_cmd_time_ = std::chrono::steady_clock::now();

    // Feedback to Isaac Sim.
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // Position lane: absolute targets in meters, Isaac Sim joint order.
    sub_metric_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/gantry/cmd_position", 10, std::bind(&GantryBridgeNode::callback_metric, this, _1));

    // Lane 1: PTP (rate-limited absolute moves).
    sub_sim_ptp_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/gantry/cmd_ptp", 10, std::bind(&GantryBridgeNode::callback_ptp, this, _1));

    // Lane 2: velocity following (high-rate position stream).
    sub_sim_velocity_ = this->create_subscription<sensor_msgs::msg::JointState>(
       "/gantry/cmd_velocity", 10, std::bind(&GantryBridgeNode::callback_velocity_stream, this, _1));

    // Lane 3: PVT trajectories.
    sub_pvt_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
       "/gantry/cmd_pvt", 10, std::bind(&GantryBridgeNode::callback_pvt, this, _1));

    error_pub_ = this->create_publisher<std_msgs::msg::Float64>("/gantry/pvt_error", 10);

    // Open the motion controller.
    if (!backend_->open()) {
      RCLCPP_ERROR(this->get_logger(), "Motion backend open failed");
    } else {
      RCLCPP_INFO(this->get_logger(), "Motion backend initialized");
      RCLCPP_INFO(this->get_logger(), "PPU loaded. X axis: %.2f", kPpu[0]);
      init_hardware();
    }

    // Feedback, watchdog and safety checks at 10 Hz.
    timer_ = this->create_wall_timer(
      kTimerPeriod, std::bind(&GantryBridgeNode::timer_callback, this));
  }

  ~GantryBridgeNode() {
      backend_->close();
      RCLCPP_INFO(this->get_logger(), "Motion backend closed");
  }

private:

  void init_hardware()
  {
    // Configure the motion profile and enable every axis.
    for (int axis : kAxisIds) {

        // Set once; move_absolute() uses this profile afterwards.
        double vel_low = kInitVelStart;
        double vel_high = kInitVelMax;
        double acc = kInitAccel;
        double dec = kInitDecel;

        // Known issue: the original intent was to halve the Y-axis speed, but its max velocity (50000) is 5x the default (10000).
        if (axis == kAxisY) {
            vel_high = kInitVelMaxY;
            acc = kInitAccelY;
            dec = kInitDecelY;
        }

        if (axis == kAxisZ) {
             vel_low = kInitVelStartZ;
             vel_high = kInitVelMaxZ;
        }

        backend_->set_profile(axis, MotionProfile{vel_low, vel_high, acc, dec});

        // Clear an error state, then enable the servo.
        AxisState state = backend_->state(axis).value_or(AxisState::Disabled);
        if (state == AxisState::ErrorStop) backend_->reset_error(axis);

        backend_->enable(axis);

        // Let the servo lock before enabling the next axis, avoiding a current spike.
        std::this_thread::sleep_for(kServoEnableSettle);
    }
    RCLCPP_INFO(this->get_logger(), "Servos enabled, motion profiles configured");
  }

  // Position lane: msg->data holds absolute targets in meters, Isaac Sim joint order.
  // Known issue: this lane bypasses the PTP lane's rate limit, busy check and deadzone.
  void callback_metric(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
      execute_motion(msg->data);
  }

  // ==========================================
  // Lane 1: PTP
  // ==========================================
  // Known issue: the PTP and position lanes do not check is_pvt_executing_, so they can issue moves while a PVT trajectory is running.
  void callback_ptp(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Enforce a minimum interval between commands in case upstream throttling fails.
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_exec_time_);
    if (elapsed.count() < kPtpMinIntervalMs) return;

    // Isaac Sim sends 5 joints: [0:X, 1:Y, 2:Z, 3:GripL, 4:GripR].
    // The first 4 are kept in sim order; execute_motion() reorders them to hardware order.
    if (msg->position.size() < 4) return;
    std::vector<double> current_targets(4);

    current_targets[0] = msg->position[0]; // X
    current_targets[1] = msg->position[1]; // Y
    current_targets[2] = msg->position[2]; // Z
    current_targets[3] = msg->position[3]; // Gripper (left finger)

    // Busy check on the main axes (X, Y): drop the command while they are busy.
    // Known issue: this check was intended to detect a moving axis but matches only the stopping state, so new PTP targets are issued while a PTP move is still in progress.
    AxisState state_x = backend_->state(kAxisIds[0]).value_or(AxisState::Disabled);
    AxisState state_y = backend_->state(kAxisIds[1]).value_or(AxisState::Disabled);

    if (state_x == AxisState::Stopping || state_y == AxisState::Stopping) {
          return;
      }

    // Deadzone: ignore the command unless some axis target changed by more than 0.1 mm.
    bool huge_change = false;

      for (size_t i = 0; i < 4; ++i) {
          if (std::abs(current_targets[i] - last_sent_targets_[i]) > kPtpDeadzoneM) {
              huge_change = true;
              break;
          }
      }

      if (!huge_change) return;

      last_exec_time_ = now;
      // Known issue: last_sent_targets_ is stored here in sim order [X, Y, Z, G], but the safety guard and the PVT end-sync use hardware order [X, Y, G, Z].
      last_sent_targets_ = current_targets;
      execute_motion(current_targets);
  }

  // Converts sim-order targets in meters to hardware order and pulses, then issues
  // absolute moves. commands: [0:X, 1:Y, 2:Z, 3:Gripper, ...].
  void execute_motion(const std::vector<double>& commands)
  {
    if (commands.size() < 4) {
          RCLCPP_ERROR(this->get_logger(), "Command too short: expected at least 4 values, got %zu", commands.size());
          return;
    }

    // Hardware order: [0]X, [1]Y, [2]Gripper, [3]Z. Z and gripper are swapped relative to sim order.
    // Known issue: the PTP and position lanes apply none of the sim-to-hardware offsets used by the velocity and PVT lanes.
    std::vector<double> hw_targets(4);

    hw_targets[0] = commands[0]; // Sim joint_1 (X) -> hardware axis 0
    hw_targets[1] = commands[1]; // Sim joint_2 (Y) -> hardware axis 1
    hw_targets[2] = commands[3]; // Sim joint_4 (gripper) -> hardware axis 2
    hw_targets[3] = commands[2]; // Sim joint_3 (Z) -> hardware axis 3

      for (size_t i = 0; i < kAxisIds.size(); ++i) {
          int axis = kAxisIds[i];

          // Meters -> pulses. kPpu is in hardware order.
          double target = hw_targets[i] * kPpu[i];

          // Skip axes already within 200 pulses of the target (filters simulation noise).
          double current_pos = backend_->actual_position(axis).value_or(0.0);
          if (std::abs(target - current_pos) < kPtpNoiseThresholdPulse) {
              continue;
          }

          // The profile was set in init_hardware(); only the target is sent here.
          if (!backend_->move_absolute(axis, target)) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
               "Axis %d move failed", axis);
          }
      }
  }

  // ==========================================
  // Lane 2: velocity following
  // ==========================================
  // Tracks a streamed position target with an adaptive P-controller by jogging
  // each axis and changing its velocity on the fly.
  // Known issue: this lane never updates last_sent_targets_, so following more than 8 cm away from the last PTP target triggers the safety-guard emergency stop.
  void callback_velocity_stream(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
      // Ignore this lane while a PVT trajectory is running.
      if (is_pvt_executing_) return;

      if (msg->name.empty() || msg->position.empty()) return;

      // Feed the watchdog.
      last_velocity_cmd_time_ = std::chrono::steady_clock::now();

      // Joint name -> position.
      std::unordered_map<std::string, double> pos_map;
      for (size_t i = 0; i < msg->name.size(); ++i) {
          if (i < msg->position.size()) pos_map[msg->name[i]] = msg->position[i];
      }

      for (size_t i = 0; i < kAxisIds.size(); ++i) {
          int axis = kAxisIds[i];

          double target_m = 0.0;
          bool target_found = false;

          // Map sim joints to hardware axes and apply the sim-to-hardware offsets.

          // X <- joint_1. Sim rest position 0.005 corresponds to hardware 0.0.
          if (axis == kAxisX && pos_map.count("joint_1")) {
              double raw_sim = pos_map["joint_1"];
              target_m = raw_sim - kSimOffsetX;
              target_found = true;
          }
          // Y <- joint_2.
          else if (axis == kAxisY && pos_map.count("joint_2")) {
              target_m = pos_map["joint_2"];
              target_found = true;
          }
          // Gripper <- joint_4 (left finger).
          else if (axis == kAxisGripper) {
             if (pos_map.count("joint_4")) {
                 double raw_sim = pos_map["joint_4"];
                 // Sim closed (-0.008) -> hardware closed (0.0); sim open (0.0) -> 0.008.
                 double calibrated_val = raw_sim + kSimOffsetGripper;
                 target_m = calibrated_val;
                 target_found = true;
             }
          }
          // Z <- joint_3, raised 15 mm relative to sim.
          else if (axis == kAxisZ && pos_map.count("joint_3")) {
              target_m = pos_map["joint_3"];
              target_m += kSimOffsetZ;
              target_found = true;
          }

          if (!target_found) continue;

          // Position limits.
          if (axis == kAxisX) target_m = std::clamp(target_m, kVelRangeX.min, kVelRangeX.max);
          if (axis == kAxisY) target_m = std::clamp(target_m, kVelRangeY.min, kVelRangeY.max);
          if (axis == kAxisZ) target_m = std::clamp(target_m, kVelRangeZ.min, kVelRangeZ.max);

          // Gripper: 0 to 12 mm, to protect the gripper mechanism.
          if (axis == kAxisGripper) target_m = std::clamp(target_m, kVelRangeGripper.min, kVelRangeGripper.max);

          AxisState state = backend_->state(axis).value_or(AxisState::Disabled);

          double current_pulse = backend_->actual_position(axis).value_or(0.0);
          double current_m = current_pulse / kPpu[i];

          double error_m = target_m - current_m;

          // Adaptive gain: the larger the error, the lower the gain, to avoid sudden jumps.
          //   |error| > 5 cm:        Kp = 1.0
          //   1 cm < |error| <= 5 cm: Kp = 5.0
          //   |error| <= 1 cm:       Kp = 15.0
          double adaptive_kp;
          double abs_err = std::abs(error_m);

          if (abs_err > kGainFarErrorM) {
              adaptive_kp = kGainFar;
          } else if (abs_err > kGainMidErrorM) {
              adaptive_kp = kGainMid;
          } else {
              adaptive_kp = kGainNear;
          }

          double cmd_vel_m = error_m * adaptive_kp;

          // A standstill axis more than 2 cm from its target aligns at a very low gain.
          if (state == AxisState::Standstill && std::abs(error_m) > kAlignErrorM) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Aligning axis %d, gap %.2f cm", axis, error_m * 100.0);
                cmd_vel_m = error_m * kAlignGain;
            }

             double cmd_vel_pulse = cmd_vel_m * kPpu[i];
             Direction target_dir = (cmd_vel_pulse >= 0) ? Direction::Positive : Direction::Negative;
             double abs_vel = std::abs(cmd_vel_pulse);

             // Velocity limit depends on state: soft start from standstill, higher limit once moving.
              double final_vel_limit;

              if (state == AxisState::Standstill) {
                final_vel_limit = kSoftStartVelLimit;
              } else {
                 final_vel_limit = kMovingVelLimit;
              }

              // Y axis (short stroke) is always capped.
              if (axis == kAxisY) final_vel_limit = std::min(final_vel_limit, kVelLimitY);

              if (abs_vel > final_vel_limit) abs_vel = final_vel_limit;

              // Deadzone: position error below 0.1 mm or velocity below 100 pulses/s.
              bool in_deadzone = (std::abs(error_m) < kVelDeadzoneM);
              if (abs_vel < kMinJogVel) in_deadzone = true;

              if (in_deadzone) {
                    // Jogging -> stop.
                    if (state == AxisState::ContinuousMotion) {
                        backend_->stop(axis, StopMode::Decelerate, kVelStopDecel);
                    }
                }
              else {
                    if (state == AxisState::Standstill) {
                        // Standstill -> jog with start velocity 0 and max velocity abs_vel (already limited above).
                        // Known issue: this profile overwrites the one set in init_hardware(), so later PTP moves run with the last jog profile.
                        backend_->set_profile(axis, MotionProfile{kJogVelStart, abs_vel, kJogAccel, kJogDecel});
                        backend_->jog(axis, target_dir);
                    }
                    else if (state == AxisState::ContinuousMotion) {
                        // Jogging -> change velocity on the fly.
                        backend_->change_velocity(axis, abs_vel, kChangeVelAccel, kChangeVelDecel);
                   }
          }
      }
  }

  // ==========================================
  // Lane 3: PVT
  // ==========================================
  // Converts a JointTrajectory into one PVT table per axis (absolute positions,
  // sim-to-hardware offsets, velocity limits) and starts every loaded axis.
    void callback_pvt(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        if (msg->points.size() < 2) return;

        // Lock out the other command lanes immediately.
        is_pvt_executing_ = true;
        pvt_start_time_ = std::chrono::steady_clock::now();

        size_t axis_count = kAxisIds.size();
        std::vector<int> sync_axis_handles;

        // Recover from errors, stop any motion and clear old PVT tables.
        for (int axis : kAxisIds) {
            AxisState state = backend_->state(axis).value_or(AxisState::Disabled);

            // An axis in ErrorStop must be reset and re-enabled.
            if (state == AxisState::ErrorStop) {
                RCLCPP_WARN(this->get_logger(), "Axis %d in error state, resetting", axis);
                backend_->reset_error(axis);
                std::this_thread::sleep_for(kPvtResetSettle);
                backend_->enable(axis);
                std::this_thread::sleep_for(kPvtEnableSettle);
            }

            backend_->stop(axis, StopMode::Emergency, kEmergencyStopDecel);
            backend_->reset_pvt(axis);
        }

        // Downsample to at most 100 points; shorter trajectories are used 1:1.
        // Known issue: Some motion controllers limit the PVT table to a few dozen entries (e.g. 42). The 100-point cap has not been verified against the controller; trajectories generated by brain.py (~16 Hz, segments <= 0.8 s) stay well below either limit.
        size_t raw_count = msg->points.size();

        size_t step = 1;
        if (raw_count > kPvtMaxPoints) {
            step = std::ceil((double)raw_count / (double)kPvtMaxPoints);
        }

        std::vector<size_t> indices;
        for (size_t k = 0; k < raw_count; k += step) indices.push_back(k);
        if (indices.back() != raw_count - 1) indices.push_back(raw_count - 1);
        size_t final_point_count = indices.size();

        for (size_t i = 0; i < axis_count; ++i) {
            int axis = kAxisIds[i];

            // Hardware axis -> sim joint index (sim order: X, Y, Z, GripL, GripR).
            int sim_joint_idx = -1;
            if (axis == kAxisX) sim_joint_idx = 0;
            else if (axis == kAxisY) sim_joint_idx = 1;
            else if (axis == kAxisGripper) sim_joint_idx = 3;
            else if (axis == kAxisZ) sim_joint_idx = 2;
            if (sim_joint_idx == -1) continue;

            // Absolute positions, not shifted to the current position.
            std::vector<double> pArray(final_point_count);
            std::vector<double> vArray(final_point_count);
            std::vector<double> tArray(final_point_count);

            double t0_sec = msg->points[0].time_from_start.sec + (msg->points[0].time_from_start.nanosec * 1e-9);

            for (size_t j = 0; j < final_point_count; ++j) {
                auto& pt = msg->points[indices[j]];
                // Known issue: positions and velocities are indexed without a size check, so a point with fewer than 4 joints or without velocities is read out of bounds.
                double raw_sim_m = pt.positions[sim_joint_idx];
                double raw_vel_m_s = pt.velocities[sim_joint_idx];
                double cur_t_sec = pt.time_from_start.sec + (pt.time_from_start.nanosec * 1e-9);

                // Sim-to-hardware offsets.
                double target_m = raw_sim_m;
                if (axis == kAxisX) target_m = raw_sim_m - kSimOffsetX;
                if (axis == kAxisZ) target_m = raw_sim_m + kSimOffsetZ;

                // Gripper: hardware closes in the negative direction, so position and velocity are both negated.
                // Known issue: the velocity lane maps the gripper as +0.008 m clamped to [0, 0.012], while this lane negates it and clamps to [-0.015, 0], so the two lanes use opposite sign conventions.
                if (axis == kAxisGripper) {
                    target_m = raw_sim_m * -1.0;
                    raw_vel_m_s = raw_vel_m_s * -1.0;
                }

                // Keep the unclamped value to detect clamping.
                double original_target = target_m;

                // Position limits.
                if (axis == kAxisX) target_m = std::clamp(target_m, kPvtRangeX.min, kPvtRangeX.max);
                if (axis == kAxisY) target_m = std::clamp(target_m, kPvtRangeY.min, kPvtRangeY.max);
                if (axis == kAxisGripper) target_m = std::clamp(target_m, kPvtRangeGripper.min, kPvtRangeGripper.max);
                // Z lower limit (-0.025) is below the pick height sent by brain.py (-0.015 after offset), so the trajectory is not truncated.
                if (axis == kAxisZ) target_m = std::clamp(target_m, kPvtRangeZ.min, kPvtRangeZ.max);

                // A clamped point gets zero velocity; a fixed position with non-zero
                // velocity would make the motor oscillate.
                if (target_m != original_target) {
                    raw_vel_m_s = 0.0;
                }

                pArray[j] = target_m * kPpu[i];

                double real_vel_pulse = raw_vel_m_s * kPpu[i];
                vArray[j] = std::clamp(real_vel_pulse, -kPvtVelLimit, kPvtVelLimit);

                // Cumulative time in ms since the first point.
                tArray[j] = (cur_t_sec - t0_sec) * 1000.0;
            }

            backend_->reset_pvt(axis);

            if (backend_->load_pvt(axis, pArray, vArray, tArray)) {
                sync_axis_handles.push_back(axis);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Axis %d PVT load failed", axis);
            }
        }

       // Start every loaded axis.
       if (sync_axis_handles.size() > 0) {
            // Known issue: axes are started one after another in this loop, not synchronized.
            for (int axis_handle : sync_axis_handles) {
                backend_->start_pvt(axis_handle);
            }
            RCLCPP_INFO(this->get_logger(), "PVT started on all loaded axes");
        } else {
            // No axis loaded: release the lock.
            is_pvt_executing_ = false;
            RCLCPP_WARN(this->get_logger(), "PVT: no axis ready");
        }
    }

  // ==========================================
  // Timer: PVT lock, watchdog, safety guard, feedback
  // ==========================================
  // Reads hardware position and velocity, converts them to meters and publishes /joint_states.
  void timer_callback()
  {
    auto now = std::chrono::steady_clock::now();

    // Step 1: release the PVT lock once every axis is at standstill.
    if (is_pvt_executing_) {
        auto pvt_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - pvt_start_time_);

        // Ignore axis states for the first 500 ms so axes that have not started yet are not read as finished.
        if (pvt_elapsed.count() > kPvtSettleMs) {
            bool all_ready = true;
            for (int axis : kAxisIds) {
                // Any state other than standstill keeps the PVT lock.
                // Known issue: an axis left in ErrorStop or Disabled never reaches standstill, so the lock is never released and the watchdog and safety guard stay disabled.
                AxisState state = backend_->state(axis).value_or(AxisState::Disabled);
                if (state != AxisState::Standstill) all_ready = false;
            }

            if (all_ready) {
            is_pvt_executing_ = false;

            // Use the final actual positions as the new reference targets, so the
            // safety guard does not see a jump when the trajectory ends.
            for (size_t i = 0; i < kAxisIds.size(); ++i) {
                double end_pos = backend_->actual_position(kAxisIds[i]).value_or(0.0);
                last_sent_targets_[i] = end_pos / kPpu[i];
            }

            RCLCPP_INFO(this->get_logger(), "PVT finished, targets synced to actual positions");
            }
        }
    }

    // Step 2: watchdog and safety guard. All motion commands here are skipped while PVT is running.
    if (!is_pvt_executing_) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_velocity_cmd_time_);

        // Watchdog: stop jogging axes when the velocity stream has stopped.
        if (elapsed.count() > kWatchdogTimeoutMs) {
            for (int axis : kAxisIds) {
                AxisState state = backend_->state(axis).value_or(AxisState::Disabled);
                if (state == AxisState::ContinuousMotion) {
                    backend_->stop(axis, StopMode::Decelerate, kWatchdogStopDecel);
                }
            }
        }

        // Safety guard: emergency-stop any axis more than 8 cm from its last target.
        // Known issue: the guard compares against the final PTP target rather than the trajectory, so any PTP move longer than 8 cm is emergency-stopped at the next timer tick.
        for (size_t i = 0; i < kAxisIds.size(); ++i) {
            int axis = kAxisIds[i];

            double current_pulse = backend_->actual_position(axis).value_or(0.0);
            double current_m = current_pulse / kPpu[i];

            double last_target = last_sent_targets_[i];
            double error_m = std::abs(last_target - current_m);

            if (error_m > kSafetyMaxErrorM) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Safety guard: axis %d error %.2f cm, emergency stop", axis, error_m * 100.0);
                backend_->stop(axis, StopMode::Emergency, kEmergencyStopDecel);
            }
        }
    }

    auto message = sensor_msgs::msg::JointState();
    message.header.stamp = this->now();
    message.name = m_joint_names;

    // Read hardware feedback (hardware order) and convert to meters.
    std::vector<double> hw_pos_metric(4);
    std::vector<double> hw_vel_metric(4);

    // X-axis command and actual position, for PVT tracking.
    double x_cmd_m = 0.0;
    double x_act_m = 0.0;

    for (size_t i = 0; i < kAxisIds.size(); ++i) {
        int axis_id = kAxisIds[i];

        // Read failures fall back to 0.
        double cur_pos = backend_->actual_position(axis_id).value_or(0.0);
        double cur_cmd = backend_->command_position(axis_id).value_or(0.0);
        double cur_vel = backend_->actual_velocity(axis_id).value_or(0.0);

        hw_pos_metric[i] = cur_pos / kPpu[i];
        hw_vel_metric[i] = cur_vel / kPpu[i];

        if (axis_id == kAxisX) {
            x_act_m = hw_pos_metric[i];
            x_cmd_m = cur_cmd / kPpu[i];
        }

        // Zero out values below the noise floor.
        if (std::abs(hw_pos_metric[i]) < kFeedbackDeadband) hw_pos_metric[i] = 0.0;
        if (std::abs(hw_vel_metric[i]) < kFeedbackDeadband) hw_vel_metric[i] = 0.0;
    }

    // While PVT runs, publish the X tracking error and log it to CSV.
    if (is_pvt_executing_) {
        if (!is_recording_) {
            std::string filename = kPvtLogPath;
            csv_file_.open(filename);
            csv_file_ << "Time_ms,X_CMD_cm,X_ACT_cm,Error_cm\n";
            is_recording_ = true;
            RCLCPP_INFO(this->get_logger(), "PVT tracking log started");
        }

        auto log_now = std::chrono::steady_clock::now();
        auto pvt_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(log_now - pvt_start_time_).count();

        double error_cm = std::abs(x_cmd_m - x_act_m) * 100.0;
        auto err_msg = std_msgs::msg::Float64();
        err_msg.data = error_cm;
        error_pub_->publish(err_msg);

        // CSV values in cm.
        if (csv_file_.is_open()) {
            csv_file_ << pvt_elapsed << ","
                      << x_cmd_m * 100.0 << ","
                      << x_act_m * 100.0 << ","
                      << error_cm << "\n";
        }
    } else {
        if (is_recording_) {
            csv_file_.close();
            is_recording_ = false;
            RCLCPP_INFO(this->get_logger(), "PVT tracking log saved");
        }
    }

    // Map 4 hardware axes to 5 sim joints, in meters.
    // Hardware order: [0:X, 1:Y, 2:Gripper, 3:Z].

    // joint_1 (X) <- axis 0
    message.position.push_back(hw_pos_metric[0]);
    message.velocity.push_back(hw_vel_metric[0]);

    // joint_2 (Y) <- axis 1
    message.position.push_back(hw_pos_metric[1]);
    message.velocity.push_back(hw_vel_metric[1]);

    // joint_3 (Z) <- axis 3
    message.position.push_back(hw_pos_metric[3]);
    message.velocity.push_back(hw_vel_metric[3]);

    // joint_4 (left finger) <- axis 2
    message.position.push_back(hw_pos_metric[2]);
    message.velocity.push_back(hw_vel_metric[2]);

    // joint_5 (right finger) <- axis 2, mirrored: one motor drives both fingers.
    message.position.push_back(hw_pos_metric[2] * -1.0);
    message.velocity.push_back(hw_vel_metric[2] * -1.0);

    message.effort = {0.0, 0.0, 0.0, 0.0, 0.0};

    publisher_->publish(message);

    // Human-readable status in cm, once per second. Does not affect the published message.
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "\n[System Status] (cm)\n"
      "   X-Axis : %7.2f cm\n"
      "   Y-Axis : %7.2f cm\n"
      "   Z-Axis : %7.2f cm\n"
      "   Gripper: %7.2f cm",
      hw_pos_metric[0] * 100.0, // X
      hw_pos_metric[1] * 100.0, // Y
      hw_pos_metric[3] * 100.0, // Z (hardware index 3)
      hw_pos_metric[2] * 100.0  // Gripper (hardware index 2)
    );

  }


  // --- Members ---
  std::unique_ptr<IMotionBackend> backend_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_pub_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_metric_;            // Position lane
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_sim_ptp_;               // Lane 1: PTP
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_sim_velocity_;          // Lane 2: velocity
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_pvt_;          // Lane 3: PVT

  // PTP rate limit: time of the last executed PTP command (initialized at startup).
  std::chrono::steady_clock::time_point last_exec_time_ = std::chrono::steady_clock::now();

  // Last targets in meters, used by the PTP deadzone and the safety guard.
  std::vector<double> last_sent_targets_ = {0.0, 0.0, 0.0, 0.0};

  // Watchdog: time of the last velocity-lane command.
  std::chrono::steady_clock::time_point last_velocity_cmd_time_;

  std::chrono::steady_clock::time_point pvt_start_time_;
  bool is_pvt_executing_ = false;

  std::vector<std::string> m_joint_names;

  // PVT tracking log.
  std::ofstream csv_file_;
  bool is_recording_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GantryBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
