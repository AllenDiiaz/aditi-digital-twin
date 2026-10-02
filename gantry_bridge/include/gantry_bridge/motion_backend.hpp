#pragma once
#include <memory>
#include <optional>
#include <vector>

namespace gantry_bridge {

// Axis states follow the PLCopen Motion Control axis state machine.
enum class AxisState {
  Disabled, Standstill, Stopping, ErrorStop, Homing,
  DiscreteMotion, ContinuousMotion, SynchronizedMotion, Other
};
enum class StopMode { Decelerate, Emergency };
enum class Direction { Positive, Negative };

// All positions in pulses, velocities in pulses/s, accelerations in pulses/s^2.
struct MotionProfile { double vel_start; double vel_max; double accel; double decel; };

class IMotionBackend {
 public:
  virtual ~IMotionBackend() = default;
  virtual bool open() = 0;
  virtual void close() = 0;
  virtual bool set_profile(int axis, const MotionProfile& profile) = 0;
  virtual bool reset_error(int axis) = 0;
  virtual bool enable(int axis) = 0;
  virtual bool move_absolute(int axis, double target) = 0;
  virtual bool jog(int axis, Direction dir) = 0;
  virtual bool change_velocity(int axis, double vel, double accel, double decel) = 0;
  virtual bool reset_pvt(int axis) = 0;
  // pos, vel and time_ms have one entry per point. time_ms is cumulative: time_ms[i] is
  // the time of point i in milliseconds measured from the first point, so time_ms[0] == 0.
  virtual bool load_pvt(int axis, const std::vector<double>& pos,
                        const std::vector<double>& vel,
                        const std::vector<double>& time_ms) = 0;
  virtual bool start_pvt(int axis) = 0;
  virtual bool stop(int axis, StopMode mode, double decel) = 0;
  virtual std::optional<AxisState> state(int axis) = 0;
  virtual std::optional<double> actual_position(int axis) = 0;
  virtual std::optional<double> command_position(int axis) = 0;
  virtual std::optional<double> actual_velocity(int axis) = 0;
};

// Implemented by the user for their own motion controller (not included in this repo).
std::unique_ptr<IMotionBackend> create_motion_backend();

}  // namespace gantry_bridge
