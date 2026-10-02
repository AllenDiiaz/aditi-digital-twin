import omni.usd
import omni.timeline
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from pxr import Usd, UsdGeom, Sdf
import numpy as np
import math

# ==========================================
# Parameters
# ==========================================
# Must point to the articulation root of the gantry.
ROBOT_PATH = "/World/gantry"
JOINT_INDICES = [0, 1, 2, 3, 4]

# World-to-joint offsets: joint = -world + offset.
ROBOT_OFFSET_X = -0.05759
ROBOT_OFFSET_Y = -0.09100

# Z heights (joint space, m).
Z_SAFE = -0.005
Z_PICK = -0.030
Z_DROP = -0.030

# Joint limits (m).
LIMITS_X = [0.005, 0.15]
LIMITS_Y = [0.000, 0.04]
LIMITS_Z = [-0.045, -0.005]

# Finger positions [left, right] (m).
GRIPPER_OPEN  = [0, 0]
GRIPPER_CLOSE = [0.015, -0.015]

class PersistentBrain:
    def __init__(self):
        self.stage = omni.usd.get_context().get_stage()
        self.timeline = omni.timeline.get_timeline_interface()

        self.robot = None
        self.controller = None
        self.is_connected = False
        self.was_playing = False

        # PVT output to hardware; the ROS 2 node is created in init_ros_node().
        self.use_pvt = True
        self.ros_node_created = False

        self.full_reset()

    def init_ros_node(self):
        """Create the ROS 2 node and PVT publisher. Disables PVT if ROS 2 is unavailable."""
        if not self.use_pvt or self.ros_node_created:
            return

        try:
            import rclpy
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
            from rclpy.duration import Duration

            self.rclpy = rclpy
            self.JointTrajectory = JointTrajectory
            self.JointTrajectoryPoint = JointTrajectoryPoint
            self.Duration = Duration

            if not self.rclpy.ok():
                self.rclpy.init()

            self.node = self.rclpy.create_node('isaac_pvt_commander')
            self.pvt_pub = self.node.create_publisher(self.JointTrajectory, '/gantry/cmd_pvt', 10)

            self.ros_node_created = True
            print("[PVT] ROS 2 publisher ready: /gantry/cmd_pvt")

        except ImportError as e:
            print("[PVT] ROS 2 module (rclpy) not found.")
            print(f"[PVT] Import error: {e}")
            print("[PVT] Falling back to simulation-only mode (PVT disabled).")
            self.use_pvt = False
        except Exception as e:
            print(f"[PVT] Initialization failed: {e}")
            self.use_pvt = False

    def trigger_intent(self, ball_name, bin_name):
        """
        Public API: queue a pick task by prim name (cube and target bin).
        Accepted only while IDLE.
        """
        if self.state == "IDLE":
            self.pending_intent = (ball_name, bin_name)
            print(f"[Intent] Bin {bin_name} selected, picking {ball_name}")
        else:
            print("[System] Robot busy, click ignored.")

    def full_reset(self):
        """Reset the task state machine. Starts in WARMUP."""
        self.state = "WARMUP"
        self.timer = 0.0
        self.start_pos = None
        self.target_ball = None
        self.target_bin = None
        print("[System] State reset, entering WARMUP")

    def reset_connection(self):
        """Called when the timeline stops: disconnect the controller and reset task progress."""
        try:
            # Command all joints to zero so the robot does not stay mid-air.
            if self.controller:
                zero_action = ArticulationAction(joint_positions=np.array([0,0,0,0,0]), joint_indices=np.array(JOINT_INDICES))
                self.controller.apply_action(zero_action)
        except: pass

        self.robot = None
        self.controller = None
        self.is_connected = False

        self.full_reset()
        print("[System] Simulation stopped, task progress cleared")

    def connect(self):
        try:
            if not self.timeline.is_playing(): return False
            if not self.robot:
                self.robot = Articulation(ROBOT_PATH)
            self.robot.initialize()
            self.controller = self.robot.get_articulation_controller()
            if self.controller:
                self.is_connected = True
                print("[Brain] Connected, waiting for a task")
                # Initialize ROS 2 here, once the simulation is playing and the ROS 2 bridge is loaded.
                self.init_ros_node()
                # No initial action is sent; the articulation holds its current state.
                return True
        except: pass
        return False

    def get_pos(self, prim_name):
        """World position of /World/<prim_name>, or None if the prim does not exist."""
        path_str = f"/World/{prim_name}"
        target_path = Sdf.Path(path_str)
        prim = self.stage.GetPrimAtPath(target_path)

        if not prim or not prim.IsValid():
            return None

        transform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        translation = transform.ExtractTranslation()
        return np.array(translation)

    def get_robot_pos(self):
        """Current X/Y in world coordinates (inverse of move()); Z is reported as Z_SAFE."""
        if not self.is_connected: return np.array([0,0,0])
        # Joint positions may be unavailable right after startup.
        try:
            j = self.robot.get_joint_positions()
            return np.array([ROBOT_OFFSET_X - j[0], ROBOT_OFFSET_Y - j[1], Z_SAFE])
        except:
            return np.array([0,0,0])

    def clamp(self, val, min_v, max_v):
        return max(min(val, max_v), min_v)

    def move(self, x, y, z, gripper):
        """Move the twin to world X/Y, joint Z and finger positions."""
        if not self.is_connected: return
        raw_x = -x + ROBOT_OFFSET_X
        raw_y = -y + ROBOT_OFFSET_Y
        raw_z = z

        cmd_x = self.clamp(raw_x, LIMITS_X[0], LIMITS_X[1])
        cmd_y = self.clamp(raw_y, LIMITS_Y[0], LIMITS_Y[1])
        cmd_z = self.clamp(raw_z, LIMITS_Z[0], LIMITS_Z[1])

        try:
            # X, Y, Z: write joint positions directly (teleport), bypassing the
            # joint drives so the twin follows without physics lag.
            self.robot.set_joint_positions(
                positions=np.array([cmd_x, cmd_y, cmd_z]),
                joint_indices=np.array([0, 1, 2])
            )

            # Fingers: driven through the articulation controller so PhysX
            # produces real contact and friction when grasping.
            # joint_positions must be passed as a keyword argument.
            action = ArticulationAction(
                joint_positions=np.array([gripper[0], gripper[1]]),
                joint_indices=np.array([3, 4])
            )
            self.controller.apply_action(action)

        except Exception:
            # Ignore single-frame failures.
            pass

    def get_joint_vec(self, w_x, w_y, w_z, w_g):
        """
        World coordinates -> joint vector [x, y, z, g_l, g_r].
        Uses the same math as move() and clamps to the joint limits so trajectories
        sent to hardware stay in range.
        """
        raw_x = -w_x + ROBOT_OFFSET_X
        raw_y = -w_y + ROBOT_OFFSET_Y
        raw_z = w_z

        j_x = self.clamp(raw_x, LIMITS_X[0], LIMITS_X[1])
        j_y = self.clamp(raw_y, LIMITS_Y[0], LIMITS_Y[1])
        j_z = self.clamp(raw_z, LIMITS_Z[0], LIMITS_Z[1])

        # w_g[0] is the left finger, w_g[1] the right finger.
        return [j_x, j_y, j_z, w_g[0], w_g[1]]

    def s_curve(self, start, end, t):
        """
        Half-cosine blend from start to end for t in [0, 1]: zero velocity at both
        ends and a bell-shaped velocity profile in between.
        """
        factor = (1.0 - math.cos(t * math.pi)) / 2.0
        return start + (end - start) * factor

    def generate_and_send_pvt(self, start_vec, end_vec, duration):
        """
        Sample a half-cosine move and publish it as a PVT trajectory.
        start_vec: [x, y, z, g_l, g_r] start joint positions (m)
        end_vec:   [x, y, z, g_l, g_r] end joint positions (m)
        duration:  move time (s)
        """
        if not self.use_pvt: return

        msg = self.JointTrajectory()
        # The bridge node reads points by index in this order.
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']

        # Fixed sampling rate of 40 / 2.5 = 16 Hz; a move yields int(duration * 16) + 1 points.
        # Known issue: int() truncates, so for durations that are not multiples of 1/16 s (e.g. 0.2, 0.6, 0.8 s) the last point stops short of end_vec with non-zero velocity.
        dur = 2.5
        freq = 40 / dur
        steps = int(duration * freq)

        for i in range(steps + 1): # steps + 1 points, starting at t = 0
            t = i / freq
            p = t / duration
            if p > 1.0: p = 1.0

            # Position factor: (1 - cos(pi * p)) / 2
            pos_factor = (1.0 - math.cos(p * math.pi)) / 2.0

            # Velocity factor, the time derivative of the position factor:
            # (pi / 2) * sin(pi * p) / duration, so velocities are in m/s.
            vel_factor = (math.pi / 2.0) * math.sin(p * math.pi) / duration

            point = self.JointTrajectoryPoint()

            current_pos = []
            current_vel = []

            for j in range(5):
                s = start_vec[j]
                e = end_vec[j]

                # P = S + (E - S) * pos_factor
                p_val = s + (e - s) * pos_factor

                # V = (E - S) * vel_factor
                v_val = (e - s) * vel_factor

                # Joint-space values; the bridge node applies the hardware offsets.
                current_pos.append(p_val)
                current_vel.append(v_val)

            point.positions = current_pos
            point.velocities = current_vel
            point.time_from_start = self.Duration(seconds=t).to_msg()

            msg.points.append(point)

        self.pvt_pub.publish(msg)
        print(f"[PVT] Trajectory sent: duration={duration}s, points={len(msg.points)}")

    def update(self, dt):
        # --- Lifecycle ---
        is_playing = self.timeline.is_playing()

        if self.use_pvt and hasattr(self, 'node'):
            # Spin every frame so queued messages are sent.
            self.rclpy.spin_once(self.node, timeout_sec=0.0)

        if self.was_playing and not is_playing:
            self.reset_connection()

        self.was_playing = is_playing
        if not is_playing: return

        if not self.is_connected:
            if not self.connect(): return

        self.timer += dt

        # Warmup: hold the home pose for 3 s before accepting tasks.
        if self.state == "WARMUP":
            if self.timer > 3.0:
                print("[System] Warmup complete, ready for tasks")
                self.state = "IDLE"
                self.timer = 0
            else:
                self.move(0.0, 0.0, Z_SAFE, GRIPPER_OPEN)
                return

        # --- Task state machine ---
        # On the first frame of each motion state the PVT trajectory is sent to
        # hardware; every frame the twin follows the same half-cosine profile.

        # IDLE: wait for an intent from trigger_intent().
        if self.state == "IDLE":
            # getattr: pending_intent does not exist until the first trigger.
            if getattr(self, 'pending_intent', None) is not None:
                ball_name, bin_name = self.pending_intent

                # Target positions are read from the USD stage.
                b_pos = self.get_pos(ball_name)
                bin_pos = self.get_pos(bin_name)

                if b_pos is not None and bin_pos is not None:
                    print(f"[System] Target locked: {ball_name} -> {bin_name}")
                    self.start_pos = self.get_robot_pos()
                    self.target_ball = b_pos
                    self.target_bin = bin_pos

                    self.state = "MOVE_TO_BALL"
                else:
                    print(f"[Error] Prim {ball_name} or {bin_name} not found. Check the USD prim names.")

                # Clear the intent and restart the timer.
                self.pending_intent = None
                self.timer = 0

        elif self.state == "MOVE_TO_BALL":
            dur = 0.8

            # First frame of the state (timer <= dt).
            if self.timer <= (dt + 0.01) and self.use_pvt:
                s_vec = self.get_joint_vec(self.start_pos[0], self.start_pos[1], Z_SAFE, GRIPPER_OPEN)
                e_vec = self.get_joint_vec(self.target_ball[0], self.target_ball[1], Z_SAFE, GRIPPER_OPEN)
                self.generate_and_send_pvt(s_vec, e_vec, dur)

            p = min(self.timer / dur, 1.0)
            cur = self.s_curve(self.start_pos, self.target_ball, p)
            self.move(cur[0], cur[1], Z_SAFE, GRIPPER_OPEN)

            if self.timer > dur + 0.2:
                self.state = "DESCEND"
                self.timer = 0

        elif self.state == "DESCEND":
            dur = 0.5

            if self.timer <= (dt + 0.01) and self.use_pvt:
                s_vec = self.get_joint_vec(self.target_ball[0], self.target_ball[1], Z_SAFE, GRIPPER_OPEN)
                e_vec = self.get_joint_vec(self.target_ball[0], self.target_ball[1], Z_PICK, GRIPPER_OPEN)
                self.generate_and_send_pvt(s_vec, e_vec, dur)

            p = min(self.timer / dur, 1.0)
            cur_z = self.s_curve(Z_SAFE, Z_PICK, p)
            self.move(self.target_ball[0], self.target_ball[1], cur_z, GRIPPER_OPEN)

            if self.timer > dur + 0.5:
                self.state = "GRASP"
                self.timer = 0

        elif self.state == "GRASP":
            dur = 0.2

            if self.timer <= (dt + 0.01) and self.use_pvt:
                s_vec = self.get_joint_vec(self.target_ball[0], self.target_ball[1], Z_PICK, GRIPPER_OPEN)
                e_vec = self.get_joint_vec(self.target_ball[0], self.target_ball[1], Z_PICK, GRIPPER_CLOSE)
                self.generate_and_send_pvt(s_vec, e_vec, dur)

            p = min(self.timer / dur, 1.0)
            g_left = self.s_curve(GRIPPER_OPEN[0], GRIPPER_CLOSE[0], p)
            g_right = self.s_curve(GRIPPER_OPEN[1], GRIPPER_CLOSE[1], p)
            self.move(self.target_ball[0], self.target_ball[1], Z_PICK, [g_left, g_right])

            if self.timer > dur + 0.5:
                self.state = "LIFT"
                self.timer = 0
                self.start_pos = self.target_ball

        elif self.state == "LIFT":
            dur = 0.5

            if self.timer <= (dt + 0.01) and self.use_pvt:
                s_vec = self.get_joint_vec(self.target_ball[0], self.target_ball[1], Z_PICK, GRIPPER_CLOSE)
                e_vec = self.get_joint_vec(self.target_ball[0], self.target_ball[1], Z_SAFE, GRIPPER_CLOSE)
                self.generate_and_send_pvt(s_vec, e_vec, dur)

            p = min(self.timer / dur, 1.0)
            cur_z = self.s_curve(Z_PICK, Z_SAFE, p)
            self.move(self.target_ball[0], self.target_ball[1], cur_z, GRIPPER_CLOSE)

            if self.timer > dur + 0.2:
                self.state = "MOVE_TO_BIN"
                self.timer = 0

        elif self.state == "MOVE_TO_BIN":
            dur = 0.6

            if self.timer <= (dt + 0.01) and self.use_pvt:
                s_vec = self.get_joint_vec(self.target_ball[0], self.target_ball[1], Z_SAFE, GRIPPER_CLOSE)
                e_vec = self.get_joint_vec(self.target_bin[0], self.target_bin[1], Z_SAFE, GRIPPER_CLOSE)
                self.generate_and_send_pvt(s_vec, e_vec, dur)

            p = min(self.timer / dur, 1.0)
            cur = self.s_curve(self.target_ball, self.target_bin, p)
            self.move(cur[0], cur[1], Z_SAFE, GRIPPER_CLOSE)

            if self.timer > dur + 0.2:
                self.state = "DESCEND_TO_BIN"
                self.timer = 0

        elif self.state == "DESCEND_TO_BIN":
            dur = 0.5

            if self.timer <= (dt + 0.01) and self.use_pvt:
                s_vec = self.get_joint_vec(self.target_bin[0], self.target_bin[1], Z_SAFE, GRIPPER_CLOSE)
                e_vec = self.get_joint_vec(self.target_bin[0], self.target_bin[1], Z_DROP, GRIPPER_CLOSE)
                self.generate_and_send_pvt(s_vec, e_vec, dur)

            p = min(self.timer / dur, 1.0)
            cur_z = self.s_curve(Z_SAFE, Z_DROP, p)
            self.move(self.target_bin[0], self.target_bin[1], cur_z, GRIPPER_CLOSE)

            if self.timer > dur + 0.5:
                self.state = "RELEASE"
                self.timer = 0

        elif self.state == "RELEASE":
            dur = 0.2

            if self.timer <= (dt + 0.01) and self.use_pvt:
                s_vec = self.get_joint_vec(self.target_bin[0], self.target_bin[1], Z_DROP, GRIPPER_CLOSE)
                e_vec = self.get_joint_vec(self.target_bin[0], self.target_bin[1], Z_DROP, GRIPPER_OPEN)
                self.generate_and_send_pvt(s_vec, e_vec, dur)

            p = min(self.timer / dur, 1.0)
            g_left = self.s_curve(GRIPPER_CLOSE[0], GRIPPER_OPEN[0], p)
            g_right = self.s_curve(GRIPPER_CLOSE[1], GRIPPER_OPEN[1], p)
            self.move(self.target_bin[0], self.target_bin[1], Z_DROP, [g_left, g_right])

            if self.timer > dur + 0.5:
                self.state = "RETRACT"
                self.timer = 0

        elif self.state == "RETRACT":
            dur = 0.5

            if self.timer <= (dt + 0.01) and self.use_pvt:
                s_vec = self.get_joint_vec(self.target_bin[0], self.target_bin[1], Z_DROP, GRIPPER_OPEN)
                e_vec = self.get_joint_vec(self.target_bin[0], self.target_bin[1], Z_SAFE, GRIPPER_OPEN)
                self.generate_and_send_pvt(s_vec, e_vec, dur)

            p = min(self.timer / dur, 1.0)
            cur_z = self.s_curve(Z_DROP, Z_SAFE, p)
            self.move(self.target_bin[0], self.target_bin[1], cur_z, GRIPPER_OPEN)
            if self.timer > dur + 0.2:
                print("[Task] Placed, returning home")
                self.state = "RETURN_HOME"
                self.timer = 0
                self.start_pos = self.target_bin

        elif self.state == "RETURN_HOME":
            dur = 0.8

            if self.timer <= (dt + 0.01) and self.use_pvt:
                s_vec = self.get_joint_vec(self.target_bin[0], self.target_bin[1], Z_SAFE, GRIPPER_OPEN)
                e_vec = self.get_joint_vec(0.0, 0.0, Z_SAFE, GRIPPER_OPEN)
                self.generate_and_send_pvt(s_vec, e_vec, dur)

            p = min(self.timer / dur, 1.0)
            home_target = np.array([0.0, 0.0, Z_SAFE])
            cur = self.s_curve(self.target_bin, home_target, p)
            self.move(cur[0], cur[1], Z_SAFE, GRIPPER_OPEN)

            if self.timer > dur + 0.2:
                print("[Task] Home reached, waiting for the next task")
                self.state = "IDLE"
                self.timer = 0
