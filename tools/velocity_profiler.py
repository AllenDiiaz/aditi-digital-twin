import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib
matplotlib.use('Agg') # Headless backend
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import threading
import time
import os

class VelocityProfiler(Node):
    """Records commanded vs. measured X velocity and saves a CSV and a plot."""
    def __init__(self):
        super().__init__('velocity_profiler')

        self.sub_cmd = self.create_subscription(JointState, '/gantry/cmd_velocity', self.cmd_cb, 10)
        self.sub_real = self.create_subscription(JointState, '/joint_states', self.real_cb, 10)

        self.start_time = None
        self.is_recording = False
        self.data_log = []

        print("Velocity Profiler ready.")
        print("Press Enter to START recording.")
        print("Press Enter again to STOP.")

    def get_time(self):
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds
        return (self.get_clock().now().nanoseconds - self.start_time) / 1e9

    def cmd_cb(self, msg):
        if not self.is_recording: return
        t = self.get_time()
        if len(msg.velocity) > 0:
            self.data_log.append({'time': t, 'type': 'sim_cmd', 'vel_x': msg.velocity[0]})

    def real_cb(self, msg):
        if not self.is_recording: return
        t = self.get_time()
        if len(msg.velocity) > 0:
            self.data_log.append({'time': t, 'type': 'real_act', 'vel_x': msg.velocity[0]})

def main():
    rclpy.init()
    node = VelocityProfiler()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()

    try:
        input() # Wait for start
        node.is_recording = True
        node.start_time = None
        node.data_log = []
        print("Recording...")

        input() # Wait for stop
        node.is_recording = False
        print("Stopped. Saving data...")

        if not node.data_log:
            print("No data collected.")
            return

        df = pd.DataFrame(node.data_log)

        # Save the CSV first so the data survives a plotting failure.
        timestamp = int(time.time())
        csv_filename = f"velocity_data_{timestamp}.csv"
        df.to_csv(csv_filename, index=False)
        print(f"CSV saved: {csv_filename}")

        df_sim = df[df['type'] == 'sim_cmd']
        df_real = df[df['type'] == 'real_act']

        plt.figure(figsize=(12, 6))

        # Plot absolute velocity (speed) so only magnitudes are compared, regardless of direction.
        plt.plot(df_sim['time'].to_numpy(), np.abs(df_sim['vel_x'].to_numpy()), 'b--', label='Sim Command (Speed)', linewidth=2)
        plt.plot(df_real['time'].to_numpy(), np.abs(df_real['vel_x'].to_numpy()), 'r-', label='Real Robot (Speed)', linewidth=2, alpha=0.8)

        plt.title(f"Sim-to-Real Speed Tracking (Absolute Value)\n{time.ctime()}", fontsize=14)
        plt.xlabel("Time (s)")
        plt.ylabel("Speed (m/s)")
        plt.legend()
        plt.grid(True)

        img_filename = f"velocity_profile_{timestamp}.png"
        plt.savefig(img_filename)
        print(f"Image saved: {img_filename}")
        print(f"Location: {os.getcwd()}")

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()
