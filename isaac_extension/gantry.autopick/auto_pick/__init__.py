import os

import omni.ext
import omni.ui as ui
import omni.usd
import omni.kit.app
from std_msgs.msg import Float64
import rclpy

from .brain import PersistentBrain

class AutoPickExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        print("[AutoPick] Extension started")

        self._window = None
        self._update_sub = None
        self._sub = None
        self._brain = None
        self.error_label = None

        # ROS 2 node that receives the live PVT tracking error from the bridge node.
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('isaac_ui_listener')
        self.error_sub = self.node.create_subscription(
            Float64,
            '/gantry/pvt_error',
            self._on_live_error_received,
            10
        )

        try:
            self._brain = PersistentBrain()

            # Per-frame update loop.
            self._update_sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
                self._on_update, name="AutoPick_Brain_Update"
            )

            # Clicking a bin in the stage triggers the matching pick task.
            self._setup_selection_listener()

            # UI window.
            self._window = ui.Window("Gantry Auto Pick", width=350, height=500)
            with self._window.frame:
                with ui.VStack(spacing=15, padding=10):
                    ui.Label("System Status: RUNNING", style={"color": 0xFF00FF00})

                    # --- Task control ---
                    with ui.CollapsableFrame("Auto Pick Tasks", name="group"):
                        with ui.VStack(spacing=10, padding=5):
                            ui.Button("Pick GREEN Cube", height=35, clicked_fn=lambda: self._trigger_color("Green"))
                            ui.Button("Pick BLUE Cube", height=35, clicked_fn=lambda: self._trigger_color("Blue"))
                            ui.Button("Pick RED Cube", height=35, clicked_fn=lambda: self._trigger_color("Red"))
                            ui.Separator(height=5)
                            ui.Button("Reset State", height=35, clicked_fn=self._on_reset)

                    # --- Sim-to-real tracking error ---
                    with ui.CollapsableFrame("Sim-to-Real Analysis", name="group"):
                        with ui.VStack(spacing=10, padding=5):
                            ui.Label("Reality Gap (Sim vs Real Error)", style={"font_size": 14})
                            with ui.HStack(height=40):
                                # Known issue: live samples from /gantry/pvt_error overwrite this label, so it shows the latest error, not the maximum.
                                ui.Label("Max Tracking Error:", width=ui.Percent(60))
                                self.error_label = ui.Label("0.0000 cm", style={"color": 0xFFFFFF00, "font_size": 18})
                            ui.Button("Load Latest Hardware Data", height=40, clicked_fn=self._on_load_pvt_data)

                    ui.Separator(height=10)
                    ui.Label("Click a bin in the stage or use the buttons above", style={"color": 0xAAFFFFFF}, word_wrap=True)

        except Exception as e:
            print(f"[AutoPick] Startup error: {e}")

    def _on_live_error_received(self, msg):
        """Show the live X tracking error (cm) published by the bridge node."""
        if self.error_label:
            val = msg.data
            self.error_label.text = f"{val:.4f} cm"

            # Below 0.05 cm green, below 0.1 cm orange, otherwise red.
            if val < 0.05:
                self.error_label.style = {"color": 0xFF00FF00} # Green
            elif val < 0.1:
                self.error_label.style = {"color": 0xFFFFA500} # Orange
            else:
                self.error_label.style = {"color": 0xFFFF0000} # Red

    def _trigger_color(self, color_name):
        """Trigger the pick task for one color."""
        if self._brain:
            print(f"[UI] Manual trigger: {color_name}")
            self._brain.trigger_intent(f"Cube_{color_name}", f"Bin_{color_name}")

    def _on_reset(self):
        if self._brain:
            print("[UI] State reset")
            self._brain.full_reset()

    def _on_load_pvt_data(self):
        """Load the PVT log written by the bridge node and show the maximum X tracking error."""
        file_path = "/tmp/gantry_pvt_log.csv"
        try:
            if not os.path.exists(file_path):
                if self.error_label:
                    self.error_label.text = "File Not Found"
                    self.error_label.style = {"color": 0xFFFF0000}
                return
            import pandas as pd
            df = pd.read_csv(file_path)
            max_error = df['Error_cm'].max()
            if self.error_label:
                self.error_label.text = f"{max_error:.4f} cm"
                self.error_label.style = {"color": 0xFF00FF00 if max_error < 0.1 else 0xFFFF0000}
        except Exception as e:
            print(f"[AutoPick] CSV read error: {e}")

    def _on_update(self, e):
        """Spin the UI ROS node so subscription callbacks run, then step the brain."""
        if rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0)

        dt = e.payload["dt"]
        if self._brain:
            self._brain.update(dt)

    def _setup_selection_listener(self):
        self._events = omni.usd.get_context().get_stage_event_stream()
        self._sub = self._events.create_subscription_to_pop(self._on_stage_event, name="AutoPick_Selection_Listener")
        print("[UI] Selection listener active")

    def _on_stage_event(self, event):
        if event.type == int(omni.usd.StageEventType.SELECTION_CHANGED):
            selected_paths = omni.usd.get_context().get_selection().get_selected_prim_paths()
            if not selected_paths: return
            selected_path = selected_paths[0]
            if getattr(self._brain, 'state', None) != "IDLE":
                omni.usd.get_context().get_selection().clear_selected_prim_paths()
                return

            if "Bin_Green" in selected_path:
                self._brain.trigger_intent("Cube_Green", "Bin_Green")
                omni.usd.get_context().get_selection().clear_selected_prim_paths()
            elif "Bin_Blue" in selected_path:
                self._brain.trigger_intent("Cube_Blue", "Bin_Blue")
                omni.usd.get_context().get_selection().clear_selected_prim_paths()
            elif "Bin_Red" in selected_path:
                self._brain.trigger_intent("Cube_Red", "Bin_Red")
                omni.usd.get_context().get_selection().clear_selected_prim_paths()

    def on_shutdown(self):
        print("[AutoPick] Extension closed")
        if self._update_sub: self._update_sub = None
        if self._sub: self._sub = None
        # Known issue: the brain's ROS node (isaac_pvt_commander) is never destroyed on shutdown.
        if self._brain: self._brain = None
        if self.node:
            self.node.destroy_node()
        if self._window:
            self._window.destroy()
            self._window = None
