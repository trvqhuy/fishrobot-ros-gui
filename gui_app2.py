import sys
import json
import time
import subprocess
import psutil
import pyqtgraph as pg
import threading
import csv
import os

from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtGui import QFont

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseArray

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

CONFIG_FILE = "config/gui_config.json"

DEFAULTS = {
    "link_number": 50,
    "fish_length": 1.220,
    "fish_width": 0.360,
    "membrance_width": 0.170,
    "membrance_length": 1.096,
}

# ===== Sensor Node Class (copied from sensor_reading.py) =====
class ImuListener(Node):
    def __init__(self):
        super().__init__('imu_listener_gui')
        self.subscription_pose = self.create_subscription(
            PoseArray,
            '/world/fish_world/dynamic_pose/info',
            self.listener_callback_pose,
            10)
        self.pos_times = []
        self.orientations = {'x': [], 'y': [], 'z': [], 'w': []}
        self.angular_velocities = {'x': [], 'y': [], 'z': []}
        self.positions = {'x': [], 'y': [], 'z': []}
        self.linear_velocities = {'x': [], 'y': [], 'z': []}

    def listener_callback_pose(self, msg):
        current_time = time.time()
        self.pos_times.append(current_time)
        pose = msg.poses[0]
        self.positions['x'].append(pose.position.x)
        self.positions['y'].append(pose.position.y)
        self.positions['z'].append(pose.position.z)
        self.orientations['x'].append(pose.orientation.x)
        self.orientations['y'].append(pose.orientation.y)
        self.orientations['z'].append(pose.orientation.z)
        self.orientations['w'].append(pose.orientation.w)

        low_pass_alpha = 0.1
        derivative_num = 10

        if len(self.positions['x']) > derivative_num:
            dt = current_time - self.pos_times[-1-derivative_num]
            dx = self.positions['x'][-1] - self.positions['x'][-1-derivative_num]
            dy = self.positions['y'][-1] - self.positions['y'][-1-derivative_num]
            dz = self.positions['z'][-1] - self.positions['z'][-1-derivative_num]
            vx = dx / dt * low_pass_alpha + self.linear_velocities['x'][-1] * (1 - low_pass_alpha)
            vy = dy / dt * low_pass_alpha + self.linear_velocities['y'][-1] * (1 - low_pass_alpha)
            vz = dz / dt * low_pass_alpha + self.linear_velocities['z'][-1] * (1 - low_pass_alpha)
            self.linear_velocities['x'].append(vx)
            self.linear_velocities['y'].append(vy)
            self.linear_velocities['z'].append(vz)

            d_orientation_x = self.orientations['x'][-1] - self.orientations['x'][-1-derivative_num]
            d_orientation_y = self.orientations['y'][-1] - self.orientations['y'][-1-derivative_num]
            d_orientation_z = self.orientations['z'][-1] - self.orientations['z'][-1-derivative_num]
            angular_velocity_x = d_orientation_x / dt * low_pass_alpha + self.angular_velocities['x'][-1] * (1 - low_pass_alpha)
            angular_velocity_y = d_orientation_y / dt * low_pass_alpha + self.angular_velocities['y'][-1] * (1 - low_pass_alpha)
            angular_velocity_z = d_orientation_z / dt * low_pass_alpha + self.angular_velocities['z'][-1] * (1 - low_pass_alpha)
            self.angular_velocities['x'].append(angular_velocity_x)
            self.angular_velocities['y'].append(angular_velocity_y)
            self.angular_velocities['z'].append(angular_velocity_z)
        else:
            self.linear_velocities['x'].append(0.0)
            self.linear_velocities['y'].append(0.0)
            self.linear_velocities['z'].append(0.0)
            self.angular_velocities['x'].append(0.0)
            self.angular_velocities['y'].append(0.0)
            self.angular_velocities['z'].append(0.0)

# ===== GUI Class (your original GUI, extended carefully) =====
class FishSimLauncher(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("🐟 Fish Robot Simulator Launcher")
        self.setWindowIcon(QtGui.QIcon())  
        self.resize(1200, 700)
        self.params = DEFAULTS.copy()
        self.ros_proc = None
        self.gz_proc = None
        self.motion_proc = None
        self.plot_thread = None
        self.sensor_node = None

        self.init_ui()
        self.update_interval()
        self.load_config()
        self.start_health_monitor()

        # ROS2 Init
        rclpy.init()
        self.sensor_node = ImuListener()
        self.start_plotting()
        self.start_csv_logging()


    def update_interval(self):
        minutes = int(self.interval_selector.currentText().split()[0])
        self.selected_interval = minutes * 60

    def init_ui(self):
        main_layout = QtWidgets.QVBoxLayout()

        # === TOP LAYOUT: SYSTEM HEALTH + ROBOT PARAMETERS ===
        # (no change to this section...)
        top_layout = QtWidgets.QHBoxLayout()  # Create a horizontal layout for both groups

        # SYSTEM HEALTH
        health_group = QtWidgets.QGroupBox("System Health")
        health_layout = QtWidgets.QVBoxLayout()

        # Top row: CPU/RAM on left, interval selector on right
        health_info_layout = QtWidgets.QHBoxLayout()

        # CPU and RAM labels on the left
        cpu_ram_widget = QtWidgets.QWidget()
        cpu_ram_layout = QtWidgets.QHBoxLayout()
        self.cpu_label = QtWidgets.QLabel("🧠 CPU: N/A%")
        self.ram_label = QtWidgets.QLabel("💾 RAM: N/A%")
        self.cpu_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        self.ram_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        cpu_ram_layout.addWidget(self.cpu_label)
        cpu_ram_layout.addWidget(self.ram_label)
        cpu_ram_layout.setContentsMargins(0, 0, 0, 0)
        cpu_ram_widget.setLayout(cpu_ram_layout)

        # Interval selector on the right
        interval_widget = QtWidgets.QWidget()
        interval_layout = QtWidgets.QHBoxLayout()
        interval_layout.addWidget(QtWidgets.QLabel("Graph Interval:"))
        self.interval_selector = QtWidgets.QComboBox()
        self.interval_selector.addItems(["1 min", "5 min", "10 min", "30 min"])
        self.interval_selector.setCurrentIndex(1)
        self.interval_selector.currentIndexChanged.connect(self.update_interval)
        self.selected_interval = 5 * 60
        interval_layout.addWidget(self.interval_selector)
        interval_layout.setContentsMargins(0, 0, 0, 0)
        interval_widget.setLayout(interval_layout)

        # Add both widgets with stretch in between (justify)
        health_info_layout.addWidget(cpu_ram_widget)
        health_info_layout.addStretch()
        health_info_layout.addWidget(interval_widget)

        health_layout.addLayout(health_info_layout)

        # GRAPH inside the health group
        self.graph_widget = pg.PlotWidget()
        self.graph_widget.setTitle("System Resource Usage (CPU, RAM)")
        self.graph_widget.setLabel('left', 'Usage (%)')
        self.graph_widget.setLabel('bottom', 'Time (s)')
        self.graph_widget.setMinimumWidth(600)
        self.graph_widget.setMaximumHeight(280)
        self.graph_widget.setBackground('white')
        self.graph_widget.showGrid(x=True, y=True, alpha=0.3)

        # # Optional: Customize X-axis ticks (every 10 seconds)
        # bottom_axis = self.graph_widget.getAxis('bottom')
        # bottom_axis.setTickSpacing(10, 10)

        # Create curves for CPU and RAM usage
        self.cpu_curve = self.graph_widget.plot(pen=pg.mkPen('r', width=2))  # Red line for CPU
        self.ram_curve = self.graph_widget.plot(pen=pg.mkPen('b', width=2))  # Blue line for RAM
        
        # Add graph widget to health layout
        health_layout.addWidget(self.graph_widget)
        health_group.setLayout(health_layout)
        # health_group.setMaximumWidth(400)  # Set max width for health group

        # ROBOT PARAMETERS
        param_group = QtWidgets.QGroupBox("Fish Robot - ROS Simulation Parameters")
        form_layout = QtWidgets.QFormLayout()
        self.widgets = {}

        for key, value in self.params.items():
            label = key.replace('_', ' ').capitalize()
            if isinstance(value, bool):
                checkbox = QtWidgets.QCheckBox()
                checkbox.setChecked(value)
                self.widgets[key] = checkbox
                form_layout.addRow(f"{label}:", checkbox)
            else:
                spinbox = QtWidgets.QDoubleSpinBox()
                spinbox.setDecimals(2)
                spinbox.setValue(float(value))  # Ensure it's numeric in case of string
                spinbox.setRange(0.01, 100.0)
                if "number" in key:
                    spinbox.setDecimals(0)
                    spinbox.setSingleStep(1)
                self.widgets[key] = spinbox
                form_layout.addRow(f"{label}:", spinbox)

        # === BUTTONS ===
        button_layout = QtWidgets.QHBoxLayout()
        # 🚀 Launch Button (Green)
        self.launch_button = QtWidgets.QPushButton("Launch")
        self.launch_button.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-weight: bold;
            }
            QPushButton:disabled {
                background-color: #A5D6A7;  /* lighter green */
                color: #eeeeee;
            }
        """)

        # 🛑 Stop Button (Red)
        self.stop_button = QtWidgets.QPushButton("Stop")
        self.stop_button.setStyleSheet("""
            QPushButton {
                background-color: #F44336;
                color: white;
                font-weight: bold;
            }
            QPushButton:disabled {
                background-color: #EF9A9A;  /* lighter red */
                color: #eeeeee;
            }
        """)
        self.stop_button.setEnabled(False)

        # 🔁 Restart Button (Orange)
        self.restart_button = QtWidgets.QPushButton("Restart")
        self.restart_button.setStyleSheet("""
            QPushButton {
                background-color: #FF9800;
                color: white;
                font-weight: bold;
            }
            QPushButton:disabled {
                background-color: #FFCC80;  /* lighter orange */
                color: #eeeeee;
            }
        """)
        self.restart_button.setEnabled(False)

        self.launch_button.clicked.connect(self.launch_all)
        self.stop_button.clicked.connect(self.stop_all)
        self.restart_button.clicked.connect(self.restart_all)

        button_layout.addStretch()
        button_layout.addWidget(self.launch_button)
        button_layout.addWidget(self.stop_button)
        button_layout.addWidget(self.restart_button)
        button_layout.addStretch()

        # ✅ Add buttons to form layout (not directly to group box)
        form_layout.addRow(button_layout)

        param_group.setLayout(form_layout)
        param_group.setMinimumWidth(250)

        # === RIGHT SIDE: Robot + Environment Params in a vertical layout ===
        right_panel_layout = QtWidgets.QVBoxLayout()
        right_panel_layout.addWidget(param_group)

        # ENVIRONMENT PARAMETERS
        env_group = QtWidgets.QGroupBox("Ocean Current Settings")
        env_main_layout = QtWidgets.QVBoxLayout()

        # Row 1: X, Y, Z spin boxes
        xyz_layout = QtWidgets.QHBoxLayout()
        self.env_x = QtWidgets.QDoubleSpinBox()
        self.env_y = QtWidgets.QDoubleSpinBox()
        self.env_z = QtWidgets.QDoubleSpinBox()

        for spin, label in zip([self.env_x, self.env_y, self.env_z], ["X", "Y", "Z"]):
            spin.setRange(-10.0, 10.0)
            spin.setDecimals(3)
            spin.setSingleStep(0.1)
            labeled = QtWidgets.QHBoxLayout()
            labeled.addWidget(QtWidgets.QLabel(f"{label}:"))
            labeled.addWidget(spin)
            xyz_layout.addLayout(labeled)

        env_main_layout.addLayout(xyz_layout)

        # Row 2: Apply + Reset buttons
        button_layout = QtWidgets.QHBoxLayout()
        self.env_apply_button = QtWidgets.QPushButton("Apply")
        self.env_reset_button = QtWidgets.QPushButton("Reset")

        self.env_apply_button.clicked.connect(self.apply_environment_params)
        self.env_reset_button.clicked.connect(self.reset_environment_params)

        button_layout.addWidget(self.env_apply_button)
        button_layout.addWidget(self.env_reset_button)

        env_main_layout.addLayout(button_layout)
        env_group.setLayout(env_main_layout)
        right_panel_layout.addWidget(env_group)

        # Add both system health and right panel to top layout
        top_layout.addWidget(health_group)
        top_layout.addLayout(right_panel_layout)

        # Add the top_layout to the main_layout
        main_layout.addLayout(top_layout)

        # (... your top layout code here ...)
        
        # === OUTPUT + MOTION + PLOT PANEL ===
        bottom_layout = QtWidgets.QHBoxLayout()

        # Log Panel
        log_group = QtWidgets.QGroupBox("System Log")
        log_layout = QtWidgets.QVBoxLayout()
        self.output = QtWidgets.QTextEdit()
        self.output.setReadOnly(True)
        self.output.setStyleSheet("background-color: #f0f0f0;")
        self.output.setMinimumHeight(150)
        log_layout.addWidget(self.output)
        log_group.setLayout(log_layout)
        bottom_layout.addWidget(log_group, 1)

        # Plots Panel
        plot_group = QtWidgets.QGroupBox("Real-Time Plots")
        plot_layout = QtWidgets.QVBoxLayout()
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot_orientation = self.plot_widget.addPlot(title="Orientation XYZ")
        self.plot_widget.nextRow()
        self.plot_position = self.plot_widget.addPlot(title="Position XYZ")
        self.plot_widget.nextRow()
        self.plot_velocity = self.plot_widget.addPlot(title="Linear Velocity XYZ")
        self.plot_widget.nextRow()
        self.plot_angular = self.plot_widget.addPlot(title="Angular Velocity XYZ")
        plot_layout.addWidget(self.plot_widget)
        plot_group.setLayout(plot_layout)
        bottom_layout.addWidget(plot_group, 2)

        main_layout.addLayout(bottom_layout)
        self.setLayout(main_layout)
        
        # === FISH MOTION CONTROL PANEL ===
        motion_group = QtWidgets.QGroupBox("Fish Motion Parameters")
        motion_layout = QtWidgets.QVBoxLayout()
        self.motion_widgets = {}

        # === Row 1: Wave type and mode ===
        wave_settings = QtWidgets.QHBoxLayout()

        self.wave_type_combo = QtWidgets.QComboBox()
        self.wave_type_combo.addItems(["linear", "quadratic", "elliptic"])
        self.wave_type_combo.currentTextChanged.connect(self.on_wave_type_change)
        wave_settings.addWidget(QtWidgets.QLabel("Wave Type:"))
        wave_settings.addWidget(self.wave_type_combo)

        self.wave_mode_combo = QtWidgets.QComboBox()
        self.wave_mode_combo.addItems(["traveling", "standing"])
        wave_settings.addWidget(QtWidgets.QLabel("Wave Mode:"))
        wave_settings.addWidget(self.wave_mode_combo)

        self.wave_number_spin = QtWidgets.QDoubleSpinBox()
        self.wave_number_spin.setRange(0.0, 10.0)
        self.wave_number_spin.setSingleStep(0.1)
        self.wave_number_spin.setValue(1.0)
        wave_settings.addWidget(QtWidgets.QLabel("Wave Number:"))
        wave_settings.addWidget(self.wave_number_spin)

        motion_layout.addLayout(wave_settings)

        # === Row 2: Membrane ON/OFF checkboxes ===
        # on_row = QtWidgets.QHBoxLayout()
        # self.m1_on = QtWidgets.QCheckBox("Membrane 1 ON"); self.m1_on.setChecked(True)
        # self.m2_on = QtWidgets.QCheckBox("Membrane 2 ON"); self.m2_on.setChecked(True)
        # on_row.addWidget(self.m1_on); on_row.addStretch(); on_row.addWidget(self.m2_on)
        # motion_layout.addLayout(on_row)

        # === Row 3: Membrane 1 and 2 parameter groups side-by-side ===
        membrane_row = QtWidgets.QHBoxLayout()

        def create_membrane_group(label_prefix, suffix):
            group = QtWidgets.QGroupBox(label_prefix)
            layout = QtWidgets.QFormLayout()

            def add_spin(key, val, minv, maxv, step=0.1):
                box = QtWidgets.QDoubleSpinBox()
                box.setRange(minv, maxv); box.setValue(val); box.setSingleStep(step)
                self.motion_widgets[f"{suffix}_{key}"] = box
                layout.addRow(key.replace('_', ' ').capitalize(), box)

            def add_check(key, checked=True):
                box = QtWidgets.QCheckBox()
                box.setChecked(checked)
                self.motion_widgets[f"{suffix}_{key}"] = box
                layout.addRow(key.replace('_', ' ').capitalize(), box)

            add_spin("frequency", 0.2, -10.0, 10.0)
            add_spin("base_amplitude", 30.0, -180.0, 180.0)
            add_spin("shifted_amplitude", 10.0, -180.0, 180.0)
            add_spin("phase_offset", 180.0 if suffix == "m1" else 0.0, -360.0, 360.0)
            add_spin("scale_factor", 1.0, 0.0, 2.0)
            add_check("forward_s.f.")

            group.setLayout(layout)
            return group

        membrane_row.addWidget(create_membrane_group("Membrane 1", "m1"))
        membrane_row.addWidget(create_membrane_group("Membrane 2", "m2"))
        motion_layout.addLayout(membrane_row)

        # === Row 4: Apply/Launch/Stop/Reset buttons ===
        motion_btns = QtWidgets.QHBoxLayout()

        self.apply_motion_btn = QtWidgets.QPushButton("Apply Motion")
        self.launch_motion_btn = QtWidgets.QPushButton("Start")
        self.stop_motion_btn = QtWidgets.QPushButton("Stop")
        self.reset_fish_btn = QtWidgets.QPushButton("Reset Position")  # ⬅️ NEW BUTTON

        # Connect buttons to their functions
        self.apply_motion_btn.clicked.connect(self.save_motion_config)
        self.launch_motion_btn.clicked.connect(self.launch_motion_publisher)
        self.stop_motion_btn.clicked.connect(self.stop_motion)
        self.reset_fish_btn.clicked.connect(self.reset_fish_model)  # ⬅️ CONNECT new function

        # Initially disable some buttons
        self.stop_motion_btn.setEnabled(False)

        # Add all buttons to the layout
        motion_btns.addWidget(self.apply_motion_btn)
        motion_btns.addWidget(self.launch_motion_btn)
        motion_btns.addWidget(self.stop_motion_btn)
        motion_btns.addWidget(self.reset_fish_btn)  # ⬅️ ADD new button

        motion_layout.addLayout(motion_btns)

        motion_group.setLayout(motion_layout)
        bottom_layout.addWidget(motion_group, 1)

        # Add to main layout
        main_layout.addLayout(bottom_layout)

        self.setLayout(main_layout)

    def start_plotting(self):
        self.plot_thread = threading.Thread(target=self.update_plot, daemon=True)
        self.plot_thread.start()

    def update_plot(self):
        while rclpy.ok():
            rclpy.spin_once(self.sensor_node, timeout_sec=0.01)
            times = np.array(self.sensor_node.pos_times)
            if len(times) == 0:
                time.sleep(0.1)
                continue
            times -= times[0]

            def safe_data(d, key):
                return np.array(d.get(key, []))

            # Update plots
            self.plot_orientation.clear()
            self.plot_position.clear()
            self.plot_velocity.clear()
            self.plot_angular.clear()

            self.plot_orientation.plot(times, safe_data(self.sensor_node.orientations, 'x'), pen='r', name='x')
            self.plot_orientation.plot(times, safe_data(self.sensor_node.orientations, 'y'), pen='g', name='y')
            self.plot_orientation.plot(times, safe_data(self.sensor_node.orientations, 'z'), pen='b', name='z')

            self.plot_position.plot(times, safe_data(self.sensor_node.positions, 'x'), pen='r')
            self.plot_position.plot(times, safe_data(self.sensor_node.positions, 'y'), pen='g')
            self.plot_position.plot(times, safe_data(self.sensor_node.positions, 'z'), pen='b')

            self.plot_velocity.plot(times, safe_data(self.sensor_node.linear_velocities, 'x'), pen='r')
            self.plot_velocity.plot(times, safe_data(self.sensor_node.linear_velocities, 'y'), pen='g')
            self.plot_velocity.plot(times, safe_data(self.sensor_node.linear_velocities, 'z'), pen='b')

            self.plot_angular.plot(times, safe_data(self.sensor_node.angular_velocities, 'x'), pen='r')
            self.plot_angular.plot(times, safe_data(self.sensor_node.angular_velocities, 'y'), pen='g')
            self.plot_angular.plot(times, safe_data(self.sensor_node.angular_velocities, 'z'), pen='b')

            time.sleep(0.1)  # Update every 100ms

    def log(self, message):
        self.output.append(f"[{time.strftime('%H:%M:%S')}] {message}")
        self.output.ensureCursorVisible()

    def closeEvent(self, event):
        self.stop_motion()
        rclpy.shutdown()
        event.accept()
    def reset_fish_model(self):
        self.log("🔄 Resetting fish model position and recalibrating camera...")
        
        # Command to reset fish model pose
        reset_pose_cmd = (
            "source /opt/ros/humble/setup.bash && "
            "ign service -s /world/fish_world/set_pose "
            "--reqtype ignition.msgs.Pose "
            "--reptype ignition.msgs.Boolean "
            "--timeout 3000 "
            "--req 'name: \"fish_model\", position: {x: 0.0, y: 0.0, z: 0.2}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}'"
        )

        # Command to recalibrate GUI camera pose
        recalib_camera_cmd = (
            "source /opt/ros/humble/setup.bash && "
            "ign topic -t /gui/camera/pose "
            "-m ignition.msgs.Pose "
            "-p 'position: {x: 3.0, y: 0.0, z: 1.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}'"
        )

        # Run reset pose
        result1 = subprocess.run(reset_pose_cmd, shell=True, executable="/bin/bash", capture_output=True, text=True)
        if result1.returncode == 0:
            self.log("✅ Fish model reset to origin.")
        else:
            self.log(f"❌ Failed to reset fish model: {result1.stderr}")

        # Run camera recalibration
        result2 = subprocess.run(recalib_camera_cmd, shell=True, executable="/bin/bash", capture_output=True, text=True)
        if result2.returncode == 0:
            self.log("✅ Camera recalibrated.")
        else:
            self.log(f"❌ Failed to recalibrate camera: {result2.stderr}")

    # === Function to update scale factor visibility ===
    def on_wave_type_change(self):
        wtype = self.wave_type_combo.currentText()
        for suffix in ["m1", "m2"]:
            sf_widget = self.motion_widgets[f"{suffix}_scale_factor"]
            sf_forward_widget = self.motion_widgets[f"{suffix}_forward_s.f."]
            if wtype == "linear":
                sf_widget.setEnabled(False)
                sf_forward_widget.setEnabled(False)
                sf_widget.setValue(1.0)
            elif wtype == "quadratic":
                sf_widget.setEnabled(True)
                sf_forward_widget.setEnabled(True)
                sf_widget.setRange(0.0, 1.0)
            elif wtype == "elliptic":
                sf_widget.setEnabled(True)
                sf_forward_widget.setEnabled(True)
                sf_widget.setRange(0.0, 1.0)

    def save_motion_config(self):
        try:
            # Save shared parameters
            self.params["wave_type"] = self.wave_type_combo.currentText()
            self.params["wave_mode"] = self.wave_mode_combo.currentText()
            self.params["wave_number"] = self.wave_number_spin.value()
            self.params["membrane_1_on"] = True  # Always on
            self.params["membrane_2_on"] = True  # Always on

            # Save membrane 1 parameters
            self.params["membrane_1"] = {
                "frequency": self.motion_widgets["m1_frequency"].value(),
                "base_amplitude": self.motion_widgets["m1_base_amplitude"].value(),
                "shifted_amplitude": self.motion_widgets["m1_shifted_amplitude"].value(),
                "scale_factor": self.motion_widgets["m1_scale_factor"].value(),
                "forward_sf": self.motion_widgets["m1_forward_s.f."].isChecked(),
                "phase_offset": self.motion_widgets["m1_phase_offset"].value()
            }

            # Save membrane 2 parameters
            self.params["membrane_2"] = {
                "frequency": self.motion_widgets["m2_frequency"].value(),
                "base_amplitude": self.motion_widgets["m2_base_amplitude"].value(),
                "shifted_amplitude": self.motion_widgets["m2_shifted_amplitude"].value(),
                "scale_factor": self.motion_widgets["m2_scale_factor"].value(),
                "forward_sf": self.motion_widgets["m2_forward_s.f."].isChecked(),
                "phase_offset": self.motion_widgets["m2_phase_offset"].value()
            }

            # Write to config file
            with open(CONFIG_FILE, "w") as f:
                json.dump(self.params, f, indent=4)
            self.log("💾 Motion parameters saved to config.")
        except Exception as e:
            self.log(f"❌ Failed to save motion parameters: {str(e)}")

    def launch_motion_publisher(self):
        if self.motion_proc is not None:
            self.log("⚠️ Motion publisher already running.")
            return
        
        self.save_motion_config()
        cmd = (
            "source /opt/ros/humble/setup.bash && "
            "/bin/python3 /home/maycuaaiz/Desktop/FishRobot-ROS/fish_control.py"
        )
        self.log("🚀 Launching fish motion publisher...")
        try:
            self.motion_proc = subprocess.Popen(cmd, shell=True, executable="/bin/bash")
            self.log(f"✅ Motion publisher started. PID: {self.motion_proc.pid}")
            self.launch_motion_btn.setEnabled(False)
            self.stop_motion_btn.setEnabled(True)
        except Exception as e:
            self.log(f"❌ Failed to launch motion publisher: {str(e)}")
            self.motion_proc = None

    def stop_motion(self):
        if self.motion_proc is None:
            self.log("⚠️ No motion publisher running.")
            return
        
        self.log("🛑 Stopping motion publisher...")
        try:
            self.motion_proc.terminate()  # Try graceful termination
            self.motion_proc.wait(timeout=5)  # Wait for process to exit
            self.log(f"✅ Motion publisher (PID: {self.motion_proc.pid}) terminated.")
        except subprocess.TimeoutExpired:
            self.log("⚠️ Motion publisher did not terminate gracefully. Forcing kill...")
            self.motion_proc.kill()  # Force kill if it doesn't terminate
            self.log(f"✅ Motion publisher (PID: {self.motion_proc.pid}) killed.")
        except Exception as e:
            self.log(f"❌ Failed to stop motion publisher: {str(e)}")
        
        self.motion_proc = None
        self.launch_motion_btn.setEnabled(True)
        self.stop_motion_btn.setEnabled(False)

    def reset_environment_params(self):
        self.env_x.setValue(0.0)
        self.env_y.setValue(0.0)
        self.env_z.setValue(0.0)
        self.log("🔁 Ocean current parameters reset to zero.")

    def apply_environment_params(self):
        x = self.env_x.value()
        y = self.env_y.value()
        z = self.env_z.value()

        cmd = (
            f"source /opt/ros/humble/setup.bash && "
            f"ign topic -t /ocean_current -m ignition.msgs.Vector3d "
            f"-p 'x: {x}, y: {y}, z: {z}'"
        )

        self.log(f"🌊 Applying ocean current: x={x}, y={y}, z={z}")
        result = subprocess.run(cmd, shell=True, executable="/bin/bash", capture_output=True, text=True)

        if result.returncode == 0:
            self.log("✅ Ocean current applied.")
        else:
            self.log("❌ Failed to apply ocean current.")
            self.log(result.stderr)
            
    def start_csv_logging(self):
        self.csv_thread = threading.Thread(target=self.update_csv_log, daemon=True)
        self.csv_thread.start()

    def update_csv_log(self):
        base_filename = f"data/fish_robot_log_{int(time.time())}.csv"
        os.makedirs("data", exist_ok=True)
        with open(base_filename, mode='w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                'Time (seconds)', 
                'Orientation X', 'Orientation Y', 'Orientation Z', 
                'Angular Velocity X', 'Angular Velocity Y', 'Angular Velocity Z',
                'Position X', 'Position Y', 'Position Z',
                'Linear Velocity X', 'Linear Velocity Y', 'Linear Velocity Z'
            ])
        
        while rclpy.ok():
            if not self.sensor_node.pos_times:
                time.sleep(1.0)
                continue

            latest_idx = -1
            start_time = self.sensor_node.pos_times[0]
            t = self.sensor_node.pos_times[latest_idx] - start_time

            writer.writerow([
                t,
                self.sensor_node.orientations['x'][latest_idx],
                self.sensor_node.orientations['y'][latest_idx],
                self.sensor_node.orientations['z'][latest_idx],
                self.sensor_node.angular_velocities['x'][latest_idx],
                self.sensor_node.angular_velocities['y'][latest_idx],
                self.sensor_node.angular_velocities['z'][latest_idx],
                self.sensor_node.positions['x'][latest_idx],
                self.sensor_node.positions['y'][latest_idx],
                self.sensor_node.positions['z'][latest_idx],
                self.sensor_node.linear_velocities['x'][latest_idx],
                self.sensor_node.linear_velocities['y'][latest_idx],
                self.sensor_node.linear_velocities['z'][latest_idx],
            ])
            time.sleep(1.0)  # Log every second

    def log(self, message):
        self.output.append(f"[{time.strftime('%H:%M:%S')}] {message}")
        self.output.ensureCursorVisible()

    def load_config(self):
        try:
            with open(CONFIG_FILE) as f:
                self.params.update(json.load(f))

            # Load shared settings
            self.wave_type_combo.setCurrentText(self.params.get("wave_type", "linear"))
            self.wave_mode_combo.setCurrentText(self.params.get("wave_mode", "traveling"))
            self.wave_number_spin.setValue(self.params.get("wave_number", 1.0))
            # No need to load membrane_1_on or membrane_2_on; assume always True

            # Load membrane 1
            m1 = self.params.get("membrane_1", {})
            self.motion_widgets["m1_frequency"].setValue(m1.get("frequency", 0.2))
            self.motion_widgets["m1_base_amplitude"].setValue(m1.get("base_amplitude", 30.0))
            self.motion_widgets["m1_shifted_amplitude"].setValue(m1.get("shifted_amplitude", 10.0))
            self.motion_widgets["m1_scale_factor"].setValue(m1.get("scale_factor", 1.0))
            self.motion_widgets["m1_phase_offset"].setValue(m1.get("phase_offset", 180.0))
            self.motion_widgets["m1_forward_s.f."].setChecked(m1.get("forward_sf", True))

            # Load membrane 2
            m2 = self.params.get("membrane_2", {})
            self.motion_widgets["m2_frequency"].setValue(m2.get("frequency", 0.2))
            self.motion_widgets["m2_base_amplitude"].setValue(m2.get("base_amplitude", 30.0))
            self.motion_widgets["m2_shifted_amplitude"].setValue(m2.get("shifted_amplitude", 10.0))
            self.motion_widgets["m2_scale_factor"].setValue(m2.get("scale_factor", 1.0))
            self.motion_widgets["m2_phase_offset"].setValue(m2.get("phase_offset", 0.0))
            self.motion_widgets["m2_forward_s.f."].setChecked(m2.get("forward_sf", True))

            # Ensure params reflect always-on membranes
            self.params["membrane_1_on"] = True
            self.params["membrane_2_on"] = True

            self.log("✅ Configuration loaded.")
        except Exception as e:
            self.log(f"⚠️ No config found or error loading: {str(e)}. Using default parameters.")
            # Set default membrane states
            self.params["membrane_1_on"] = True
            self.params["membrane_2_on"] = True

    def update_params(self):
        for key, widget in self.widgets.items():
            if isinstance(widget, QtWidgets.QCheckBox):
                self.params[key] = widget.isChecked()
            else:
                self.params[key] = (
                    int(widget.value()) if "number" in key else float(widget.value())
                )
        with open(CONFIG_FILE, "w") as f:
            json.dump(self.params, f, indent=4)
        self.log("💾 Parameters saved.")

    def run_model_generator(self):
        self.update_params()
        self.log("⚙️ Generating model...")
        result = subprocess.run(["/bin/python3", "model_generator.py"], capture_output=True, text=True)
        if result.returncode == 0:
            self.log("✅ Model generation complete.")
        else:
            self.log("❌ Error generating model.")
            self.log(result.stderr)

    def launch_all(self):
        self.run_model_generator()
        self.log("🚀 Launching ROS bridge and Gazebo...")

        bash_command = "source /opt/ros/humble/setup.bash && ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=config/final.bridge.yaml"
        self.ros_proc = subprocess.Popen(bash_command, shell=True, executable="/bin/bash")
        self.log(f"✅ ROS Bridge started. PID: {self.ros_proc.pid}")

        gazebo_command = "ign gazebo config/final.fish_world.sdf"
        self.gz_proc = subprocess.Popen(gazebo_command, shell=True, executable="/bin/bash")
        self.log(f"✅ Gazebo started. PID: {self.gz_proc.pid}")

        self.launch_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.restart_button.setEnabled(True)

    def stop_all(self):
        def kill_processes_by_name(process_name):
            try:
                result = subprocess.check_output(f"ps aux | grep -i {process_name}", shell=True, text=True)
                lines = result.splitlines()
                pids = []
                for line in lines:
                    if process_name in line and "grep" not in line:
                        pid = line.split()[1]
                        pids.append(pid)
                if pids:
                    kill_command = ["kill", "-9"] + pids
                    subprocess.run(kill_command)
                    return True
                return False
            except subprocess.CalledProcessError:
                return False
        
        self.log("🛑 Stopping processes...")

        # Stop motion publisher if running
        if self.motion_proc is not None:
            self.stop_motion()

        ros_stopped = kill_processes_by_name("parameter_bridge")
        gazebo_stopped = kill_processes_by_name("gazebo")

        if ros_stopped:
            self.log("✅ ROS processes terminated.")
        else:
            self.log("❌ No ROS processes found to terminate.")

        if gazebo_stopped:
            self.log("✅ Gazebo processes terminated.")
        else:
            self.log("❌ No Gazebo processes found to terminate.")

        self.launch_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.restart_button.setEnabled(False)

  # Optional: Better default look
    def restart_all(self):
        self.log("🔁 Restarting simulation...")
        self.stop_all()
        QtCore.QTimer.singleShot(1000, self.launch_all)

    def start_health_monitor(self):
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_health)
        self.timer.start(2000)  # Update every 2 seconds

    def update_health(self):
        cpu = psutil.cpu_percent()
        mem = psutil.virtual_memory()
        self.cpu_label.setText(f"🧠 CPU: {cpu}%")
        self.ram_label.setText(f"💾 RAM: {mem.percent}% of {round(mem.total / (1024 ** 3), 1)} GB")

        current_time = int(time.time()) - self.start_time

        # Append new data
        self.time_data.append(current_time)
        self.cpu_data.append(cpu)
        self.ram_data.append(mem.percent)

        # Remove data outside selected interval
        cutoff_time = current_time - self.selected_interval
        while self.time_data and self.time_data[0] < cutoff_time:
            self.time_data.pop(0)
            self.cpu_data.pop(0)
            self.ram_data.pop(0)

        # Update graph only if enough data exists
        if len(self.time_data) > 1:
            self.cpu_curve.setData(self.time_data, self.cpu_data)
            self.ram_curve.setData(self.time_data, self.ram_data)
            self.graph_widget.setXRange(self.time_data[0], self.time_data[-1], padding=0.05)
            self.graph_widget.setYRange(0, 100, padding=0.05)

        # # Update time axis to show every 10th second
        # time_axis = self.graph_widget.getAxis('bottom')
        # time_axis.setTickSpacing(10, 10)
        # time_axis.setLabel("Time (s)", units="s")
        # time_axis.setTicks([[(t, str(t)) for t in self.time_data[::10]]])  # Show every 10th second


# ===== Main Entry =====
def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setFont(QFont("Ubuntu", 10))
    app.setStyle("Fusion")
    win = FishSimLauncher()
    win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()

