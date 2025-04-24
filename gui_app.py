import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseArray
import threading
import csv
import os
import time
import sys
import json
import time
import subprocess
import psutil
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtGui import QFont
import multiprocessing as mp
from queue import Empty

CONFIG_FILE = "config/gui_config.json"

DEFAULTS = {
    "link_number": 50,
    "fish_length": 1.220,
    "fish_width": 0.360,
    "membrance_width": 0.170,
    "membrance_length": 1.096,
}


class DataRecorder(Node):
    def __init__(self, gui=None, plot_queue=None):
        super().__init__('data_recorder')
        self.subscription = self.create_subscription(
            PoseArray,
            '/world/fish_world/dynamic_pose/info',
            self.pose_callback,
            10
        )
        self.recording = False
        self.file = None
        self.writer = None
        self.start_time = None
        self.last_positions = None
        self.last_orientations = None
        self.last_time = None

        # Buffers for CSV recording
        self.time_buffer = []
        self.position_buffer = {'x': [], 'y': [], 'z': []}
        self.orientation_buffer = {'x': [], 'y': [], 'z': []}
        self.linear_velocity_buffer = {'x': [], 'y': [], 'z': []}
        self.angular_velocity_buffer = {'x': [], 'y': [], 'z': []}
        
        self.low_pass_alpha = 0.05
        self.derivative_num = 10
        self.gui = gui
        self.plot_queue = plot_queue
        self.max_buffer_size = 999999999  # ~30 seconds at 10 Hz

    def get_new_file_path(self, base_path='data/simulation.csv'):
        if not os.path.exists(base_path):
            return base_path
        counter = 1
        while True:
            new_path = f"{os.path.splitext(base_path)[0]}_{counter}.csv"
            if not os.path.exists(new_path):
                return new_path
            counter += 1

    def start_recording(self):
        if not self.recording:
            file_path = self.get_new_file_path()
            self.file = open(file_path, mode='w', newline='')
            self.writer = csv.writer(self.file)
            self.writer.writerow([
                'Time (seconds)', 
                'Orientation X', 'Orientation Y', 'Orientation Z',
                'Angular Velocity X', 'Angular Velocity Y', 'Angular Velocity Z',
                'Position X', 'Position Y', 'Position Z',
                'Linear Velocity X', 'Linear Velocity Y', 'Linear Velocity Z'
            ])
            self.start_time = time.time()
            self.last_positions = None
            self.last_orientations = None
            self.last_time = None
            self.recording = True
            self.time_buffer.clear()
            for buf in [self.position_buffer, self.orientation_buffer, 
                       self.linear_velocity_buffer, self.angular_velocity_buffer]:
                for axis in ['x', 'y', 'z']:
                    buf[axis].clear()
            if self.gui:
                self.gui.log(f"💾 Saving fish trajectory to {file_path}")

    def stop_recording(self):
        if self.recording:
            if self.file:
                self.file.close()
            self.file = None
            self.writer = None
            self.recording = False

    def pose_callback(self, msg):
        current_time = time.time()
        if not self.recording or self.writer is None:
            return

        pose = msg.poses[0]
        elapsed = current_time - self.start_time

        pos_x, pos_y, pos_z = pose.position.x, pose.position.y, pose.position.z
        ori_x, ori_y, ori_z, ori_w = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w

        if self.last_positions is not None and self.last_time is not None:
            dt = current_time - self.last_time
            vx = (pos_x - self.last_positions[0]) / dt * self.low_pass_alpha + \
                 (self.linear_velocity_buffer['x'][-1] if self.linear_velocity_buffer['x'] else 0.0) * (1 - self.low_pass_alpha)
            vy = (pos_y - self.last_positions[1]) / dt * self.low_pass_alpha + \
                 (self.linear_velocity_buffer['y'][-1] if self.linear_velocity_buffer['y'] else 0.0) * (1 - self.low_pass_alpha)
            vz = (pos_z - self.last_positions[2]) / dt * self.low_pass_alpha + \
                 (self.linear_velocity_buffer['z'][-1] if self.linear_velocity_buffer['z'] else 0.0) * (1 - self.low_pass_alpha)
            wx = (ori_x - self.last_orientations[0]) / dt * self.low_pass_alpha + \
                 (self.angular_velocity_buffer['x'][-1] if self.angular_velocity_buffer['x'] else 0.0) * (1 - self.low_pass_alpha)
            wy = (ori_y - self.last_orientations[1]) / dt * self.low_pass_alpha + \
                 (self.angular_velocity_buffer['y'][-1] if self.angular_velocity_buffer['y'] else 0.0) * (1 - self.low_pass_alpha)
            wz = (ori_z - self.last_orientations[2]) / dt * self.low_pass_alpha + \
                 (self.angular_velocity_buffer['z'][-1] if self.angular_velocity_buffer['z'] else 0.0) * (1 - self.low_pass_alpha)
        else:
            vx = vy = vz = 0.0
            wx = wy = wz = 0.0

        self.last_positions = (pos_x, pos_y, pos_z)
        self.last_orientations = (ori_x, ori_y, ori_z)
        self.last_time = current_time

        self.writer.writerow([
            elapsed,
            ori_x, ori_y, ori_z,
            wx, wy, wz,
            pos_x, pos_y, pos_z,
            vx, vy, vz
        ])

        self.time_buffer.append(elapsed)
        self.position_buffer['x'].append(pos_x)
        self.position_buffer['y'].append(pos_y)
        self.position_buffer['z'].append(pos_z)
        self.orientation_buffer['x'].append(ori_x)
        self.orientation_buffer['y'].append(ori_y)
        self.orientation_buffer['z'].append(ori_z)
        self.linear_velocity_buffer['x'].append(vx)
        self.linear_velocity_buffer['y'].append(vy)
        self.linear_velocity_buffer['z'].append(vz)
        self.angular_velocity_buffer['x'].append(wx)
        self.angular_velocity_buffer['y'].append(wy)
        self.angular_velocity_buffer['z'].append(wz)

        if len(self.time_buffer) > self.max_buffer_size:
            self.time_buffer.pop(0)
            for buf in [self.position_buffer, self.orientation_buffer, 
                       self.linear_velocity_buffer, self.angular_velocity_buffer]:
                for axis in ['x', 'y', 'z']:
                    buf[axis].pop(0)

        if self.plot_queue:
            try:
                self.plot_queue.put_nowait({
                    'time': elapsed,
                    'position': {'x': pos_x, 'y': pos_y, 'z': pos_z},
                    'orientation': {'x': ori_x, 'y': ori_y, 'z': ori_z},
                    'linear_velocity': {'x': vx, 'y': vy, 'z': vz},
                    'angular_velocity': {'x': wx, 'y': wy, 'z': wz}
                })
            except mp.queues.Full:
                pass


class FishSimLauncher(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Fish Robot Simulator Launcher")
        self.setWindowIcon(QtGui.QIcon("config/fish_icon.png"))
        self.resize(800, 600)
        self.params = DEFAULTS.copy()
        self.ros_proc = None
        self.gz_proc = None
        self.motion_proc = None  # Track motion publisher process
        rclpy.init()
        self.data_recorder = DataRecorder(gui=self)
        self.data_thread = threading.Thread(target=rclpy.spin, args=(self.data_recorder,), daemon=True)
        self.data_thread.start()

        self.init_ui()
        # Curves for fish motion plots
        self.fish_curves = {
            'orientation': {},
            'position': {},
            'angular_velocity': {},
            'linear_velocity': {}
        }

        colors = {'x': 'r', 'y': 'g', 'z': 'b'}

        for key in ['orientation', 'position', 'angular_velocity', 'linear_velocity']:
            for axis in ['x', 'y', 'z']:
                self.fish_curves[key][axis] = self.fish_plots[key].plot(pen=pg.mkPen(colors[axis], width=2), name=f"{axis.upper()}")

        self.update_interval()  # ← Ensures selected_interval is initialized correctly
        self.load_config()
        self.start_health_monitor()

        # Data buffers for graph
        self.cpu_data = []
        self.ram_data = []
        self.time_data = []
        self.start_time = int(time.time())  # Initialize start_time here

        self.fish_plot_timer = QtCore.QTimer()
        self.fish_plot_timer.timeout.connect(self.update_fish_motion_plots)
        self.fish_plot_timer.start(100)  # Update every 100 ms (10 Hz)

        self.motion_countdown_timer = QtCore.QTimer()
        self.motion_countdown_timer.timeout.connect(self.update_motion_countdown)
        self.remaining_motion_time = 0  # in seconds

    def update_interval(self):
        minutes = float(self.interval_selector.currentText().split()[0])
        self.selected_interval = minutes * 60

    def init_ui(self):
        main_layout = QtWidgets.QVBoxLayout()

        # === TOP LAYOUT: SYSTEM HEALTH + ROBOT PARAMETERS ===
        top_layout = QtWidgets.QHBoxLayout()  # Create a horizontal layout for both groups

        # SYSTEM HEALTH
        health_group = QtWidgets.QGroupBox("System Health")
        health_layout = QtWidgets.QVBoxLayout()

        # Top row: CPU/RAM on left, interval selector on right
        health_info_layout = QtWidgets.QHBoxLayout()

        # CPU and RAM labels on the left
        cpu_ram_widget = QtWidgets.QWidget()
        cpu_ram_layout = QtWidgets.QHBoxLayout()
        self.cpu_label = QtWidgets.QLabel("🔲 CPU: N/A%")
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
        self.interval_selector.addItems(["0.5 min", "1 min", "5 min", "10 min", "30 min"])
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
                spinbox.setValue(float(value))
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

        button_layout.addWidget(self.launch_button)
        button_layout.addWidget(self.stop_button)
        button_layout.addWidget(self.restart_button)

        # ✅ Add buttons to form layout (not directly to group box)
        form_layout.addRow(button_layout)

        param_group.setLayout(form_layout)
        param_group.setMinimumWidth(250)

        right_panel_layout = QtWidgets.QVBoxLayout()
        right_panel_layout.addWidget(param_group, 1)

        # === ENVIRONMENT PARAMETERS ===
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

        # === CAMERA MONITOR GROUP ===
        camera_group = QtWidgets.QGroupBox("Camera Monitor")
        camera_layout = QtWidgets.QVBoxLayout()

        # View Mode selection on the same row
        view_mode_layout = QtWidgets.QHBoxLayout()
        view_mode_layout.addWidget(QtWidgets.QLabel("View Mode:"))
        self.camera_mode_selector = QtWidgets.QComboBox()
        self.camera_mode_selector.addItems(["Global", "Horizontal", "Vertical"])
        view_mode_layout.addWidget(self.camera_mode_selector)
        camera_layout.addLayout(view_mode_layout)
        self.camera_mode_selector.currentTextChanged.connect(self.update_camera_view_mode)

        # Tracking toggle button
        self.camera_tracking_btn = QtWidgets.QPushButton("Start Tracking")
        self.camera_tracking_btn.clicked.connect(self.toggle_camera_tracking_mode)
        camera_layout.addWidget(self.camera_tracking_btn)

        camera_group.setLayout(camera_layout)

        # === Combine Ocean Current + Camera Monitor in 1 horizontal layout ===
        env_camera_row = QtWidgets.QHBoxLayout()
        env_camera_row.addWidget(env_group)
        env_camera_row.addWidget(camera_group)

        # === Wrap into right panel layout ===
        right_panel_layout = QtWidgets.QVBoxLayout()
        right_panel_layout.addWidget(param_group)
        right_panel_layout.addLayout(env_camera_row)

        # === Add to top layout ===
        top_layout.addWidget(health_group)
        top_layout.addLayout(right_panel_layout)

        # === Final main layout ===
        main_layout.addLayout(top_layout)


        # === OUTPUT + MOTION PANEL ===
        bottom_layout = QtWidgets.QHBoxLayout()

        # Log Panel
        log_group = QtWidgets.QGroupBox("System Log")
        log_layout = QtWidgets.QVBoxLayout()

        self.output = QtWidgets.QTextEdit()
        self.output.setReadOnly(True)
        self.output.setStyleSheet("background-color: #f0f0f0;")
        self.output.setMinimumHeight(200)

        log_layout.addWidget(self.output)
        log_group.setLayout(log_layout)

        bottom_layout.addWidget(log_group, 1)  # takes 2/3 of space

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

        # === Row 4: Launch/Stop/Reset/Camera Track Toggle buttons ===
        motion_btns = QtWidgets.QHBoxLayout()

        self.launch_motion_btn = QtWidgets.QPushButton("Start Motion")
        self.stop_motion_btn = QtWidgets.QPushButton("Stop Motion")
        self.reset_fish_btn = QtWidgets.QPushButton("Reset Position")
        # self.track_camera_btn = QtWidgets.QPushButton("Start Tracking")  # ⬅️ NEW TOGGLE BUTTON (initial text)

        # Connect buttons to their functions
        self.launch_motion_btn.clicked.connect(self.start_recording_and_launch_motion)
        self.stop_motion_btn.clicked.connect(self.stop_recording_and_stop_motion)
        self.reset_fish_btn.clicked.connect(self.reset_fish_model)
        # self.track_camera_btn.clicked.connect(self.toggle_camera_tracking)  # ⬅️ TOGGLE FUNCTION

        # Initially disable some buttons
        self.stop_motion_btn.setEnabled(False)

        # Add all buttons to the layout
        motion_btns.addWidget(self.launch_motion_btn)
        motion_btns.addWidget(self.stop_motion_btn)
        motion_btns.addWidget(self.reset_fish_btn)
        # motion_btns.addWidget(self.track_camera_btn)

        motion_layout.addLayout(motion_btns)

        motion_group.setLayout(motion_layout)
        bottom_layout.addWidget(motion_group, 1)

        # Add to main layout
        main_layout.addLayout(bottom_layout)

        # === FISH MOTION REAL-TIME PLOTS ===
        fish_motion_group = QtWidgets.QGroupBox("Fish Motion - Real-time Data")
        fish_motion_main_layout = QtWidgets.QVBoxLayout()

        # 1st row: all 4 plots side-by-side
        fish_motion_plots_layout = QtWidgets.QHBoxLayout()

        self.fish_plots = {}

        def create_plot(title, y_label):
            plot = pg.PlotWidget()
            plot.setTitle(title)
            # plot.setLabel('left', y_label)
            # plot.setLabel('bottom', 'Time (s)')
            # plot.addLegend()
            plot.setBackground('w')
            plot.showGrid(x=True, y=True, alpha=0.3)
            # plot.setFixedWidth(320)  # Make plots fixed width for nice alignment
            plot.setMinimumHeight(280)
            plot.setMinimumWidth(280)
            return plot

        # Create 4 horizontally distributed plots
        self.fish_plots['orientation'] = create_plot("Orientation vs Time", "Orientation (rad)")
        self.fish_plots['position'] = create_plot("Position vs Time", "Position (m)")
        self.fish_plots['angular_velocity'] = create_plot("Angular Velocity vs Time", "Angular Velocity (rad/s)")
        self.fish_plots['linear_velocity'] = create_plot("Linear Velocity vs Time", "Velocity (m/s)")

        for key in ['orientation', 'position', 'angular_velocity', 'linear_velocity']:
            fish_motion_plots_layout.addWidget(self.fish_plots[key])

        fish_motion_main_layout.addLayout(fish_motion_plots_layout)

        fish_motion_group.setLayout(fish_motion_main_layout)

        # Add it into your main layout
        main_layout.addWidget(fish_motion_group)

        self.setLayout(main_layout)

        # Internal tracking state
        self.tracking_enabled = False  # ⬅️ Add this somewhere in your __init__ or setup

    def update_fish_motion_plots(self):
        # Only update if recording
        if not self.data_recorder.recording:
            return
        
        recorder = self.data_recorder
        time_data = recorder.time_buffer

        if len(time_data) < 2:
            return

        for key, buffer in [
            ('orientation', recorder.orientation_buffer),
            ('position', recorder.position_buffer),
            ('angular_velocity', recorder.angular_velocity_buffer),
            ('linear_velocity', recorder.linear_velocity_buffer)
        ]:
            for axis in ['x', 'y', 'z']:
                self.fish_curves[key][axis].setData(time_data, buffer[axis])

        # Optional: auto-scroll x-axis
        for plot in self.fish_plots.values():
            plot.setXRange(max(0, time_data[-1] - 30), time_data[-1])  # Show last 30 seconds

    def update_camera_view_mode(self, new_mode):
        if getattr(self, 'tracking_enabled', False):
            self.log(f"🔄 Updating camera view to [{new_mode}] mode...")

            offset = {
                "Global": (-1.5, -1.5, 2.0),
                "Horizontal": (-1.0, 0.0, 0.0),
                "Vertical": (0.0, 0.0, 2.0),
            }.get(new_mode, (-1.5, -1.5, 2.0))

            offset_cmd = (
                f"source /opt/ros/humble/setup.bash && "
                f"ign service -s /gui/follow/offset "
                f"--reqtype ignition.msgs.Vector3d "
                f"--reptype ignition.msgs.Boolean "
                f"--timeout 3000 "
                f"--req 'x: {offset[0]}, y: {offset[1]}, z: {offset[2]}'"
            )

            result = subprocess.run(offset_cmd, shell=True, executable="/bin/bash", capture_output=True, text=True)
            if result.returncode == 0:
                self.log(f"✅ View mode updated to [{new_mode}]")
            else:
                self.log(f"❌ Failed to update view mode: {result.stderr}")

    def toggle_camera_tracking_mode(self):
        selected_mode = self.camera_mode_selector.currentText()

        if not getattr(self, 'tracking_enabled', False):
            self.log(f"🎯 Activating camera tracking [{selected_mode}]...")

            # Start tracking fish model
            track_cmd = (
                "source /opt/ros/humble/setup.bash && "
                "ign service -s /gui/follow "
                "--reqtype ignition.msgs.StringMsg "
                "--reptype ignition.msgs.Boolean "
                "--timeout 3000 "
                "--req 'data: \"fish_model\"'"
            )

            # Choose camera offset by mode
            offset = {
                "Global":  (-1.5, -1.5, 2.0),
                "Horizontal": (-1.0, 0, 0.0),
                "Vertical": (0.0, 0.0, 2.0),
            }.get(selected_mode, (-1.5, -1.5, 2.0))

            offset_cmd = (
                f"source /opt/ros/humble/setup.bash && "
                f"ign service -s /gui/follow/offset "
                f"--reqtype ignition.msgs.Vector3d "
                f"--reptype ignition.msgs.Boolean "
                f"--timeout 3000 "
                f"--req 'x: {offset[0]}, y: {offset[1]}, z: {offset[2]}'"
            )

            result1 = subprocess.run(track_cmd, shell=True, executable="/bin/bash", capture_output=True, text=True)
            result2 = subprocess.run(offset_cmd, shell=True, executable="/bin/bash", capture_output=True, text=True)

            if result1.returncode == 0 and result2.returncode == 0:
                self.log(f"✅ Camera now tracking fish [{selected_mode}]")
                self.camera_tracking_btn.setText("Stop Tracking")
                self.tracking_enabled = True
            else:
                self.log(f"❌ Failed to start tracking: {result1.stderr} {result2.stderr}")
        else:
            # Stop tracking
            self.log("🛑 Deactivating camera tracking...")

            stop_cmd = (
                "source /opt/ros/humble/setup.bash && "
                "ign service -s /gui/follow "
                "--reqtype ignition.msgs.StringMsg "
                "--reptype ignition.msgs.Boolean "
                "--timeout 3000 "
                "--req 'data: \"\"'"
            )

            result = subprocess.run(stop_cmd, shell=True, executable="/bin/bash", capture_output=True, text=True)
            if result.returncode == 0:
                self.log("✅ Camera tracking stopped.")
                self.camera_tracking_btn.setText("Start Tracking")
                self.tracking_enabled = False
            else:
                self.log(f"❌ Failed to stop tracking: {result.stderr}")

    def reset_fish_model(self):
        self.log("🔄 Resetting fish model position and recalibrating camera...")

        # Command to reset fish model pose
        reset_pose_cmd = (
            "source /opt/ros/humble/setup.bash && "
            "ign service -s /world/fish_world/set_pose "
            "--reqtype ignition.msgs.Pose "
            "--reptype ignition.msgs.Boolean "
            "--timeout 3000 "
            "--req 'name: \"fish_model\", position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}'"
        )

        # Command to recalibrate GUI camera pose
        recalib_camera_cmd = (
            "source /opt/ros/humble/setup.bash && "
            "ign service -s /gui/move_to/pose "
            "--reqtype ignition.msgs.GUICamera "
            "--reptype ignition.msgs.Boolean "
            "--timeout 3000 "
            "--req 'pose: {position: {x: -1.5, y: -1.5, z: 2.0}, orientation: {x: -0.1, y: 0.2, z: 0.5, w: 1.0}}'"
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

        self.apply_environment_params()

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
        self.cpu_label.setText(f"🔲 CPU: {cpu}%")
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
        
    def start_recording_and_launch_motion(self):
        self.save_motion_config()
        self.log("🚀 Launching fish motion publisher and starting recording & real-time plotting...")
        
        # === ✨ Reset plots and buffers before starting
        self.reset_fish_motion_plots()

        # Start fresh recording
        self.data_recorder.start_recording()
        self.launch_motion_publisher()

        # ✨ Setup countdown
        self.remaining_motion_time = self.selected_interval  # from system health interval
        self.update_motion_countdown_button_text()
        self.motion_countdown_timer.start(1000)  # 1000 ms = 1 second

    def update_motion_countdown_button_text(self):
        self.launch_motion_btn.setText(f"Remaining: {self.remaining_motion_time}s")

    def update_motion_countdown(self):
        if self.remaining_motion_time > 0:
            self.remaining_motion_time -= 1
            self.update_motion_countdown_button_text()
        else:
            self.motion_countdown_timer.stop()
            self.log("🛑 Motion timeout reached, stopping automatically...")
            self.stop_recording_and_stop_motion()

    def reset_fish_motion_plots(self):
        self.log("🔄 Resetting fish motion plots and buffers...")

        # Clear data buffers
        self.data_recorder.time_buffer.clear()
        for buffer in [
            self.data_recorder.position_buffer,
            self.data_recorder.orientation_buffer,
            self.data_recorder.linear_velocity_buffer,
            self.data_recorder.angular_velocity_buffer
        ]:
            for axis in ['x', 'y', 'z']:
                buffer[axis].clear()

        # Clear graphical curves
        for key in self.fish_curves:
            for axis in self.fish_curves[key]:
                self.fish_curves[key][axis].clear()

    def stop_recording_and_stop_motion(self):
        self.log("🛑 Stopping recording and stopping motion publisher...")
        self.data_recorder.stop_recording()
        self.stop_motion()

        # ✨ Stop countdown timer and reset button text
        self.motion_countdown_timer.stop()
        self.launch_motion_btn.setText("Start Motion")

    def closeEvent(self, event):
        self.data_recorder.stop_recording()
        rclpy.shutdown()
        self.data_thread.join()
        event.accept()

def main():
    app = QtWidgets.QApplication(sys.argv)

    # Apply Ubuntu default font
    app.setFont(QFont("Ubuntu", 10))
    app.setStyle("Fusion")
    win = FishSimLauncher()
    win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
