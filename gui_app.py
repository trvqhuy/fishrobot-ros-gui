import sys
import json
import time
import subprocess
import psutil
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtGui import QFont

CONFIG_FILE = "config/gui_config.json"

DEFAULTS = {
    "link_number": 50,
    "fish_length": 1.220,
    "fish_width": 0.360,
    "membrance_width": 0.170,
    "membrance_length": 1.096,
    "membrane_1_on": True,
    "membrane_2_on": True,
}

class FishSimLauncher(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("🐟 Fish Robot Simulator Launcher")
        self.setWindowIcon(QtGui.QIcon())  # Add icon if needed
        self.resize(800, 600)
        self.params = DEFAULTS.copy()
        self.ros_proc = None
        self.gz_proc = None

        self.init_ui()
        self.update_interval()  # ← Ensures selected_interval is initialized correctly
        self.load_config()
        self.start_health_monitor()

        # Data buffers for graph
        self.cpu_data = []
        self.ram_data = []
        self.time_data = []
        self.start_time = int(time.time())  # Initialize start_time here

    def update_interval(self):
        minutes = int(self.interval_selector.currentText().split()[0])
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
        param_group = QtWidgets.QGroupBox("Fish Robot - Simulation Parameters")
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
                spinbox.setValue(value)
                spinbox.setRange(0.01, 100.0)
                if "number" in key:
                    spinbox.setDecimals(0)
                    spinbox.setSingleStep(1)
                self.widgets[key] = spinbox
                form_layout.addRow(f"{label}:", spinbox)

        param_group.setLayout(form_layout)
        param_group.setMinimumWidth(250)  # Set minimum width for the parameters group

        # === RIGHT SIDE: Robot + Environment Params in a vertical layout ===
        right_panel_layout = QtWidgets.QVBoxLayout()
        right_panel_layout.addWidget(param_group)

        # ENVIRONMENT PARAMETERS
        env_group = QtWidgets.QGroupBox("🌊 Ocean Current Settings")
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

        button_layout.addStretch()
        button_layout.addWidget(self.env_apply_button)
        button_layout.addWidget(self.env_reset_button)
        button_layout.addStretch()

        env_main_layout.addLayout(button_layout)
        env_group.setLayout(env_main_layout)
        right_panel_layout.addWidget(env_group)


        # Add both system health and right panel to top layout
        top_layout.addWidget(health_group)
        top_layout.addLayout(right_panel_layout)

        # Add the top_layout to the main_layout
        main_layout.addLayout(top_layout)

        # === BUTTONS ===
        button_layout = QtWidgets.QHBoxLayout()
        self.launch_button = QtWidgets.QPushButton("🚀 Launch")
        self.launch_button.clicked.connect(self.launch_all)

        self.stop_button = QtWidgets.QPushButton("🛑 Stop")
        self.stop_button.clicked.connect(self.stop_all)
        self.stop_button.setEnabled(False)

        self.restart_button = QtWidgets.QPushButton("🔁 Restart")
        self.restart_button.clicked.connect(self.restart_all)
        self.restart_button.setEnabled(False)

        button_layout.addStretch()
        button_layout.addWidget(self.launch_button)
        button_layout.addWidget(self.stop_button)
        button_layout.addWidget(self.restart_button)
        button_layout.addStretch()
        main_layout.addLayout(button_layout)

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

        bottom_layout.addWidget(log_group, 2)  # takes 2/3 of space

        # === FISH MOTION CONTROL PANEL ===
        motion_group = QtWidgets.QGroupBox("Fish Motion Parameters")
        motion_layout = QtWidgets.QFormLayout()
        self.motion_widgets = {}

        def add_spin(name, default, minval, maxval, step=0.1, decimals=3):
            box = QtWidgets.QDoubleSpinBox()
            box.setRange(minval, maxval)
            box.setSingleStep(step)
            box.setDecimals(decimals)
            box.setValue(default)
            self.motion_widgets[name] = box
            motion_layout.addRow(name.replace('_', ' ').capitalize(), box)

        def add_check(name, default):
            chk = QtWidgets.QCheckBox()
            chk.setChecked(default)
            self.motion_widgets[name] = chk
            motion_layout.addRow(name.replace('_', ' ').capitalize(), chk)

        add_spin("frequency_1", 1.0, 0.01, 10.0)
        add_spin("frequency_2", 1.0, 0.01, 10.0)
        add_spin("wave_number", 1.0, 0.0, 10.0)
        add_spin("scale_factor", 1.0, 0.0, 2.0)
        add_spin("shifted_amplitude", 60.0, 0.0, 180.0)
        add_check("shifted_upward", True)
        add_check("forward_movement", True)
        add_check("spin_movement", False)
        add_check("forward_sf", True)

        motion_btn_layout = QtWidgets.QHBoxLayout()
        self.apply_motion_btn = QtWidgets.QPushButton("Apply Motion")
        self.launch_motion_btn = QtWidgets.QPushButton("Launch Motion")

        self.apply_motion_btn.clicked.connect(self.save_motion_config)
        self.launch_motion_btn.clicked.connect(self.launch_motion_publisher)

        motion_btn_layout.addWidget(self.apply_motion_btn)
        motion_btn_layout.addWidget(self.launch_motion_btn)

        motion_layout.addRow(motion_btn_layout)
        motion_group.setLayout(motion_layout)

        bottom_layout.addWidget(motion_group, 1)  # take 1/3 of space

        # Add to main layout
        main_layout.addLayout(bottom_layout)

        self.setLayout(main_layout)


    def save_motion_config(self):
        for key, widget in self.motion_widgets.items():
            if isinstance(widget, QtWidgets.QCheckBox):
                self.params[key] = widget.isChecked()
            else:
                self.params[key] = float(widget.value())

        with open(CONFIG_FILE, "w") as f:
            json.dump(self.params, f, indent=4)
        self.log("💾 Motion parameters saved to config.")

    def launch_motion_publisher(self):
        self.save_motion_config()
        cmd = (
            "source /opt/ros/humble/setup.bash && "
            "/bin/python3 /home/maycuaaiz/Desktop/FishRobot-ROS/publisher_member_function.py"
        )
        self.log("🚀 Launching fish motion publisher...")
        subprocess.Popen(cmd, shell=True, executable="/bin/bash")

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

    def log(self, message):
        self.output.append(f"[{time.strftime('%H:%M:%S')}] {message}")
        self.output.ensureCursorVisible()

    def load_config(self):
        try:
            with open(CONFIG_FILE) as f:
                self.params.update(json.load(f))
                for key, widget in self.widgets.items():
                    if isinstance(widget, QtWidgets.QCheckBox):
                        widget.setChecked(self.params[key])
                    else:
                        widget.setValue(self.params[key])
            self.log("✅ Configuration loaded.")
        except Exception:
            self.log("⚠️ No config found. Using default parameters.")

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
