# 🐟 FishRobot ROS GUI

A PyQt5-based graphical interface for launching, monitoring, and controlling a fish robot simulation in ROS 2 and Gazebo.

## 📦 Features

- Launches ROS 2 bridge and Gazebo simulation
- Real-time CPU & RAM usage graphs
- Configurable robot parameters
- Ocean current environment control
- Fish motion control panel with live parameter tuning

## 🚀 Run the GUI Program

1. Navigate to the project directory:

   ```bash
   cd ~/Desktop/FishRobot-ROS
   ```

2. Activate the Python virtual environment:

   ```bash
   source .venv/bin/activate
   ```

3. Install dependencies:

   ```bash
   pip install -r requirements.txt
   ```

4. Launch the GUI:

   ```bash
   python3 gui_app.py
   ```

## 📂 Folder Structure

```
FishRobot-ROS/
├── gui_app.py                    # Main PyQt5 application
├── config/                       # Contains config files (.json, .yaml, .sdf)
├── model_generator.py            # Model building script
├── publisher_member_function.py  # ROS 2 motion control node
└── README.md
```

## 🛠 Requirements

- Python 3.8+
- ROS 2 Humble
- `PyQt5`, `pyqtgraph`, `psutil`, `numpy`
- Gazebo + ROS-Gazebo bridge

## 📄 License

MIT License. See `LICENSE` for more information.

---

Created with ❤️ by [trvqhuy](https://github.com/trvqhuy)