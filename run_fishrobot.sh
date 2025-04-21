#!/bin/bash
# Run Fish Robot Simulator GUI

# 1. Source ROS2 Humble
source /opt/ros/humble/setup.bash

# 2. Activate Python virtual environment (optional, if you use it)
source /home/maycuaaiz/Desktop/FishRobot-ROS/.venv/bin/activate

# 3. Move to project folder (optional if you run inside it already)
cd /home/maycuaaiz/Desktop/FishRobot-ROS

# 4. Launch GUI application
python3 gui_app.py
