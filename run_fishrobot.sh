#!/bin/bash
# -------------------------------------------
# Fish Robot Simulator GUI Launcher Script
# -------------------------------------------

echo "[INFO] Initializing Fish Robot Simulator GUI..."

# === 1. Source ROS2 Environment ===
ROS_SETUP="/opt/ros/humble/setup.bash"
if [ -f "$ROS_SETUP" ]; then
    source "$ROS_SETUP"
    echo "[INFO] ROS2 environment sourced."
else
    echo "[ERROR] ROS2 setup file not found: $ROS_SETUP"
    exit 1
fi

# === 2. Activate Python Virtual Environment ===
VENV_PATH="/home/maycuaaiz/Desktop/FishRobot-ROS/.venv/bin/activate"
if [ -f "$VENV_PATH" ]; then
    source "$VENV_PATH"
    echo "[INFO] Python virtual environment activated."
else
    echo "[WARNING] Virtual environment not found, running without it."
fi

# === 3. Navigate to Project Directory ===
PROJECT_DIR="/home/maycuaaiz/Desktop/FishRobot-ROS"
cd "$PROJECT_DIR" || {
    echo "[ERROR] Failed to navigate to project directory: $PROJECT_DIR"
    exit 1
}
echo "[INFO] Changed directory to $PROJECT_DIR"

# === 4. Run GUI Application ===
echo "[INFO] Launching GUI application..."
python3 gui_app.py

# Optional: Log success
echo "[INFO] GUI application exited."
