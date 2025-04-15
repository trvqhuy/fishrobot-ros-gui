cd /home/maycuaaiz/Desktop/FishRobot-ROS
source /opt/ros/humble/setup.bash
/bin/python3 model_generator.py & ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=config/final.bridge.yaml 

cd /home/maycuaaiz/Desktop/FishRobot-ROS
source /opt/ros/humble/setup.bash
/bin/python3 model_generator.py & ign gazebo config/final.fish_world.sdf

source /opt/ros/humble/setup.bash
/bin/python3 /home/maycuaaiz/Desktop/FishRobot-ROS/sensor_reading.py

source /opt/ros/humble/setup.bash
/bin/python3 /home/maycuaaiz/Desktop/FishRobot-ROS/publisher_member_function.py

source /opt/ros/humble/setup.bash
ign topic -t /ocean_current -m ignition.msgs.Vector3d -p 'x: 0, y:0, z:0.2'

# RUN GUI PROGRAM

cd /home/maycuaaiz/Desktop/FishRobot-ROS
source .venv/bin/activate
python3 gui_app.py