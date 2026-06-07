echo '
source /opt/ros/humble/setup.zsh
colcon build
source install/setup.zsh
ros2 launch oculus_reader teleop_double_piper_x.launch.py
'
