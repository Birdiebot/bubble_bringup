export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=1
cd /home/nv/Desktop/bubble/
. install/setup.sh
ros2 launch bubble_protocol bcp_api_core_launch.py