# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export ROS_LOCALHOST_ONLY=1
# cd /home/nv/Desktop/bubble/

# sleep 5

# sudo chmod 777 /dev/ttyTHS0
# . install/setup.sh
# ros2 launch /home/nv/Desktop/bubble/src/bubble_bringup/launch/sentryup_launch.py &
# sleep 10
# sudo ps -ef | grep ros | grep -v grep | awk '{print $2}' | xargs sudo kill -9
# wait
# sleep 5
# ros2 launch /home/nv/Desktop/bubble/src/bubble_bringup/launch/sentryup_launch.py

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=1
cd /home/nv/Desktop/bubble/


sudo chmod 777 /dev/ttyTHS0
. install/setup.sh
sudo ps -ef | grep ros | grep -v grep | awk '{print $2}' | xargs sudo kill -9
gnome-terminal -x bash -c "ros2 launch /home/nv/Desktop/bubble/src/bubble_bringup/launch/sentryup_launch.py"