#!/bin/bash
# WIFI
#docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:humble udp4 --port 8090 -v4
# serial
#docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host 192.168.2.51:5000/micro-ros-agent:humble serial --dev /dev/myserial -b 2000000 -v4
# serial
#docker start -i 7026de7b7d9b
#docker stop 7026de7b7d9b

sleep 5
gnome-terminal --title=mircoROS_Agent -- bash -c "export ROS_DOMAIN_ID=30 && source /opt/ros/humble/setup.bash && source /home/jetson/mircoROS_agent/install/setup.bash && ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/myserial -b 2000000"

