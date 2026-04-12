#!/bin/bash
sleep 10
source /opt/ros/humble/setup.bash
source /home/jetson/yahboomcar_ws/install/setup.bash
source /home/jetson/M3Pro_ws/install/setup.bash
export ROS_DOMAIN_ID=30
gnome-terminal -- bash -c "cd ~/joy_control && /usr/bin/python3 start_joy_controller.py; exec bash"


