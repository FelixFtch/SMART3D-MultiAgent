#!/bin/bash
pkill -9 -f "cartograph|lslidar|talker|imu_qos|ddsm115|static_transform" 2>/dev/null || true
sleep 2
rm -rf /dev/shm/fastrtps_* /dev/shm/*fastdds* /dev/shm/*SHM* /tmp/*.log 2>/dev/null
mknod /dev/ttyACM3 c 166 3 2>/dev/null || true
chmod 666 /dev/ttyACM3 2>/dev/null || true
/root/mk_imu_dev.sh 2>/dev/null || true
/root/mk_m10p_dev.sh 2>/dev/null || true
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source /root/m10p_ws/install/setup.bash
source /root/imu_ws/install/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0.05 0 0 0 base_link laser_link &
sleep 2
ros2 run lslidar_driver lslidar_driver_node --ros-args --params-file /root/m10p_ws/m10p_uart.yaml > /tmp/lidar.log 2>&1 &
sleep 3
ros2 run hipnuc_imu talker --ros-args -p serial_port:=/dev/imu -p baud_rate:=115200 -p imu_topic:=/IMU_data -p frame_id:=base_link > /tmp/imu_talker.log 2>&1 &
sleep 2
python3 /root/imu_qos_bridge.py > /tmp/imu_bridge.log 2>&1 &
sleep 3
ros2 launch /root/cartographer_config/cartographer_launch.py config_dir:=/root/cartographer_config config_basename:=robotcar_2d.lua > /tmp/cartographer.log 2>&1 &
sleep 8
echo "=== SCAN ===" && timeout 3s ros2 topic hz /scan 2>&1 | head -2
echo "=== TF ===" && timeout 3s ros2 topic hz /tf 2>&1 | head -2
echo "=== BRIDGE ===" && tail -2 /tmp/imu_bridge.log
echo "=== CARTO ===" && tail -3 /tmp/cartographer.log
wait
