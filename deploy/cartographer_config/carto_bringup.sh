#!/bin/bash
###############################################################################
# RobotCar Cartographer Bringup Script
# Run this ON THE HOST (not inside container)
#
# What it does:
#   1. Cleans up zombie processes + DDS shared memory
#   2. Starts LiDAR (M10P -> /scan)
#   3. Starts IMU  (N300 Pro -> /IMU_data)
#   4. Starts correct TF tree: base_link -> laser_link (static)
#   5. Starts Cartographer SLAM (map -> odom -> base_link)
#
# Usage:
#   chmod +x /home/robotcar/carto_bringup.sh
#   /home/robotcar/carto_bringup.sh
#
# To stop everything:
#   /home/robotcar/carto_bringup.sh stop
###############################################################################

set -e
CONTAINER="ros2-humble"
DOMAIN_ID=42

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log()  { echo -e "${GREEN}[BRINGUP]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
err()  { echo -e "${RED}[ERROR]${NC} $1"; }

# ─── STOP MODE ──────────────────────────────────────────────────────────────
if [ "${1}" = "stop" ]; then
    log "Stopping all nodes..."
    docker exec "$CONTAINER" bash -c '
        pkill -9 -f cartographer || true
        pkill -9 -f occupancy_grid || true
        pkill -9 -f "hipnuc_imu/talker" || true
        pkill -9 -f lslidar || true
        pkill -9 -f static_transform || true
        pkill -9 -f imu_to_tf || true
        pkill -9 -f ros2_daemon || true
        rm -rf /dev/shm/fastrtps_* /dev/shm/*fastdds* 2>/dev/null
        rm -rf /tmp/ros2cli_daemon_* 2>/dev/null
    ' 2>/dev/null || true
    log "All nodes stopped."
    exit 0
fi

# ─── STEP 0: CLEANUP ───────────────────────────────────────────────────────
log "Step 0: Cleaning up old processes and DDS state..."
docker exec "$CONTAINER" bash -c '
    # Kill everything ROS-related
    pkill -9 -f cartographer 2>/dev/null || true
    pkill -9 -f occupancy_grid 2>/dev/null || true
    pkill -9 -f "hipnuc_imu/talker" 2>/dev/null || true
    pkill -9 -f lslidar 2>/dev/null || true
    pkill -9 -f static_transform 2>/dev/null || true
    pkill -9 -f imu_to_tf 2>/dev/null || true
    pkill -9 -f ros2_daemon 2>/dev/null || true
    sleep 1
    # Clean DDS shared memory (critical!)
    rm -rf /dev/shm/fastrtps_* /dev/shm/*fastdds* /dev/shm/*ros* 2>/dev/null || true
    rm -rf /tmp/ros2cli_daemon_* /root/.ros/ros2cli_daemon_* 2>/dev/null || true
    echo "Cleanup done."
' 2>/dev/null
sleep 1

# ─── STEP 1: VERIFY DEVICES ────────────────────────────────────────────────
log "Step 1: Verifying serial devices..."
docker exec "$CONTAINER" bash -c '
    /root/mk_imu_dev.sh 2>/dev/null || echo "mk_imu_dev.sh not found"
    /root/mk_m10p_dev.sh 2>/dev/null || echo "mk_m10p_dev.sh not found"
    echo "IMU:   $(ls -la /dev/imu 2>/dev/null || echo MISSING)"
    echo "LiDAR: $(ls -la /dev/m10p 2>/dev/null || echo "using ttyACM1 directly")"
'

# ─── STEP 2: START LIDAR ───────────────────────────────────────────────────
log "Step 2: Starting M10P LiDAR..."
docker exec -d "$CONTAINER" bash -lc "
    source /opt/ros/humble/setup.bash
    source /root/m10p_ws/install/setup.bash
    export ROS_DOMAIN_ID=$DOMAIN_ID
    ros2 run lslidar_driver lslidar_driver_node --ros-args \
        --params-file /root/m10p_ws/m10p_uart.yaml \
        > /tmp/lidar.log 2>&1
"
sleep 2

# Verify LiDAR
docker exec "$CONTAINER" bash -lc "
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=$DOMAIN_ID
    timeout 3s ros2 topic list --no-daemon | grep -q '/scan' && \
        echo '  /scan OK' || echo '  WARNING: /scan not found!'
"

# ─── STEP 3: START IMU ─────────────────────────────────────────────────────
log "Step 3: Starting N300 Pro IMU..."
docker exec -d "$CONTAINER" bash -lc "
    source /opt/ros/humble/setup.bash
    source /root/imu_ws/install/setup.bash
    export ROS_DOMAIN_ID=$DOMAIN_ID
    /root/mk_imu_dev.sh 2>/dev/null || true
    ros2 run hipnuc_imu talker --ros-args \
        -p serial_port:=/dev/imu \
        -p baud_rate:=115200 \
        -p imu_topic:=/IMU_data \
        -p frame_id:=base_link \
        > /tmp/imu_talker.log 2>&1
"
sleep 2

# Verify IMU
docker exec "$CONTAINER" bash -lc "
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=$DOMAIN_ID
    timeout 3s ros2 topic list --no-daemon | grep -q '/IMU_data' && \
        echo '  /IMU_data OK' || echo '  WARNING: /IMU_data not found!'
"

# ─── STEP 4: START STATIC TF ───────────────────────────────────────────────
log "Step 4: Starting TF: base_link -> laser_link..."
# IMPORTANT: cartographer expects base_link -> laser_link (NOT the reverse!)
# Adjust x,y,z if you know the actual offset of the LiDAR from base_link center
docker exec -d "$CONTAINER" bash -lc "
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=$DOMAIN_ID
    # Args: x y z yaw pitch roll frame_id child_frame_id
    # LiDAR is ~5cm above base_link center (adjust if needed)
    ros2 run tf2_ros static_transform_publisher \
        0 0 0.05 0 0 0 base_link laser_link \
        > /tmp/tf_static.log 2>&1
"
# NOTE: Do NOT run imu_to_tf.py when using cartographer!
# Cartographer handles IMU fusion internally.
sleep 1

# ─── STEP 5: START CARTOGRAPHER ────────────────────────────────────────────
log "Step 5: Starting Cartographer SLAM..."
docker exec -d "$CONTAINER" bash -lc "
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=$DOMAIN_ID
    ros2 launch /root/cartographer_config/cartographer_launch.py \
        config_dir:=/root/cartographer_config \
        config_basename:=robotcar_2d.lua \
        > /tmp/cartographer.log 2>&1
"
sleep 3

# ─── STEP 6: VERIFY ────────────────────────────────────────────────────────
log "Step 6: Verifying system..."
docker exec "$CONTAINER" bash -lc "
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=$DOMAIN_ID
    echo '=== Active Topics ==='
    ros2 topic list --no-daemon 2>/dev/null | grep -E 'scan|IMU|map|tf|odom|submap'
    echo ''
    echo '=== Active Nodes ==='
    ros2 node list --no-daemon 2>/dev/null || echo 'node list failed (daemon issue, ignore)'
"

echo ""
log "=========================================="
log " Bringup Complete!"
log "=========================================="
log ""
log " Topics you should see:"
log "   /scan         - LiDAR data"
log "   /IMU_data     - IMU data"
log "   /map          - Occupancy grid map"
log "   /tf           - Dynamic transforms"
log "   /tf_static    - Static transforms"
log "   /submap_list  - Cartographer submaps"
log ""
log " TF tree:"
log "   map -> odom -> base_link -> laser_link"
log ""
log " To view in RViz (from ros2-gui container):"
log "   Fixed Frame: map"
log "   Add: Map (topic: /map)"
log "   Add: LaserScan (topic: /scan)"
log "   Add: TF"
log ""
log " To check cartographer logs:"
log "   docker exec ros2-humble cat /tmp/cartographer.log"
log ""
log " To stop: $0 stop"
