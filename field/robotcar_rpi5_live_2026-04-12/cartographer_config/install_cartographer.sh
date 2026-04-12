#!/bin/bash
###############################################################################
# Cartographer Installation Script
# Run INSIDE the ros2-humble container:
#   docker exec -it ros2-humble bash
#   bash /root/cartographer_config/install_cartographer.sh
###############################################################################

set -e

echo "============================================"
echo " Installing Cartographer for ROS2 Humble"
echo "============================================"

# Source ROS
source /opt/ros/humble/setup.bash

# Step 1: Install cartographer
echo ""
echo "[1/4] Installing ros-humble-cartographer-ros..."
apt-get update -qq
apt-get install -y --no-install-recommends \
    ros-humble-cartographer-ros \
    python3-serial
echo "  Done."

# Step 2: Verify installation
echo ""
echo "[2/4] Verifying installation..."
if ros2 pkg list 2>/dev/null | grep -q cartographer_ros; then
    echo "  cartographer_ros: OK"
else
    echo "  ERROR: cartographer_ros not found!"
    exit 1
fi

# Step 3: Verify config files exist
echo ""
echo "[3/4] Checking config files..."
for f in /root/cartographer_config/robotcar_2d.lua \
         /root/cartographer_config/cartographer_launch.py \
         /root/cartographer_config/stp23l_range.py; do
    if [ -f "$f" ]; then
        echo "  $(basename $f): OK"
    else
        echo "  $(basename $f): MISSING!"
    fi
done

# Step 4: Make scripts executable
echo ""
echo "[4/4] Setting permissions..."
chmod +x /root/cartographer_config/*.py 2>/dev/null || true
chmod +x /root/cartographer_config/*.sh 2>/dev/null || true

echo ""
echo "============================================"
echo " Installation Complete!"
echo "============================================"
echo ""
echo " Config dir: /root/cartographer_config/"
echo " Lua config: robotcar_2d.lua"
echo " Launch:     cartographer_launch.py"
echo " STP-23L:    stp23l_range.py"
echo ""
echo " Quick test (inside container):"
echo "   source /opt/ros/humble/setup.bash"
echo "   ros2 launch /root/cartographer_config/cartographer_launch.py"
echo ""
echo " Or use the bringup script from HOST:"
echo "   bash /home/robotcar/carto_bringup.sh"
echo ""
