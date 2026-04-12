#!/bin/bash
# =============================================================================
# SMART-3D M3 PRO Setup Script
# Run on Jetson Orin NX (M3 PRO) ONCE after USB WiFi is plugged in
#
# Prerequisites:
#   - RTL8812AU USB WiFi plugged in and driver working
#   - Can see "robotcar" AP in: nmcli device wifi list
#   - start_agent.sh already works (micro-ROS for STM32 base)
#
# Usage:
#   chmod +x m3pro_setup.sh
#   bash m3pro_setup.sh
# =============================================================================

set -e

echo "============================================="
echo "  SMART-3D M3 PRO Environment Setup"
echo "============================================="
echo ""

# ---- 1. Install CycloneDDS ----
echo "[1/6] Installing CycloneDDS..."
if dpkg -l ros-humble-rmw-cyclonedds-cpp 2>/dev/null | grep -q "^ii"; then
    echo "  → Already installed."
else
    sudo apt update
    sudo apt install -y ros-humble-rmw-cyclonedds-cpp
    echo "  → Installed."
fi

# ---- 2. Detect USB WiFi interface ----
echo ""
echo "[2/6] Detecting USB WiFi interface..."
USB_WIFI=""
for iface in $(ls /sys/class/net/); do
    if [[ "$iface" == wlx* ]]; then
        USB_WIFI="$iface"
        break
    fi
done

if [ -z "$USB_WIFI" ]; then
    echo "  ✗ No USB WiFi (wlx...) detected!"
    echo "    Check: nmcli device status"
    echo "    If RTL8812AU not showing, driver may need installation."
    echo ""
    echo "    To install RTL8812AU driver on Jetson:"
    echo "      sudo apt install -y dkms git build-essential linux-headers-\$(uname -r)"
    echo "      git clone https://github.com/aircrack-ng/rtl8812au.git"
    echo "      cd rtl8812au && sudo make dkms_install"
    echo "      sudo modprobe 88XXau"
    exit 1
fi
echo "  → Found: $USB_WIFI"

# ---- 3. Connect to robotcar AP ----
echo ""
echo "[3/6] Connecting to robotcar AP..."
CURRENT=$(nmcli -t -f DEVICE,CONNECTION device status | grep "^${USB_WIFI}:" | cut -d: -f2)
if [ "$CURRENT" = "robotcar" ]; then
    echo "  → Already connected to robotcar."
else
    ROBOTCAR_WIFI_PASSWORD="${ROBOTCAR_WIFI_PASSWORD:-CHANGE_ME}"
    nmcli device wifi connect "robotcar" password "$ROBOTCAR_WIFI_PASSWORD" ifname "$USB_WIFI" || {
        echo "  ✗ Failed to connect to robotcar AP."
        echo "    Set ROBOTCAR_WIFI_PASSWORD before running this script."
        echo "    Check: nmcli device wifi list --ifname $USB_WIFI"
        exit 1
    }
    sleep 2
    echo "  → Connected."
fi

# Get IP on robotcar network
M3PRO_IP=$(ip -4 addr show "$USB_WIFI" | grep -oP '10\.42\.\d+\.\d+' | head -1)
if [ -z "$M3PRO_IP" ]; then
    echo "  ✗ No 10.42.x.x IP assigned! Check AP connection."
    exit 1
fi
echo "  → M3 PRO IP on robotcar AP: $M3PRO_IP"

# ---- 4. Create CycloneDDS config ----
echo ""
echo "[4/6] Creating CycloneDDS config..."

# M3 PRO config: specify USB WiFi interface for cross-network,
# add peers for both i9 and Pi5 car
cat > /tmp/cyclone.xml << XMLEOF
<CycloneDDS>
  <Domain id="42">
    <General>
      <AllowMulticast>true</AllowMulticast>
      <Interfaces>
        <NetworkInterface name="${USB_WIFI}"/>
      </Interfaces>
    </General>
    <Discovery>
      <Peers>
        <Peer address="10.42.0.160"/>
        <Peer address="10.42.0.1"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
XMLEOF
echo "  → /tmp/cyclone.xml created (interface=$USB_WIFI)"

# Also create a "local-only" config for micro-ROS agent + Cartographer
# These need to talk to each other locally AND to the client
cat > /tmp/cyclone_local.xml << XMLEOF
<CycloneDDS>
  <Domain id="42">
    <General>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer address="10.42.0.160"/>
        <Peer address="10.42.0.1"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
XMLEOF
echo "  → /tmp/cyclone_local.xml created (no interface restriction, for local nodes)"

# ---- 5. Create modified start_agent.sh ----
echo ""
echo "[5/6] Creating SMART-3D compatible start_agent.sh..."
cat > ~/start_agent_smart3d.sh << 'AGENTEOF'
#!/bin/bash
# Modified start_agent.sh for SMART-3D
# Uses DOMAIN_ID=42 + CycloneDDS for cross-network communication
#
# IMPORTANT: Run this INSTEAD of the original start_agent.sh
#            when using SMART-3D multi-agent mode.

sleep 3
gnome-terminal --title=microROS_Agent -- bash -c "
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file:///tmp/cyclone_local.xml
  source /opt/ros/humble/setup.bash
  source /home/jetson/mircoROS_agent/install/setup.bash
  echo '=== micro-ROS Agent (SMART-3D mode, DOMAIN_ID=42) ==='
  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/myserial -b 2000000
"
AGENTEOF
chmod +x ~/start_agent_smart3d.sh
echo "  → ~/start_agent_smart3d.sh created"

# ---- 6. Create run script for m3pro_client ----
echo ""
echo "[6/6] Creating run scripts..."

# Cartographer launcher (with SMART-3D DDS config)
cat > ~/run_cartographer_smart3d.sh << 'CARTEOF'
#!/bin/bash
# Launch Cartographer + sensors in SMART-3D mode
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///tmp/cyclone_local.xml
source /opt/ros/humble/setup.bash

echo "=== Cartographer + Sensors (SMART-3D mode) ==="
echo "  DOMAIN_ID=42, CycloneDDS"
echo ""

# Launch base + camera + cartographer
# These come from Yahboom's packages
ros2 launch slam_mapping cartographer.launch.py &
CART_PID=$!

ros2 launch M3Pro_navigation base_bringup.launch.py &
BASE_PID=$!

ros2 launch orbbec_camera dabai_dcw2.launch.py &
CAM_PID=$!

echo ""
echo "=== All sensors started. PIDs: cart=$CART_PID base=$BASE_PID cam=$CAM_PID ==="
echo "Press Ctrl+C to stop all."

trap "kill $CART_PID $BASE_PID $CAM_PID 2>/dev/null; exit" INT TERM
wait
CARTEOF
chmod +x ~/run_cartographer_smart3d.sh

# M3 PRO SMART-3D client
cat > ~/run_m3pro_client.sh << 'CLIENTEOF'
#!/bin/bash
# Launch SMART-3D thin client for M3 PRO
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///tmp/cyclone.xml
source /opt/ros/humble/setup.bash

echo "=== SMART-3D M3 PRO Client ==="
echo "  DOMAIN_ID=42, CycloneDDS"
echo "  Namespace: m3pro"
echo ""

python3 ~/m3pro_client.py --ros-args \
    -p car_ns:=m3pro \
    -p robot_speed:=0.25
CLIENTEOF
chmod +x ~/run_m3pro_client.sh

# Copy m3pro_client.py to home if not there
if [ ! -f ~/m3pro_client.py ]; then
    echo ""
    echo "  ⚠ ~/m3pro_client.py not found!"
    echo "    Copy it from your dev machine:"
    echo "    scp m3pro_client.py jetson@${M3PRO_IP}:~/"
fi

echo ""
echo "============================================="
echo "  Setup Complete!"
echo "============================================="
echo ""
echo "  M3 PRO IP (robotcar AP): $M3PRO_IP"
echo "  USB WiFi interface:      $USB_WIFI"
echo ""
echo "  --- Update i9 CycloneDDS config ---"
echo "  Add this peer to /tmp/cyclone.xml on the i9:"
echo "    <Peer address=\"$M3PRO_IP\"/>"
echo ""
echo "  --- Startup Order ---"
echo "  Terminal 1: bash ~/start_agent_smart3d.sh"
echo "              (wait 5s for micro-ROS agent)"
echo "  Terminal 2: bash ~/run_cartographer_smart3d.sh"
echo "              (wait for scan + TF to appear)"
echo "  Terminal 3: bash ~/run_m3pro_client.sh"
echo ""
echo "  --- i9 Side ---"
echo "  python3 i9_planner.py --ros-args -p car_names:=\"car1,m3pro\""
echo "============================================="
