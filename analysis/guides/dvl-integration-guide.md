# DVL Integration Quick Start

## When You Get the DVL Hardware

### Step 1: Physical Connection
1. Mount DVL on bottom of AUV (pointing down)
2. Connect DVL Ethernet to Raspberry Pi network switch
3. OR connect DVL serial to Pixhawk TELEM port

### Step 2: Network Configuration
```bash
# Set static IP for DVL
sudo ip addr add 192.168.2.100/24 dev eth0

# Ping DVL to verify connection
ping 192.168.2.95
```

### Step 3: Create DVL Driver Package
```bash
cd ~/ROS_workspaces/Duburi_ws/src
ros2 pkg create --build-type ament_python dvl_driver

# Copy template from analysis/design-decisions/dvl-integration.md
# into dvl_driver/dvl_driver/dvl_driver_node.py
```

### Step 4: Enable in Configuration
Edit `src/mavlink_inspector/config/defaults.yaml`:
```yaml
dvl:
  enabled: true
  interface: "ethernet"
  ip_address: "192.168.2.95"
```

### Step 5: Test
```bash
colcon build --packages-select dvl_driver
source install/setup.bash
ros2 run dvl_driver dvl_driver_node
```

**See:** analysis/design-decisions/dvl-integration.md for full details
