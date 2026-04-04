# DVL Integration Plan - Nortek Nucleus 1000

## Overview

The Nortek Nucleus 1000 DVL will provide accurate velocity measurements for closed-loop control, replacing/augmenting the IMU-based velocity estimation.

**DVL Specifications:**
- Frequency: 1000 kHz
- Range: 0.2m to 50m altitude
- Accuracy: ±0.2% of velocity ±0.2 cm/s
- Update Rate: Up to 15 Hz
- Beam Configuration: 4-beam Janus
- Interface: Serial (UART) or Ethernet

## Integration Points

### 1. Hardware Connection

**Option A: Serial via Pixhawk**
- Connect DVL UART to Pixhawk TELEM2/3 port
- ArduSub forwards data via MAVLink VISION_POSITION_DELTA

**Option B: Direct Ethernet to Companion**
- Connect DVL Ethernet to Raspberry Pi
- Parse DVL data directly in ROS2 node
- Recommended for higher data rates

### 2. MAVLink Message Format

ArduSub can receive DVL data via:

**VISION_POSITION_DELTA (Message ID: 11011)**
```
uint64_t time_usec         // Timestamp
float angle_delta[3]       // Roll, pitch, yaw deltas (radians)
float position_delta[3]    // X, Y, Z position deltas (meters)
float confidence           // Confidence value (0.0 - 1.0)
```

**VISION_SPEED_ESTIMATE (Message ID: 103)**
```
uint64_t usec             // Timestamp
float x, y, z             // Velocity in NED frame (m/s)
uint8_t reset_counter     // Sensor reset counter
float covariance[9]       // Covariance matrix
```

Recommended: Use VISION_SPEED_ESTIMATE for simplicity.

### 3. Software Architecture

```
┌─────────────────────────────────────────────────────────────┐
│ DVL Sensor (Nortek Nucleus 1000)                            │
│  • Measures bottom-relative velocity                        │
│  • Detects bottom lock status                               │
│  • Outputs velocity in body frame                           │
└─────────────────────────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│ dvl_driver_node (NEW ROS2 node to create)                   │
│  • Parses DVL serial/ethernet data                          │
│  • Checks bottom_lock flag                                  │
│  • Publishes /dvl/velocity topic                            │
│  • Sends VISION_SPEED_ESTIMATE to ArduSub                   │
└─────────────────────────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│ sensor_sources.py (MODIFY in mavlink_inspector)             │
│  • Receives /dvl/velocity topic                             │
│  • Checks bottom_lock status                                │
│  • Fuses DVL + IMU velocity                                 │
│  • Fallback to IMU if DVL unavailable                       │
└─────────────────────────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│ velocity_control.py (MODIFY)                                 │
│  • Use DVL velocity as primary source                       │
│  • Use IMU velocity as backup                               │
│  • Update cascade controller with accurate velocity         │
└─────────────────────────────────────────────────────────────┘
```

## Implementation Steps

### Phase 5.1: DVL Driver Node (NEW)
**Package:** `dvl_driver` (create new package)

**Node:** `dvl_driver_node.py`

**Responsibilities:**
- Parse DVL serial data (PD0/PD6 format) or JSON (if Ethernet)
- Extract velocity (vx, vy, vz) in body frame
- Extract bottom_lock flag
- Publish ROS2 topic: `/dvl/velocity` (geometry_msgs/TwistStamped)
- Send VISION_SPEED_ESTIMATE to ArduSub via MAVLink

**Key Code Structure:**
```python
class DVLDriver(Node):
    def __init__(self):
        self.create_subscription('/mavlink/dvl_raw', self._handle_dvl)
        self.velocity_pub = self.create_publisher('dvl/velocity', TwistStamped)
        self.mavlink_conn = mavutil.mavlink_connection(...)
    
    def _handle_dvl(self, msg):
        # Parse DVL data
        velocity = parse_dvl_pd0(msg.data)
        bottom_lock = velocity.bottom_lock
        
        if bottom_lock:
            # Publish velocity
            twist = TwistStamped()
            twist.twist.linear.x = velocity.vx
            twist.twist.linear.y = velocity.vy
            twist.twist.linear.z = velocity.vz
            self.velocity_pub.publish(twist)
            
            # Send to ArduSub
            self._send_vision_speed_estimate(velocity)
```

### Phase 5.2: Sensor Fusion (MODIFY sensor_sources.py)
**File:** `src/mavlink_inspector/mavlink_inspector/sensor_sources.py`

**Add DVL handling:**
```python
class SensorSources:
    def __init__(self):
        self.dvl_velocity = None
        self.dvl_bottom_lock = False
        self.dvl_last_update = 0.0
        self.create_subscription('/dvl/velocity', self._handle_dvl_velocity)
    
    def _handle_dvl_velocity(self, msg):
        self.dvl_velocity = (msg.twist.linear.x,
                             msg.twist.linear.y,
                             msg.twist.linear.z)
        self.dvl_bottom_lock = True  # Assume locked if receiving data
        self.dvl_last_update = time.time()
    
    def get_velocity(self):
        """Get best available velocity estimate"""
        # Prefer DVL if available and locked
        if self.dvl_bottom_lock and (time.time() - self.dvl_last_update) < 0.5:
            return self.dvl_velocity
        else:
            # Fallback to IMU velocity estimator
            return self.velocity_estimator.get_velocity()
```

### Phase 5.3: Velocity Control Integration (MODIFY velocity_control.py)
**File:** `src/mavlink_inspector/mavlink_inspector/velocity_control.py`

**Update VelocityEstimator:**
```python
class VelocityEstimator:
    def update(self, imu_msg, orientation_quat=None, dvl_velocity=None):
        """
        Update velocity estimate
        
        Args:
            imu_msg: IMU acceleration data
            orientation_quat: Current orientation for gravity compensation
            dvl_velocity: DVL velocity if available (overrides IMU)
        """
        if dvl_velocity is not None:
            # Use DVL directly (most accurate)
            self._velocity_x = dvl_velocity[0]
            self._velocity_y = dvl_velocity[1]
            self._velocity_z = dvl_velocity[2]
            self._stopped_duration = 0.0  # Reset ZUPT
        else:
            # Fallback to IMU integration (with gravity compensation)
            # ... existing IMU integration code ...
```

## DVL Data Formats

### PD0 Format (Binary)
Nortek DVL outputs PD0 format over serial. Parser needed.

**Key Fields:**
- Header: 0x7F7F
- Velocity: 4 bytes per beam (mm/s)
- Bottom lock: Status byte (bit 0)
- Altitude: Distance to bottom (cm)

**Python Parser Example:**
```python
def parse_pd0(data):
    if data[0:2] != b'\x7F\x7F':
        return None
    
    # Extract velocity (4 beams)
    vx = struct.unpack('<h', data[10:12])[0] / 1000.0  # mm/s → m/s
    vy = struct.unpack('<h', data[12:14])[0] / 1000.0
    vz = struct.unpack('<h', data[14:16])[0] / 1000.0
    
    # Bottom lock status
    status = data[16]
    bottom_lock = (status & 0x01) != 0
    
    return DVLData(vx, vy, vz, bottom_lock)
```

### JSON Format (Ethernet)
If using Ethernet interface, DVL may output JSON:
```json
{
  "velocity": {
    "x": 0.12,
    "y": -0.05,
    "z": 0.02
  },
  "bottom_lock": true,
  "altitude": 2.35,
  "timestamp": 1234567890
}
```

## Configuration Parameters

Add to `defaults.yaml`:
```yaml
# DVL Configuration
dvl:
  enabled: false                    # Enable DVL integration
  interface: "ethernet"             # "serial" or "ethernet"
  ip_address: "192.168.2.95"        # If ethernet
  port: 16171                       # DVL port
  serial_port: "/dev/ttyUSB0"       # If serial
  baud_rate: 115200                 # Serial baud rate
  timeout: 1.0                      # Data timeout (seconds)
  
  # Velocity fusion
  dvl_weight: 0.8                   # DVL weight in fusion (0.0-1.0)
  imu_weight: 0.2                   # IMU weight in fusion
  bottom_lock_required: true        # Only use DVL if bottom locked
  
  # Watchdog
  stale_timeout: 0.5                # Seconds before considering DVL stale
```

## Testing Procedure

### 1. Bench Test (No Water)
```bash
# Start DVL driver
ros2 run dvl_driver dvl_driver_node

# Echo velocity topic
ros2 topic echo /dvl/velocity

# Expected: Zero velocity (no bottom lock)
```

### 2. Pool Test (Shallow)
```bash
# Start full stack
ros2 launch mavlink_inspector inspector.launch.py dvl_enabled:=true

# Monitor DVL status
ros2 topic echo /duburi/diagnostics | grep dvl

# Run mission with DVL
ros2 run mavlink_runner runner missions/test_square.txt

# Expected: Improved position accuracy
```

### 3. Validation Metrics
- Position error: <10cm over 10m mission (with DVL)
- Velocity accuracy: ±0.2 cm/s (DVL spec)
- Bottom lock detection: >95% success rate
- Fallback to IMU: Seamless transition

## DVL Endpoints and Commands

### Ethernet Interface
**Base URL:** `http://192.168.2.95:16171` (default DVL IP)

**REST API Endpoints:**
```
GET /velocity
Response: {"vx": 0.12, "vy": -0.05, "vz": 0.02, "bottom_lock": true}

GET /status
Response: {"firmware": "1.2.3", "temperature": 25.4, "altitude": 2.35}

POST /config
Body: {"update_rate": 10, "beam_mode": "janus"}

GET /diagnostics
Response: {"beam_health": [1,1,1,1], "snr": [30, 32, 31, 29]}
```

### Serial Commands
**PD Commands (ASCII over serial):**
```
CR1     # Output data format (PD0)
CR2     # Output rate (Hz)
CX      # Start pinging
CZ      # Stop pinging
```

## Integration Checklist

- [ ] Create dvl_driver package
- [ ] Implement PD0/JSON parser
- [ ] Add /dvl/velocity topic publisher
- [ ] Modify sensor_sources.py for DVL handling
- [ ] Update velocity_control.py for DVL input
- [ ] Add DVL parameters to defaults.yaml
- [ ] Create DVL testing guide
- [ ] Document fallback behavior (DVL → IMU)
- [ ] Test in pool (with/without bottom lock)
- [ ] Validate position accuracy improvement

## References

- **Nortek DVL Manual:** [ceruleansonar.com/c/dvl-75](https://docs.ceruleansonar.com/c/dvl-75)
- **ArduSub DVL Guide:** [discuss.bluerobotics.com DVL integration](https://discuss.bluerobotics.com)
- **MAVLink VISION messages:** [mavlink.io/en/messages/common.html](https://mavlink.io/en/messages/common.html)
- **BlueOS DVL plugin:** [blueos.cloud DVL extension](https://blueos.cloud/docs)

## Future Enhancements

- **Multi-DVL fusion:** Use multiple DVLs for redundancy
- **Terrain-relative navigation:** Use altitude for 3D mapping
- **Current estimation:** Detect water currents from DVL drift
- **Automatic calibration:** Auto-tune velocity fusion weights
