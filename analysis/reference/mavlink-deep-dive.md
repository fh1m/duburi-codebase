# 20 — ArduSub & MAVLink Deep Dive: Library Learnings

> **Purpose:** Document the understanding of ArduSub, pymavlink, and related
> libraries gained during development. Essential reading for anyone working
> with the MAVLink layer.

---

## Table of Contents

1. [ArduSub Overview](#1-ardusub-overview)
2. [MAVLink Protocol Fundamentals](#2-mavlink-protocol-fundamentals)
3. [pymavlink Library Usage](#3-pymavlink-library-usage)
4. [RC Override Mechanics](#4-rc-override-mechanics)
5. [Flight Modes and Their Behaviors](#5-flight-modes-and-their-behaviors)
6. [Arming and Safety](#6-arming-and-safety)
7. [Telemetry Messages](#7-telemetry-messages)
8. [Common Pitfalls and Solutions](#8-common-pitfalls-and-solutions)
9. [YASMIN FSM Library](#9-yasmin-fsm-library)
10. [simple-pid vs Custom PID](#10-simple-pid-vs-custom-pid)
11. [Ultralytics YOLO Integration](#11-ultralytics-yolo-integration)
12. [Reference Resources](#12-reference-resources)

---

## 1. ArduSub Overview

ArduSub is ArduPilot firmware specifically for underwater vehicles (ROVs, AUVs).

### Key Characteristics
- **Thruster mixing**: Handles frame-specific motor mixing (BlueROV2 is `vectored6dof`)
- **Depth hold**: Built-in depth PID using pressure sensor
- **Attitude stabilization**: AHRS-based roll/pitch/yaw stabilization
- **Failsafes**: Battery, leak, GCS timeout, pilot input timeout

### Frame Configuration
Our BlueROV2-style frame uses:
- **6 thrusters**: 4 horizontal (vectored), 2 vertical
- **Motor mixing**: ArduSub handles PWM distribution to individual motors
- **We control**: Body-frame axes (forward, lateral, throttle, yaw)

### Channel Mapping
```
CH1 = Pitch     (not used in MANUAL mode)
CH2 = Roll      (not used in MANUAL mode)
CH3 = Throttle  (vertical movement / depth control input)
CH4 = Yaw       (rotation)
CH5 = Forward   (surge)
CH6 = Lateral   (sway)
CH7 = Camera tilt (optional)
CH8 = Lights (optional)
```

---

## 2. MAVLink Protocol Fundamentals

### Message Types
1. **HEARTBEAT**: Connection keepalive, mode/armed state
2. **COMMAND_LONG**: Request actions (arm, mode change, etc.)
3. **RC_CHANNELS_OVERRIDE**: Direct channel control
4. **Telemetry**: ATTITUDE, AHRS2, SYS_STATUS, SCALED_PRESSURE, etc.

### System IDs
- **System ID 1**: The vehicle (Pixhawk/ArduSub)
- **System ID 255**: Ground Control Station (our ROS node)
- **Component ID 1**: Autopilot
- **Component ID 0**: All components (broadcast)

### Critical Lesson: HEARTBEAT Filtering
```python
# WRONG: Process all HEARTBEATs
def _handle_heartbeat(self, msg):
    self._armed = bool(msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)

# RIGHT: Only process vehicle HEARTBEATs
def _handle_heartbeat(self, msg):
    if msg.get_srcSystem() != 1:  # Ignore GCS echoes
        return
    self._armed = bool(msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
```

Without this filter, GCS HEARTBEATs (system_id=255) cause armed/mode flapping
because they echo different state.

---

## 3. pymavlink Library Usage

### Installation
```bash
pip install pymavlink
```

### Connection Setup
```python
from pymavlink import mavutil

# Serial connection
conn = mavutil.mavlink_connection(
    '/dev/ttyACM0',
    baud=115200,
    source_system=255,  # GCS system ID
    source_component=0   # All components
)

# Wait for heartbeat
conn.wait_heartbeat()
print(f"Connected to system {conn.target_system}")
```

### Sending HEARTBEAT (Required at 1 Hz)
```python
conn.mav.heartbeat_send(
    mavutil.mavlink.MAV_TYPE_GCS,           # type
    mavutil.mavlink.MAV_AUTOPILOT_INVALID,  # autopilot
    0,  # base_mode
    0,  # custom_mode
    mavutil.mavlink.MAV_STATE_ACTIVE        # system_status
)
```

### Sending COMMAND_LONG
```python
conn.mav.command_long_send(
    conn.target_system,     # target_system
    conn.target_component,  # target_component
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # command
    0,      # confirmation
    1,      # param1: 1=arm, 0=disarm
    0, 0, 0, 0, 0, 0  # param2-7
)
```

### Sending RC Override
```python
conn.mav.rc_channels_override_send(
    conn.target_system,     # target_system
    conn.target_component,  # target_component
    1500, 1500, 1500, 1500,  # channels 1-4
    1600, 1500, 0, 0         # channels 5-8 (0 = no override)
)
```

### Reading Messages
```python
# Non-blocking read
msg = conn.recv_match(blocking=False)
if msg:
    msg_type = msg.get_type()
    if msg_type == 'ATTITUDE':
        print(f"Roll: {msg.roll}, Pitch: {msg.pitch}, Yaw: {msg.yaw}")
```

---

## 4. RC Override Mechanics

### Timing Requirements
- **Minimum rate**: 20 Hz (50ms interval)
- **Timeout**: ArduSub disarms after ~1s without RC input
- **Neutral = must send**: Cannot stop sending to stop movement

### PWM Values
```
NEUTRAL = 1500    # No movement
MIN = 1100        # Full negative
MAX = 1900        # Full positive
RANGE = 400       # 1100-1500 or 1500-1900
```

### Speed to PWM Conversion
```python
def percent_to_pwm(speed_percent: int) -> int:
    """Convert 0-100% speed to PWM offset 0-400."""
    return int(speed_percent * 4)  # 100% → 400 PWM offset

# Forward at 50%: 1500 + 200 = 1700
# Backward at 50%: 1500 - 200 = 1300
```

### Diagonal Movement
```python
def build_diagonal_channels(forward: int, lateral: int) -> dict:
    """Scale diagonal movement to maintain constant speed."""
    # 1/√2 scaling for 45° diagonals
    scale = 0.7071  # 1/sqrt(2)
    return {
        CH_FORWARD: int(forward * scale),
        CH_LATERAL: int(lateral * scale),
    }
```

### The Idle Problem
```python
# WRONG: Stop sending when idle
if not moving:
    return  # ArduSub will disarm!

# RIGHT: Send neutral when idle
if not moving:
    send_rc_override([1500] * 8)
```

---

## 5. Flight Modes and Their Behaviors

### MANUAL Mode
- **What it does**: Direct RC → thruster mapping
- **No stabilization**: Vehicle drifts freely
- **Use case**: Direct thruster control for testing

### STABILIZE Mode
- **What it does**: Maintains level attitude (roll/pitch = 0)
- **Throttle**: Direct vertical control
- **Use case**: Easier piloting, auto-levels

### ALT_HOLD Mode
- **What it does**: Maintains target depth automatically
- **Throttle channel**: Sets depth target, not direct control
- **Requires**: Good depth sensor (pressure)
- **Use case**: Autonomous depth maintenance

### Mode Switching
```python
# Set mode via COMMAND_LONG
conn.mav.command_long_send(
    target_system, target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,  # confirmation
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    STABILIZE,  # custom_mode: 0=STABILIZE, 2=ALT_HOLD, 19=MANUAL
    0, 0, 0, 0, 0
)
```

### Mode Numbers
```python
STABILIZE = 0
ACRO = 1
ALT_HOLD = 2
AUTO = 3
GUIDED = 4
# ...
MANUAL = 19
```

---

## 6. Arming and Safety

### Arming Requirements
1. **HEARTBEAT received**: Vehicle must see GCS heartbeat
2. **Pre-arm checks pass**: Battery OK, sensors OK, etc.
3. **Throttle centered**: CH3 near 1500 (safety check)

### Arming Command
```python
conn.mav.command_long_send(
    target_system, target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,    # confirmation
    1,    # param1: 1=arm, 0=disarm
    0,    # param2: 0=normal, 21196=force
    0, 0, 0, 0, 0
)
```

### Force Arm (Bypass Pre-arm Checks)
```python
# param2 = 21196 bypasses checks (DANGEROUS)
conn.mav.command_long_send(
    ...,
    1,      # arm
    21196,  # force arm magic number
    ...
)
```

### Waiting for Arm Confirmation
```python
# Check HEARTBEAT for armed state
def is_armed(msg):
    return bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

# Wait loop
start = time.time()
while time.time() - start < timeout:
    msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
    if msg and msg.get_srcSystem() == 1 and is_armed(msg):
        return True
return False
```

### Disarming
```python
# Always disarm on exit
conn.mav.command_long_send(
    target_system, target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0,  # param1: 0=disarm
    0, 0, 0, 0, 0, 0
)
```

---

## 7. Telemetry Messages

### HEARTBEAT (1 Hz from vehicle)
```python
msg.base_mode      # Bit flags including ARMED
msg.custom_mode    # Flight mode number
msg.system_status  # MAV_STATE_* enum
```

### ATTITUDE (~10 Hz)
```python
msg.roll   # Radians
msg.pitch  # Radians
msg.yaw    # Radians
msg.rollspeed   # rad/s
msg.pitchspeed  # rad/s
msg.yawspeed    # rad/s
```

### AHRS2 (~10 Hz)
```python
msg.roll      # Radians
msg.pitch     # Radians
msg.yaw       # Radians
msg.altitude  # Meters (barometric, not reliable underwater)
msg.lat       # Latitude (if GPS available)
msg.lng       # Longitude (if GPS available)
```

### SYS_STATUS (~2 Hz)
```python
msg.voltage_battery  # mV
msg.current_battery  # cA (10 mA units)
msg.battery_remaining  # 0-100%
```

### SCALED_PRESSURE (~10 Hz)
```python
msg.press_abs   # hPa (absolute pressure)
msg.temperature # centi-degrees C

# Depth calculation
SURFACE_PRESSURE = 1013.25  # hPa at surface
WATER_DENSITY = 1000  # kg/m³ (freshwater)
GRAVITY = 9.81  # m/s²

depth_m = (msg.press_abs - SURFACE_PRESSURE) * 100 / (WATER_DENSITY * GRAVITY)
```

### SERVO_OUTPUT_RAW (~10 Hz)
```python
msg.servo1_raw  # PWM output to motor 1
msg.servo2_raw  # PWM output to motor 2
# ... up to servo8_raw
```

---

## 8. Common Pitfalls and Solutions

### Pitfall 1: Vehicle Disarms After Movement
**Cause**: Stopped sending RC override after movement completed.
**Solution**: Always send neutral (1500) when idle, never stop sending.

### Pitfall 2: Armed/Mode State Flapping
**Cause**: Processing HEARTBEAT messages from GCS (system_id=255).
**Solution**: Filter to only process system_id=1 (vehicle).

### Pitfall 3: Commands Ignored
**Cause**: Wrong target_system or target_component.
**Solution**: Use `conn.target_system` and `conn.target_component` from connection.

### Pitfall 4: RC Override Has No Effect
**Cause**: Not armed, or not in MANUAL mode, or RC override disabled.
**Solution**: Check armed state, check mode, verify ArduSub parameter RC_OVERRIDE_TIME.

### Pitfall 5: Connection Drops
**Cause**: HEARTBEAT not sent frequently enough (must be ~1 Hz).
**Solution**: Timer-based HEARTBEAT at exactly 1 Hz.

### Pitfall 6: Depth Reading Wrong
**Cause**: Using surface pressure from wrong location/time.
**Solution**: Calibrate surface pressure at test location before diving.

### Pitfall 7: Yaw Wrapping
**Cause**: Yaw goes from 359° to 0°, PID sees 359° error.
**Solution**: Normalize yaw error to [-180, 180]:
```python
def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle
```

### Pitfall 8: Blocking the ROS Executor
**Cause**: Calling `motors_armed_wait()` in callback.
**Solution**: Arm/disarm in a separate thread, check armed state asynchronously.

---

## 9. YASMIN FSM Library

### Overview
YASMIN (Yet Another State MachINe) is a ROS 2-native FSM library with:
- Hierarchical state machines
- Blackboard for state sharing
- Visual debugger (YASMIN viewer)

### Basic State
```python
from yasmin import State, Blackboard

class MyState(State):
    def __init__(self):
        super().__init__(outcomes=['succeeded', 'failed', 'aborted'])

    def execute(self, blackboard: Blackboard) -> str:
        # Do work
        if success:
            return 'succeeded'
        return 'failed'
```

### State Machine
```python
from yasmin import StateMachine

sm = StateMachine(outcomes=['mission_complete', 'mission_failed'])

sm.add_state('SUBMERGE', SubmergeState(),
             transitions={'succeeded': 'SEARCH', 'failed': 'mission_failed'})
sm.add_state('SEARCH', SearchState(),
             transitions={'found': 'ALIGN', 'not_found': 'SURFACE'})
sm.add_state('ALIGN', AlignState(),
             transitions={'aligned': 'DRIVE', 'lost_target': 'SEARCH'})
sm.add_state('DRIVE', DriveState(),
             transitions={'succeeded': 'SURFACE'})
sm.add_state('SURFACE', SurfaceState(),
             transitions={'succeeded': 'mission_complete'})

# Execute
outcome = sm.execute(Blackboard())
```

### Blackboard Usage
```python
# Set data
blackboard['target_depth'] = 1.5
blackboard['context'] = planner_context

# Get data
depth = blackboard['target_depth']
ctx = blackboard['context']
```

### YASMIN Viewer
```bash
# Start viewer
ros2 run yasmin_viewer yasmin_viewer

# In code, enable visualization
sm.set_viewer(True)
```

---

## 10. simple-pid vs Custom PID

### simple-pid Library
```python
from simple_pid import PID

pid = PID(Kp=1.0, Ki=0.1, Kd=0.05, setpoint=1.5)
pid.output_limits = (-400, 400)

# Use
output = pid(current_depth)
```

### Why We Use Custom PID
1. **Deadband**: simple-pid doesn't have deadband (output=0 when |error| < threshold)
2. **Rate limiting**: No limit on output change rate
3. **Derivative-on-measurement**: simple-pid uses derivative-on-error (causes kick)
4. **Anti-windup**: simple-pid has basic anti-windup, but not conditional integration

### Our PidController Features
```python
class PidController:
    def __init__(self, kp, ki, kd, max_output, deadband, max_rate, derivative_filter):
        # All features parameterized

    def compute(self, setpoint, measurement):
        error = setpoint - measurement

        # Deadband
        if abs(error) < self.deadband:
            return 0.0

        # Derivative on measurement (not error)
        d_measurement = measurement - self._last_measurement
        derivative = -self.kd * d_measurement  # Note: negative

        # EMA filter on derivative
        self._filtered_derivative = (
            self.derivative_filter * derivative +
            (1 - self.derivative_filter) * self._filtered_derivative
        )

        # Conditional integration (anti-windup)
        if abs(self._output) < self.max_output:
            self._integral += error * self.dt

        # Rate limiting
        raw_output = self.kp * error + self.ki * self._integral + self._filtered_derivative
        delta = raw_output - self._output
        if abs(delta) > self.max_rate:
            delta = self.max_rate * (1 if delta > 0 else -1)
        self._output += delta

        return clamp(self._output, -self.max_output, self.max_output)
```

---

## 11. Ultralytics YOLO Integration

### Installation
```bash
pip install ultralytics
```

### Basic Usage
```python
from ultralytics import YOLO

# Load model
model = YOLO('yolov8n.pt')  # or custom model

# Inference
results = model(image, conf=0.5)

# Process results
for result in results:
    boxes = result.boxes
    for box in boxes:
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        confidence = box.conf[0].item()
        class_id = int(box.cls[0].item())
        class_name = model.names[class_id]
```

### ROS Integration
```python
def _on_image(self, msg: Image):
    # Convert ROS Image to CV2
    cv_image = ros_image_to_cv2(msg)

    # Run inference
    results = self.model(cv_image, conf=self.confidence_threshold)

    # Convert to ROS message
    detection_array = DetectionArray()
    for result in results:
        for box in result.boxes:
            detection = Detection()
            detection.class_name = self.model.names[int(box.cls[0])]
            detection.confidence = float(box.conf[0])
            detection.bbox = [float(x) for x in box.xyxy[0].tolist()]
            detection_array.detections.append(detection)

    self.detection_pub.publish(detection_array)
```

### Performance Tips
1. **Use smaller model**: yolov8n (nano) for real-time on Jetson
2. **Lower resolution**: 320x320 or 416x416 instead of 640x640
3. **TensorRT export**: `model.export(format='engine')` for 2-3x speedup
4. **Batch size 1**: Real-time inference doesn't benefit from batching

---

## 12. Reference Resources

### ArduSub Documentation
- ArduSub Parameters: https://ardusub.com/developers/full-parameter-list.html
- ArduSub Pymavlink: https://www.ardusub.com/developers/pymavlink.html
- ArduPilot Dev Wiki: https://ardupilot.org/dev/

### MAVLink
- MAVLink Protocol: https://mavlink.io/en/
- Message Definitions: https://mavlink.io/en/messages/common.html
- pymavlink Docs: https://mavlink.io/en/mavgen_python/

### YASMIN
- GitHub: https://github.com/uleroboticsgroup/yasmin
- ROS 2 Integration: https://github.com/uleroboticsgroup/yasmin/tree/main/yasmin_ros

### YOLO
- Ultralytics Docs: https://docs.ultralytics.com/
- YOLOv8 Guide: https://docs.ultralytics.com/modes/predict/

### Reference Codebases
- BlueROV2 Pymavlink Examples: https://github.com/bluerobotics/ardusub-gitbook
- Our Reference: `/home/duburi/old_stuff/auv/src/mishu/mishu/`

---

## Quick Reference Card

### Essential MAVLink Commands
```python
# Connect
conn = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200, source_system=255)
conn.wait_heartbeat()

# Send GCS heartbeat (1 Hz)
conn.mav.heartbeat_send(MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE)

# Arm
conn.mav.command_long_send(ts, tc, MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0,0,0,0,0)

# Set mode
conn.mav.command_long_send(ts, tc, MAV_CMD_DO_SET_MODE, 0, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_num, 0,0,0,0,0)

# RC override (20 Hz)
conn.mav.rc_channels_override_send(ts, tc, ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8)

# Read
msg = conn.recv_match(blocking=False)
```

### Channel Values
```
Neutral: 1500
Full forward/up/right: 1900
Full backward/down/left: 1100
50% speed offset: ±200
```

### Mode Numbers
```
STABILIZE = 0
ALT_HOLD = 2
MANUAL = 19
```

---

*Document created from codebase analysis and development experience at commit `d0a48d6`.*
