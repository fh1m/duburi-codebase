# Duburi 4.2 Movement Vocabulary Reference

This document provides a comprehensive reference for all movement commands
available in the Duburi AUV control stack.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          COMMAND SOURCES                                 │
├───────────────┬───────────────┬───────────────┬─────────────────────────┤
│  CLI Runner   │ Mission Files │   Planner     │   Vision/Perception     │
│(command_      │(mission_      │(PlannerContext│   (DuburiClient or      │
│ parser.py)    │ parser.py)    │ .send())      │    TeleopCommand)       │
└───────┬───────┴───────┬───────┴───────┬───────┴──────────┬──────────────┘
        │               │               │                  │
        └───────────────┴───────────────┴──────────────────┘
                                │
                       DriverCommand message
                       /driver/command topic
                                │
                                ▼
            ┌───────────────────────────────────────────┐
            │         CommandHandler.handle()           │
            │   (mavlink_inspector/command_handler.py)  │
            │                                           │
            │  Registry lookup → Handler dispatch       │
            └───────────────────────┬───────────────────┘
                                    │
                                    ▼
            ┌───────────────────────────────────────────┐
            │        RcController.apply_movement()      │
            │   (trapezoidal ramp, phase-aware brake)   │
            └───────────────────────┬───────────────────┘
                                    │
                                    ▼
            ┌───────────────────────────────────────────┐
            │           RC_CHANNELS_OVERRIDE            │
            │              (20 Hz to Pixhawk)           │
            │                                           │
            │  CH1=Pitch, CH2=Roll, CH3=Throttle,       │
            │  CH4=Yaw, CH5=Forward, CH6=Lateral        │
            └───────────────────────────────────────────┘
```

## Channel Mapping

| Channel | Axis | PWM Range | Positive | Negative |
|---------|------|-----------|----------|----------|
| CH1 | Pitch | 1100-1900 | Pitch down | Pitch up |
| CH2 | Roll | 1100-1900 | Roll right | Roll left |
| CH3 | Throttle | 1100-1900 | Descend | Ascend |
| CH4 | Yaw | 1100-1900 | Rotate right | Rotate left |
| CH5 | Forward | 1100-1900 | Forward | Backward |
| CH6 | Lateral | 1100-1900 | Strafe right | Strafe left |

**Neutral PWM:** 1500 (no thrust)
**PWM Range:** ±400 from neutral (1100-1900)

## Canonical Commands (25 total)

### Translation Commands (7)

```
Command         Channels    Description
─────────────────────────────────────────────────────────────
move_forward    CH5+        Thrust forward
move_back       CH5-        Thrust backward
move_left       CH6-        Strafe left
move_right      CH6+        Strafe right
move_up         CH3-        Ascend (negative throttle = up)
move_down       CH3+        Descend (positive throttle = down)
move_at         CH5,CH6     Arbitrary angle via cos/sin decomposition
```

**Body-Frame Reference:**
```
            Forward (0°)
               ↑ +X
               │
    Left ←─────┼─────→ Right
    (-Y)       │       (+Y)
               │
               ↓
           Back (180°)
```

### Heading Commands (4)

```
Command             Method      Description
─────────────────────────────────────────────────────────────
yaw_to_heading      Bang-bang   Fast but may overshoot
pid_yaw_to_heading  PID         Smooth, recommended for precision
yaw_left            Open-loop   Rotate left for duration
yaw_right           Open-loop   Rotate right for duration
```

**Heading Convention:**
- 0° = North
- 90° = East
- 180° = South
- 270° = West

### Depth Commands (4)

```
Command         Mode        Description
─────────────────────────────────────────────────────────────
set_depth       ALT_HOLD    Firmware depth hold (requires mode switch)
pid_depth       Software    Software PID depth hold (any mode)
pid_depth_off   —           Disable software depth hold
surface         Composite   Ascend to surface with throttle burst
```

**Depth Convention:**
- Positive = below surface
- 0.5m = 50cm underwater

### Compound Commands (2)

```
Command    Axes           Description
─────────────────────────────────────────────────────────────
go         Trans+Yaw      Move in direction while holding heading
cruise     Trans+Yaw+Dep  Full 3-axis coordinated movement
```

### System Commands (5)

```
Command          Transport   Description
─────────────────────────────────────────────────────────────
arm              Service     Arm vehicle for operation
disarm           Service     Disarm vehicle
set_mode         Service     Change flight mode
stop             Service     Emergency stop (all neutral)
calibrate_depth  Service     Set current depth as surface
```

### Actuator Commands (2)

```
Command         Description
─────────────────────────────────────────────────────────────
open_grabber    Open gripper mechanism
close_grabber   Close gripper mechanism
```

## Command Parameters

| Parameter | Type | Range | Description |
|-----------|------|-------|-------------|
| speed_pct | float | 0-100 | Speed as percentage |
| duration | float | ≥0 | Duration in seconds (0=indefinite) |
| target_heading | float | 0-360 | Compass heading in degrees |
| target_depth | float | ≥0 | Depth in meters |
| bearing | float | 0-360 | Body-frame thrust direction |
| direction | string | — | 'forward', 'back', 'left', 'right', or compound |
| bypass_ramp | bool | — | Skip PWM ramping (instant thrust) |
| use_pid | bool | — | Use PID variant |

## Movement Diagrams

### Forward/Backward
```
     ┌───────────┐
     │    ↑↑↑    │  move_forward: CH5 = 1500 + offset
     │   ╔═══╗   │
     │   ║ ● ║   │  AUV (top view)
     │   ╚═══╝   │
     │    ↓↓↓    │  move_back: CH5 = 1500 - offset
     └───────────┘
```

### Lateral (Strafe)
```
     ┌───────────┐
     │← ← ╔═══╗→ →│
     │    ║ ● ║   │
     │    ╚═══╝   │
     └───────────┘
     move_left: CH6 = 1500 - offset
     move_right: CH6 = 1500 + offset
```

### Yaw (Rotation)
```
         ╭─────╮
       ↺ │     │ ↻
         │  ●  │
         │     │
         ╰─────╯
    yaw_left   yaw_right
    CH4 < 1500  CH4 > 1500
```

### Diagonal Movement (Compound)
```
    ↖ forward-left    ↗ forward-right
      ╲    ↑    ╱
       ╲   │   ╱
     ←──╔══╪══╗──→
        ║  ●  ║
     ←──╚═════╝──→
       ╱   │   ╲
      ╱    ↓    ╲
    ↙ back-left    ↘ back-right

    Diagonal uses √2 scaling:
    forward-right → CH5 = 1500 + offset/√2
                    CH6 = 1500 + offset/√2
```

### Cruise (3-Axis Coordinated)
```
    ┌──────────────────────────────────┐
    │  cruise bearing=45 heading=90    │
    │  depth=0.5 speed=50              │
    │                                  │
    │  ╔═══╗ ↗ bearing (thrust dir)    │
    │  ║ ● ║ → heading (facing)        │
    │  ╚═══╝   depth PID active        │
    │          yaw PID active          │
    └──────────────────────────────────┘
```

## PWM Ramping Profile

```
PWM
  ▲
  │    ┌──────────────────────────┐
  │   ╱                            ╲
  │  ╱     cruising phase           ╲
  │ ╱                                ╲──┐ braking
  │╱                                    └──
  ├────────────────────────────────────────── time
  0   ramp_up   cruise   decel   brake

  Parameters:
  - ramp_rate: 800 PWM/s (default)
  - decel_time: 1.0s before end
  - brake_strength: 0.3 (30% reverse)
  - brake_duration: 0.5s
```

## Python API Usage

### Using DuburiClient (Recommended)

```python
from mavlink_driver.duburi_client import DuburiClient

# Initialize
duburi = DuburiClient(node)

# System
duburi.arm()
duburi.set_mode('STABILIZE')

# Movement
duburi.move_forward(speed=50, duration=3.0)
duburi.move_at(bearing=45, speed=40)  # 45° = forward-right

# Heading
duburi.yaw_to(90, method='pid')  # Face East
duburi.turn(45, direction='left')  # Relative turn

# Depth
duburi.pid_depth(0.5)  # Hold at 50cm

# Compound
duburi.go(direction='forward', heading=90, speed=50, duration=5.0)
duburi.cruise(bearing=0, heading=90, depth=0.5, speed=50, duration=10.0)

# Stop
duburi.stop()
duburi.disarm()
```

### Perception Integration

```python
# Vision-based adjustment
def on_detection(detection):
    bearing_to_target = detection.bearing
    distance = detection.distance
    
    # Move toward detected object
    duburi.move_at(bearing=bearing_to_target, speed=30, duration=0.5)
    
    # Maintain heading toward target
    duburi.yaw_to(bearing_to_target, method='pid')

# Depth-based tracking
def track_depth_target(target_depth):
    duburi.pid_depth(target_depth)
```

## CLI Usage

```bash
# Movement
move forward 50% 5s
move left 30%
forward-right 40% 3s
just forward 50%      # Bypass ramp

# Heading
heading 90
~heading 90           # PID variant
turn left 45

# Depth
depth 0.5
~depth 0.5            # PID variant
surface

# Compound
go forward 90 50% 5s
cruise 0 90 0.5 60% 10s
```

## MAVLink Layer

### RC_CHANNELS_OVERRIDE

```python
# Sent at 20 Hz
master.mav.rc_channels_override_send(
    target_system,
    target_component,
    ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8,
    ch9, ch10, ch11, ch12, ch13, ch14, ch15, ch16, ch17, ch18
)
```

### SET_MODE

```python
master.mav.command_long_send(
    target_system, target_component,
    MAV_CMD_DO_SET_MODE,
    0,  # confirmation
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id,  # from mode_mapping()
    0, 0, 0, 0, 0
)
```

### ARM/DISARM

```python
master.mav.command_long_send(
    target_system, target_component,
    MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1,  # 1=arm, 0=disarm
    0, 0, 0, 0, 0, 0
)
```

## Configuration

See `mavlink_inspector/config/defaults.yaml` for tunable parameters:
- PID gains (depth_kp/ki/kd, yaw_kp/ki/kd)
- Ramp settings (ramp_rate, decel_time)
- Brake settings (brake_enabled, brake_strength)
- Safety (rc_watchdog_timeout)

All parameters are live-tunable via:
```bash
ros2 param set /mavlink_inspector depth_kp 500.0
```
