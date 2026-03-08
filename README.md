# BRACU Duburi 4.2 - Control & Perception Software

ROS 2 workspace for the BRACU Duburi AUV 4.2. Controls via Pixhawk 2.4.8 / ArduSub / pymavlink. Perception via YOLO object detection with GPU-accelerated inference.

---

## Table of Contents

1. [Hardware](#hardware)
2. [Packages](#packages)
3. [Build & Quick Start](#build--quick-start)
4. [Architecture](#architecture)
5. [Using the Duburi CLI (Runner)](#using-the-duburi-cli-runner)
6. [Planning Missions Without the Runner](#planning-missions-without-the-runner)
7. [Logging](#logging)
8. [Vision & Perception](#vision--perception)
9. [Topics & Messages](#topics--messages)
10. [Inspector Parameters](#inspector-parameters)
11. [Analysis & Documentation](#analysis--documentation)
12. [Dependencies](#dependencies)
13. [License](#license)

---

## Hardware

| Component | Spec |
|-----------|------|
| **Flight Controller** | Pixhawk 2.4.8 |
| **Firmware** | ArduSub (ArduPilot) |
| **Connection** | `/dev/ttyACM0` (serial, 115200 baud) |
| **Thrusters** | Blue Robotics T200 |
| **Channels** | Ch 1–4 lateral, Ch 5–8 depth |
| **Barometer** | BAR30 |
| **Camera** | USB camera (V4L2 compatible) |
| **GPU** | CUDA 12.8 (for YOLO inference) |

---

## Packages

| Package | Description |
|---------|-------------|
| `duburi_interfaces` | Shared messages: `DriverCommand`, `MavlinkEvent`, `VehicleState`, `VehicleDiagnostics`, `Detection`, `DetectionArray` |
| `mavlink_inspector` | Main Pixhawk connection hub. Owns serial port, executes commands via MAVLink, publishes events and vehicle state |
| `mavlink_driver` | Movement control: `mission_executor` (predefined missions), `teleop_driver` (Twist → DriverCommand) |
| `mavlink_runner` | Interactive CLI with `Duburi >` prompt for quick testing and file-based missions |
| `mavlink_logger` | Logs MAVLink activity to `logs/` (workspace-relative, session folders) |
| `vision_manager` | Camera management: device enumeration, live testing, streaming (`/camera/image_raw`), checkerboard calibration |
| `vision` | YOLO object detection: subscribes to camera, publishes `DetectionArray`, annotated images, terminal output |

---

## Build & Quick Start

```bash
cd /home/duburi/workspaces/duburi_ws
colcon build
source install/setup.bash
```

**Typical workflow (controls only):**

```bash
# Terminal 1: Start inspector (connects to Pixhawk) – run first
ros2 run mavlink_inspector inspector

# Terminal 2: Start Duburi CLI
ros2 run mavlink_runner runner

# Terminal 3 (optional): Logger (logs to logs/<session>/)
ros2 run mavlink_logger logger
```

**Typical workflow (perception):**

```bash
# Terminal 1: Camera streaming
ros2 run vision_manager camera_node --ros-args -p device_id:=0

# Terminal 2: YOLO detection (GPU default)
ros2 run vision detector_node --ros-args -p enable_display:=True
```

**Launch inspector + logger together:**

```bash
ros2 launch mavlink_inspector duburi_control.launch.py
# With options:
ros2 launch mavlink_inspector duburi_control.launch.py connection_port:=/dev/ttyACM0 enable_logger:=true log_directory:=/path/to/auv_logs
```

---

## Architecture

```
┌──────────────────── CONTROLS ─────────────────────┐   ┌────────── PERCEPTION ──────────┐
│                                                    │   │                                │
│                 Pixhawk (/dev/ttyACM0)             │   │  USB Camera (/dev/videoN)      │
│                         │                          │   │         │                      │
│                 mavlink_inspector                   │   │   vision_manager               │
│                         │                          │   │   (camera_node)                │
│       /mavlink/events   │   /mavlink/vehicle_state │   │         │                      │
│                         │                          │   │  /camera/image_raw             │
│                /driver/command                     │   │  /camera/camera_info           │
│                         │                          │   │         │                      │
│     +-------------------+-------------------+      │   │      vision                    │
│     │                   │                   │      │   │   (detector_node)              │
│  mavlink_runner   mission_executor    teleop_driver│   │         │                      │
│  (CLI, missions)  (Python missions)  (/cmd_vel)    │   │  /vision/detections            │
│     │                   │                   │      │   │  /vision/annotated_image       │
│     +-------------------+-------------------+      │   │                                │
│                         │                          │   └────────────────────────────────┘
│                 mavlink_logger → logs/              │
│                                                    │       ┌──────────────────┐
└────────────────────────────────────────────────────┘       │ duburi_interfaces│
                                                            │ (shared messages)│
          Both sides share duburi_interfaces ◄──────────────│                  │
          Future: planning node bridges them                └──────────────────┘
```

---

## Using the Duburi CLI (Runner)

The runner provides an interactive prompt for direct control and file-based missions.

### Naming Convention

| Prefix | Meaning | Example |
|--------|---------|--------|
| *(bare)* | ArduSub firmware / standard | `depth 0.5`, `heading 90`, `turn left 45` |
| `~` | Software PID (smooth, closed-loop) | `~depth 0.5`, `~heading 90`, `~turn left 45` |
| `move` | Single/compound directional thrust | `move forward 50%`, `move forward-right 50%` |
| `go` | Movement + PID heading hold | `go forward 90 50%`, `go forward-right 45 50%` |

All old command names (`dive`, `p_dive`, `yaw`, `p_yaw`, `p_turn`) are kept as backward-compatible aliases.

### Starting the Runner

```bash
ros2 run mavlink_runner runner
```

### Input Features

- **Up/Down** – Command history
- **Left/Right** – Cursor movement
- History stored in `~/.duburi_history`

---

### Movement Commands

| Command | Example | Description |
|---------|---------|-------------|
| `move forward [gain%] [Ns]` | `move forward 50% 7s` | Move forward |
| `move back [gain%] [Ns]` | `move back 50% 3s` | Move backward |
| `move left [gain%] [Ns]` | `move left 50% 10s` | Strafe left |
| `move right [gain%] [Ns]` | `move right 70% 5s` | Strafe right |
| `move up [gain%] [Ns]` | `move up 40%` | Move up (indefinite) |
| `move down [gain%] [Ns]` | `move down 50% 2s` | Move down |
| `forward [gain%] [Ns]` | `forward 50% 5s` | Shorthand (no `move`) |

- **Gain** 0–100% (default 50%)
- **Duration** in seconds (`Ns`); omit for indefinite
- Order of gain/duration doesn't matter

---

### Diagonal Movement

Move in two horizontal directions at once. Speed is automatically scaled by $1/\sqrt{2} ≈ 71\%$ per axis so the resultant velocity vector matches your requested speed — the AUV doesn't go faster diagonally.

| Command | Example | Description |
|---------|---------|-------------|
| `move forward-right [gain%] [Ns]` | `move forward-right 60% 5s` | Diagonal forward-right |
| `move forward-left [gain%] [Ns]` | `forward-left 50% 3s` | Diagonal forward-left |
| `move back-right [gain%] [Ns]` | `back-right 40%` | Diagonal back-right |
| `move back-left [gain%] [Ns]` | `move back-left 50% 2s` | Diagonal back-left |

**Valid combinations:** Any pair of `{forward, back}` × `{left, right}`. Conflicting directions (e.g. `forward-back`) are rejected.

> **Why only horizontal?** Vertical movement (up/down) should use `~depth` or `depth` for PID-controlled depth hold. Raw throttle without PID drifts and is unreliable in water.

---

### Depth Control

Two depth control strategies are available. Use bare command for ArduSub firmware, `~` prefix for software PID:

| Command | Example | Description |
|---------|---------|-------------|
| `depth <m>` | `depth 0.5` | ArduSub ALT_HOLD firmware depth hold |
| `~depth` | `~depth` | Software PID — hold current depth (auto STABILIZE) |
| `~depth <m>` | `~depth 0.5` | Software PID — hold specific depth (auto STABILIZE) |
| `~depth off` | `~depth off` | Disable software depth PID |
| `surface` | `surface` | Ascend to surface (stops everything) |

- **`depth`** uses ArduSub’s built-in ALT_HOLD. Pixhawk’s firmware PID controls the throttle.
- **`~depth`** is our software PID running at 20 Hz. Auto-switches to STABILIZE mode. Overrides CH_THROTTLE each tick. Uses derivative-on-measurement and conditional integration to prevent overshoot.
- Both depth modes can be combined with forward/lateral movement — they only control the vertical axis.
- **Aliases:** `dive` = `depth`, `p_dive` = `~depth`

---

### Heading Control

Use bare command for bang-bang (fast, snappy), `~` prefix for PID (smooth, precise):

| Command | Example | Description |
|---------|---------|-------------|
| `heading <deg> [gain%]` | `heading 260 50%` | Bang-bang yaw to heading |
| `~heading <deg> [gain%]` | `~heading 260 50%` | PID smooth yaw to heading |
| `heading left [gain%] [Ns]` | `heading left 50% 5s` | Open-loop rotate left |
| `heading right [gain%] [Ns]` | `heading right 50% 5s` | Open-loop rotate right |

- `heading` completes once the heading is within 5° of target.
- `~heading` completes once within 3° (PID is smoother and more precise).
- Both are software-only — ArduSub doesn’t provide heading PID in MANUAL mode.
- **Aliases:** `yaw` = `heading`, `p_yaw` = `~heading`

### Relative Turn (from current heading)

| Command | Example | Description |
|---------|---------|-------------|
| `turn left <deg> [gain%]` | `turn left 90` | Bang-bang relative turn left |
| `turn right <deg> [gain%]` | `turn right 45 60%` | Bang-bang relative turn right |
| `~turn left <deg> [gain%]` | `~turn left 90` | PID smooth relative turn left |
| `~turn right <deg> [gain%]` | `~turn right 45` | PID smooth relative turn right |

- Turns are computed from the current heading (requires telemetry from inspector).
- **Aliases:** `p_turn` = `~turn`

---

### Simultaneous Move + Heading (`go` commands)

The `go` command is the most powerful: it moves the AUV in a direction **while simultaneously** PID-yawing to a target heading. This is essential for competition tasks where you need to approach a gate at a specific angle.

| Command | Example | Description |
|---------|---------|-------------|
| `go forward <deg> [gain%] [Ns]` | `go forward 90 60% 5s` | Move forward + PID yaw to 90° |
| `go back <deg> [gain%] [Ns]` | `go back 270 50% 3s` | Reverse + PID yaw to 270° |
| `go left <deg> [gain%] [Ns]` | `go left 0 50% 5s` | Strafe left + hold 0° heading |
| `go right <deg> [gain%] [Ns]` | `go right 180 40%` | Strafe right + hold 180° |
| `go forward-right <deg> [gain%] [Ns]` | `go forward-right 45 60% 5s` | Diagonal + heading |

#### How `go` Works — Step by Step

```
go forward 90 60% 5s
```

1. **Tick 0 (instantly):** Inspector sets BOTH:
   - `_current_movement` → CH_FORWARD=1740 (60%), CH_LATERAL=1500, CH_THROTTLE=1500, CH_YAW=1500
   - `_yaw_to_heading` → PID targeting 90°
2. **Every 50ms (20 Hz):** The RC override layer builds one combined PWM message:
   - Layer 2: forward thrust from `_current_movement`
   - Layer 3: depth PID overrides CH_THROTTLE (if `p_dive` active)
   - Layer 4: yaw PID overrides CH_YAW with correction toward 90°
3. **The AUV moves forward AND rotates toward 90° simultaneously from the first tick.**
4. **When heading 90° is reached** (within 3°): yaw PID stops. CH_YAW goes neutral. Forward thrust continues.
5. **After 5 seconds:** movement expires. All channels go neutral. AUV stops.

> **Key insight:** `go` doesn't "first turn, then move." Translation and rotation happen in parallel from the very start.

#### Combining `go` with Depth

`go` only controls horizontal translation + yaw. To also hold depth:

```bash
Duburi > p_dive 0.5; go forward 90 60% 10s
```

> Can also be written with new names: `Duburi > ~depth 0.5; go forward 90 60% 10s`

This holds 0.5m depth (Layer 3) while moving forward toward 90° (Layers 2+4).

---

### How the RC Override Layering Works

Every 50ms, the inspector builds a single PWM message from 4 layers:

```
Layer 1: Neutral (1500) on all channels ─── baseline
Layer 2: Active movement ─── sets CH_FORWARD, CH_LATERAL, CH_THROTTLE, CH_YAW
Layer 3: Depth PID ─── OVERRIDES CH_THROTTLE (if p_dive active)
Layer 4: Yaw PID ─── OVERRIDES CH_YAW (if go/p_yaw/yaw active)
```

Higher layers override lower ones. This means:
- `move forward` alone → Layer 2 sets forward thrust
- `move forward` + `p_dive 0.5` → Layer 2 sets forward, Layer 3 overwrites throttle
- `go forward 90` → Layer 2 sets forward + neutral yaw, Layer 4 overwrites yaw with PID
- `go forward-right 45` + `p_dive 0.3` → Layers 2+3+4 all active simultaneously

| Channel | What controls it | Priority |
|---------|------------------|----------|
| CH_FORWARD (5) | Movement commands | Layer 2 |
| CH_LATERAL (6) | Movement commands | Layer 2 |
| CH_THROTTLE (3) | `move up/down`, OR `p_dive`/`dive` | Layer 3 overrides Layer 2 |
| CH_YAW (4) | `yaw left/right`, OR `go`/`p_yaw` | Layer 4 overrides Layer 2 |

---

### Mode & Arm

| Command | Example |
|---------|---------|
| `mode <MODE>` | `mode MANUAL` / `mode ALT_HOLD` / `mode STABILIZE` |
| `arm` | Arm motors (non-blocking) |
| `disarm` | Disarm motors (non-blocking) |

Arm/disarm print confirmation when events are received.

---

### Stop & Actuators

| Command | Example |
|---------|---------|
| `stop` | Stop all thrusters |
| `grabber open` | Open grabber |
| `grabber close` | Close grabber |

---

### Chained Commands

Execute multiple commands on one line (runner waits for duration before next):

```bash
Duburi > arm; move forward 50% 5s; move left 50% 2s; stop; disarm
```

---

### File-Based Missions

```bash
Duburi > run <mission_name>
Duburi > run example_gate
Duburi > list missions
```

**Mission file locations (in order):**

1. `./missions/` (current directory)
2. `mavlink_runner/missions/`
3. `~/.duburi/missions/`

**Example mission file** (`missions/example_gate.txt`):

```
# Example gate mission — approach gate at heading 90°
arm
mode MANUAL
sleep 2
~depth 0.5                    # hold 0.5m depth (software PID)
sleep 3
go forward 90 60% 8s          # move forward + PID yaw to 90°
sleep 9
forward-right 50% 3s          # diagonal adjustment
sleep 4
stop
surface
sleep 5
disarm
```

```
# Compound movement demo
arm
mode MANUAL
sleep 2
~depth 0.3                    # hold depth (PID)
sleep 3
go forward-right 45 50% 5s    # diagonal + heading 45°
sleep 6
go left 270 60% 3s            # strafe left at heading 270°
sleep 4
stop
disarm
```

- One command per line (or semicolon-separated on a line)
- `#` lines are comments
- `sleep <seconds>` / `wait <seconds>` — pause between commands
- `pause` — pause mission until Enter (runner) or external resume (executor)
- Runner waits for duration between commands
- **Ctrl+C during mission executor** aborts gracefully (sends stop, doesn't kill node)
- **Second Ctrl+C** forces exit

---

### Other Commands

| Command | Description |
|---------|-------------|
| `help` | Show all commands |
| `status` | Show vehicle status topic info |
| `quit` / `exit` / `q` | Exit runner |

---

## Planning Missions Without the Runner

You can plan and run missions without using the interactive runner.

---

### Option 1: Mission Executor Node

Predefined missions run as a ROS 2 node.

```bash
# Start inspector first
ros2 run mavlink_inspector inspector

# Run built-in pool test mission (after ~3s delay)
ros2 run mavlink_driver mission_executor

# With custom mission name (if supported by executor)
ros2 run mavlink_driver mission_executor --ros-args -p mission:=pool_test
```

The executor publishes `DriverCommand` messages to `/driver/command` with delays between steps.

---

### Option 2: Direct Topic Publishing

Control the AUV by publishing `DriverCommand` to `/driver/command`.

```bash
# Arm
ros2 topic pub --once /driver/command duburi_interfaces/msg/DriverCommand "{command: 'arm'}"

# Move forward 50% for 5 seconds
ros2 topic pub --once /driver/command duburi_interfaces/msg/DriverCommand "{command: 'move_forward', duration: 5.0, speed: 50}"

# Diagonal: forward-right for 3 seconds
ros2 topic pub --once /driver/command duburi_interfaces/msg/DriverCommand "{command: 'move_forward_right', duration: 3.0, speed: 50}"

# Software PID depth hold at 0.5m
ros2 topic pub --once /driver/command duburi_interfaces/msg/DriverCommand "{command: 'pid_depth', depth: 0.5}"

# Go forward + PID yaw to 90°, 60% for 5s
ros2 topic pub --once /driver/command duburi_interfaces/msg/DriverCommand "{command: 'go_forward', angle: 90.0, duration: 5.0, speed: 60}"

# Go diagonal forward-right + PID yaw to 45°
ros2 topic pub --once /driver/command duburi_interfaces/msg/DriverCommand "{command: 'go_forward_right', angle: 45.0, duration: 5.0, speed: 50}"

# PID yaw to heading 260°
ros2 topic pub --once /driver/command duburi_interfaces/msg/DriverCommand "{command: 'pid_yaw_to_heading', angle: 260.0, speed: 50}"

# Stop
ros2 topic pub --once /driver/command duburi_interfaces/msg/DriverCommand "{command: 'stop'}"

# Disarm
ros2 topic pub --once /driver/command duburi_interfaces/msg/DriverCommand "{command: 'disarm'}"
```

---

### Option 3: Custom Python Mission Node

Use `mavlink_driver.driver_client` to build your own mission node.

```python
#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from duburi_interfaces.msg import DriverCommand
from mavlink_driver.driver_client import (
    arm, disarm, move_forward, move_left, move_combo,
    go_forward, go_combo, pid_depth, pid_depth_off,
    set_mode, stop, surface, set_depth,
    yaw_to_heading, pid_yaw,
    turn_left, pid_turn_left,
)

class MyMissionNode(Node):
    def __init__(self):
        super().__init__('my_mission')
        self._pub = self.create_publisher(
            DriverCommand, '/driver/command',
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        )
        self._timer = self.create_timer(3.0, self._run_mission_once)

    def _publish(self, cmd, delay=0.5):
        self._pub.publish(cmd)
        if delay > 0:
            time.sleep(delay)

    def _run_mission_once(self):
        self._timer.cancel()
        self._publish(set_mode('MANUAL'))
        self._publish(arm(), delay=3.0)

        # Hold depth at 0.5m (software PID, works in MANUAL)
        self._publish(pid_depth(0.5), delay=3.0)

        # Move forward at 60% for 5s while PID-yawing to 90°
        self._publish(go_forward(angle=90, duration=5, speed=60))
        time.sleep(6)

        # Diagonal: forward-right at 50% for 3s
        self._publish(move_combo('forward-right', duration=3, speed=50))
        time.sleep(4)

        # Diagonal + heading: forward-left while yawing to 45°
        self._publish(go_combo('forward-left', angle=45, duration=4, speed=50))
        time.sleep(5)

        # Simple forward
        self._publish(move_forward(duration=3, speed=50))
        time.sleep(4)

        # Clean up
        self._publish(pid_depth_off())
        self._publish(stop())
        self._publish(surface(), delay=5.0)
        self._publish(disarm())

def main():
    rclpy.init()
    node = MyMissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### Option 4: Mission File as Reference

Use `missions/*.txt` files as mission descriptions even when not using the runner:

1. **With Runner:** `run example_gate`
2. **Without Runner:** Replicate the sequence in `mission_executor` or your custom node
3. **Scripting:** Parse the file and publish `DriverCommand` via a small script

---

### Option 5: Teleop via /cmd_vel

Use `teleop_driver` to drive the AUV with Twist messages (e.g., from a joystick or nav stack).
All 4 axes (forward/lateral/vertical/yaw) are combined into a **single `teleop` command** per tick — no per-axis stomping. When the joystick returns to centre, a single `teleop_idle` is sent that clears movement without disrupting active depth PID or heading hold.

```bash
ros2 run mavlink_driver teleop_driver
# Publish to /cmd_vel (geometry_msgs/Twist)
# Single axis:
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
# Multi-axis (forward + right + up + yaw simultaneously):
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.5, z: 0.3}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
```

The `teleop` command encodes PWM offsets in DriverCommand fields: `speed`=forward, `duration`=lateral, `depth`=throttle, `angle`=yaw. The inspector decodes all 4 and sets channels in one `_current_movement`.

---

## Logging

The logger creates **session folders** with rotating log files inside the workspace:

```
logs/
└── 2026-03-06_14-30-00/     # session timestamp
    ├── session.log           # all events + commands (RotatingFileHandler, 5 MB max, 3 backups)
    ├── events.log            # MAVLink events only
    ├── commands.log          # DriverCommand only
    └── state.csv             # throttled telemetry (1 Hz default)
```

```bash
# Start logger with defaults (logs/ in workspace root)
ros2 run mavlink_logger logger

# Custom log directory
ros2 run mavlink_logger logger --ros-args -p log_directory:=/path/to/logs

# Adjust rotation (10 MB max, 5 backups)
ros2 run mavlink_logger logger --ros-args -p max_log_bytes:=10485760 -p backup_count:=5

# Throttle state logging to 0.5 Hz
ros2 run mavlink_logger logger --ros-args -p state_log_interval:=2.0
```

### Runner Health Monitor

The runner automatically monitors the inspector connection. If no `VehicleState` messages arrive for 5+ seconds, a yellow warning appears:

```
⚠ No telemetry for 8s — is mavlink_inspector running?
```

The warning auto-clears when messages resume. Also visible in the `status` dashboard as "Telemetry stale."

---

## Vision & Perception

The perception stack handles camera input and object detection, publishing results as ROS 2 topics for downstream use.

### Quick Start

```bash
# Standalone test (camera + YOLO, no ROS pipeline needed)
ros2 run vision detector_standalone --ros-args -p device_id:=0

# Full ROS pipeline
# Terminal 1: Camera streaming
ros2 run vision_manager camera_node --ros-args -p device_id:=0

# Terminal 2: YOLO detection (GPU by default)
ros2 run vision detector_node --ros-args -p enable_display:=True

# Terminal 3: See detections
ros2 topic echo /vision/detections
```

### vision_manager – Camera Management

| Executable | Description |
|------------|-------------|
| `camera_node` | Streams camera → `/camera/image_raw` + `/camera/camera_info` |
| `camera_enum` | Lists all V4L2 cameras (one-shot, prints table) |
| `camera_test` | Interactive preview with FPS overlay, snapshots (`s`), info (`i`) |
| `camera_calibrate` | Checkerboard calibration → YAML file (ROS CameraInfo compatible) |

**camera_node parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `device_id` | `0` | V4L2 device index (`/dev/videoN`) |
| `frame_width` | `640` | Capture width |
| `frame_height` | `480` | Capture height |
| `fps` | `30` | Target framerate |
| `camera_name` | `duburi_cam` | Frame ID for image headers |
| `calibration_file` | `""` | Path to calibration YAML |

```bash
# Enumerate cameras
ros2 run vision_manager camera_enum

# Test a specific camera
ros2 run vision_manager camera_test --ros-args -p device_id:=1

# Calibrate (9x6 checkerboard, 2.5cm squares)
ros2 run vision_manager camera_calibrate --ros-args \
    -p board_width:=9 -p board_height:=6 -p square_size:=0.025

# Launch camera node with calibration
ros2 launch vision_manager camera.launch.py \
    device_id:=0 calibration_file:=calibration/calibration_video0_20260306.yaml
```

### vision – YOLO Object Detection

| Executable | Description |
|------------|-------------|
| `detector_node` | ROS node: subscribes to image topic, runs YOLO, publishes detections |
| `detector_standalone` | Direct camera+YOLO test (no camera_node needed) |

**detector_node parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model` | `yolov8n.pt` | YOLO model file |
| `confidence` | `0.5` | Detection confidence threshold |
| `device` | `cuda:0` | Inference device (`cpu` or `cuda:0`) |
| `image_topic` | `/camera/image_raw` | Input image topic |
| `enable_display` | `False` | Show OpenCV preview window |
| `publish_annotated` | `True` | Publish annotated image with bounding boxes |
| `max_det` | `50` | Max detections per frame |
| `classes` | `""` | Comma-separated class filter (empty = all) |
| `iou` | `0.45` | NMS IoU threshold |

```bash
# Basic detection (GPU, prints to terminal)
ros2 run vision detector_node

# With display and custom model
ros2 run vision detector_node --ros-args \
    -p enable_display:=True -p model:=yolov8s.pt -p confidence:=0.3

# Filter specific classes (e.g. only persons and cars)
ros2 run vision detector_node --ros-args -p classes:="0,2"

# Force CPU mode
ros2 run vision detector_node --ros-args -p device:=cpu

# Via launch file
ros2 launch vision vision.launch.py enable_display:=True confidence:=0.4
```

### Detection Message Format

**`Detection.msg`** – single detection:

| Field | Type | Description |
|-------|------|-------------|
| `class_name` | string | Class label (e.g. `person`, `gate`) |
| `class_id` | int32 | Numeric class ID from YOLO model |
| `confidence` | float32 | Detection confidence (0.0–1.0) |
| `bbox_x`, `bbox_y` | int32 | Top-left corner (pixels) |
| `bbox_w`, `bbox_h` | int32 | Bounding box size (pixels) |
| `center_x`, `center_y` | float32 | Normalized center (0.0–1.0, resolution-independent) |

**`DetectionArray.msg`** – per frame:

| Field | Type | Description |
|-------|------|-------------|
| `header` | Header | Timestamp + frame_id |
| `detections` | Detection[] | List of detections |
| `image_width` | int32 | Source image width |
| `image_height` | int32 | Source image height |

### Vision Data Flow

```
 USB Camera ──► camera_node ──► detector_node ──► /vision/detections
 /dev/video0    (V4L2+OpenCV)    (YOLO GPU)        (DetectionArray)
                    │                │
             /camera/image_raw   /vision/annotated_image
             (sensor_msgs/Image) (with bounding boxes)
```

Normalized center coordinates (`center_x`, `center_y`) are designed for easy future MAVLink integration:

```
  center_x < 0.4 → target is left  → yaw_left
  center_x > 0.6 → target is right → yaw_right
  0.4–0.6        → centered        → move_forward
```

---

## Topics & Messages

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/driver/command` | `DriverCommand` | → Inspector | Movement and control commands |
| `/mavlink/events` | `MavlinkEvent` | Inspector → | Arm/disarm, mode, movement events |
| `/mavlink/vehicle_state` | `VehicleState` | Inspector → | Armed, mode, depth, yaw, voltage (10 Hz) |
| `/mavlink/diagnostics` | `VehicleDiagnostics` | Inspector → | Heading rate, pressure, servos, RC, CPU (2 Hz) |
| `/cmd_vel` | `Twist` | → teleop_driver | Teleop input (when using teleop_driver) |
| `/camera/image_raw` | `Image` | camera_node → | Raw camera frames (bgr8, 30 Hz) |
| `/camera/camera_info` | `CameraInfo` | camera_node → | Camera intrinsics / calibration |
| `/vision/detections` | `DetectionArray` | detector_node → | YOLO detections per frame |
| `/vision/annotated_image` | `Image` | detector_node → | Frames with bounding boxes drawn |

**DriverCommand fields:**

| Field | Description |
|-------|-------------|
| `command` | See command reference below |
| `mode` | For `set_mode`: `MANUAL`, `ALT_HOLD`, `STABILIZE` |
| `depth` | Target depth in meters (for `set_depth`, `pid_depth`) |
| `angle` | Target heading in degrees 0–360 (for `yaw_to_heading`, `pid_yaw_to_heading`, `go_*`) |
| `duration` | Duration in seconds (0 = indefinite) |
| `speed` | Gain 0–100 (percent) |

**Command reference:**

| Command | Category | Description |
|---------|----------|-------------|
| `move_forward` | Movement | Single-axis forward |
| `move_back` | Movement | Single-axis backward |
| `move_left` | Movement | Single-axis strafe left |
| `move_right` | Movement | Single-axis strafe right |
| `move_up` | Movement | Single-axis up (prefer `pid_depth`) |
| `move_down` | Movement | Single-axis down (prefer `pid_depth`) |
| `move_forward_right` | Diagonal | Horizontal diagonal (√2 scaled) |
| `move_forward_left` | Diagonal | Horizontal diagonal (√2 scaled) |
| `move_back_right` | Diagonal | Horizontal diagonal (√2 scaled) |
| `move_back_left` | Diagonal | Horizontal diagonal (√2 scaled) |
| `go_forward` | Go | Move + PID yaw to `angle` |
| `go_back` | Go | Move + PID yaw to `angle` |
| `go_left` | Go | Move + PID yaw to `angle` |
| `go_right` | Go | Move + PID yaw to `angle` |
| `go_forward_right` | Go | Diagonal + PID yaw to `angle` |
| `go_forward_left` | Go | Diagonal + PID yaw to `angle` |
| `go_back_right` | Go | Diagonal + PID yaw to `angle` |
| `go_back_left` | Go | Diagonal + PID yaw to `angle` |
| `yaw_to_heading` | Heading | Bang-bang yaw to `angle` |
| `pid_yaw_to_heading` | Heading | PID yaw to `angle` |
| `yaw_left` | Heading | Open-loop rotate left |
| `yaw_right` | Heading | Open-loop rotate right |
| `set_depth` | Depth | ALT_HOLD firmware depth to `depth` |
| `pid_depth` | Depth | Software PID depth to `depth` (0 = current) |
| `pid_depth_off` | Depth | Disable software depth PID |
| `surface` | Depth | Ascend to surface |
| `arm` | Control | Arm motors |
| `disarm` | Control | Disarm motors |
| `set_mode` | Control | Set flight mode (`mode` field) |
| `stop` | Control | Stop all movement + clear all PIDs |
| `open_grabber` | Actuator | Open grabber servo |
| `close_grabber` | Actuator | Close grabber servo |
| `teleop` | Teleop | All 4 axes in one command (used by teleop_driver) |
| `teleop_idle` | Teleop | Clear movement without nuking PIDs (used by teleop_driver) |

---

## Inspector Parameters

The inspector node accepts the following ROS parameters for tuning:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `connection_port` | `/dev/ttyACM0` | Pixhawk serial port |
| `baud` | `115200` | Serial baud rate |
| `yaw_source` | `attitude` | Which MAVLink message updates yaw: `attitude` (ATTITUDE, recommended), `ahrs2` (AHRS2), or `both` (legacy, causes jitter) |
| `depth_kp` | `500.0` | Depth PID proportional gain |
| `depth_ki` | `25.0` | Depth PID integral gain |
| `depth_kd` | `200.0` | Depth PID derivative gain |
| `depth_max_integral` | `0.5` | Depth PID max integral accumulator (anti-windup cap) |
| `yaw_kp` | `2.0` | Yaw PID proportional gain |
| `yaw_ki` | `0.05` | Yaw PID integral gain |
| `yaw_kd` | `0.5` | Yaw PID derivative gain |
| `yaw_max_integral` | `50.0` | Yaw PID max integral accumulator |

```bash
# Override parameters at launch
ros2 run mavlink_inspector inspector --ros-args \
  -p connection_port:=/dev/ttyACM1 \
  -p yaw_source:=attitude \
  -p depth_kp:=300.0 \
  -p depth_ki:=15.0
```

### PID Implementation Notes

- **Derivative on measurement** — both depth and yaw PIDs use derivative-on-measurement (not derivative-on-error) to avoid PWM spikes when setpoint changes. Depth uses `(depth - prev_depth)/dt`, yaw uses the gyro-measured heading rate from ATTITUDE messages.
- **Conditional integration** — integral accumulation pauses when PID output is saturated (`|output| ≥ 400 PWM`), preventing windup during large transients.
- **Yaw speed scaling** — the `speed` parameter on `pid_yaw_to_heading` now clamps max PID output (lower speed = gentler corrections).

---

## Analysis & Documentation

The `analysis/` folder contains detailed technical documentation:

| Document | Description |
|----------|-------------|
| `00_OVERVIEW.md` | High-level system overview |
| `01_ARCHITECTURE.md` | Detailed architecture and data flow |
| `02_DESIGN_DECISIONS.md` | Rationale for key design choices (12 decisions) |
| `03_INSPECTOR_LINE_BY_LINE.md` | Inspector node code walkthrough |
| `04_RUNNER_LINE_BY_LINE.md` | Runner CLI code walkthrough |
| `05_DRIVER_LINE_BY_LINE.md` | Driver client library walkthrough |
| `06_INTERFACES.md` | Message definitions and field semantics |
| `07_ARDUSUB_CONSTRAINTS.md` | ArduSub firmware constraints and gotchas |
| `08_AGENT_GUIDE.md` | Guide for AI agents working on this codebase |
| `09_KNOWN_ISSUES_AND_GOTCHAS.md` | Known issues, edge cases, and fixes (21 entries) |
| `10_DESIGN_ISSUES.md` | 7 architectural concerns with severity/effort ratings |
| `11_DESK_TESTING_GUIDE.md` | Step-by-step desk testing procedures for bug fixes |

---

## Dependencies

### Controls
- **ROS 2** (Humble or later)
- **pymavlink:** `pip install pymavlink` or `sudo apt install python3-pymavlink`

### Perception
- **OpenCV:** `pip install opencv-python` or `sudo apt install python3-opencv`
- **Ultralytics YOLO:** `pip install ultralytics`
- **PyTorch with CUDA:** Required for GPU inference (auto-detected by ultralytics)
- **v4l-utils (optional):** `sudo apt install v4l-utils` — for `camera_enum` detailed device info

---

## License

Apache-2.0
