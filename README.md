# BRACU Duburi 4.2 - Control Software

ROS 2 workspace for the BRACU Duburi AUV 4.2 control stack. Uses Pixhawk 2.4.8, ArduSub, and pymavlink.

---

## Table of Contents

1. [Hardware](#hardware)
2. [Packages](#packages)
3. [Build & Quick Start](#build--quick-start)
4. [Architecture](#architecture)
5. [Using the Duburi CLI (Runner)](#using-the-duburi-cli-runner)
6. [Planning Missions Without the Runner](#planning-missions-without-the-runner)
7. [Logging](#logging)
8. [Topics & Messages](#topics--messages)
9. [Dependencies](#dependencies)
10. [License](#license)

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

---

## Packages

| Package | Description |
|---------|-------------|
| `duburi_interfaces` | Shared messages: `DriverCommand`, `MavlinkEvent`, `VehicleState`, `VehicleDiagnostics` |
| `mavlink_inspector` | Main Pixhawk connection hub. Owns serial port, executes commands via MAVLink, publishes events and vehicle state |
| `mavlink_driver` | Movement control: `mission_executor` (predefined missions), `teleop_driver` (Twist → DriverCommand) |
| `mavlink_runner` | Interactive CLI with `Duburi >` prompt for quick testing and file-based missions |
| `mavlink_logger` | Logs MAVLink activity to `logs/` (workspace-relative, session folders) |

---

## Build & Quick Start

```bash
cd /home/duburi/workspaces/duburi_ws
colcon build
source install/setup.bash
```

**Typical workflow:**

```bash
# Terminal 1: Start inspector (connects to Pixhawk) – run first
ros2 run mavlink_inspector inspector

# Terminal 2: Start Duburi CLI
ros2 run mavlink_runner runner

# Terminal 3 (optional): Logger (logs to logs/<session>/)
ros2 run mavlink_logger logger
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
                    Pixhawk (/dev/ttyACM0)
                            |
                    mavlink_inspector
                            |
          /mavlink/events   |   /mavlink/vehicle_state
                            |
                   /driver/command
                            |
        +-------------------+-------------------+
        |                   |                   |
  mavlink_runner    mission_executor      teleop_driver
  (CLI, missions)   (Python missions)    (/cmd_vel → DriverCommand)
        |                   |                   |
        +-------------------+-------------------+
                            |
                    mavlink_logger → auv_logs/
```

---

## Using the Duburi CLI (Runner)

The runner provides an interactive prompt for direct control and file-based missions.

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

> **Why only horizontal?** Vertical movement (up/down) should use `p_dive` or `dive` for PID-controlled depth hold. Raw throttle without PID drifts and is unreliable in water.

---

### Depth Control

Two depth control strategies are available:

| Command | Example | Description |
|---------|---------|-------------|
| `dive <m>` | `dive 0.5` | Firmware depth hold (switches to ALT_HOLD mode) |
| `p_dive` | `p_dive` | Software PID — hold current depth (any mode) |
| `p_dive <m>` | `p_dive 0.5` | Software PID — hold specific depth |
| `p_dive off` | `p_dive off` | Disable software depth PID |
| `surface` | `surface` | Ascend to surface (stops everything) |

- **`dive`** uses ArduSub's built-in ALT_HOLD. Pixhawk's firmware PID controls the throttle.
- **`p_dive`** is our software PID running at 20 Hz. Works in MANUAL mode. Overrides CH_THROTTLE each tick.
- Both depth modes can be combined with forward/lateral movement — they only control the vertical axis.

---

### Heading Control

| Command | Example | Description |
|---------|---------|-------------|
| `yaw <deg> [gain%]` | `yaw 260 50%` | Yaw to heading (bang-bang, snaps to heading) |
| `p_yaw <deg> [gain%]` | `p_yaw 260 50%` | PID yaw to heading (smooth, oscillation-free) |
| `yaw left [gain%] [Ns]` | `yaw left 50% 5s` | Rotate left (open-loop, no heading target) |
| `yaw right [gain%] [Ns]` | `yaw right 50% 5s` | Rotate right (open-loop) |

- `yaw` completes once the heading is within 5° of target.
- `p_yaw` completes once within 3° (PID is smoother and more precise).
- Both are software-only — ArduSub doesn't provide heading PID in MANUAL mode.

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
p_dive 0.5                    # hold 0.5m depth (software PID)
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
p_dive 0.3                    # hold depth
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
    set_mode, stop, surface,
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
Supports **multi-axis**: simultaneous horizontal axes are combined into diagonal commands (e.g. `move_forward_right`), vertical and yaw are sent as separate commands.

```bash
ros2 run mavlink_driver teleop_driver
# Publish to /cmd_vel (geometry_msgs/Twist)
# Single axis:
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
# Multi-axis (diagonal forward-right + yaw):
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
```

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

## Topics & Messages

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/driver/command` | `DriverCommand` | → Inspector | Movement and control commands |
| `/mavlink/events` | `MavlinkEvent` | Inspector → | Arm/disarm, mode, movement events |
| `/mavlink/vehicle_state` | `VehicleState` | Inspector → | Armed, mode, depth, yaw, voltage (10 Hz) |
| `/mavlink/diagnostics` | `VehicleDiagnostics` | Inspector → | Heading rate, pressure, servos, RC, CPU (2 Hz) |
| `/cmd_vel` | `Twist` | → teleop_driver | Teleop input (when using teleop_driver) |

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

---

## Dependencies

- **ROS 2** (Humble or later)
- **pymavlink:** `pip install pymavlink` or `sudo apt install python3-pymavlink`

---

## License

Apache-2.0
