# 12 — Code Reference: Post-Refactor Module Map

> **Commit:** `d0a48d6` — "BlueOS+Jetson bring-up guide and HEARTBEAT source gating fix"  
> **Date:** 2026-03-28  
> **Total:** 80 Python source files across 10 packages (~10,556 lines)

This document maps every package, module, class, and public function in the
BRACU Duburi AUV 4.2 ROS 2 codebase after the Phase 1 modularization refactor
and the addition of the YASMIN FSM mission planner.

---

## Table of Contents

1. [Package Overview](#1-package-overview)
2. [mavlink_inspector](#2-mavlink_inspector)
3. [mavlink_driver](#3-mavlink_driver)
4. [mavlink_runner](#4-mavlink_runner)
5. [mavlink_logger](#5-mavlink_logger)
6. [vision](#6-vision)
7. [vision_inspector](#7-vision_inspector)
8. [duburi_planner](#8-duburi_planner)
9. [duburi_common](#9-duburi_common)
10. [duburi_blueos](#10-duburi_blueos)
11. [duburi_interfaces](#11-duburi_interfaces)
12. [Cross-Package Dependency Graph](#12-cross-package-dependency-graph)
13. [Import Map](#13-import-map)

---

## 1. Package Overview

| Package | Role | Modules | Lines | ROS Nodes |
|---|---|---|---|---|
| `mavlink_inspector` | MAVLink ↔ ROS bridge | 7 | ~2,301 | `mavlink_inspector` |
| `mavlink_driver` | High-level command API + missions | 5 | ~1,044 | `mission_executor`, `teleop_driver` |
| `mavlink_runner` | Human-facing CLI | 4 | ~917 | `duburi_runner` |
| `mavlink_logger` | Topic logging to CSV/JSON | 1 | ~233 | `mavlink_logger` |
| `vision` | YOLO detection + visual servo + Kalman tracking | 6 | ~1,374 | `detector_node` |
| `vision_inspector` | Camera management, calibration, recording | 10 | ~1,380 | `camera_manager`, `camera_enumerator`, `camera_tester`, `camera_calibrator`, `camera_recorder`, `camera_playback` |
| `duburi_planner` | YASMIN FSM mission planner | 18 | ~2,100 | `mission_planner` |
| `duburi_common` | Shared constants and command vocabulary | 2 | ~207 | — |
| `duburi_blueos` | BlueOS REST API companion client | 6 | ~450 | `blueos_monitor` |
| `duburi_interfaces` | ROS 2 msg/srv definitions | — | — | — |

---

## 2. mavlink_inspector

> **Purpose:** Owns the MAVLink serial connection, translates between MAVLink
> and ROS 2. All hardware interaction passes through this node.

### 2.1 inspector_node.py (641 lines)

**Role:** Thin orchestrator. Creates and wires the five peer modules,
owns all ROS timers, publishes vehicle state.

| Item | Kind | Purpose |
|---|---|---|
| `MavlinkInspectorNode` | class (Node) | Main ROS 2 node |
| `__init__()` | method | Declares ROS parameters, instantiates ConnectionManager / TelemetryParser / RcController / PidController × 2 / CommandHandler, creates publishers/subscribers/timers |
| `_read_loop()` | timer (50 Hz) | Reads MAVLink messages, routes to TelemetryParser |
| `_send_heartbeat()` | timer (1 Hz) | Sends GCS heartbeat via ConnectionManager |
| `_publish_state()` | timer (10 Hz) | Publishes `VehicleState` from telemetry fields |
| `_publish_diagnostics()` | timer (2 Hz) | Publishes `VehicleDiagnostics` |
| `_rc_override_tick()` | timer (20 Hz) | 4-layer RC builder: neutral → movement → depth PID → yaw PID |
| `_on_driver_command()` | subscriber | Delegates incoming `DriverCommand` to CommandHandler |
| `/driver/teleop` subscription | subscriber | `TeleopCommand` → `CommandHandler.handle_teleop()` |
| `_send_command_long()` | method | Constructs and sends MAV_CMD via MAVLink |
| `_handle_command_ack()` | method | Processes COMMAND_ACK, resolves pending futures |
| `_arm_disarm()` | method | Arm/disarm with optional checks |
| `_set_mode()` | method | Mode change (MANUAL / STABILIZE / ALT_HOLD) |
| `_set_target_depth()` | method | Depth PID setpoint + ALT_HOLD engagement |
| `_set_servo_pwm()` | method | Direct servo PWM (used by grabber) |

**Depends on:** `connection_manager`, `telemetry_parser`, `rc_controller`,
`pid_controller`, `command_handler`

---

### 2.2 command_handler.py (440 lines)

**Role:** Routes `DriverCommand` messages to the correct handler. Owns the
system-command dispatch table and delegates movements to the MOVEMENTS registry.

| Item | Kind | Purpose |
|---|---|---|
| `CommandHandler` | class | Stateless-ish command router, holds ref to inspector node |
| `handle_command()` | method | Entry point — strips, lowercases, routes via dispatch |
| `system_dispatch` | dict | Maps system commands → handler methods (`stop`, `arm`, `disarm`, `mode`, `depth`, `grab_open`, `grab_close`) |
| `_handle_stop()` | method | Zero all channels, cancel timers, reset PIDs |
| `_handle_arm()` / `_handle_disarm()` | methods | Delegate to inspector's `_arm_disarm()` |
| `_handle_mode()` | method | Mode switching via inspector's `_set_mode()` |
| `_handle_depth()` | method | Depth targeting via inspector's `_set_target_depth()` |
| `_handle_grab_open/close()` | methods | Servo PWM for gripper |
| `handle_go()` | method | Routes `go_*` compound-movement prefixes to `movement_commands.handle_go()` |
| `handle_compound_move()` | method | Routes `compound_move` commands to `movement_commands.handle_compound_move()` |
| `_parse_speed_duration()` | static | Extracts (speed, end_time) from DriverCommand fields |
| `create_depth_pid()` / `create_yaw_pid()` | factory | Returns configured PidController instances |
| `handle_teleop()` | method | Applies multi-axis RC from `TeleopCommand` (`/driver/teleop`); `idle` clears movement |

**Dispatch flow:**
```
handle_command(cmd)
  ├─ system_dispatch[cmd] ?  → _handle_stop / _handle_arm / ...
  ├─ cmd in MOVEMENTS ?      → MOVEMENTS[cmd](node, speed, end_time)
  ├─ cmd.startswith("just_") → MOVEMENTS[cmd](node, speed, end_time)
  ├─ cmd.startswith("go_")   → handle_go(cmd, speed, duration)
  └─ unknown                 → log warning
```

**Depends on:** `movement_commands` (MOVEMENTS registry), `pid_controller`, `duburi_common.constants` (`UNARMED_ALLOWED_INSPECTOR`)

---

### 2.3 movement_commands.py (399 lines)

**Role:** Pure movement handlers. Each function applies channel overrides to
the inspector's RC controller. Registered in the `MOVEMENTS` dict.

| Item | Kind | Purpose |
|---|---|---|
| `MOVEMENTS` | dict | `str → Callable` registry, 30+ entries including aliases |
| `cmd_move_forward()` | handler | CH_FORWARD + speed |
| `cmd_move_back()` | handler | CH_FORWARD − speed |
| `cmd_move_left()` | handler | CH_LATERAL − speed |
| `cmd_move_right()` | handler | CH_LATERAL + speed |
| `cmd_move_up()` | handler | CH_THROTTLE + speed |
| `cmd_move_down()` | handler | CH_THROTTLE − speed |
| `cmd_yaw_left()` | handler | CH_YAW − speed |
| `cmd_yaw_right()` | handler | CH_YAW + speed |
| `cmd_surface()` | handler | Full CH_THROTTLE up for configurable duration |
| `cmd_cruise()` | handler | CH_FORWARD + speed, no auto-stop timer |
| `cmd_teleop()` | handler | Legacy `DriverCommand` `teleop` path (field overloading); prefer `/driver/teleop` + `TeleopCommand` |
| `handle_go()` | function | Parses `go_forward_left` → diagonal via `build_diagonal_channels()` |
| `handle_compound_move()` | function | Parses `compound_move` with explicit axes |
| Movement aliases | entries | `'forward' → cmd_move_forward`, `'back' → cmd_move_back`, etc. |
| `just_*` variants | entries | Instant versions (bypass ramp): `just_forward`, `just_back`, etc. |

**Handler signature:** `(node: MavlinkInspectorNode, speed: int, end_time: float) → None`

**Depends on:** `rc_controller` (channel constants, `percent_to_pwm`, `build_diagonal_channels`)

---

### 2.4 connection_manager.py (246 lines)

**Role:** Serial port lifecycle — discovery, connection, reconnection with
exponential backoff, heartbeat transmission, message reading.

| Item | Kind | Purpose |
|---|---|---|
| `ConnectionManager` | class | Owns `mavutil.mavlink_connection` handle |
| `connect()` | method | Connects with exponential backoff (2 s → 15 s max) |
| `reconnect()` | method | Teardown + re-connect cycle |
| `send_heartbeat()` | method | GCS heartbeat, tracks last-sent time, triggers reconnect on timeout |
| `read_message()` | method | Non-blocking `recv_match()` with `HEARTBEAT` filter |
| `send()` | method | Raw `mav.send()` wrapper |
| `close()` | method | Graceful teardown |
| `_detect_port()` | static | Scans `/dev/ttyACM*` and `/dev/ttyUSB*` for ArduSub |

**Reconnection logic:** After `heartbeat_timeout` seconds (default 3.0) without
a heartbeat response, the manager tears down the connection and enters an
exponential backoff loop: 2 s → 4 s → 8 s → 15 s (capped).

**Depends on:** `pymavlink.mavutil`

---

### 2.5 rc_controller.py (206 lines)

**Role:** PWM computation and RC channel management.

| Item | Kind | Purpose |
|---|---|---|
| **Channel constants** | module-level | `CH_PITCH=1`, `CH_ROLL=2`, `CH_THROTTLE=3`, `CH_YAW=4`, `CH_FORWARD=5`, `CH_LATERAL=6`, `NEUTRAL_PWM=1500`, `PWM_RANGE=400` |
| `percent_to_pwm()` | function | Converts speed percentage (0–100) to PWM offset (0–400). Testable pure function. |
| `build_diagonal_channels()` | function | Computes forward + lateral channels at 1/√2 scaling for diagonal movement. Testable pure function. |
| `RcController` | class | Manages 8-channel PWM state with trapezoidal velocity ramp |
| `set_movement()` | method | Sets target channels + auto-stop timer |
| `get_channels()` | method | Returns current 8-channel list, applying ramp towards targets |
| `stop()` | method | Zeros all channels to neutral |
| `is_active()` | property | True if any channel ≠ neutral |

**Ramp behavior:** When `ramp_rate` > 0, PWM values change at most `ramp_rate`
PWM/second towards targets. This produces smooth acceleration/deceleration.

**Depends on:** nothing (standalone)

---

### 2.6 pid_controller.py (162 lines)

**Role:** Generic PID controller used for both depth hold and yaw hold.

| Item | Kind | Purpose |
|---|---|---|
| `PidController` | class | Configurable PID with advanced features |
| `__init__()` | method | Takes Kp, Ki, Kd, max_output, deadband, max_rate, derivative_filter |
| `compute()` | method | Returns control output given setpoint and measurement |
| `reset()` | method | Zeros integral and derivative state |
| `set_setpoint()` | method | Updates target value |

**Features:**
- **Deadband:** Output = 0 when |error| < tolerance (prevents oscillation at setpoint)
- **Anti-windup:** Conditional integration — integral accumulates only when output is not saturated
- **EMA derivative:** Exponential moving average on derivative-on-measurement (not on error), configurable filter coefficient
- **Rate limiting:** Output change limited to `max_rate` per tick (prevents PWM jumps)

**Usage:** Two instances created by `CommandHandler.create_depth_pid()` and
`create_yaw_pid()` with different gains. Both called from `_rc_override_tick()`.

**Depends on:** nothing (standalone, fully unit-testable)

---

### 2.7 telemetry_parser.py (159 lines)

**Role:** Converts incoming MAVLink messages to vehicle state fields.

| Item | Kind | Purpose |
|---|---|---|
| `TelemetryParser` | class | Dispatch-table message router |
| `parse()` | method | Looks up `msg.get_type()` in `_dispatch` dict, calls handler |
| `_dispatch` | dict | 7 entries: HEARTBEAT, AHRS2, ATTITUDE, SYS_STATUS, SCALED_PRESSURE, SERVO_OUTPUT_RAW, RC_CHANNELS |
| `_handle_heartbeat()` | method | Extracts armed state, flight mode, system status; emits arm/disarm/mode transition events via callback |
| `_handle_ahrs2()` | method | Lat/lng/altitude from AHRS2 |
| `_handle_attitude()` | method | Roll/pitch/yaw from ATTITUDE |
| `_handle_sys_status()` | method | Battery voltage/current/remaining |
| `_handle_scaled_pressure()` | method | External pressure → depth calculation (configurable surface pressure) |
| `_handle_servo_output()` | method | 8 servo output channels |
| `_handle_rc_channels()` | method | 8 RC input channels |

**Event callbacks:** On arm/disarm or mode transition, calls
`node.get_logger().info()` and publishes `MavlinkEvent`.

**Depends on:** nothing (standalone, testable with mock messages)

---

## 3. mavlink_driver

> **Purpose:** High-level command construction API and mission execution.
> Other nodes import from this package to build `DriverCommand` messages.

### 3.1 driver_client.py (262 lines)

**Role:** Factory functions for creating `DriverCommand` messages. This is the
public API that `mavlink_runner` and `mission_executor` import.

| Item | Kind | Purpose |
|---|---|---|
| `make_command()` | function | Core factory: `make_command(command, speed, duration, depth, angle, gain) → DriverCommand` |
| `move_forward()` | function | `make_command('move_forward', speed, duration)` |
| `move_back()` | function | Backward movement |
| `move_left()` / `move_right()` | functions | Lateral movement |
| `move_up()` / `move_down()` | functions | Vertical movement |
| `yaw_left()` / `yaw_right()` | functions | Yaw rotation |
| `resolve_relative_yaw()` | function | Computes absolute heading from relative angle + current heading |
| `turn_left()` / `turn_right()` | functions | Heading-based turn via `resolve_relative_yaw()` |
| `set_depth()` | function | Depth target command |
| `set_heading()` | function | Heading hold command |
| `arm()` / `disarm()` | functions | Arm/disarm commands |
| `stop()` | function | Stop all movement |
| `surface()` | function | Surface command |
| `go_forward_left()` ... | functions | 4 diagonal compound movements |
| `cruise()` | function | Cruise with heading hold |

**Depends on:** `duburi_interfaces.msg.DriverCommand`

---

### 3.2 just_commands.py (115 lines)

**Role:** Instant (no-ramp) variants of all movement commands. Prefixed with
`just_` to indicate they bypass the trapezoidal velocity ramp.

| Item | Kind | Purpose |
|---|---|---|
| `just_forward()` | function | Instant forward movement |
| `just_back()` | function | Instant backward |
| `just_left()` / `just_right()` | functions | Instant lateral |
| `just_up()` / `just_down()` | functions | Instant vertical |
| `just_yaw_left()` / `just_yaw_right()` | functions | Instant yaw |

All functions call `make_command()` from `driver_client.py` with the `just_`
prefix on the command string.

**Depends on:** `driver_client.make_command`

---

### 3.3 mission_parser.py (245 lines)

**Role:** Parses a single line from a mission file into a `DriverCommand`.

| Item | Kind | Purpose |
|---|---|---|
| `parse_file_command()` | function | `(current_heading, cmd, args, logger) → DriverCommand` |

**Parsing pipeline:**
```
raw line → strip → lowercase
  ├─ prefix "just " → add just_ prefix, recurse
  ├─ prefix "~"     → alias lookup, recurse
  ├─ prefix "move " → strip prefix, recurse
  ├─ movement commands → driver_client.move_forward(speed, duration) etc.
  ├─ depth/heading   → driver_client.set_depth/set_heading(value)
  ├─ turn left/right → driver_client.turn_left/right(angle, current_heading)
  ├─ go_*            → driver_client.go_*(speed, duration)
  ├─ cruise          → driver_client.cruise(speed, duration, heading)
  ├─ grab open/close → make_command('grab_open/close')
  ├─ arm / disarm    → driver_client.arm/disarm()
  └─ unknown         → log warning, return None
```

**Depends on:** `driver_client` (all factory functions), `duburi_common.command_vocabulary` (`resolve_prefixes`, `HORIZONTAL_DIRS`)

---

### 3.4 mission_executor.py (318 lines)

**Role:** Loads and runs mission files or built-in missions sequentially.

| Item | Kind | Purpose |
|---|---|---|
| `MissionExecutorNode` | class (Node) | ROS 2 node for autonomous missions |
| `__init__()` | method | Subscribes to `/mavlink/vehicle_state`, creates `/driver/command` publisher |
| `run_mission_file()` | method | Loads `.txt` file, iterates lines, calls `_parse_file_command()` |
| `run_builtin()` | method | Runs hardcoded mission sequences (e.g., `gate`) |
| `_parse_file_command()` | method | Delegates to `mission_parser.parse_file_command()` |
| `_publish_command()` | method | Publishes `DriverCommand` and waits for duration |
| `_interruptible_sleep()` | method | Sleeps with abort/pause checks |
| `_abort()` | method | SIGINT handler — stops, disarms, shuts down |
| `_pause()` / `_resume()` | methods | Pause/resume mission execution |

**Mission file format:**
```
# Comment lines start with #
forward 50 3.0      # command speed duration
depth 1.5            # set depth
turn left 90         # relative turn
just forward 80 2.0  # instant (no-ramp) variant
~ my_alias 60 3.0   # alias lookup
```

**Depends on:** `mission_parser`, `driver_client`, `duburi_common.constants` (`MISSION_PATHS`)

---

### 3.5 teleop_driver.py (104 lines)

**Role:** Converts `geometry_msgs/Twist` messages to `TeleopCommand` for
joystick/gamepad control.

| Item | Kind | Purpose |
|---|---|---|
| `TeleopDriverNode` | class (Node) | Subscribes to `/cmd_vel`, publishes to `/driver/teleop` |
| `_twist_cb()` | method | Maps Twist axes to `TeleopCommand` fields with 0.1 dead-zone; publishes `idle=True` when centred |

**Axis mapping:**
| Twist field | TeleopCommand field | Meaning |
|---|---|---|
| `linear.x` | `linear_x` | Forward / back |
| `linear.y` | `linear_y` | Left / right |
| `linear.z` | `linear_z` | Up / down |
| `angular.z` | `angular_z` | Yaw |

`speed` is set from the `max_speed` parameter (PWM cap). Design Issue 7 (DriverCommand field overloading) is **resolved** — see `10_DESIGN_ISSUES.md`.

**Depends on:** `duburi_interfaces.msg.TeleopCommand`, `geometry_msgs.msg.Twist`

---

## 4. mavlink_runner

> **Purpose:** Human-facing interactive CLI for sending commands and running
> missions. Intended for poolside use during testing.

### 4.1 command_parser.py (470 lines)

**Role:** Parses human-typed command strings into `DriverCommand` messages.

| Item | Kind | Purpose |
|---|---|---|
| `parse_command()` | function | `(node, line) → DriverCommand \| None` — main entry point |
| `try_compound_move()` | function | Parses `go_forward_left`-style compounds |
| Duration/gain regexes | module-level | `r'(\d+\.?\d*)\s*s'` for duration, etc. |

**Parsing pipeline:**
```
user input → strip → lowercase
  ├─ "help" / "status" / "quit" → special handling (print, return None)
  ├─ "mission <name>" → load and run mission file
  ├─ prefix "just " → just_ prefix, continue
  ├─ prefix "~"     → alias lookup, continue
  ├─ speed extraction → regex for percentage or raw int
  ├─ duration extraction → regex for "Ns" or float
  ├─ movement commands → driver_client.move_*(speed, duration)
  ├─ depth / heading → driver_client.set_depth/heading()
  ├─ turn → driver_client.turn_left/right()
  ├─ go_* → driver_client.go_*()
  ├─ cruise → driver_client.cruise()
  ├─ grab → make_command('grab_open/close')
  ├─ arm / disarm / stop → direct commands
  └─ unknown → error message, return None
```

**Depends on:** `duburi_interfaces.msg.DriverCommand`, `duburi_common.command_vocabulary` (`DIRECTION_TO_COMMAND`, `HORIZONTAL_DIRS`, `build_command_name`, `build_compound_name`, `resolve_prefixes`), `constants` (`HELP_TEXT`), `status_display`

---

### 4.2 runner.py (268 lines)

**Role:** The interactive REPL node. Runs a prompt loop in a background thread.

| Item | Kind | Purpose |
|---|---|---|
| `DuburiRunnerNode` | class (Node) | ROS 2 node with interactive prompt |
| `__init__()` | method | Creates publisher, subscriber, readline setup |
| `_prompt_loop()` | method | Thread: `input()` → `_parse_one()` → publish |
| `_parse_one()` | method | Delegates to `command_parser.parse_command()` |
| `_on_vehicle_state()` | method | Caches latest state for status display |
| `_run_mission()` | method | Reads mission file, parses line-by-line |
| `_safe_disarm()` | method | Best-effort disarm on exit |

**Readline integration:** Loads/saves command history from
`~/.duburi_history`. Tab completion is not implemented.

**Depends on:** `command_parser`, `constants`, `status_display`, `duburi_common.constants` (`UNARMED_ALLOWED` for disarmed command gating)

---

### 4.3 constants.py (104 lines)

**Role:** Runner-specific help text; re-exports shared paths from `duburi_common`.

| Item | Kind | Purpose |
|---|---|---|
| `HELP_TEXT` | str | Long help string displayed by `help` command |
| `MISSION_PATHS`, `HISTORY_FILE` | re-export | From `duburi_common.constants` (canonical definitions) |

**Depends on:** `duburi_common.constants` (re-exports only)

---

### 4.4 status_display.py (75 lines)

**Role:** Formatted vehicle status dashboard printed to terminal.

| Item | Kind | Purpose |
|---|---|---|
| `print_status()` | function | ANSI-colored dashboard: armed state, mode, battery bar, heading compass, depth, attitude, servo/RC values |

**Depends on:** nothing (standalone, testable with mock VehicleState)

---

## 5. mavlink_logger

> **Purpose:** Logs subscribed topics (events, vehicle state, commands) to CSV and JSON files for post-dive analysis.

### 5.1 logger_node.py (233 lines)

| Item | Kind | Purpose |
|---|---|---|
| `MavlinkLoggerNode` | class (Node) | Subscribes to events, vehicle state, and commands; writes to files |
| Subscriptions | 3 topics | `MavlinkEvent`, `VehicleState` (`/mavlink/vehicle_state`), `DriverCommand` |
| CSV writer | method | Appends timestamped rows to `<workspace>/logs/YYYY-MM-DD_HH-MM-SS.csv` |
| JSON writer | method | Appends JSON lines to corresponding `.jsonl` file |

**Log directory:** `<workspace>/logs/` with automatic session-based filenames.

**Depends on:** `duburi_interfaces.msg.*`

---

## 6. vision

> **Purpose:** YOLO-based object detection for underwater gate/buoy recognition,
> Kalman-filtered bounding box tracking, and closed-loop visual servoing.

### 6.1 detector_node.py (263 lines)

| Item | Kind | Purpose |
|---|---|---|
| `DetectorNode` | class (Node) | Subscribes to camera image, runs YOLO, publishes detections |
| `_on_image()` | method | Converts ROS Image → CV2 → YOLO → DetectionArray + annotated image |
| Model loading | `__init__` | Loads YOLO `.pt` model with configurable path and confidence threshold |

**Depends on:** `image_utils`, `ultralytics`, `duburi_interfaces.msg.DetectionArray`

---

### 6.2 alignment_controller.py (635 lines)

**Role:** Visual servo controller that aligns AUV to a detected target (gate post,
buoy). Provides three control modes: PID, proportional, and bang-bang.

| Item | Kind | Purpose |
|---|---|---|
| `AlignmentController` | class (Node) | Subscribes to detections, publishes `TeleopCommand` for closed-loop alignment |
| `ControlMode` | enum | `PID`, `PROPORTIONAL`, `BANG_BANG` |
| `_on_detection()` | method | Computes error from bounding box center to image center, applies control law |
| `_pid_control()` | method | Full PID with integral anti-windup and derivative filtering |
| `_proportional_control()` | method | Simple gain × error |
| `_bangbang_control()` | method | Fixed-speed control above deadband threshold |
| `_publish_alignment_status()` | method | Publishes `AlignmentStatus` msg for mission planner feedback |
| `reset()` | method | Zeros PID integral terms, called on target loss |

**Parameters:** `kp_yaw`, `ki_yaw`, `kd_yaw`, `kp_forward`, `target_class`, `deadband_x`, `deadband_area`, `control_mode`

**Depends on:** `duburi_interfaces.msg.TeleopCommand`, `duburi_interfaces.msg.AlignmentStatus`, `duburi_interfaces.msg.DetectionArray`

---

### 6.3 kalman_tracker.py (238 lines)

**Role:** Kalman filter for smoothing YOLO bounding box detections and handling
temporary dropouts (occlusion, missed frames).

| Item | Kind | Purpose |
|---|---|---|
| `KalmanTracker` | class | Tracks single target with position + velocity state |
| `predict()` | method | State extrapolation step (called every frame) |
| `update()` | method | Measurement incorporation when detection available |
| `get_smoothed_bbox()` | method | Returns filtered bounding box (cx, cy, w, h) |
| `is_valid()` | property | True if tracker has recent measurements (within dropout threshold) |
| `reset()` | method | Reinitializes filter state for new target |

**Features:**
- 8-state vector: [cx, cy, w, h, vx, vy, vw, vh]
- Configurable process/measurement noise (Q, R matrices)
- Dropout handling: continues prediction for N frames without measurement
- Automatic reset on large innovation (new target detection)

**Depends on:** `numpy` (standalone, fully unit-testable)

---

### 6.4 detector_standalone.py (175 lines)

| Item | Kind | Purpose |
|---|---|---|
| `StandaloneDetector` | class | Runs YOLO on camera feed without ROS (for testing/demo) |
| `run()` | method | OpenCV capture → YOLO → display with bounding boxes |

**Depends on:** `ultralytics`, `cv2`

---

### 6.5 image_utils.py (63 lines)

**Role:** cv_bridge replacement for ROS 2 Image ↔ OpenCV conversion.

| Item | Kind | Purpose |
|---|---|---|
| `ros_image_to_cv2()` | function | `Image → np.ndarray` — supports `bgr8`, `rgb8`, `mono8` |
| `cv2_to_ros_image()` | function | `np.ndarray → Image` — reverse conversion |

**Depends on:** `numpy`, `sensor_msgs.msg.Image`

---

### 6.6 config.py (~50 lines)

**Role:** Centralized vision configuration constants.

| Item | Kind | Purpose |
|---|---|---|
| `DEFAULT_MODEL_PATH` | constant | Default YOLO model location |
| `DEFAULT_CONFIDENCE` | constant | Detection confidence threshold |
| `DEFAULT_IMAGE_SIZE` | constant | YOLO inference resolution |

**Depends on:** nothing (standalone)

---

## 7. vision_inspector

> **Purpose:** Camera device management, calibration, recording/playback, and raw image publishing.

### 7.1 camera_manager_node.py (~250 lines)

| Item | Kind | Purpose |
|---|---|---|
| `CameraManagerNode` | class (Node) | Multi-camera orchestrator. Discovers cameras, creates `FramePublisher` per camera, publishes `/camera/<name>/image_raw` |
| Parameters | ROS params | `camera_configs` (YAML list of camera name→device mappings) |
| `_on_status_timer()` | method | Publishes `CameraStatusArray` to `/vision_inspector/status` at 1 Hz |

> **Note:** The old single-camera `camera_node.py` has been replaced by `camera_manager_node.py` + `frame_publisher.py` + `camera_device.py` for multi-camera support.

---

### 7.2 camera_calibrator.py (276 lines)

| Item | Kind | Purpose |
|---|---|---|
| `CameraCalibratorNode` | class (Node) | Chessboard calibration with OpenCV |
| `calibrate()` | method | Collects frames, computes intrinsic matrix, saves to YAML |

---

### 7.3 camera_enumerator.py (165 lines)

| Item | Kind | Purpose |
|---|---|---|
| `CameraEnumeratorNode` | class (Node) | Discovers and lists available V4L2 cameras |
| `enumerate()` | method | Scans `/dev/video*`, probes capabilities, publishes list |

---

### 7.4 camera_tester.py (151 lines)

| Item | Kind | Purpose |
|---|---|---|
| `CameraTesterNode` | class (Node) | Opens camera and displays live feed for verification |
| `test()` | method | Captures frames, shows in OpenCV window |

---

### 7.5 camera_recorder.py (~180 lines)

| Item | Kind | Purpose |
|---|---|---|
| `CameraRecorderNode` | class (Node) | Records camera stream to video file (MP4/AVI) |
| `_on_image()` | method | Writes frames to video writer with timestamp overlay |
| `_start_recording()` / `_stop_recording()` | methods | Lifecycle control |

**Parameters:** `output_dir`, `codec`, `fps`, `record_on_start`

---

### 7.6 camera_playback.py (~150 lines)

| Item | Kind | Purpose |
|---|---|---|
| `CameraPlaybackNode` | class (Node) | Replays recorded video as ROS Image topic |
| `_playback_loop()` | method | Timer-driven frame publishing at original FPS |

**Parameters:** `video_file`, `loop`, `start_paused`

---

### 7.7 frame_publisher.py (~120 lines)

| Item | Kind | Purpose |
|---|---|---|
| `FramePublisherNode` | class (Node) | Publishes individual image files as ROS Images |
| `publish_frame()` | method | Loads image from disk, publishes once |

**Use case:** Testing detection pipeline with static images.

---

### 7.8 calibration_store.py (~100 lines)

| Item | Kind | Purpose |
|---|---|---|
| `CalibrationStore` | class | YAML-based camera calibration storage |
| `load()` / `save()` | methods | Read/write intrinsic matrix, distortion coefficients |
| `get_undistort_maps()` | method | Returns OpenCV remap arrays for undistortion |

**Depends on:** `pyyaml`, `numpy`, `cv2`

---

## 8. duburi_planner

> **Purpose:** YASMIN-based finite state machine mission planner for autonomous
> task execution. Implements hierarchical state machines for RoboSub missions.

### 8.1 mission_node.py (~250 lines)

**Role:** Main ROS 2 node that orchestrates mission execution via YASMIN FSM.

| Item | Kind | Purpose |
|---|---|---|
| `MissionPlannerNode` | class (Node) | Creates state machine, manages mission lifecycle |
| `__init__()` | method | Builds FSM from mission config, creates PlannerContext |
| `run_mission()` | method | Executes selected mission FSM |
| `_on_abort()` | method | SIGINT handler — transitions to safe state, surfaces |

**Parameters:** `mission_name`, `enable_viz` (YASMIN viewer)

**Depends on:** `yasmin`, `planner_context`, `missions/*`

---

### 8.2 planner_context.py (~200 lines)

**Role:** Shared context object passed to all states via YASMIN blackboard.
Holds ROS publishers, subscribers, and cached vehicle state.

| Item | Kind | Purpose |
|---|---|---|
| `PlannerContext` | class | Container for shared ROS resources |
| `command_pub` | attribute | Publisher for `/driver/command` |
| `teleop_pub` | attribute | Publisher for `/driver/teleop` |
| `vehicle_state` | property | Cached latest `VehicleState` |
| `detections` | property | Cached latest `DetectionArray` |
| `alignment_status` | property | Cached latest `AlignmentStatus` |
| `send_command()` | method | Publishes `DriverCommand` with optional wait |
| `wait_for_depth()` | method | Blocks until depth within tolerance |
| `wait_for_alignment()` | method | Blocks until alignment converged |

**Depends on:** `duburi_interfaces.msg.*`, `mavlink_driver.driver_client`

---

### 8.3 planner_config.py (~150 lines)

**Role:** Dataclasses for mission and task configuration.

| Item | Kind | Purpose |
|---|---|---|
| `TaskConfig` | dataclass | Per-task tunables (timeouts, thresholds, speeds) |
| `MissionConfig` | dataclass | Mission-level settings (task sequence, abort behavior) |
| `load_config()` | function | Loads config from YAML file |
| `GateConfig`, `BuoyConfig`, etc. | dataclasses | Task-specific configuration |

---

### 8.4 bb_utils.py (~80 lines)

**Role:** YASMIN blackboard utility functions.

| Item | Kind | Purpose |
|---|---|---|
| `get_context()` | function | Retrieves `PlannerContext` from blackboard |
| `set_task_result()` | function | Stores task outcome in blackboard |
| `get_task_result()` | function | Retrieves previous task outcome |

---

### 8.5 states/ Directory (8 files, ~800 lines total)

**Role:** YASMIN state implementations for mission primitives.

#### states/arm.py

| Item | Kind | Purpose |
|---|---|---|
| `ArmState` | class (State) | Arms the vehicle, sets mode |
| Outcomes | | `succeeded`, `failed` |

#### states/submerge.py

| Item | Kind | Purpose |
|---|---|---|
| `SubmergeState` | class (State) | Descends to target depth via PID |
| Outcomes | | `succeeded`, `failed`, `aborted` |

#### states/drive.py

| Item | Kind | Purpose |
|---|---|---|
| `DriveState` | class (State) | Drives forward through gate or toward target |
| `execute()` | method | Sends forward command for configured duration |
| Outcomes | | `succeeded`, `aborted` |

#### states/surface.py

| Item | Kind | Purpose |
|---|---|---|
| `SurfaceState` | class (State) | Ascends to surface, disarms |
| `execute()` | method | Sends surface command, waits, disarms |
| Outcomes | | `succeeded`, `failed` |

#### states/search.py

| Item | Kind | Purpose |
|---|---|---|
| `SearchState` | class (State) | Searches for target object (yaw sweep or forward cruise) |
| Outcomes | | `found`, `not_found`, `aborted` |

#### states/align.py

| Item | Kind | Purpose |
|---|---|---|
| `AlignState` | class (State) | Visual servo alignment to detected target |
| `execute()` | method | Enables alignment controller, waits for convergence |
| Outcomes | | `aligned`, `lost_target`, `timeout`, `aborted` |

#### states/wait_feedback.py

| Item | Kind | Purpose |
|---|---|---|
| `WaitFeedbackState` | class (State) | Waits for `DriverCommandFeedback` confirmation |
| Outcomes | | `reached`, `timeout`, `rejected` |

#### states/send_command.py

| Item | Kind | Purpose |
|---|---|---|
| `SendCommandState` | class (State) | Publishes a `DriverCommand` and optionally waits |
| Outcomes | | `succeeded`, `failed` |

---

### 8.6 missions/ Directory (2 files)

**Role:** Mission-specific state machine definitions.

#### missions/gate.py

| Item | Kind | Purpose |
|---|---|---|
| `create_gate_fsm()` | function | Builds YASMIN state machine for gate task |

**State sequence:**
```
START → ArmState → SubmergeState → SearchState → AlignState → DriveState → SurfaceState → END
                                         ↑              │
                                         └──lost_target─┘
```

#### missions/demo_square.py

| Item | Kind | Purpose |
|---|---|---|
| `create_demo_square_fsm()` | function | Builds state machine for a simple square movement pattern (testing/demo) |

> **Note:** There is no `mission_registry.py` or `buoy_mission.py`. Missions are loaded by name in `mission_node.py`.

---

## 9. duburi_common

> **Purpose:** Shared Python library (no ROS nodes): command vocabulary and stack-wide constants so `mavlink_runner` and `mavlink_driver` do not duplicate parsing rules, paths, or allow-lists.

### 9.1 command_vocabulary.py

| Item | Kind | Purpose |
|---|---|---|
| `ALIASES` | dict | User-facing alias → canonical command (or tuple with PID flag), e.g. `dive` → `depth` |
| `DIRECTION_TO_COMMAND` | dict | Direction word → `DriverCommand.command` name (`forward` → `move_forward`, etc.) |
| `HORIZONTAL_DIRS` | frozenset | Valid horizontal directions for compound / `go_*` parsing |
| `resolve_prefixes()` | function | Resolves `just` prefix, `~` PID prefix, and alias expansion on tokenized input |
| `build_command_name()` | function | Builds normalized movement command strings (incl. `just_` variants) |
| `build_compound_name()` | function | Builds hyphenated compound command names from horizontal parts |

**Imported by:** `mavlink_runner.command_parser`, `mavlink_driver.mission_parser`

### 9.2 constants.py

| Item | Kind | Purpose |
|---|---|---|
| `MISSION_PATHS` | list[Path] | Search paths for mission files |
| `HISTORY_FILE` | Path | CLI readline history location (`~/.duburi_history`) |
| `DEFAULT_SPEED` | int | Default movement gain for parsers / UX |
| `ARM_WAIT`, `DISARM_WAIT`, `SURFACE_WAIT` | float | Timing constants for arm/disarm/surface sequences |
| `UNARMED_ALLOWED` | frozenset | Commands permitted when disarmed (runner gate) |
| `UNARMED_ALLOWED_INSPECTOR` | frozenset | Superset used by inspector (extra passthrough / alignment commands) |

**Imported by:** `mavlink_runner.constants` (re-exports paths), `mavlink_runner.runner`, `mavlink_driver.mission_executor`, `mavlink_inspector.command_handler`

---

## 10. duburi_blueos

> **Purpose:** REST API client for the BlueOS companion computer (Raspberry Pi 4B).
> Monitors system health, manages endpoints, and publishes status to ROS 2.

### 10.1 blueos_monitor_node.py

| Item | Kind | Purpose |
|---|---|---|
| `BlueosMonitorNode` | class (Node) | ROS 2 node that periodically polls BlueOS endpoints and publishes system status |
| `_status_timer()` | method | Timer callback that queries health and publishes to `/blueos/system_status` |

### 10.2 blueos_client.py

| Item | Kind | Purpose |
|---|---|---|
| `BlueosClient` | class | HTTP client for BlueOS REST API (system info, MAVLink endpoints, extensions) |
| `get_system_info()` | method | Fetches CPU, memory, disk, temperature from BlueOS |
| `get_mavlink_endpoints()` | method | Lists active MAVLink endpoint configurations |

### 10.3 blueos_config.py

| Item | Kind | Purpose |
|---|---|---|
| Configuration | module | BlueOS connection settings (IP, port, endpoints) |

### 10.4 endpoints.py

| Item | Kind | Purpose |
|---|---|---|
| `ENDPOINTS` | dict | Mapping of BlueOS REST API endpoint paths |

### 10.5 health_checker.py

| Item | Kind | Purpose |
|---|---|---|
| `HealthChecker` | class | Evaluates system health metrics against thresholds (CPU temp, disk space, memory) |

**Depends on:** `requests`, `duburi_interfaces.msg` (for status messages)

---

## 11. duburi_interfaces

> **Purpose:** ROS 2 message and service definitions shared by all packages.

### Messages (msg/)

| Message | Fields | Used By |
|---|---|---|
| `DriverCommand` | `command`, `mode`, `depth`, `angle`, `duration`, `speed`, `status` | driver, runner, planner → inspector, logger |
| `TeleopCommand` | `linear_x`, `linear_y`, `linear_z`, `angular_z`, `speed`, `idle` | teleop_driver, alignment_controller → inspector |
| `VehicleState` | `armed`, `mode`, `heading`, `depth`, `roll`, `pitch`, `yaw`, `battery_voltage`, `battery_current`, `battery_remaining`, `servos[8]`, `rc_channels[8]` | inspector → all |
| `VehicleDiagnostics` | `cpu_temp`, `board_voltage`, `system_status`, `errors_count` | inspector (not subscribed by mavlink_logger) |
| `MavlinkEvent` | `event_type`, `description`, `timestamp` | inspector → logger |
| `DetectionArray` | `detections[]` — each with `class_name`, `confidence`, `bbox` | vision → planner, alignment_controller |
| `AlignmentStatus` | `aligned`, `error_x`, `error_y`, `error_area`, `target_visible` | alignment_controller → planner |
| `CameraStatus` | `device_id`, `width`, `height`, `fps`, `is_recording` | vision_inspector → logger |

---

## 12. Cross-Package Dependency Graph

```
duburi_interfaces (ROS msgs/srv)              duburi_common (Python library)
            │                                           │
            ├──────────────┬────────────────────────────┼──────────────┐
            ▼              ▼                            ▼              ▼
   mavlink_logger   mavlink_inspector            mavlink_driver   mavlink_runner
                          │                            │              │
                          │                            └──────┬───────┘
                          │                                   │
                     pymavlink                          runner imports
                    + internal mods                       driver_client,
                                                            just_commands

            vision ──────────────────────────► duburi_interfaces + ultralytics
               │                                         ▲
               │ (AlignmentStatus, DetectionArray)       │
               ▼                                         │
         duburi_planner ─────────────────────────────────┘
               │
               └──► yasmin (YASMIN FSM library)

            vision_inspector ──► duburi_interfaces only

            duburi_blueos ──► requests (HTTP client)
```

**Key dependency rules:**
- `mavlink_inspector` imports `duburi_interfaces` and `duburi_common` (allow-list only); it does not import `mavlink_driver` or `mavlink_runner`
- `mavlink_driver` depends on `duburi_interfaces` and `duburi_common`
- `mavlink_runner` imports from `mavlink_driver` (`driver_client`, `just_commands`) and `duburi_common`
- `vision` depends on `duburi_interfaces` and `ultralytics`
- `duburi_planner` depends on `duburi_interfaces`, `mavlink_driver.driver_client`, `yasmin`, and subscribes to `vision` topics
- All inter-package communication is via ROS 2 topics (loose coupling)

---

## 13. Import Map

### What imports what (Python-level)

```
mavlink_runner.command_parser
  ← duburi_interfaces.msg.DriverCommand  (inline message construction)
  ← mavlink_runner.constants        (HELP_TEXT)
  ← mavlink_runner.status_display   (print_status)
  ← duburi_common.command_vocabulary (resolve_prefixes, DIRECTION_TO_COMMAND, …)

mavlink_runner.runner
  ← mavlink_runner.command_parser   (parse_command)
  ← mavlink_runner.constants        (HELP_TEXT, HISTORY_FILE)
  ← mavlink_runner.status_display   (print_status)
  ← duburi_common.constants         (UNARMED_ALLOWED)

mavlink_driver.just_commands
  ← mavlink_driver.driver_client    (make_command)

mavlink_driver.mission_parser
  ← mavlink_driver.driver_client    (all factory functions)
  ← duburi_common.command_vocabulary (resolve_prefixes, HORIZONTAL_DIRS)

mavlink_driver.mission_executor
  ← mavlink_driver.mission_parser   (parse_file_command)
  ← mavlink_driver.driver_client    (stop, disarm)
  ← duburi_common.constants         (MISSION_PATHS)

mavlink_inspector.inspector_node
  ← mavlink_inspector.connection_manager
  ← mavlink_inspector.telemetry_parser
  ← mavlink_inspector.rc_controller
  ← mavlink_inspector.pid_controller
  ← mavlink_inspector.command_handler

mavlink_inspector.command_handler
  ← mavlink_inspector.movement_commands  (MOVEMENTS, handle_go, handle_compound_move)
  ← mavlink_inspector.pid_controller     (PidController)
  ← duburi_common.constants              (UNARMED_ALLOWED_INSPECTOR)

mavlink_inspector.movement_commands
  ← mavlink_inspector.rc_controller      (channel constants, percent_to_pwm, build_diagonal_channels)

vision.detector_node
  ← vision.image_utils              (ros_image_to_cv2, cv2_to_ros_image)

vision.alignment_controller
  ← vision.kalman_tracker           (KalmanTracker)
  ← duburi_interfaces.msg           (TeleopCommand, AlignmentStatus, DetectionArray)

duburi_planner.planner_context
  ← mavlink_driver.driver_client    (make_command, arm, disarm, stop, etc.)
  ← duburi_interfaces.msg           (DriverCommand, VehicleState, DetectionArray, AlignmentStatus)

duburi_planner.states.*
  ← duburi_planner.planner_context  (PlannerContext)
  ← duburi_planner.bb_utils         (get_context, set_task_result)
  ← yasmin                          (State, Blackboard)

duburi_planner.missions.*
  ← duburi_planner.states.*         (all state classes)
  ← yasmin                          (StateMachine)
```

### No circular dependencies

After the fix in commit `f62781f`, the import graph is a strict DAG.
The circular dependency between `driver_client.py` ↔ `just_commands.py` was
resolved by removing the unused re-export block from `driver_client.py`.

---

## Quick Lookup: "Where is X?"

| Looking for… | File | Package |
|---|---|---|
| `percent_to_pwm()` | `rc_controller.py` | mavlink_inspector |
| `build_diagonal_channels()` | `rc_controller.py` | mavlink_inspector |
| `PidController` | `pid_controller.py` | mavlink_inspector |
| `MOVEMENTS` dispatch dict | `movement_commands.py` | mavlink_inspector |
| `ConnectionManager` | `connection_manager.py` | mavlink_inspector |
| `TelemetryParser` | `telemetry_parser.py` | mavlink_inspector |
| `RcController` | `rc_controller.py` | mavlink_inspector |
| `CommandHandler` | `command_handler.py` | mavlink_inspector |
| `make_command()` | `driver_client.py` | mavlink_driver |
| `just_forward()` etc. | `just_commands.py` | mavlink_driver |
| `parse_file_command()` | `mission_parser.py` | mavlink_driver |
| `MissionExecutorNode` | `mission_executor.py` | mavlink_driver |
| `parse_command()` (CLI) | `command_parser.py` | mavlink_runner |
| `DuburiRunnerNode` | `runner.py` | mavlink_runner |
| `print_status()` | `status_display.py` | mavlink_runner |
| `HELP_TEXT` | `constants.py` | mavlink_runner |
| `ALIASES`, `resolve_prefixes`, compound helpers | `command_vocabulary.py` | duburi_common |
| `MISSION_PATHS`, `UNARMED_ALLOWED*` | `constants.py` | duburi_common |
| Channel constants | `rc_controller.py` | mavlink_inspector |
| Image conversion | `image_utils.py` | vision |
| YOLO detection | `detector_node.py` | vision |
| `AlignmentController` | `alignment_controller.py` | vision |
| `KalmanTracker` | `kalman_tracker.py` | vision |
| Camera publishing | `camera_manager_node.py` | vision_inspector |
| Camera recording | `camera_recorder.py` | vision_inspector |
| `MissionPlannerNode` | `mission_node.py` | duburi_planner |
| `PlannerContext` | `planner_context.py` | duburi_planner |
| FSM states | `states/*.py` | duburi_planner |
| Mission definitions | `missions/*.py` | duburi_planner |

---

*Document updated from full codebase audit. 80 Python source files across 10 packages.*
