# Message Interfaces

## Overview

All inter-node communication uses custom messages in `duburi_interfaces`. This document explains each message, field semantics, and usage patterns.

---

## DriverCommand.msg

**Purpose:** Single abstraction for all AUV control commands. Used by runner, mission_executor, teleop_driver, and any custom mission node.

**Topic:** `/driver/command` (publish-only from drivers; subscribe-only in inspector)

### Fields

| Field | Type | Semantics | Notes |
|-------|------|-----------|-------|
| `command` | string | Action to perform | See command list below |
| `mode` | string | Flight mode name **or** heading (cruise) | `set_mode`: MANUAL/ALT_HOLD/STABILIZE. `cruise`/`just_cruise`: target heading in degrees (parsed back to float). |
| `depth` | float32 | Target depth in meters | Positive = below surface. For `set_depth`, `p_dive`, `cruise`. Inspector negates for ArduSub (negative = below). **Teleop:** repurposed as throttle PWM offset. |
| `angle` | float32 | Heading/bearing in degrees 0-360 | For `yaw_angle`, `yaw_to_heading`, `pid_yaw_to_heading`, `go_*` (target heading), `move_at`/`just_move_at` (body-frame bearing), `cruise`/`just_cruise` (thrust bearing). **Teleop:** repurposed as yaw PWM offset. |
| `duration` | float32 | Seconds to sustain movement | 0 = indefinite. Used by move_*, yaw_left, yaw_right, go_*, cruise, etc. **Teleop:** repurposed as lateral PWM offset. |
| `speed` | int32 | Gain or PWM offset | 0-100 = percent (inspector converts via `percent_to_pwm`). >100 = raw PWM. **Teleop:** repurposed as forward PWM offset. |
| `status` | string | Optional metadata | Reserved for future use |

### Command Values

**Movement (basic):**
- `move_forward`, `move_back`, `move_left`, `move_right`, `move_up`, `move_down`
- Compound diagonals: `move_forward_right`, `move_forward_left`, `move_back_right`, `move_back_left`

**Movement (vector):**
- `move_at` — body-frame vector movement at arbitrary bearing (uses `angle` field)

**Yaw:**
- `yaw_angle` — legacy SET_ATTITUDE_TARGET (may not work in MANUAL)
- `yaw_to_heading` — bang-bang thruster rotation to target heading
- `pid_yaw_to_heading` — PID-controlled thruster rotation to target heading
- `yaw_left`, `yaw_right` — open-loop yaw

**Simultaneous movement + heading (go):**
- `go_forward`, `go_back`, `go_left`, `go_right` — single-axis movement + PID heading hold
- `go_forward_right`, `go_forward_left`, `go_back_right`, `go_back_left` — diagonal + heading

**Coordinated manoeuvre (cruise):**
- `cruise` — vector movement + depth PID + heading PID simultaneously
- Uses: `angle` = thrust bearing, `mode` = target heading, `depth` = target depth

**Depth:**
- `set_depth` (or `depth`) — firmware ALT_HOLD depth target
- `p_dive` (or `dive`) — software PID depth hold via throttle channel

**Lifecycle:** `arm`, `disarm`, `set_mode`, `stop`

**Actuators:** `open_grabber`, `close_grabber`

**Surface:** `surface` — clears all movement/PID and commands ascent

**Teleop:** `teleop` — direct multi-axis control (field overloading — see Issue 7 in 10_DESIGN_ISSUES.md)

**Instant (`just_*`) variants:**
All movement commands accept a `just_` prefix for bypass-ramp (instant PWM) operation:
`just_move_forward`, `just_move_back`, `just_move_left`, `just_move_right`,
`just_move_up`, `just_move_down`, `just_move_at`, `just_move_forward_right`, etc.,
`just_surface`, `just_go_forward`, `just_go_forward_right`, etc.,
`just_cruise`, `just_teleop`

### Design Rationale

- **Single topic:** All control flows through one topic. Inspector is the only subscriber. Simplifies architecture.
- **speed as percent or PWM:** 0-100 maps to percent (user-friendly). Values >100 allow direct PWM for advanced use. Inspector branches on `0 < raw_speed <= 100`.
- **duration 0 = indefinite:** Allows "move left" without timeout; user sends `stop` when done.
- **`mode` field dual-use:** The `mode` string field carries either a flight mode name (for `set_mode`) or a heading value (for `cruise`). Since these commands are mutually exclusive, the overloading is unambiguous.

---

## DriverCommandFeedback.msg

**Purpose:** Command acknowledgement published by the inspector for every handled command. Enables callers (mission executor, custom nodes) to verify command acceptance, track completion, or detect errors.

**Topic:** `/driver/feedback` (published by inspector; subscribed by mission nodes, logger)

### Fields

| Field | Type | Semantics |
|-------|------|-----------|
| `header` | std_msgs/Header | Timestamp (`frame_id='inspector'`) |
| `command` | string | The command string that was handled |
| `status` | string | One of: `accepted`, `reached`, `rejected`, `timeout`, `completed` |
| `error` | float32 | Numeric error value (e.g. heading error in degrees, depth error in metres) |
| `detail` | string | Human-readable detail about the status |

### Status Values

| Status | Meaning | Example |
|--------|---------|---------|
| `accepted` | Command received and processing started | `move_forward accepted` |
| `reached` | Target condition met (heading, depth) | `yaw_to_heading reached, error=1.2°` |
| `rejected` | Command invalid or cannot execute | `unknown command rejected` |
| `timeout` | Timed operation expired without reaching target | `yaw_to_heading timeout` |
| `completed` | Timed movement finished its duration | `go_forward completed, duration expired` |

### Design Rationale

- **Non-blocking:** Published asynchronously; callers don't need to wait.
- **Mission integration:** Mission executor can wait for `reached` before proceeding.
- **Safe shutdown:** `_publish_feedback()` catches exceptions and checks `rclpy.ok()`.

---

## MavlinkEvent.msg

**Purpose:** Broadcast MAVLink-related events for logging, UI, and confirmation.

**Topic:** `/mavlink/events`

### Fields

| Field | Type | Semantics |
|-------|------|-----------|
| `header` | std_msgs/Header | Timestamp, frame_id |
| `event_type` | string | Category: armed, disarmed, movement, mode_change, connected, etc. |
| `description` | string | Human-readable message |
| `raw_data` | string | Optional JSON/dump for debugging |

### Event Types

- `armed`, `disarmed`, `arm_failed`, `disarm_failed` — Arm/disarm confirmation
- `movement` — Any movement or stop
- `mode_change` — Flight mode changed
- `connected`, `connection_failed`, `connection_lost` — Connection state
- `command_ack` — MAVLink COMMAND_ACK received

### Design Rationale

- **Runner subscribes for arm/disarm:** Non-blocking; prints `[Armed.]` when event arrives.
- **Logger subscribes:** Writes all events to session/events logs.
- **Extensible:** New event types can be added without changing message definition.

---

## VehicleState.msg

**Purpose:** Current telemetry snapshot. Published at 10 Hz by inspector.

**Topic:** `/mavlink/vehicle_state`

### Fields

| Field | Type | Semantics |
|-------|------|-----------|
| `armed` | bool | Motors armed |
| `flight_mode` | string | ArduSub mode name |
| `depth` | float32 | Depth in m (negative below surface) |
| `yaw`, `pitch`, `roll` | float32 | Attitude in degrees |
| `voltage` | float32 | Battery voltage (V) |
| `current` | float32 | Battery current (A) |

### Design Rationale

- **depth 1:** Avoids multiple subscribers hammering MAVLink for AHRS2.
- **yaw 0-360:** Normalized for display and yaw_to_heading error calculation.

---

## Adding New Commands

1. Add command string to `DriverCommand.msg` comments.
2. Add handler in `inspector_node.py` `_on_driver_command`.
3. Add helper in `driver_client.py` if used by mission nodes.
4. Add parser branch in `runner.py` `_parse_one` if CLI support needed.
5. Add parser branch in `mission_executor.py` `_parse_file_command` for mission file support.
6. Update command reference: `analysis/12_COMMAND_REFERENCE.md`.
