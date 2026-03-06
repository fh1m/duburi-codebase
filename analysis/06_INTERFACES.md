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
| `mode` | string | Flight mode name | Only for `set_mode`: MANUAL, ALT_HOLD, STABILIZE |
| `depth` | float32 | Target depth in meters | Positive = below surface. For `set_depth`. Inspector negates for ArduSub (negative = below). |
| `angle` | float32 | Heading in degrees 0-360 | For `yaw_angle`, `yaw_to_heading` |
| `duration` | float32 | Seconds to sustain movement | 0 = indefinite. Used by move_*, yaw_left, yaw_right. |
| `speed` | int32 | Gain or PWM offset | 0-100 = percent (inspector converts via percent_to_pwm). >100 = PWM offset from 1500. |
| `status` | string | Optional metadata | Reserved for future use |

### Command Values

- **Movement:** `move_forward`, `move_back`, `move_left`, `move_right`, `move_up`, `move_down`
- **Yaw:** `yaw_angle` (legacy set_attitude), `yaw_to_heading` (thruster-based), `yaw_left`, `yaw_right`
- **Depth:** `set_depth` (or `depth`)
- **Lifecycle:** `arm`, `disarm`, `set_mode`, `stop`
- **Actuators:** `open_grabber`, `close_grabber`

### Design Rationale

- **Single topic:** All control flows through one topic. Inspector is the only subscriber. Simplifies architecture.
- **speed as percent or PWM:** 0-100 maps to percent (user-friendly). Values >100 allow direct PWM for advanced use. Inspector branches on `0 < raw_speed <= 100`.
- **duration 0 = indefinite:** Allows "move left" without timeout; user sends `stop` when done.

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
