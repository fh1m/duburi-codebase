# Design Issues — Architectural & Structural Concerns

> These are **not bugs** — the code works correctly. These are structural
> decisions that affect maintainability, testability, and future development
> velocity. Ordered by severity, then effort.

---

## Issue 1 — God Object: `MavlinkInspectorNode` (1300+ lines)

**Severity:** HIGH  |  **Effort to fix:** HIGH

### What It Is

`inspector_node.py` contains a single 1300-line class that handles everything:

| Responsibility | Lines | Description |
|---|---|---|
| Serial connection & reconnect | ~L100–L328 | Port scanning, heartbeat, auto-reconnect with backoff |
| MAVLink message parsing | ~L396–L520 | 8 message types (HEARTBEAT, AHRS2, ATTITUDE, etc.) |
| State publishing | ~L521–L590 | VehicleState + VehicleDiagnostics at 10Hz / 2Hz |
| PID controllers (depth + yaw) | ~L788–L862 | Two separate control loops embedded in RC timer |
| RC override (20 Hz) | ~L756–L869 | 4-layer priority channel builder |
| Command dispatch | ~L871–L1204 | 25+ if/elif branches for DriverCommand handling |
| Arm/disarm, mode, servo | ~L1210–L1306 | Actuator & mode control helpers |

### Why It's a Problem

- **Single Responsibility Principle violation.** Changing PID tuning, adding a
  MAVLink message handler, or fixing a connection bug all touch the same
  1300-line file. Merge conflicts are frequent in a team environment.
- **Untestable in isolation.** Testing the PID controller requires instantiating
  the entire node with a MAVLink connection.
- **Cognitive load.** New team members need to understand the entire file to
  modify any part of it. The `_send_rc_override()` method alone is 113 lines
  with 4 nested layers of channel manipulation.
- **Tight coupling.** PID state, connection state, movement state, and command
  parsing share instance variables freely. No clear boundaries.

### Ideal Refactoring

Break into focused modules — the node becomes a thin orchestrator:

```
inspector_node.py          # ~150 lines — wires everything, owns timers
├── connection_manager.py  # serial connect, reconnect, heartbeat
├── telemetry_parser.py    # MAVLink msg → state fields
├── command_handler.py     # DriverCommand dispatch table
├── pid_controller.py      # reusable PID class (depth, yaw, future heading)
└── rc_override.py         # 4-layer channel builder + sender
```

Each module is independently testable. PID controller becomes a generic class
reusable for any control channel.

### Why We Haven't Done It Yet

The single-file architecture works reliably in competition. Refactoring carries
risk of introducing timing bugs in the RC override pipeline. It should be done
in the off-season with thorough integration testing.

---

## Issue 2 — No Unit Tests Exist

**Severity:** HIGH  |  **Effort to fix:** MEDIUM (pure-function tests: LOW)

### What It Is

Zero test files in the entire workspace. No `pytest.ini`, `conftest.py`, or
`test_*.py` anywhere. Every code change can only be validated by deploying on
hardware — or hoping for the best.

### Trivially Testable Functions

These are pure functions with zero hardware dependencies:

| Function | File | Lines | What to Test |
|---|---|---|---|
| `percent_to_pwm()` | inspector_node.py | ~L57 | Boundary: 0→1500, 100→1900, -100→1100 |
| `_build_diagonal_channels()` | inspector_node.py | ~L63 | Forward+right → √2 scaling, conflicting axes → None |
| `_angle_error()` | inspector_node.py | ~L748 | Wraparound: 350→10 = +20, 10→350 = -20 |
| `resolve_relative_yaw()` | driver_client.py | ~L93 | Relative from 350° + 30° = 20° |
| `_parse_file_command()` | mission_executor.py | ~L293 | "forward 5 50" → (forward, dur=5, speed=50) |
| `percent_to_pwm()` (runner) | runner.py | ~L42 | Same function, duplicated in runner |

### What Should Exist

1. **`test_math.py`** — Pure-function tests for `percent_to_pwm`,
   `_angle_error`, `_build_diagonal_channels`, `resolve_relative_yaw`.
2. **`test_pid.py`** — PID controller convergence tests (step response,
   derivative kick immunity, integral windup limits).
3. **`test_command_parse.py`** — Mission file parsing, runner input parsing.
4. **`test_teleop.py`** — Twist → DriverCommand encoding, dead-zone behavior.
5. **Integration tests** — Mocked MAVLink connection (`unittest.mock.patch` on
   `mavutil.mavlink_connection`) testing arm/disarm sequences, mode switching.

### Effort

Pure-function tests (items 1, 3) are ~2 hours of work and catch the most
common regressions (angle math bugs, PWM computation errors). PID and
integration tests require more infrastructure.

---

## Issue 3 — Hardcoded Timing Constants Scattered Throughout

**Severity:** MEDIUM  |  **Effort to fix:** LOW

### What It Is

Timing values are buried as literals across multiple files. No central
configuration. Changing pool behavior requires editing source code.

| Constant | File | Value | ROS Param? |
|---|---|---|---|
| Heartbeat timeout | inspector_node.py | `3.0` s | No |
| Reconnect backoff initial | inspector_node.py | `2.0` s | No |
| Reconnect backoff max | inspector_node.py | `15.0` s | No |
| MAVLink read loop rate | inspector_node.py | `0.02` s (50 Hz) | No |
| State publish rate | inspector_node.py | `0.1` s (10 Hz) | No |
| RC override rate | inspector_node.py | `0.05` s (20 Hz) | No |
| Diagnostics rate | inspector_node.py | `0.5` s (2 Hz) | No |
| Depth target resend | inspector_node.py | `0.5` s (2 Hz) | No |
| ACK timeout | inspector_node.py | `3.0` s | No |
| Surface throttle duration | inspector_node.py | `10.0` s | No |
| Teleop dead-zone | teleop_driver.py | `0.1` | No |
| Mission startup delay | mission_executor.py | `3.0` s | No |
| State log interval | logger_node.py | `1.0` s | No |
| Depth PID gains | inspector_node.py | Kp/Ki/Kd | **Yes** ✓ |
| Yaw PID gains | inspector_node.py | Kp/Ki/Kd | **Yes** ✓ |
| Connection port | inspector_node.py | `/dev/ttyACM0` | **Yes** ✓ |

### Ideal Solution

1. Declare ALL timing values as ROS parameters (like the PID gains already are).
2. Create `config/defaults.yaml` for standard values.
3. Create `config/competition.yaml` and `config/pool_test.yaml` overrides.
4. Load via ROS2 launch files: `--params-file config/pool_test.yaml`.

### Why It Matters

Pool conditions differ from competition (water density, tether length, current).
Currently, tuning requires code edits, rebuild, and redeploy. With YAML configs,
the team can swap profiles on launch day without touching code.

---

## Issue 4 — if/elif Command Dispatch Chain (25+ branches)

**Severity:** MEDIUM  |  **Effort to fix:** MEDIUM

### What It Is

`_on_driver_command()` (~L871–L1204) is a 330-line method with 25+ `if/elif`
branches for routing commands:

```python
if c == 'stop':
    self._stop_all()
elif c in ('move_forward', 'forward'):
    set_movement({CH_FORWARD: speed, ...}, 'Moving forward')
elif c in ('move_back', 'back', 'backward'):
    set_movement({CH_FORWARD: NEUTRAL_PWM - ...}, 'Moving backward')
# ... 22 more branches ...
elif c == 'teleop':
    clamp = lambda v: ...
# ... etc
else:
    self.get_logger().warn(f'Unknown command: {c}')
```

The same pattern exists in `mission_executor.py` `_parse_file_command()`
(~L293–L437, ~140 lines).

### Why It's a Problem

- **O(n) lookup** — Python evaluates each branch sequentially.
- **Hard to extend** — Adding a command requires finding the right insertion
  point in a 330-line method. Can accidentally break adjacent branches.
- **No discovery** — No programmatic way to list available commands, validate
  inputs, or generate help text.
- **Untestable** — Can't test one command handler without the entire method.

### Ideal Solution

Dispatch table with handler registration:

```python
_DISPATCH = {
    'stop':         _cmd_stop,
    'move_forward': _cmd_move_direction,
    'forward':      _cmd_move_direction,  # alias
    'teleop':       _cmd_teleop,
    # ...
}

def _on_driver_command(self, cmd):
    handler = self._DISPATCH.get(cmd.command.strip().lower())
    if handler:
        handler(self, cmd, speed, end_time)
    else:
        self.get_logger().warn(f'Unknown command: {cmd.command}')
```

Each handler becomes a focused method. Commands are discoverable via
`self._DISPATCH.keys()`. Aliases are explicit dictionary entries.

---

## Issue 5 — if/elif MAVLink Message Parsing (same pattern)

**Severity:** MEDIUM  |  **Effort to fix:** LOW

### What It Is

`_process_message()` (~L396–L520) handles 8 MAVLink message types with an
if/elif chain. The COMMAND_ACK handler alone is ~55 lines nested inside.

### Ideal Solution

Same dispatch-table pattern as Issue 4:

```python
_MSG_HANDLERS = {
    'HEARTBEAT':         _handle_heartbeat,
    'AHRS2':             _handle_ahrs2,
    'ATTITUDE':          _handle_attitude,
    'SYS_STATUS':        _handle_sys_status,
    'SCALED_PRESSURE':   _handle_scaled_pressure,
    'SERVO_OUTPUT_RAW':  _handle_servo_output,
    'RC_CHANNELS':       _handle_rc_channels,
    'COMMAND_ACK':       _handle_command_ack,
}
```

Each handler becomes a testable method. New message types (GPS_RAW_INT,
STATUSTEXT, NAMED_VALUE_FLOAT) are added with zero risk to existing handlers.

---

## Issue 6 — Continuous Neutral RC Override When Idle

**Severity:** LOW  |  **Effort to fix:** LOW

### What It Is

When the AUV is idle (no movement, no PID, no heading hold), the RC override
timer still fires at 20 Hz, sending all-neutral PWM values (1500 on 6
channels). That's ~46 bytes × 20 Hz = 920 bytes/sec of pure neutral noise on
the serial link.

### Why It Matters (Slightly)

- **Serial bandwidth:** On a 115200-baud link, 920 B/s is ~8% of raw capacity.
  Not catastrophic, but it crowds out telemetry when bandwidth is tight
  (competition tether scenarios, wireless links).
- **Log noise:** Every neutral RC message shows up in MAVLink logs, making
  actual movement events harder to find.
- **Pixhawk CPU:** Processing 20 messages/second of pure neutral values is
  unnecessary work for the flight controller.

### Why It's Currently Acceptable

ArduSub's failsafe triggers after ~3 seconds of RC silence when armed. The
continuous neutral stream prevents this. The fix requires careful state tracking
to start/stop RC sending without triggering failsafe — slightly tricky to get
right, low reward.

### Ideal Solution

Three-state RC mode:

```
IDLE    → don't send RC at all (disarmed, or armed + cooldown expired)
ACTIVE  → send at 20 Hz (movement, PID, or heading hold active)
STOPPING → send neutral at 20 Hz for 1 second after last movement, then → IDLE
```

When armed and in IDLE, resume ACTIVE on any new command. The 1-second cooldown
ensures ArduSub sees a clean stop before we go silent.

---

## Issue 7 — DriverCommand Message Field Overloading (Teleop)

**Severity:** MEDIUM  |  **Effort to fix:** MEDIUM

### What It Is

The `teleop` command repurposes established `DriverCommand.msg` fields for
entirely different semantics:

| Field | Normal Meaning | Teleop Meaning |
|---|---|---|
| `speed` (int32) | PWM offset or percentage | Forward/back PWM offset |
| `duration` (float32) | Duration in seconds | Lateral (left/right) PWM offset |
| `depth` (float32) | Target depth in meters | Throttle (up/down) PWM offset |
| `angle` (float32) | Target heading in degrees | Yaw PWM offset |

This was a pragmatic fix for BUG3 (teleop multi-axis broken) — rather than
modifying the `DriverCommand.msg` interface (which requires rebuilding all
packages), we reused existing fields with documented conventions.

### Why It's a Problem

- **Logger confusion:** The logger records `depth=150.0` for a teleop command
  that actually means "throttle up +150 PWM", not "dive to 150m".
- **Semantic opacity:** Any node subscribing to `/driver/command` must know the
  special teleop encoding. Self-describing message semantics are violated.
- **Future collisions:** If `DriverCommand` gains new commands that need
  `depth` to mean actual depth, the teleop overloading creates ambiguity.

### Ideal Solution (When Interface Rebuild Is Acceptable)

**Option A:** Add explicit fields to `DriverCommand.msg`:

```
float32 forward_offset   # PWM offset for forward/back (teleop only)
float32 lateral_offset   # PWM offset for lateral (teleop only)
float32 throttle_offset  # PWM offset for throttle (teleop only)
float32 yaw_offset       # PWM offset for yaw (teleop only)
```

**Option B:** Create a separate `TeleopCommand.msg`:

```
# TeleopCommand.msg
float32 forward_pwm    # +forward, -back
float32 lateral_pwm    # +right, -left
float32 throttle_pwm   # +up, -down
float32 yaw_pwm        # +CCW, -CW
```

Inspector subscribes to both `/driver/command` and `/driver/teleop`. Each
message type has unambiguous field semantics.

### Current Mitigation

The overloading is well-documented in comments at the `teleop` handler
(inspector_node.py ~L1172) and in teleop_driver.py's module docstring. The
logger would need a conditional format for `command=='teleop'` lines if
accurate log interpretation is needed.

---

## Summary Table

| # | Issue | Severity | Effort | Priority |
|---|---|---|---|---|
| 1 | God Object (inspector 1300+ lines) | **HIGH** | High | Off-season refactor |
| 2 | No unit tests | **HIGH** | Medium | **Immediate** — start with pure functions |
| 3 | Hardcoded timing constants | Medium | Low | Next sprint |
| 4 | if/elif command dispatch | Medium | Medium | With God Object refactor |
| 5 | if/elif message parsing | Medium | Low | With God Object refactor |
| 6 | Continuous neutral RC override | Low | Low | Nice-to-have |
| 7 | DriverCommand field overloading | Medium | Medium | Next interface version |

### Recommended Approach

1. **Unit tests first** (Issue 2) — zero risk, catch regressions immediately.
   Start with `percent_to_pwm`, `_angle_error`, `_build_diagonal_channels`.
2. **Parameterise timing** (Issue 3) — low effort, high payoff for pool testing.
3. **God Object decomposition** (Issue 1) + dispatch tables (Issues 4, 5) —
   do together in the off-season as a major refactor sprint.
4. **Interface cleanup** (Issue 7) — when next breaking change to
   `duburi_interfaces` is needed anyway.
5. **RC idle optimization** (Issue 6) — bonus, do after integration tests exist.
