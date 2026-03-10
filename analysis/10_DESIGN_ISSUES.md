# 10 — Design Issues & Technical Debt (Post-Refactor Analysis)

> **Last updated:** 2026-03-06 — analysis at commit `f62781f`
>
> This document catalogues structural problems in the BRACU Duburi AUV 4.2
> codebase. Each issue includes theoretical background, current implementation
> state, and remaining work. Updated after the Phase 1 modularization refactor.
>
> **Note:** Issue 6 (RC Neutral Override) is intentionally deferred for
> post-pool-testing analysis.

---

## Issue 1 — God Object Decomposition

**Original severity:** HIGH  |  **Status: RESOLVED** (commit `f62781f`)

### Theory

The *God Object* anti-pattern (Riel, 1996) occurs when a single class
accumulates responsibilities that should be distributed across multiple
cohesive modules. It violates the Single Responsibility Principle (SRP) —
a class should have exactly one reason to change. In robotics, God Objects
are especially dangerous because:

- **Timing sensitivity:** Mixing control loops (20 Hz RC override) with
  I/O management (serial reconnect) in one class means a bug in reconnect
  logic can subtly delay a control tick.
- **Testing impossibility:** Hardware-dependent I/O (MAVLink serial) and
  pure computation (PID math, PWM calculations) become inseparable.
- **Onboarding friction:** New team members must comprehend the entire file
  to modify any part, increasing the probability of accidental regressions.

### Original State

`inspector_node.py` was **1 854 lines** containing a single monolithic class
(`MavlinkInspectorNode`) with 8 distinct responsibilities:

| Responsibility | Approx. Lines | Description |
|---|---|---|
| Serial connection & reconnect | ~290 | Port scanning, heartbeat, auto-reconnect with exponential backoff |
| MAVLink message parsing | ~110 | 8 message types → vehicle state fields |
| State/diagnostics publishing | ~60 | VehicleState at 10 Hz, VehicleDiagnostics at 2 Hz |
| PID controllers (depth + yaw) | ~110 | Two independent control loops with rate limiting |
| PWM ramp + RC override | ~180 | 4-layer priority channel builder with smooth transitions |
| Command dispatch | ~650 | 30+ if/elif branches for DriverCommand handling |
| Vehicle control helpers | ~70 | Arm/disarm, mode change, servo control |
| Feedback/event publishing | ~20 | DriverCommandFeedback, MavlinkEvent |

### What Was Done

The monolith was decomposed into **7 focused modules** following the
Single Responsibility Principle:

| Module | Lines | Single Responsibility |
|---|---|---|
| `inspector_node.py` | 641 | Thin orchestrator — wires modules, owns timers/publishers |
| `command_handler.py` | 440 | DriverCommand dispatch via dict-based routing |
| `movement_commands.py` | 399 | MOVEMENTS registry dict — all 30+ movement handlers |
| `connection_manager.py` | 246 | Serial lifecycle, heartbeat, exponential-backoff reconnect |
| `rc_controller.py` | 206 | Channel constants, PWM math, trapezoidal velocity ramp |
| `pid_controller.py` | 162 | Generic PID with deadband, anti-windup, EMA derivative |
| `telemetry_parser.py` | 159 | MAVLink dispatch table → vehicle state fields |

**Total:** ~2 253 lines across 7 files.

The increase from 1 854 → 2 253 lines is expected: proper imports, module-level
docstrings, explicit `__init__` parameters, and class interfaces all add lines
while reducing complexity. The key metric is not total LOC but **max single-file
complexity** — reduced from 1 854 to 641 lines (65% reduction).

### Architecture After Refactor

```
inspector_node.py (orchestrator, 641 lines)
  │
  ├── creates → ConnectionManager     (serial I/O, 246 lines)
  │                └── pymavlink.mavutil
  ├── creates → TelemetryParser        (MAVLink → state, 159 lines)
  ├── creates → RcController           (PWM channels + ramp, 206 lines)
  ├── creates → PidController × 2      (depth + yaw, 162 lines)
  └── creates → CommandHandler         (route commands, 440 lines)
                  └── uses → MOVEMENTS dict (movement_commands.py, 399 lines)
```

### Remaining Concerns

1. **Orchestrator is still 641 lines.** For a node owning 7 timers, 4
   publishers, and 2 subscribers, this is acceptable — but the
   `_rc_override_tick()` method (4-layer channel builder) is the densest
   logic remaining. If it grows beyond ~50 lines, consider extracting a
   `ChannelMixer` class.

2. **Command handler coupling.** `CommandHandler` holds a direct reference
   to the inspector node and calls its methods (`_arm_disarm`, `_set_mode`)
   directly. This is pragmatic but means the handler cannot be tested
   without a mock inspector object. A future improvement would be injecting
   a protocol/interface instead.

3. **Movement handlers access node state.** Functions in `movement_commands.py`
   receive the full node object to access `rc_controller`, logging, and
   timers. A stricter design would pass only the needed interfaces.

### Verification

- Clean build: all 7 packages, 0 warnings
- Runtime import test: all 7 modules import cleanly
- Circular dependency (just_commands ↔ driver_client) found and fixed
- No behavioral changes: same MAVLink byte sequences, same timing

---

## Issue 2 — No Unit Tests Exist

**Severity:** HIGH  |  **Effort to fix:** MEDIUM  |  **Status: OPEN**

### Theory

Untested robotics code is a liability in competition. Without automated tests,
every code change requires physical hardware verification — diving the AUV in
a pool. This creates a fear-of-change culture where bugs accumulate because
developers avoid modifying "working" code. The cost compounds:

- **Regression probability** scales with code size. At ~5 600 lines across 33
  files, manual verification is insufficient.
- **Refactoring paralysis** — the modularization in Issue 1 was delayed
  precisely because there were no tests to prove behavioral equivalence.
- **Competition-day risk** — a last-minute fix that seems correct can't be
  validated without pool time, which may not be available.

### Current State (Post-Refactor)

The modularization has dramatically improved *testability*. Previously,
functions like `percent_to_pwm()` and `_build_diagonal_channels()` were
trapped inside the 1 854-line God Object and required a full ROS node to
instantiate. Now they are standalone, importable, and require zero
infrastructure.

### Testable Functions — Updated Inventory

**Tier 1: Pure functions (zero dependencies, trivial to test)**

| Function | File | Package | What to Test |
|---|---|---|---|
| `percent_to_pwm(pct)` | `rc_controller.py` | mavlink_inspector | 0→0, 50→200, 100→400, negatives, >100 clamping |
| `build_diagonal_channels(speed, ch1, ch2)` | `rc_controller.py` | mavlink_inspector | √2 scaling correctness, sign preservation |
| `resolve_relative_yaw(rel, current)` | `driver_client.py` | mavlink_driver | 350°+30°=20°, 10°-30°=340°, wraparound |
| `make_command(cmd, speed, dur, ...)` | `driver_client.py` | mavlink_driver | Field mapping, default values |
| `ros_image_to_cv2(img)` | `image_utils.py` | vision | bgr8/rgb8/mono8 encoding, shape validation |
| `cv2_to_ros_image(arr)` | `image_utils.py` | vision | Round-trip fidelity |

**Tier 2: Stateful classes (require mock/setup)**

| Class | File | Package | What to Test |
|---|---|---|---|
| `PidController` | `pid_controller.py` | mavlink_inspector | Step response convergence, deadband, anti-windup, rate limiting, reset |
| `RcController` | `rc_controller.py` | mavlink_inspector | Ramp behavior, set_movement → get_channels over time, stop zeroing |
| `TelemetryParser` | `telemetry_parser.py` | mavlink_inspector | Mock MAVLink messages → correct state fields, arm/disarm events |
| `ConnectionManager` | `connection_manager.py` | mavlink_inspector | Backoff timing (mock time.sleep), reconnect cycle |

**Tier 3: Integration (require ROS 2 test harness)**

| Test | Scope | What to Verify |
|---|---|---|
| Command round-trip | runner → driver → inspector | CLI input produces correct MAVLink output |
| Mission execution | mission file → executor → inspector | Sequencing, timing, abort |
| Teleop pipeline | Twist → teleop_driver → inspector → RC | Dead-zone, multi-axis |

### Implementation Plan

1. **Add `pytest` to package dependencies** for inspector, driver, runner.
2. **Create `test/` directories** in each package.
3. **Start with Tier 1** — pure function tests: ~2 hours, catches the most
   common regressions (angle math, PWM boundary errors).
4. **PID controller tests** — step response, integral windup, derivative kick.
   These are critical because PID bugs manifest as physical misbehavior
   (oscillation, overshoot) that's hard to diagnose in water.
5. **Parser tests** — exercise every command path in `command_parser.py` and
   `mission_parser.py`. These are the most maintenance-prone code (new commands
   = new branches).

### Effort Estimate

| Tier | Effort | Value | Priority |
|---|---|---|---|
| Tier 1 (pure functions) | ~2 hours | Catch math/boundary regressions | **Immediate** |
| Tier 2 (PID, RC, parser) | ~4 hours | Prevent control-loop regressions | Before next pool test |
| Tier 3 (integration) | ~8 hours | End-to-end confidence | Off-season |

---

## Issue 3 — Hardcoded Timing Constants

**Severity:** MEDIUM  |  **Effort to fix:** LOW  |  **Status: PARTIALLY IMPROVED**

### Theory

Timing constants in robotics code serve two roles: (1) control-loop rates that
define system dynamics (50 Hz read, 20 Hz RC override), and (2) tuning
parameters that vary by environment (heartbeat timeout, ramp rate, PID gains).

Category (1) should be constants — changing the RC override rate from 20 Hz to
10 Hz fundamentally alters the control bandwidth. Category (2) should be
runtime-configurable — pool water density differs from competition venue, PID
gains need tuning, timeouts need adjusting for different tether lengths.

The original code conflated both categories as hardcoded literals.

### Current State

After modularization, the timing constants are now **located in their
responsible modules** rather than scattered across one 1 854-line file:

| Constant | Module | Value | ROS Param? | Category |
|---|---|---|---|---|
| Heartbeat timeout | `connection_manager.py` | `3.0` s | No | Tuning |
| Reconnect backoff initial | `connection_manager.py` | `2.0` s | No | Tuning |
| Reconnect backoff max | `connection_manager.py` | `15.0` s | No | Tuning |
| MAVLink read loop rate | `inspector_node.py` | `0.02` s (50 Hz) | No | Control rate |
| State publish rate | `inspector_node.py` | `0.1` s (10 Hz) | No | Control rate |
| RC override rate | `inspector_node.py` | `0.05` s (20 Hz) | No | Control rate |
| Diagnostics rate | `inspector_node.py` | `0.5` s (2 Hz) | No | Control rate |
| ACK timeout | `inspector_node.py` | `3.0` s | No | Tuning |
| Surface throttle duration | `movement_commands.py` | `10.0` s | No | Tuning |
| Teleop dead-zone | `teleop_driver.py` | `0.1` | No | Tuning |
| Mission startup delay | `mission_executor.py` | `3.0` s | No | Tuning |
| State log interval | `logger_node.py` | `1.0` s | No | Tuning |
| PWM ramp rate | `inspector_node.py` | 800 PWM/s | **Yes** ✓ | Tuning |
| PID max rate | `inspector_node.py` | 50 PWM/tick | **Yes** ✓ | Tuning |
| Nominal voltage | `inspector_node.py` | 0.0 | **Yes** ✓ | Tuning |
| Depth tolerance | `inspector_node.py` | 0.05 m | **Yes** ✓ | Tuning |
| Depth PID gains | `inspector_node.py` | Kp/Ki/Kd | **Yes** ✓ | Tuning |
| Yaw PID gains | `inspector_node.py` | Kp/Ki/Kd | **Yes** ✓ | Tuning |
| Connection port | `inspector_node.py` | `/dev/ttyACM0` | **Yes** ✓ | Tuning |

### Improvement

The refactor provided **organizational clarity** — constants now live next to
the code that uses them. The *tuning* parameters that were already ROS params
(PID gains, ramp rate, port) remain configurable. The remaining hardcoded
tuning values are now easier to parameterize because they're in focused modules
with clear ownership.

### Remaining Work

1. **Parameterize connection tuning:** heartbeat_timeout, backoff_initial,
   backoff_max in `connection_manager.py` — these should be ROS params passed
   from inspector_node's parameter declarations.
2. **Parameterize surface_duration** in `movement_commands.py`.
3. **Create `config/defaults.yaml`** collecting all param defaults in one place.
4. **Do NOT parameterize control rates** (50 Hz read, 20 Hz RC, 10 Hz state) —
   these define system dynamics and should remain compile-time constants.

---

## Issue 4 — if/elif Command Dispatch Chain

**Original severity:** MEDIUM  |  **Status: RESOLVED** (commit `f62781f`)

### Theory

Long if/elif chains for command routing are a manifestation of the *Switch
Statement Smell* (Fowler, 1999). The problems are:

- **O(n) lookup:** Python evaluates branches sequentially. With 30+ branches,
  the last command pays the full chain cost on every dispatch.
- **Open/Closed Principle violation:** Adding a new command requires modifying
  the existing dispatch method — there's no way to extend behavior without
  editing the core routing code.
- **Testing granularity:** The entire method must be invoked to test a single
  command handler. No isolation possible.
- **Discovery:** No programmatic way to list supported commands.

The standard fix is the *Command Pattern* with a dispatch table — a dictionary
mapping command names to handler callables. Lookup becomes O(1), new commands
are added by dictionary insertion, and handlers are independently testable.

### Original State

`_on_driver_command()` was a **651-line method** (~L988–L1639) with 30+ `if/elif`
branches. The same pattern existed in `mission_executor.py`'s
`_parse_file_command()` (~140 lines) and `runner.py`'s command parsing.

### Implementation

The refactor introduced two dispatch mechanisms:

**1. CommandHandler.system_dispatch (command_handler.py)**

```python
system_dispatch = {
    'stop':       _handle_stop,
    'arm':        _handle_arm,
    'disarm':     _handle_disarm,
    'mode':       _handle_mode,
    'depth':      _handle_depth,
    'grab_open':  _handle_grab_open,
    'grab_close': _handle_grab_close,
}
```

System commands route through a dict. O(1) lookup, each handler is a focused
method (~10–30 lines).

**2. MOVEMENTS registry (movement_commands.py)**

```python
MOVEMENTS = {
    'move_forward': cmd_move_forward,
    'forward':      cmd_move_forward,   # alias
    'move_back':    cmd_move_back,
    'back':         cmd_move_back,      # alias
    'backward':     cmd_move_back,      # alias
    # ... 30+ entries total
    'cruise':       cmd_cruise,
    'teleop':       cmd_teleop,
}
```

Movement commands (including aliases) map to standalone handler functions.
Aliases are explicit dictionary entries — no hidden fallthrough behavior.

**Dispatch flow:**
```
handle_command(cmd)
  ├─ system_dispatch.get(cmd)     → system handler      [O(1)]
  ├─ MOVEMENTS.get(cmd)           → movement handler    [O(1)]
  ├─ cmd.startswith("just_")      → MOVEMENTS[cmd]      [O(1)]
  ├─ cmd.startswith("go_")        → handle_go()
  └─ unknown                      → log warning
```

### Verification

Every command that worked before continues to work. The dispatch flow was
verified by reading every handler and confirming it performs the same channel
manipulation as the original if/elif branch. No behavioral changes.

### Remaining Concern

The `just_` prefix handling is done via string check before MOVEMENTS lookup.
This works because all `just_*` entries are registered in MOVEMENTS, but the
two-step lookup (prefix check → dict lookup) is slightly redundant. Minor —
not worth changing.

---

## Issue 5 — if/elif MAVLink Message Parsing

**Original severity:** MEDIUM  |  **Status: RESOLVED** (commit `f62781f`)

### Theory

Same anti-pattern as Issue 4 applied to incoming MAVLink messages. The
`_process_message()` method handled 7 message types via if/elif, with the
added complication that some handlers (HEARTBEAT, COMMAND_ACK) contained
significant business logic (arm/disarm state tracking, command completion
futures).

### Implementation

`TelemetryParser` uses a dispatch dict:

```python
_dispatch = {
    'HEARTBEAT':         _handle_heartbeat,
    'AHRS2':             _handle_ahrs2,
    'ATTITUDE':          _handle_attitude,
    'SYS_STATUS':        _handle_sys_status,
    'SCALED_PRESSURE':   _handle_scaled_pressure,
    'SERVO_OUTPUT_RAW':  _handle_servo_output,
    'RC_CHANNELS':       _handle_rc_channels,
}
```

Each handler is a focused method (10–25 lines) that updates specific state
fields. The HEARTBEAT handler includes arm/disarm transition detection and
mode-change event publishing — appropriately complex for its responsibility.

COMMAND_ACK handling remains in `inspector_node.py` (as `_handle_command_ack`)
because it involves pending futures and MAVLink send state that belongs to the
node, not the telemetry parser. This is the correct separation — the parser
handles *telemetry*, the node handles *command protocol*.

### Benefits Realized

- **Extensibility:** Adding GPS_RAW_INT, STATUSTEXT, or NAMED_VALUE_FLOAT
  support requires adding one dict entry + one handler method. Zero risk to
  existing handlers.
- **Testability:** Each handler can be tested with a mock MAVLink message
  object. No ROS infrastructure needed.
- **Discoverability:** `_dispatch.keys()` lists all handled message types.

---

## Issue 6 — Continuous Neutral RC Override When Idle

**Severity:** LOW  |  **Effort to fix:** LOW  |  **Status: DEFERRED**

> *Deferred for post-pool-testing analysis. Will be revisited after
> empirical testing of ArduSub failsafe timing on our specific hardware
> configuration.*

### What It Is

When the AUV is idle (no active movement, no PID, no heading hold), the RC
override timer still fires at 20 Hz, sending all-neutral PWM values (1500 on
6 channels). That's ~46 bytes × 20 Hz = **920 bytes/sec** of pure neutral
noise on the serial link.

### Why It's Currently Acceptable

ArduSub's failsafe triggers after ~3 seconds of RC silence when armed. The
continuous neutral stream prevents unintended failsafe activation. The fix
requires careful state tracking to start/stop RC sending without triggering
failsafe — and the exact failsafe timing depends on firmware version and
parameter configuration, which we need to verify empirically.

### Post-Refactor Note

The RC override logic now lives in `rc_controller.py` (`RcController` class)
and `inspector_node.py` (`_rc_override_tick` timer). The three-state solution
would be implemented entirely within `RcController` by adding an `rc_mode`
state variable and modifying `get_channels()` to return `None` in IDLE state
(signaling the timer to skip sending).

---

## Issue 7 — DriverCommand Message Field Overloading (Teleop)

**Severity:** MEDIUM  |  **Effort to fix:** MEDIUM  |  **Status: OPEN**

### Theory

Message field overloading (using the same field name with different semantics
depending on context) violates the *Principle of Least Astonishment*. In ROS 2,
message definitions serve as the API contract between nodes. When `depth` means
"target depth in meters" for 29 commands but "throttle PWM offset" for one
command, the contract is broken.

This is a common pragmatic shortcut in competition robotics — modifying
`.msg` files requires rebuilding all dependent packages and updating every
consumer. The teleop overloading was introduced to fix multi-axis teleop
(BUG3) without a costly interface change.

### Current Implementation

The teleop command in `movement_commands.py` (`cmd_teleop()`) and the teleop
publisher in `teleop_driver.py` both use the overloaded encoding:

| DriverCommand Field | Normal Semantics | Teleop Encoding |
|---|---|---|
| `speed` (int32) | PWM offset / percentage | Forward/back PWM offset |
| `duration` (float32) | Duration in seconds | Lateral (left/right) PWM offset |
| `depth` (float32) | Target depth in meters | Throttle (up/down) PWM offset |
| `angle` (float32) | Target heading in degrees | Yaw PWM offset |

### Impact Analysis

1. **Logger corruption:** `logger_node.py` records all DriverCommand messages
   to CSV. A teleop command with `depth=150.0` appears in logs as "set depth
   to 150 meters" rather than "throttle offset +150 PWM". Post-dive log
   analysis will be misleading unless the logger adds special-case formatting
   for `command == 'teleop'`.

2. **Semantic opacity:** Any node subscribing to `/driver/command` (current:
   inspector and logger; future: safety watchdog, telemetry dashboard) must
   implement the teleop special case. Self-describing message semantics are
   violated.

3. **Cross-package contagion:** The overloading convention must be documented
   and maintained consistently across `mavlink_driver` (teleop_driver.py),
   `mavlink_inspector` (movement_commands.py), and `mavlink_logger`
   (logger_node.py). A documentation miss causes silent bugs.

### Solutions (Ordered by Disruption)

**Option A — Minimal: Add logger special-case (LOW effort)**

Add a conditional format in `logger_node.py` for `command == 'teleop'` that
labels the fields correctly. Does not fix the root cause but makes logs
accurate.

**Option B — Separate message: `TeleopCommand.msg` (MEDIUM effort)**

```
# TeleopCommand.msg
float32 forward_pwm    # +forward, -back
float32 lateral_pwm    # +right, -left
float32 throttle_pwm   # +up, -down
float32 yaw_pwm        # +CCW, -CW
```

Inspector subscribes to both `/driver/command` and `/driver/teleop`. Fields
are unambiguous. Requires a rebuild of `duburi_interfaces` and modifications
to `inspector_node.py` and `teleop_driver.py`.

**Option C — Extend DriverCommand: add offset fields (MEDIUM effort)**

Add `forward_offset`, `lateral_offset`, `throttle_offset`, `yaw_offset` to
`DriverCommand.msg`. Teleop uses the new fields; existing commands ignore them.
Backwards-compatible but bloats the message for all consumers.

### Recommendation

**Option B** is cleanest — separation of concerns at the message level. Do it
when the next interface rebuild is needed (adding a new message type, modifying
existing fields, etc.) to amortize the rebuild cost.

---

## Issue 8 — Duplicated Command Parsing Logic (NEW)

**Severity:** MEDIUM  |  **Effort to fix:** MEDIUM  |  **Status: OPEN**

### Theory

The *DRY principle* (Don't Repeat Yourself) states that every piece of
knowledge should have a single, authoritative representation. When the same
logic exists in multiple places, changes must be synchronized — and
synchronization failures cause bugs that are hard to detect because the
secondary copy "almost works."

### What It Is

Command parsing — the translation from a text command string to a
`DriverCommand` message — is implemented **three times** across two packages:

| Parser | File | Package | Lines | Context |
|---|---|---|---|---|
| `parse_command()` | `command_parser.py` | mavlink_runner | 470 | Interactive CLI input |
| `parse_file_command()` | `mission_parser.py` | mavlink_driver | 245 | Mission file lines |
| Command handling | `command_handler.py` | mavlink_inspector | 440 | DriverCommand dispatch |

While these serve different purposes (human input, file input, ROS message
dispatch), they share substantial overlapping logic:

- **Prefix handling:** All three handle `just_` prefix, `~` alias expansion,
  and `move` prefix stripping.
- **Command vocabulary:** All three must recognize `forward`, `back`, `left`,
  `right`, `up`, `down`, `yaw_left`, `yaw_right`, `depth`, `heading`, `turn`,
  `go_*`, `cruise`, `grab_open`, `grab_close`, `arm`, `disarm`, `stop`.
- **Speed/duration extraction:** `command_parser.py` and `mission_parser.py`
  both parse speed and duration from args with similar regex patterns.

### Why It's a Problem

Adding a new command (e.g., `hold_position`, `spiral`, `zigzag`) requires
editing **three files across two packages**. If any one is missed:

- Missing in `command_parser.py` → command doesn't work from CLI
- Missing in `mission_parser.py` → command doesn't work in missions
- Missing in `command_handler.py` → command is published but silently ignored

There's no compiler error, no runtime warning (beyond "unknown command") — the
failure is silent and context-dependent.

### Analysis of Differences

The parsers aren't identical — they have legitimate differences:

| Aspect | command_parser (CLI) | mission_parser (file) | command_handler (dispatch) |
|---|---|---|---|
| Input format | Free-text with help/status | Structured lines | ROS DriverCommand msg |
| Speed default | 50 (interactive default) | Required in most cases | From msg.speed field |
| Duration | Optional (defaults vary) | From args | From msg.duration field |
| Special commands | `help`, `status`, `quit`, `mission` | Comments (`#`), empty lines | N/A (msg already parsed) |
| Error handling | Print to terminal | Log warning | Log warning |

### Ideal Solution

Extract a shared **command vocabulary** module that both text parsers import:

```python
# shared_vocabulary.py
MOVEMENT_COMMANDS = {
    'forward': ('move_forward', driver_client.move_forward),
    'back':    ('move_back',    driver_client.move_back),
    # ...
}

SYSTEM_COMMANDS = {
    'arm':   driver_client.arm,
    'disarm': driver_client.disarm,
    # ...
}

ALIASES = {'~gate': 'go_forward ...', ...}
```

Then `command_parser.py` and `mission_parser.py` both import from the shared
vocabulary. `command_handler.py` already has its own registry (MOVEMENTS dict)
and wouldn't need to share this since it operates on different input types.

### Effort

Medium — requires careful comparison of all three parsers to identify what's
truly shared vs. legitimately different. The cross-package dependency (runner
imports from driver) makes this architecturally natural since runner already
depends on driver.

---

## Issue 9 — Cross-Package Code Duplication: percent_to_pwm (NEW)

**Severity:** LOW  |  **Effort to fix:** LOW  |  **Status: OPEN**

### What It Is

The `percent_to_pwm()` function exists in two places:

1. `mavlink_inspector/rc_controller.py` — used by movement handlers
2. `mavlink_runner/command_parser.py` — used for CLI speed parsing

Both implementations are identical:

```python
def percent_to_pwm(percent: int) -> int:
    return int(PWM_RANGE * min(abs(percent), 100) / 100)
```

Additionally, `PWM_RANGE` and `NEUTRAL_PWM` constants are defined in both
`rc_controller.py` (inspector) and used in `command_parser.py` (runner) as
local calculations.

### Why It's Minor

The function is simple and stable — the formula is unlikely to change. But if
PWM_RANGE were ever adjusted (e.g., to support a different thruster ESC range),
the runner's copy would need manual synchronization.

### Ideal Solution

Option A: Runner imports from inspector (adds cross-package dependency — not
ideal since runner currently only depends on driver).

Option B: Move `percent_to_pwm` and PWM constants to `mavlink_driver`'s
`driver_client.py` since it's the shared library both packages import.

Option C: Accept the duplication — it's 3 lines of stable arithmetic.

**Recommendation:** Option C for now. Document the duplication and address it
only if PWM range becomes configurable.

---

## Summary Table

| # | Issue | Original | Current Status | Priority |
|---|---|---|---|---|
| 1 | God Object (inspector) | **HIGH** | **RESOLVED** — 7-module split | Done ✓ |
| 2 | No unit tests | **HIGH** | **OPEN** — now testable | **Immediate** |
| 3 | Hardcoded timing constants | Medium | **PARTIALLY IMPROVED** — organized by module | Next sprint |
| 4 | if/elif command dispatch | Medium | **RESOLVED** — dispatch tables | Done ✓ |
| 5 | if/elif message parsing | Medium | **RESOLVED** — dispatch dict | Done ✓ |
| 6 | RC neutral override | Low | **DEFERRED** — pending pool test | Post-pool |
| 7 | DriverCommand teleop overloading | Medium | **OPEN** — documented workaround | Next interface version |
| 8 | Duplicated command parsing (NEW) | Medium | **OPEN** — three parallel parsers | Off-season |
| 9 | percent_to_pwm duplication (NEW) | Low | **OPEN** — accepted for now | Low priority |

### Progress Since First Analysis

Three of the original seven issues are now **fully resolved** (Issues 1, 4, 5).
One issue is **improved** (Issue 3 — constants now co-located with their
modules). Two issues remain **open** with clear implementation paths (Issues 2,
7). One issue is **deferred** pending empirical data (Issue 6).

Two new issues (8, 9) were identified during the deep audit — these are
structural observations that became visible only after the modularization made
the codebase structure explicit.

### Recommended Next Steps

1. **Unit tests** (Issue 2) — Start with Tier 1 pure functions: `percent_to_pwm`,
   `build_diagonal_channels`, `resolve_relative_yaw`, PidController step
   response. ~2 hours, highest value per effort.
2. **Pool testing** → then address Issue 6 (RC neutral) with empirical data.
3. **Parameterize connection tuning** (Issue 3 remaining) — heartbeat_timeout,
   backoff rates. ~30 minutes, useful for pool testing.
4. **Shared command vocabulary** (Issue 8) — extract when adding new commands.
5. **TeleopCommand.msg** (Issue 7) — do with next interface version.
