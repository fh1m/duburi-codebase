# mavlink_driver – Line-by-Line Analysis

**Package:** `src/mavlink_driver/`

Contains: `driver_client.py`, `mission_parser.py`, `mission_executor.py`, `just_commands.py`, `teleop_driver.py`.

---

## driver_client.py

### make_command()

**Why:** Single factory for `DriverCommand`. Ensures all fields are set and avoids typos.

### speed Parameter

```python
speed: int = 50
```
**Why:** Default 50 = 50% gain. Inspector maps 0–100 to PWM via `percent_to_pwm`.

### Helper Functions (move_forward, etc.)

**Why:** Convenience wrappers so mission code stays readable: `move_forward(duration=5, speed=80)` instead of `make_command('move_forward', duration=5, speed=80)`.

### yaw_angle vs yaw_to_heading

**Note:** `driver_client` exposes `yaw_angle` (legacy set_attitude_target). For thruster-based yaw, use `make_command('yaw_to_heading', angle=260, speed=50)`.

---

## mission_executor.py

### Timer-Based Run

```python
self._run_mission_timer = self.create_timer(3.0, self._run_mission_once)
```
**Why:** 3 s delay lets the inspector connect before commands are sent.

```python
self._run_mission_timer.cancel()
```
**Why:** Run mission once, then stop the timer.

### _publish with time.sleep(delay)

```python
def _publish(self, cmd: DriverCommand, delay: float = 0.5):
    self._cmd_pub.publish(cmd)
    if delay > 0:
        time.sleep(delay)
```
**Why:** `time.sleep` blocks the executor. Used so movement commands have time to run before the next one. For missions with duration, sleep should match the movement duration.

### _run_pool_test Structure

**Why:** Example mission: MANUAL → arm → ALT_HOLD → set_depth → move_forward → move_left → move_right → stop. Matches typical pool test flow.

---

## mission_parser.py (added post-refactor)

> **New file.** Extracted from `mission_executor.py`'s inline `_parse_file_command()` during the Phase 2 refactoring (unified command parsing).

### parse_file_command()

```python
def parse_file_command(current_heading, cmd, args, logger) -> DriverCommand:
```
**Why:** Single function to parse a mission file line into a `DriverCommand`. Uses `duburi_common.command_vocabulary` for alias resolution and prefix handling (`just`, `~`, `move`). Returns `None` for unknown commands. This eliminated the duplicated parsing logic that previously existed in both runner and executor.

### Parsing Pipeline

```
raw line → strip → lowercase
  ├─ prefix "just " → add just_ prefix, recurse
  ├─ prefix "~"     → alias lookup via command_vocabulary, recurse
  ├─ prefix "move " → strip prefix, recurse
  ├─ movement commands → driver_client.move_forward(speed, duration) etc.
  ├─ depth/heading   → driver_client.set_depth/set_heading(value)
  ├─ turn left/right → driver_client.turn_left/right(angle, current_heading)
  ├─ go_*            → driver_client.go_*(speed, duration)
  ├─ cruise          → driver_client.cruise(speed, duration, heading)
  └─ unknown         → log warning, return None
```

**Depends on:** `driver_client` (all factory functions), `duburi_common.command_vocabulary` (`resolve_prefixes`, `HORIZONTAL_DIRS`)

---

## just_commands.py (added post-refactor)

> **New file.** Provides instant (no-ramp) variants of all movement commands via the `just_` prefix.

### Pattern

All functions call `make_command()` from `driver_client.py` with the `just_` prefix on the command string:

```python
def just_forward(duration=0, speed=DEFAULT_SPEED):
    return make_command('just_move_forward', speed=speed, duration=duration)
```

**Why:** Separating `just_*` commands into their own module keeps `driver_client.py` focused on the canonical ramped variants. `just_commands.py` provides the same API surface with instant (bypass-ramp) semantics.

**Depends on:** `driver_client.make_command`

---

## teleop_driver.py

### Twist to TeleopCommand

**Why:** Converts `/cmd_vel` (geometry_msgs/Twist) to a dedicated `TeleopCommand` message on `/driver/teleop`. This is the **updated** approach — the old version converted Twist to `DriverCommand` one-axis-at-a-time, which was limiting.

### Multi-Axis Mapping

> **Updated:** The teleop driver now supports simultaneous multi-axis control via the `TeleopCommand` message, replacing the old single-axis-at-a-time approach.

```python
# New approach: all axes mapped simultaneously
teleop_cmd = TeleopCommand()
teleop_cmd.linear_x = msg.linear.x * max_speed  # Forward/back
teleop_cmd.linear_y = msg.linear.y * max_speed  # Left/right
teleop_cmd.linear_z = msg.linear.z * max_speed  # Up/down
teleop_cmd.angular_z = msg.angular.z * max_speed # Yaw
teleop_cmd.idle = all_axes_below_deadzone
```

| Twist field | TeleopCommand field | Meaning |
|---|---|---|
| `linear.x` | `linear_x` | Forward / back |
| `linear.y` | `linear_y` | Left / right |
| `linear.z` | `linear_z` | Up / down |
| `angular.z` | `angular_z` | Yaw |

**Why:** `TeleopCommand` is a dedicated message type (see `06_INTERFACES.md`) that avoids the field overloading problem of the old approach (which repurposed `DriverCommand.speed`, `duration`, `depth`, `angle` for PWM offsets). When the joystick returns to centre, `idle=true` clears movement without disrupting active depth PID or heading hold.

### Dead-Zone

```python
_DZ = 0.1  # Dead-zone threshold (should be a ROS parameter — see 11_REFACTORING_PLAN.md item 1.7)
```
**Why:** Prevents drift from joystick noise. Axes below 0.1 magnitude are treated as zero.
