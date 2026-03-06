# Known Issues and Gotchas

## For Agents: Critical Edge Cases

### 1. KeyboardInterrupt → RCLError on Exit

**Symptom:** When user presses Ctrl+C during runner, you may see:
```
rclpy._rclpy_pybind11.RCLError: Failed to publish: publisher's context is invalid
rclpy._rclpy_pybind11.RCLError: failed to shutdown: rcl_shutdown already called
```

**Cause:** The `except KeyboardInterrupt` block calls `self._publish(DriverCommand(command='stop'))` but by then the ROS context may already be shutting down. Publishing after shutdown fails.

**Fix:** Wrap the stop publish in try/except, or check `rclpy.ok()` before publishing. Avoid publishing in exception handlers during shutdown. **Status: Fixed** — all shutdown paths now guard with try/except.

### 2. Mission File: Must Wait After Every Command With Duration

**Symptom:** Mission runs but thrusters don't move / movements overwrite each other.

**Cause:** If `_execute_chain` only waits when `i < len(parts) - 1`, single-command lines (each mission line) never wait. Commands fire back-to-back and overwrite `_current_movement` before execution.

**Fix:** Always `time.sleep(wait_sec)` when `wait_sec > 0`, regardless of chain position. **Status: Fixed.**

### 3. Arm Before Movement in Missions

**Symptom:** Mission sends move forward but vehicle doesn't move.

**Cause:** Arming takes 2–4 seconds. If we send move immediately after arm, the vehicle may not be armed yet.

**Fix:** Return `wait_sec=4.0` for arm and `wait_sec=2.0` for disarm from `_parse_one`, so the mission runner sleeps before the next command. **Status: Fixed.**

### 4. mode MANUAL Before Arm in Mission Files

**Symptom:** RC override doesn't work, thrusters don't respond.

**Cause:** ArduSub requires MANUAL mode for RC_CHANNELS_OVERRIDE to drive thrusters. Other modes (ALT_HOLD, STABILIZE) may use different control paths.

**Fix:** First line of mission should be `mode MANUAL` before `arm`.

### 5. Connection Port Parameter

**Note:** Default may be `/dev/ttyACM0` or `/dev/ttyACM1` depending on system. Override with:
```bash
ros2 run mavlink_inspector inspector --ros-args -p connection_port:=/dev/ttyACM0
```

### 6. yaw_angle vs yaw_to_heading

- **yaw_angle:** Uses `set_attitude_target` (SET_ATTITUDE_TARGET MAVLink). Works in ALT_HOLD/STABILIZE. May be ignored in MANUAL.
- **yaw_to_heading:** Uses thrusters (RC override on yaw channel) with closed-loop feedback. Works in MANUAL. Runner maps `yaw <deg>` to `yaw_to_heading`.
- **Note:** Both runner and mission executor now consistently use `yaw_to_heading` for the `yaw` command. `yaw_angle` is retained in driver_client/inspector as a legacy option but is not used by default.

### 7. speed Field Semantics

- **0–100:** Treated as percentage. Inspector converts via `percent_to_pwm()` → 1100–1900.
- **>100:** Treated as PWM offset from 1500 (e.g. 200 → 1700).
- **0:** Defaults to 50 (50% gain).

### 8. Depth Sign Convention

- User specifies positive depth (e.g. 0.5 = half meter below surface).
- ArduSub expects negative Z for below surface. Inspector does: `d = -abs(d) if d > 0 else d`.

### 9. readline Availability

- `import readline` enables history and cursor editing on Unix.
- On Windows, readline may not exist. Code uses `try/except` and exception catches include `NameError`; missing readline is non-fatal. **Status: Fixed** — `NameError` is now caught properly.

### 10. Thread Safety in Inspector

- `_current_movement` and `_yaw_to_heading` are protected by `_movement_lock` because:
  - `_on_driver_command` (callback) writes them
  - `_send_rc_override` (timer) reads them
  - Both run in different executor contexts; lock prevents races.
- **Minor PID race:** When changing heading targets mid-flight, there's a brief (~50ms) window where stale PID integral/error may be stamped into a new target dict. Self-corrects on the next RC tick.

### 11. Mission Executor Ctrl+C Behaviour

- **First Ctrl+C:** Sets `_abort` flag, publishes `stop()`, halts mission gracefully.
- **Second Ctrl+C:** Raises `SystemExit(1)` for forced exit.
- Interruptible sleep checks abort every 0.1s, so response time is <100ms.
- The `pause` command blocks on a `threading.Event`; abort unblocks it immediately.

### 12. Mission File Format Differences (Runner vs Executor)

- **Runner** (`run <mission>` in CLI): Uses regex to parse `move forward 50% 5s`, handles gain `%` and duration `Ns` anywhere in the line.
- **Mission Executor** (`ros2 run mavlink_driver mission_executor`): Uses positional args `forward 5 50` (command duration speed).
- **Both** now support: `sleep`, `wait`, `pause`, `go`, compound diagonals, `move <direction>`.
- **The `move` prefix** works in both systems. The executor recursively delegates `move forward 3 50` → `forward 3 50`.

### 13. Teleop Driver — Depth PID Interaction

- Teleop sends vertical commands (`move_up`/`move_down`) as separate DriverCommands, not as part of a compound.
- If `p_dive` is active, Layer 3 overrides CH_THROTTLE and the vertical teleop command has no effect.
- Users should disable `p_dive` before using vertical teleop axes.

### 14. Logger Path Changes

- Logger now stores logs in `<workspace>/logs/<YYYY-MM-DD_HH-MM-SS>/` (was `~/auv_logs/`).
- Session folder is created at startup. Each session has: `session.log`, `events.log`, `commands.log`, `state.csv`.
- RotatingFileHandler: 5 MB max, 3 backups per log file.
