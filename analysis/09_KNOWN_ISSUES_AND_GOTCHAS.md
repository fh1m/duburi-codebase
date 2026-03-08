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

- ~~Teleop sends vertical commands (`move_up`/`move_down`) as separate DriverCommands, not as part of a compound.~~
- **Status: Fixed** — Teleop now sends a single `teleop` command with all 4 axes combined. No more per-axis stomping.
- If `p_dive` is active, Layer 3 overrides CH_THROTTLE and the vertical teleop axis has no effect.
- Users should disable `p_dive` before using vertical teleop axes.

### 14. Logger Path Changes

- Logger now stores logs in `<workspace>/logs/<YYYY-MM-DD_HH-MM-SS>/` (was `~/auv_logs/`).
- Session folder is created at startup. Each session has: `session.log`, `events.log`, `commands.log`, `state.csv`.
- RotatingFileHandler: 5 MB max, 3 backups per log file.

---

## Bug Fixes (Audit Batch — 2026-03)

### 15. BUG1 — PID Derivative Kick (inspector_node.py)

**Problem:** Both depth and yaw PID controllers computed the derivative term as `Kd * (error - last_error) / dt`. When the setpoint changes (e.g. new target depth), the error jumps instantly, producing a massive derivative spike that sends a full-range PWM pulse. This is the textbook "derivative kick" problem.

**Fix:** Changed to derivative-on-measurement:
- Depth PID: `d_out = -Kd * (current_depth - prev_depth) / dt` — responds only to actual depth changes, not setpoint jumps.
- Yaw PID: `d_out = -Kd * heading_rate` — uses the gyro-measured yaw rate from ATTITUDE messages directly, which is smoother than differencing discrete samples.

**Files:** `inspector_node.py` (depth PID ~L770, yaw PID ~L810)

### 16. BUG2 — Yaw PID Speed Parameter Ignored (inspector_node.py)

**Problem:** `pid_yaw_to_heading` accepted a `speed` parameter and stored it as `gain_offset`, but the PID loop clamped output to `±PWM_RANGE` (400) regardless. A user requesting `~yaw 90 30%` got the same max thrust as `~yaw 90 100%`.

**Fix:** PID output is now clamped to `±gain_offset` instead of `±PWM_RANGE`. Lower speed = gentler maximum PID correction.

**Files:** `inspector_node.py` (yaw PID output clamp ~L820)

### 17. BUG3 — Teleop Multi-Axis Broken (teleop_driver.py)

**Problem:** The teleop driver published up to 3 separate DriverCommands per Twist callback (horizontal, vertical, yaw). Each one replaced `_current_movement` in the inspector, so only the last command took effect. Moving diagonally with vertical thrust was impossible.

**Fix:** New `teleop` command carries all 4 axes in a single DriverCommand:
- `speed` → forward/back PWM offset
- `duration` → lateral PWM offset (repurposed field)
- `depth` → throttle PWM offset
- `angle` → yaw PWM offset

Inspector decodes all 4 and sets channels in one `_current_movement`.

**Files:** `teleop_driver.py` (complete rewrite of `_twist_cb`), `inspector_node.py` (new `teleop` command handler)

### 18. BUG4 — Teleop Stop Floods (teleop_driver.py)

**Problem:** When all joystick axes returned to centre, the teleop driver published a `stop` command **every tick**. The `stop` handler clears `_depth_pid`, `_yaw_to_heading`, and `_alt_hold_target` — so any active depth hold or heading lock was destroyed whenever the joystick was idle.

**Fix:**
1. New `teleop_idle` command in inspector — clears `_current_movement` only, preserves PIDs and heading lock.
2. Teleop driver tracks `_last_was_idle` flag and sends `teleop_idle` only once when entering dead-zone.

**Files:** `teleop_driver.py` (idle tracking), `inspector_node.py` (new `teleop_idle` command handler)

### 19. BUG5 — `time.sleep(0.5)` Blocks Callback Thread (inspector_node.py)

**Problem:** The `set_depth` handler called `time.sleep(0.5)` after switching to ALT_HOLD mode to "wait for mode switch to take effect". This blocked the ROS callback thread for 500ms, delaying all other incoming commands during that window.

**Fix:** Removed the `sleep()`. The depth target is sent immediately after the mode switch — the 2Hz resend timer (`_resend_alt_hold`) ensures the target is retransmitted if the first one arrives before the mode switch completes.

**Files:** `inspector_node.py` (`set_depth` handler ~L1090)

### 20. BUG6 — Dual Yaw Source Jitter (inspector_node.py)

**Problem:** Both `AHRS2` and `ATTITUDE` MAVLink messages wrote to `self._yaw`, but at different rates and from different EKF filters. When both are active, `_yaw` alternates between two slightly different values, causing PID oscillation and heading hold jitter.

**Fix:** New ROS parameter `yaw_source` (default: `'attitude'`) selects which message updates `self._yaw`:
- `'attitude'` — ATTITUDE only (recommended, higher rate, primary ArduSub AHRS)
- `'ahrs2'` — AHRS2 only
- `'both'` — legacy behaviour (both update, not recommended)

Depth always comes from AHRS2 regardless of yaw source.

**Files:** `inspector_node.py` (AHRS2/ATTITUDE handlers ~L410-430, new `yaw_source` parameter ~L140)

### 21. BUG7 — Integral Windup Aggressive Defaults (inspector_node.py)

**Problem:** Depth PID defaults were Ki=100, max_integral=2.0 → maximum integral contribution = Ki × max_integral = 200 PWM (50% of the 400-PWM range). On descent, the integral quickly saturated, and upon reaching target depth the accumulated integral drove a massive overshoot requiring many seconds to unwind.

**Fix:**
1. **Reduced defaults:** Ki: 100→25, max_integral: 2.0→0.5 → max I contribution = 12.5 PWM (3% of range). Now integrals only compensate for buoyancy drift, not overpower proportional control.
2. **Conditional integration:** Integral accumulation pauses when PID output is saturated (|output| ≥ PWM_RANGE). Prevents windup during large transients.
3. All defaults remain overridable via ROS parameters (`depth_ki`, `depth_max_integral`).

**Files:** `inspector_node.py` (depth PID defaults ~L155, depth PID loop ~L770)
