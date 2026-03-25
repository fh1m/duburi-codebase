# Desk Testing Guide — Verifying the 7 Bug Fixes

> **Prerequisites:** Pixhawk 2.4.8 connected via USB (`/dev/ttyACM0`), ROS 2
> Humble sourced, workspace built. No water needed — all tests verify logic,
> telemetry, and command routing on the bench.

---

## Quick Start

```bash
# Source workspace (every terminal)
cd ~/workspaces/duburi_ws && source install/setup.bash

# Terminal 1 — Inspector + Logger
ros2 launch mavlink_inspector duburi_control.launch.py

# Terminal 2 — Runner CLI (interactive testing)
ros2 run mavlink_runner runner

# Terminal 3 — Monitoring (pick what you need)
ros2 topic echo /mavlink/vehicle_state          # depth, yaw, armed, mode
ros2 topic echo /mavlink/events         # movement events
ros2 topic echo /driver/command         # command traffic
ros2 topic echo /mavlink/diagnostics    # heading_rate, pressure
```

---

## Test 1 — BUG1: PID Derivative Kick (Fixed)

**What was broken:** Setting a new depth/yaw target caused a PWM spike (full
thrust for one tick) because the derivative term reacted to the setpoint jump.

**How to verify on desk:**

### Depth PID (derivative on measurement)

1. In the runner CLI:
   ```
   arm
   mode MANUAL
   ~dive 0.5        # start PID depth hold at 0.5m
   ```
2. In the monitoring terminal, watch throttle channel:
   ```bash
   ros2 topic echo /mavlink/events
   ```
3. **While PID is active**, change the target:
   ```
   ~dive 1.0        # change target mid-hold
   ```
4. **Expected:** Throttle channel changes **gradually** (proportional ramp).
   No single-tick spike to ±400 PWM.

   **Before fix:** You'd see `PID depth: throttle_offset=±400` on the tick
   immediately after target change.

   **After fix:** The derivative term uses `(depth - prev_depth)/dt` which is
   near-zero at the moment of target change (depth hasn't moved yet), so only
   the P term ramps up.

### Yaw PID (derivative on heading rate)

1. ```
   ~yaw 90          # PID yaw to 90°
   ```
2. While holding, change target:
   ```
   ~yaw 270         # 180° turn
   ```
3. **Expected:** Yaw PWM ramps up proportionally. No spike.
   The derivative term now uses `heading_rate` from the gyro (ATTITUDE message),
   which reads near-zero when the AUV hasn't started turning yet.

### What to look for in logs

```
# Good (gradual):
PID depth: throttle_offset=50
PID depth: throttle_offset=85
PID depth: throttle_offset=120

# Bad (spike — means fix didn't work):
PID depth: throttle_offset=400   ← full range spike
PID depth: throttle_offset=120
```

---

## Test 2 — BUG2: Yaw PID Speed Parameter (Fixed)

**What was broken:** `~yaw 90 30%` and `~yaw 90 100%` both allowed the same
max PID output (±400 PWM). Speed was stored but ignored.

**How to verify:**

1. ```
   ~yaw 180 30%    # slow PID yaw
   ```
2. Watch events — max yaw PWM offset should be ~120 (30% of 400).
3. Stop, then:
   ```
   ~yaw 180 100%   # fast PID yaw
   ```
4. Max yaw PWM offset should be ~400 (full range).

### What to look for

```
# 30% speed → clamped to ±120:
movement: PID yaw to heading 180° ...
# yaw channel shows offsets ≤120

# 100% speed → clamped to ±400:
# yaw channel shows offsets up to 400
```

The `gain_offset` is now used as the PID output clamp in the yaw PID loop
(`max_pwm = y2h.get('gain_offset', PWM_RANGE)`).

---

## Test 3 — BUG3: Teleop Multi-Axis (Fixed)

**What was broken:** Moving diagonally (forward + right) while also
commanding vertical (up) sent 3 separate DriverCommands. Each overwrote
`_current_movement` — only the last axis worked.

**How to verify:**

1. Start teleop driver:
   ```bash
   # Terminal 4
   ros2 run mavlink_driver teleop_driver
   ```

2. Publish a multi-axis Twist (simulating joystick):
   ```bash
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 1.0, y: 0.5, z: 0.3}, angular: {z: 0.0}}"
   ```

3. Monitor the command sent:
   ```bash
   ros2 topic echo /driver/command
   ```

4. **Expected:** A SINGLE `teleop` command with all axes encoded:
   ```
   command: teleop
   speed: 50         # forward (1.0 × scale_linear=50)
   duration: 25.0    # lateral right (0.5 × 50)
   depth: 15.0       # throttle up (0.3 × 50)
   angle: 0.0        # no yaw
   ```

   **Before fix:** You'd see THREE separate commands: `move_forward`,
   `move_right`, `move_up` — only `move_up` (last) would take effect.

---

## Test 4 — BUG4: Teleop Stop Floods (Fixed)

**What was broken:** When joystick returned to centre, `stop` was sent every
tick, nuking depth PID and heading hold.

**How to verify:**

1. Activate a depth PID hold:
   ```
   ~dive 0.5
   ```
2. Send a zero Twist (simulating joystick centred):
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}" -r 10
   ```
3. **Expected:**
   - ONE `teleop_idle` command appears in `/driver/command` (not repeated).
   - Depth PID remains active (events still show "PID depth: ...").
   - No "Stop - all thrusters neutral" events flooding.

   **Before fix:** You'd see `stop` every 100ms, and "PID depth hold OFF"
   immediately, killing the depth hold.

4. **Verify idle dedup** — publish zero Twist continuously at 10 Hz for 5
   seconds. You should see `teleop_idle` exactly ONCE in /driver/command,
   not 50 times.

---

## Test 5 — BUG5: sleep(0.5) in set_depth (Fixed)

**What was broken:** `set_depth` called `time.sleep(0.5)` which blocked the
ROS callback thread, delaying all other commands for 500ms.

**How to verify:**

1. Send a depth command and immediately a movement command:
   ```
   dive 0.5
   forward 50% 3s
   ```
   (Type both quickly in runner, or chain in a mission file)

2. **Expected:** Both commands are processed immediately — no visible lag
   between "ALT_HOLD depth target" event and "Moving forward" event.

   **Before fix:** The "Moving forward" event would appear ~500ms after the
   depth event because the callback thread was sleeping.

3. **Programmatic test** — Publish two commands 100ms apart via:
   ```bash
   ros2 topic pub --once /driver/command duburi_interfaces/msg/DriverCommand \
     "{command: 'set_depth', depth: 0.5}" &
   sleep 0.1
   ros2 topic pub --once /driver/command duburi_interfaces/msg/DriverCommand \
     "{command: 'move_forward', speed: 50, duration: 3.0}"
   ```
   Both events should appear within ~100ms of each other in `/mavlink/events`.

---

## Test 6 — BUG6: Dual Yaw Source Jitter (Fixed)

**What was broken:** Both AHRS2 and ATTITUDE messages updated `self._yaw` at
different rates, causing the yaw value to alternate between two slightly
different readings → PID oscillation.

**How to verify:**

1. **Check default yaw source:**
   ```bash
   ros2 param get /mavlink_inspector yaw_source
   ```
   Should return `attitude` (the default).

2. **Monitor yaw stability:**
   ```bash
   ros2 topic echo /mavlink/vehicle_state --field yaw
   ```
   With the Pixhawk on the desk, yaw should be stable (±0.1° jitter max).

   **Before fix:** Yaw would jitter ±1-3° as AHRS2 and ATTITUDE wrote
   different values alternately.

3. **Test switching sources:**
   ```bash
   ros2 param set /mavlink_inspector yaw_source ahrs2
   ```
   (Note: runtime parameter changes won't take effect because the value is
   read at startup. Restart inspector with:)
   ```bash
   ros2 run mavlink_inspector inspector --ros-args -p yaw_source:=ahrs2
   ```
   Yaw should still be stable, just from AHRS2.

4. **Test 'both' (legacy, expect jitter):**
   ```bash
   ros2 run mavlink_inspector inspector --ros-args -p yaw_source:=both
   ```
   Should show the old jitter behavior — confirms the fix is actually filtering.

---

## Test 7 — BUG7: Integral Windup (Fixed)

**What was broken:** Ki=100, max_integral=2.0 → max integral contribution =
200 PWM (half the range). On descent, integral quickly saturated, causing
massive overshoot when reaching target.

**How to verify:**

1. **Check new defaults:**
   ```bash
   ros2 param get /mavlink_inspector depth_ki
   # Should return 25.0 (was 100.0)
   ros2 param get /mavlink_inspector depth_max_integral
   # Should return 0.5 (was 2.0)
   ```
   Max integral contribution: 25 × 0.5 = 12.5 PWM (~3% of range). Safe.

2. **Test conditional integration:**
   - Start PID depth hold to a far target (forces PID saturation):
     ```
     ~dive 3.0         # very deep — PID will saturate
     ```
   - Wait a few seconds, then check the integral hasn't wound up:
     ```
     # The old bug: integral would hit max (2.0) within seconds,
     # contributing 200 PWM of offset that persists after reaching target.
     # New behavior: integral pauses accumulation when output is saturated.
     ```
   - Change target to a closer depth:
     ```
     ~dive 0.3
     ```
   - **Expected:** No massive overshoot. Throttle should reverse promptly
     because the integral wasn't wound up during the saturated phase.

3. **Override to old values (regression test):**
   ```bash
   ros2 run mavlink_inspector inspector --ros-args \
     -p depth_ki:=100.0 \
     -p depth_max_integral:=2.0
   ```
   Repeat the test — you should see much larger integral-driven overshoot.
   This confirms the new defaults are better.

---

## Quick Smoke Test Checklist (All 7 Bugs)

Run these in the runner CLI with Pixhawk on USB:

```bash
# Source workspace
cd ~/workspaces/duburi_ws && source install/setup.bash

# Start inspector
ros2 launch mavlink_inspector duburi_control.launch.py &

# Start runner
ros2 run mavlink_runner runner
```

In the runner:

```
# Basic connectivity
status                  # Should show depth, yaw, mode

# BUG6 — Yaw source (visual check)
status                  # Yaw should be stable (no jitter)

# BUG7 — PI defaults
# (check with ros2 param get in another terminal)

# BUG5 — No blocking on depth set
arm
mode MANUAL
dive 0.5                # Should be instant, no 500ms lag
forward 50% 2s          # Should start immediately if typed quickly after dive

# BUG1 — No derivative kick
~dive 0.5               # Start PID depth
~dive 1.0               # Change target — no throttle spike in events

# BUG2 — Speed matters for yaw PID
~yaw 90 30%             # Watch max yaw PWM ≤ 120
stop
~yaw 90 100%            # Watch max yaw PWM ≤ 400

# BUG3+4 — Teleop (in another terminal)
# ros2 run mavlink_driver teleop_driver
# ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
#   "{linear: {x: 1.0, y: 0.5, z: 0.3}, angular: {z: 0.0}}"
# Check /driver/command shows ONE 'teleop' command

# Cleanup
stop
disarm
```

---

## Monitoring Commands Reference

```bash
# Real-time state (depth, yaw, armed, mode)
ros2 topic echo /mavlink/vehicle_state

# Movement events (PID output, command execution)
ros2 topic echo /mavlink/events

# Raw commands entering inspector
ros2 topic echo /driver/command

# Diagnostics (heading_rate, voltage, pressure)
ros2 topic echo /mavlink/diagnostics

# Check a parameter
ros2 param get /mavlink_inspector depth_ki
ros2 param get /mavlink_inspector yaw_source

# List all inspector parameters
ros2 param list /mavlink_inspector

# Topic Hz (verify publish rates)
ros2 topic hz /mavlink/vehicle_state           # expect ~10 Hz
ros2 topic hz /mavlink/diagnostics     # expect ~2 Hz
```
