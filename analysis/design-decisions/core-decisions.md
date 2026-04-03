# Design Decisions

## 1. RC Override Must Be Sent Continuously (20 Hz)

**Decision**: A timer at 50 ms (20 Hz) sends RC_CHANNELS_OVERRIDE whenever movement is active or idle.

**Why**: ArduSub failsafes (and can disarm) if RC override stops for ~3 seconds. From ArduSub docs: *"When the autopilot is being commanded to move via RC_CHANNELS_RAW or MANUAL_CONTROL messages, the messages must be sent at a constant rate."*

**Implementation**: `_send_rc_override` runs every 0.05 s. When `_current_movement` is set, it sends movement channels; when idle, it sends neutral (1500) on all channels.

---

## 2. Movement State Instead of One-Shot RC

**Decision**: Store `_current_movement = {channels, end_time}` and let the RC timer drive output for the full duration.

**Why**: Sending RC once and scheduling a stop in a thread does not satisfy ArduSub’s requirement for continuous RC. The old mishu `basic.py` used a `while` loop that repeatedly called `set_rc_channel_pwm` for the whole duration.

**Implementation**: `_on_driver_command` sets `_current_movement`. `_send_rc_override` sends those channels each tick until `end_time`, then clears and sends stop.

---

## 3. Idle Sends Neutral RC

**Decision**: When no movement is active, still send neutral (1500) on all channels at 20 Hz.

**Why**: If we stop sending RC after a movement ends, ArduSub sees no RC and can failsafe/disarm. Neutral keeps the link alive.

---

## 4. Yaw-to-Heading via Thrusters, Not set_attitude_target

**Decision**: `yaw 260` uses `yaw_to_heading`, which drives the yaw channel based on heading error until the target is reached.

**Why**: `set_attitude_target` (SET_ATTITUDE_TARGET) is for stabilized modes. In MANUAL mode it may be ignored. Thrusters work in all modes.

**Implementation**: `_yaw_to_heading = {target_deg, gain_offset, tolerance_deg}`. Each RC tick, compute angle error, apply yaw PWM, clear when within tolerance.

---

## 5. Arm/Disarm in a Thread

**Decision**: `_arm_disarm` runs in a daemon thread that calls `motors_armed_wait()` / `motors_disarmed_wait()`.

**Why**: These calls block on `recv_match`. Blocking in the ROS callback would stall the executor. A thread keeps the node responsive.

---

## 6. Non-Blocking Arm/Disarm in Runner

**Decision**: Runner publishes arm/disarm and returns immediately. Confirmation is printed when events arrive.

**Why**: Blocking on `_wait_for_ack` caused the CLI to hang when events were delayed or lost. Non-blocking avoids stuck prompts.

---

## 7. Mission Wait Logic: Always Wait for `wait_sec`

**Decision**: In `_execute_chain`, we always `time.sleep(wait_sec)` when `wait_sec > 0`, not only when there is a next command.

**Why**: The previous logic only waited when `i < len(parts) - 1`. For mission files, each line is one part, so we never waited. Commands overwrote each other before they could run.

---

## 8. Arm/Disarm Return wait_sec in Runner

**Decision**: `arm` returns `(True, 4.0)`, `disarm` returns `(True, 2.0)`.

**Why**: Missions need time for the vehicle to arm/disarm before the next command. These fixed delays approximate that.

---

## 9. Speed as 0–100 Percent or PWM Offset

**Decision**: If `0 < speed <= 100`, treat as percent and use `percent_to_pwm()`. Otherwise treat as PWM offset from 1500.

**Why**: Matches old `control_utility.py` (`percent_to_pwm`). CLI uses percent; programmatic use can use raw PWM.

---

## 10. Connection in Background Thread

**Decision**: `_connect()` runs in a daemon thread started in `__init__`.

**Why**: `wait_heartbeat()` blocks. Running it in the main thread would block node startup. A thread lets the node start and connect asynchronously.

---

## 11. Mission Search Paths

**Decision**: Missions are searched in: `./missions/`, `mavlink_runner/missions/`, `~/.duburi/missions/`.

**Why**: Supports workspace-local missions, package-installed missions, and user-specific missions.

---

## 12. readline for History/Cursor

**Decision**: `import readline` before `input()` to enable Up/Down history and Left/Right cursor.

**Why**: Default `input()` has no history or editing. readline is standard on Unix and improves usability.

---

# Control Redesign V2 Decisions

## 13. All V2 Features Disabled by Default

**Decision**: Every V2 control feature (convergence, rotate-in-place, cascade, gain scheduling, DVL) is disabled by default via `*_enabled: false` parameters.

**Why**: Untested control algorithms can damage hardware. Safe defaults let us test incrementally. Enable features one at a time after pool validation.

---

## 14. Configuration-Driven Architecture

**Decision**: 74+ ROS2 parameters control all V2 behavior. No magic numbers in code.

**Why**: 
- Tune without recompiling
- Different robots may need different values
- Parameters are documented in defaults.yaml

---

## 15. Modular Control Classes

**Decision**: Each control concept is a separate class (VelocityEstimator, ConvergenceGate, GainScheduler, etc.) in dedicated modules.

**Why**:
- Single responsibility per class
- Unit testable in isolation
- Hot-swappable at runtime if needed

---

## 16. ZUPT for Drift Correction

**Decision**: When IMU accelerometer reads near-zero for N seconds, reset velocity estimate to zero.

**Why**: IMU integration drifts unboundedly. ZUPT (Zero-velocity Update) provides periodic correction when stationary. Simple, effective, no external sensors needed.

---

## 17. Convergence Gates Between Commands

**Decision**: Movement commands can optionally wait for vehicle to stabilize (velocity < threshold for N ms) before completing.

**Why**: Prevents inertia from one command affecting the next. Critical for mission accuracy.

---

## 18. Rotate-in-Place with Translation Lock

**Decision**: During sharp yaw commands, lock translation channels (forward/lateral/throttle) to neutral.

**Why**: Normal yaw commands allow translation to continue, causing drift during turns. Locking translation ensures vehicle rotates on its axis.

---

## 19. Two-Zone Yaw PID

**Decision**: Yaw PID uses reduced gains near target (precision zone: 5-10°, final zone: <5°).

**Why**: Full gains near target cause overshoot. Reduced gains allow precise final positioning without sacrificing responsiveness far from target.

---

## 20. Settling Time for Yaw

**Decision**: Yaw is not "reached" until target is held for N ms (settling time).

**Why**: A momentary pass through target due to oscillation doesn't count as reached. Settling time ensures stable convergence.

---

## 21. Cascade Position Control

**Decision**: Position control uses cascade architecture: Position PID → Velocity Setpoint → Velocity PID → Thrust.

**Why**: 
- Position-only control is sluggish (needs high integrator, causes windup)
- Velocity-only control can't hold position
- Cascade gives fast response AND accurate positioning

---

## 22. Gain Scheduling by Speed

**Decision**: Three sets of PID gains for low (0-30%), medium (30-60%), and high (60-100%) speed ranges.

**Why**: Gains tuned for 30% are too aggressive at 90%. Different speeds need different gains for optimal performance.

---

## 23. Acceleration Limiting

**Decision**: Maximum acceleration rate (50%/sec default). Commands ramp up gradually.

**Why**: Instant full-throttle causes overshoot and instability. Ramping allows controller to track smoothly.

---

## 24. Sensor Source Priority Fallback

**Decision**: Multiple sensor sources (DVL, external compass, Pixhawk) with priority-based fallback.

**Why**: Sensors fail. DVL loses bottom lock, compass cables disconnect. System must degrade gracefully to less accurate but available sources.

---

## 25. Sensor Staleness Detection

**Decision**: Each sensor source has a timeout. If no update within timeout, source is marked invalid and fallback activates.

**Why**: Stale data is worse than no data. A 5-second-old velocity reading will cause incorrect control. Better to use IMU estimate than stale DVL.

---

## 26. Message-Type-Agnostic External Yaw

**Decision**: External yaw source supports multiple message types (Float32, Imu, Vector3Stamped) via configuration.

**Why**: Different compass sensors publish different message types. Supporting multiple types allows easy sensor swapping without code changes.
