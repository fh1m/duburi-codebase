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
