# ArduSub Constraints That Drive Design

This document explains ArduSub/MAVLink requirements that directly influenced implementation. **Do not violate these** when modifying the codebase.

---

## 1. RC Override Must Be Sent Continuously

**Source:** [ArduSub pymavlink docs](https://www.ardusub.com/developers/pymavlink.html)

> "When the autopilot is being commanded to move via RC_CHANNELS_RAW or MANUAL_CONTROL messages, the messages must be sent at a constant rate like the HEARTBEAT message. Otherwise, the autopilot will execute a failsafe if it has not received an updated command after a timeout period."

**Timeout:** ~3 seconds typical.

**Implication:**
- We cannot send RC_CHANNELS_OVERRIDE once and stop. We must send at 20–50 Hz for the entire duration of movement.
- When idle (no movement), we must still send neutral (1500) on all channels, or ArduSub will failsafe and **disarm**.
- Implementation: `_send_rc_override` timer at 20 Hz (0.05 s). When `_current_movement` is set, sends movement channels. When None, sends neutral.

---

## 2. Heartbeat Required

**Source:** Same ArduSub docs.

> "All system components that communicate via MAVLink are expected to send a HEARTBEAT message at a constant rate of at least 1 Hz."

**Implication:**
- Inspector sends heartbeat at 1 Hz via `_heartbeat_timer`.
- Without it, ArduSub triggers failsafe.

---

## 3. Channel Mapping (Duburi 4.2 / ArduSub)

**Source:** Old mishu codebase, ArduSub Sub docs.

| Channel | Function | Neutral | Range |
|---------|----------|---------|-------|
| 1 | Pitch | 1500 | 1100–1900 |
| 2 | Roll | 1500 | 1100–1900 |
| 3 | Throttle (depth) | 1500 | 1100–1900 |
| 4 | Yaw | 1500 | 1100–1900 |
| 5 | Forward | 1500 | 1100–1900 |
| 6 | Lateral | 1500 | 1100–1900 |
| 7–8 | Reserved (grabber, etc.) | — | — |

**PWM convention:** 1500 = neutral. 1100–1500 = one direction, 1500–1900 = opposite. ±400 from 1500 for full deflection.

**percent_to_pwm:** `1500 + (percent/100) * 400` maps -100..100 to 1100..1900.

---

## 4. Depth Convention

**ArduSub:** Negative Z = below surface. `set_position_target_global_int` uses depth as negative for underwater.

**DriverCommand:** User specifies positive depth (e.g. 0.5 m). Inspector negates: `d = -abs(d)` before sending.

---

## 5. Arm/Disarm Blocking

**pymavlink:** `motors_armed_wait()` and `motors_disarmed_wait()` block until HEARTBEAT confirms state. They consume MAVLink messages (recv_match).

**Implication:**
- Must run in a separate thread. Blocking in ROS callback would block executor and RC override timer.
- Implementation: `_do_arm_disarm` runs in daemon thread. Publishes events when done.

---

## 6. MAVLink 2

**Setting:** `os.environ['MAVLINK20'] = '1'` before importing pymavlink.

**Reason:** MAVLink 2 supports 18 RC channels, MAVLink 1 only 8. We use RC_CHANNELS_OVERRIDE with 18 slots.

---

## 7. set_attitude_target vs RC Override

**set_attitude_target:** Used for attitude hold in STABILIZE/ALT_HOLD. May be ignored or behave differently in MANUAL.

**RC override:** Works in MANUAL. Direct thruster control.

**Implication:** `yaw_angle` (set_attitude_target) may not work in MANUAL. We added `yaw_to_heading` which uses RC override on yaw channel with closed-loop error.

---

## 8. Servo/Grabber

**MAV_CMD_DO_SET_SERVO:** `servo_n + 8` because MAIN outputs 1–8 are thrusters; AUX 1–8 map to servo instances 9–16. So servo 1 → instance 9.
