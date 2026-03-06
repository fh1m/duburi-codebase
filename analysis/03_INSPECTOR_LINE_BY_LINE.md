# mavlink_inspector – Line-by-Line Analysis

**File:** `src/mavlink_inspector/mavlink_inspector/inspector_node.py`

This node owns the single MAVLink connection to the Pixhawk. All control flows through it.

---

## Imports and Environment

```python
os.environ['MAVLINK20'] = '1'
```
**Why:** Forces MAVLink 2. Must be set before importing pymavlink. MAVLink 2 is required for ArduSub.

```python
from pymavlink.quaternion import QuaternionBase
```
**Why:** Used for `set_attitude_target_send` and `set_target_attitude`. ArduSub expects quaternions for attitude targets.

---

## Channel Mapping Constants

```python
CH_FORWARD = 5
CH_LATERAL = 6
CH_THROTTLE = 3
CH_YAW = 4
```
**Why:** ArduSub/BlueROV2 convention. Ch 5 = forward/back, Ch 6 = left/right, Ch 3 = depth, Ch 4 = yaw. Matches old mishu codebase and ArduSub docs.

```python
NEUTRAL_PWM = 1500
PWM_RANGE = 400  # 1100-1900, so ±400 from 1500
```
**Why:** Standard RC PWM: 1500 = no movement, 1100–1900 = full range. ±400 gives ±100% in our percent mapping.

---

## percent_to_pwm()

```python
def percent_to_pwm(percent: float) -> int:
    percent = max(-100, min(100, percent))
    return int(1500 + (percent / 100) * PWM_RANGE)
```
**Why:** Same formula as mishu `control_utility.py`: `1500 + (percent/100)*400`. Clamps to avoid invalid PWM.

---

## __init__ – Connection in Thread

```python
self._connect_thread = threading.Thread(target=self._connect, daemon=True)
self._connect_thread.start()
```
**Why:** `wait_heartbeat()` blocks. Running in a thread keeps the node responsive. Daemon so it does not block shutdown.

---

## Timer Frequencies

| Timer | Period | Purpose |
|-------|--------|---------|
| `_read_timer` | 0.02 s (50 Hz) | Read MAVLink messages |
| `_state_timer` | 0.1 s (10 Hz) | Publish VehicleState |
| `_heartbeat_timer` | 1.0 s | Send GCS heartbeat |
| `_rc_override_timer` | 0.05 s (20 Hz) | Send RC_CHANNELS_OVERRIDE |

**Why:** ArduSub expects heartbeat ≥1 Hz and RC override at a steady rate. 20 Hz RC is enough to avoid the ~3 s failsafe timeout.

---

## _send_rc_override() – Critical Logic

```python
if mv is not None and now >= mv['end_time']:
    self._stop_all()
    mv = None
```
**Why:** When movement duration expires, clear movement and stop.

```python
if mv is not None:
    for ch, pwm in mv.get('channels', {}).items():
        self._set_rc_channel_pwm(ch, pwm)
else:
    for ch in [CH_FORWARD, CH_LATERAL, CH_THROTTLE, CH_YAW]:
        self._set_rc_channel_pwm(ch, NEUTRAL_PWM)
```
**Why:** ArduSub failsafes if RC stops. When idle we must still send neutral at 20 Hz so it does not disarm.

---

## _set_rc_channel_pwm() – RC_CHANNELS_OVERRIDE

```python
rc = [65535] * 18
rc[channel_id - 1] = int(max(1100, min(1900, pwm)))
```
**Why:** 65535 = “unchanged” in MAVLink. Only the requested channel is set; others stay at 65535.

---

## _current_movement Structure

```python
{'channels': {CH_FORWARD: 1700, CH_LATERAL: 1500, ...}, 'end_time': float}
```
**Why:** Stores per-channel PWM and end time. `end_time = inf` for indefinite movement.

---

## _yaw_to_heading – Thruster-Based Yaw

```python
self._yaw_to_heading = {
    'target_deg': cmd.angle % 360,
    'gain_offset': min(PWM_RANGE, gain_offset),
    'tolerance_deg': 5.0,
}
```
**Why:** `set_attitude_target` is unreliable in MANUAL. We use RC override on the yaw channel and close the loop on heading. 5° tolerance avoids oscillation.

```python
err = (target - current) % 360
if err > 180:
    err -= 360
```
**Why:** Shortest-path angle error in [-180, 180]. Ensures we turn the shorter way.

---

## Speed Interpretation

```python
if 0 < raw_speed <= 100:
    speed = percent_to_pwm(raw_speed)
else:
    speed = NEUTRAL_PWM + max(-PWM_RANGE, min(PWM_RANGE, int(raw_speed)))
```
**Why:** 0–100 = percent (e.g. 50 → 1700). Other values = PWM offset from 1500 for compatibility.

---

## Movement Channel Math

**Forward:** `CH_FORWARD: speed` (speed > 1500)  
**Back:** `CH_FORWARD: NEUTRAL_PWM - (speed - NEUTRAL_PWM)`  
**Left:** `CH_LATERAL: NEUTRAL_PWM - (speed - NEUTRAL_PWM)`  
**Right:** `CH_LATERAL: speed`  
**Up:** `CH_THROTTLE: NEUTRAL_PWM - (speed - NEUTRAL_PWM)` (less throttle = rise)  
**Down:** `CH_THROTTLE: speed`

**Why:** Sign convention from mishu. Forward/right/down use positive offset; back/left/up use negative.

---

## _arm_disarm in Thread

```python
threading.Thread(target=_do_arm_disarm, daemon=True).start()
```
**Why:** `motors_armed_wait()` blocks on MAVLink. Running in a thread avoids blocking the ROS executor and callbacks.

---

## set_target_depth – Depth Sign

```python
d = -abs(d) if d > 0 else d  # ArduSub: negative = below surface
```
**Why:** User depth is positive (e.g. 2 m). ArduSub uses negative Z for below surface.

---

## Grabber – servo_n + 8

```python
servo_n + 8  # MAV_CMD_DO_SET_SERVO instance
```
**Why:** ArduSub: outputs 1–8 are main (thrusters), 9+ are AUX. Servo 1 → instance 9.
