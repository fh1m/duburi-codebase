# mavlink_driver – Line-by-Line Analysis

**Package:** `src/mavlink_driver/`

Contains: `driver_client.py`, `mission_executor.py`, `teleop_driver.py`.

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

## teleop_driver.py

### Twist to DriverCommand

**Why:** Converts `/cmd_vel` (geometry_msgs/Twist) to `DriverCommand` for joystick/teleop nodes.

### Axis Priority

```python
if abs(msg.linear.x) > 0.1:
    cmd.command = 'move_forward' if msg.linear.x > 0 else 'move_back'
elif abs(msg.linear.y) > 0.1:
    ...
```
**Why:** One axis at a time. Prevents conflicting commands when multiple axes are non-zero.

### scale_linear / scale_angular

**Why:** Maps Twist magnitude to speed. Default 50 → 50% gain. Tune for joystick sensitivity.
