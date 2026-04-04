# Control Flow - V2 Architecture

**Last Updated:** April 2026
**Status:** [DONE] Production Ready - All 30 bug fixes complete

---

## Table of Contents

1. [Mission Execution Flow](#mission-execution-flow)
2. [Control Loop Architecture](#control-loop-architecture)
3. [Telemetry Processing Pipeline](#telemetry-processing-pipeline)
4. [Command Dispatch Flow](#command-dispatch-flow)
5. [Safety & Watchdog Systems](#safety--watchdog-systems)
6. [Convergence & Settling](#convergence--settling)

---

## Mission Execution Flow

Complete flow from mission file to motor commands:

```

 Mission Execution Flow

 Mission Runner
 Parse mission file
 Execute commands sequentially
 Wait for convergence (if enabled)

 MAVLink Inspector (Core Control Node)

 Telemetry Parser
 AHRS2 → Yaw/Pitch/Roll
 SCALED_PRESSURE → Depth (not altitude!)
 SCALED_IMU2 → Acceleration
 Watchdog: Detect stale messages

 Velocity Estimator (V2 Phase 1)
 Gravity rotation via quaternion
 Integrate body-frame acceleration
 ZUPT: Zero-velocity updates
 Drift mitigation: 0.5 m/s² threshold

 Cascade Controller (V2 Phase 3)
 Position Loop: Target → Velocity
 Velocity Loop: Velocity → Thrust
 Per-DOF integrals (surge/sway/heave)
 Gain scheduling based on speed

 RC Controller (V2 Phase 2)
 Ramping: Smooth acceleration/deceleration
 Active braking: Reduce overshoot
 Thread-safe _ramped dict
 PWM limits: 1100-1900

 MAVLink Connection
 RC_CHANNELS_OVERRIDE @ 20Hz (500ms watchdog)
 HEARTBEAT @ 2Hz (ArduSub GCS failsafe)
 source_system=1 (GCS identification)

 ArduSub Flight Controller
 Receives RC overrides and commands
 Sends telemetry (AHRS2, SCALED_PRESSURE, IMU)
 Motor mixing and ESC control

```

---

## Control Loop Architecture

### V2 Control Stack Layers

```

 V2 Control Stack
 (20Hz Control Loop)

 Layer 1: Sensor Fusion & State Estimation

 TelemetryParser: AHRS2, SCALED_PRESSURE, SCALED_IMU2
 SensorSourceManager: DVL/External IMU/Pixhawk priority
 Message Watchdog: Detect stale data (1Hz check)

 Layer 2: Velocity Estimation (Phase 1)

 Quaternion-based gravity compensation
 - Eliminates 49 m/s drift at 30° pitch
 IMU integration: a_world = a_body - g_rotated
 ZUPT correction: 0.5 m/s² threshold, 1.0s window
 Convergence gate: Wait for settling before next command

 Layer 3: Position Control (Phase 3)

 PositionEstimator: Dead reckoning from velocity
 CascadeController:
 - Outer loop: Position error → Target velocity
 - Inner loop: Velocity error → Thrust
 Per-DOF integral terms (surge, sway, heave)
 Anti-windup: Clamp integrals to prevent saturation

 Layer 4: Gain Scheduling & Limiting (Phase 4)

 GainScheduler: 3 speed ranges (low/medium/high)
 - Low (<0.2 m/s): Aggressive gains, quick response
 - Medium (0.2-0.5 m/s): Balanced gains
 - High (>0.5 m/s): Conservative gains, prevent overshoot
 AccelerationLimiter: Max 50% change per second

 Layer 5: RC Output & Safety (Phase 2)

 RCController:
 - Trapezoidal ramping (smooth accel/decel)
 - Active braking (skip ramping during brake)
 - Thread-safe _ramped dict (all accesses locked)
 PWM conversion: -100% to +100% → 1100-1900 μs
 Channel mapping: CH1-4 lateral, CH5-8 depth
 Continuous 20Hz output (10x faster than ArduSub requires)

 ArduSub Firmware (Pixhawk)
 RC Override watchdog: 500ms timeout
 GCS Heartbeat watchdog: Must receive 2Hz HEARTBEAT
 Motor mixing: Vectored thrust → 8 ESC channels

```

---

## Telemetry Processing Pipeline

### Message Flow (ArduSub → Inspector)

```

 ArduSub Telemetry Messages (115200 baud serial)

 MAVLink Messages @ ~50Hz

 TelemetryParser.on_mavlink_message()

 Message Routing:

 AHRS2 (msg.id = 178)
 Roll, Pitch, Yaw (quaternion)
 Used for attitude, NOT altitude

 SCALED_PRESSURE (msg.id = 29)
 Absolute pressure (mbar)
 Auto-calibrate surface pressure on startup
 depth = (P_current - P_surface) * 0.01 meters

 SCALED_IMU2 (msg.id = 116)
 Raw acceleration (mg units)
 Convert to m/s²: acc = raw * 9.81 / 1000
 Body frame: surge/sway/heave

 Timestamp Tracking (Watchdog):
 last_ahrs2_time, last_pressure_time, last_imu_time
 Check every 1Hz: Warn if >2 seconds stale

 Parsed Telemetry

 VehicleState Message

 attitude:
 yaw: 45.2° (from AHRS2 quaternion)
 pitch: -5.1° (from AHRS2 quaternion)
 roll: 2.3° (from AHRS2 quaternion)

 depth: 1.52m (from SCALED_PRESSURE, NOT altitude)

 acceleration:
 surge: 0.12 m/s² (from SCALED_IMU2, body frame)
 sway: -0.03 m/s²
 heave: 9.81 m/s² (includes gravity!)

 Published @ 20Hz

 /mavlink/vehicle_state topic
 Available to all ROS2 nodes
 Used by planner, vision, logger

```

### Critical Bug Fix: Depth from Pressure Sensor

**Before (Bug #3):**
```python
# WRONG: Used MSL altitude estimate from AHRS2
depth = -msg.altitude # MSL altitude (GPS + baro fusion)
```

**After (Fixed):**
```python
# [DONE] CORRECT: Use SCALED_PRESSURE sensor
if not self.surface_pressure_calibrated:
 self.surface_pressure = msg.press_abs # Auto-calibrate

depth = (msg.press_abs - self.surface_pressure) * 0.01 # meters
```

**Impact:** Accurate depth readings for underwater control

---

## Command Dispatch Flow

### From User Input to Motor Commands

```

 Command Sources

 Runner CLI: "forward 50% 5s"
 Mission File: missions/pool_test.txt
 Planner: YASMIN state machine
 Vision: Visual servoing alignment

 DriverCommand message

 CommandParser (mavlink_runner)

 Parse syntax:

 INPUT: "forward 50% 5s wait"

 PARSED:
 direction: "forward" → surge
 speed: "50%" → 50
 duration: "5s" → 5.0
 modifiers: ["wait"] → wait_for_convergence=True

 Publish to /driver/command

 InspectorNode.on_driver_command()

 Command Validation:
 Check if armed (if not in UNARMED_ALLOWED list)
 Validate speed (0-100%)
 Validate duration (>0)
 Dynamic dt calculation (not fixed 0.05s)

 Dispatch to movement handler

 MovementCommands Registry (decorator-based)

 @register("forward", aliases=["f", "fwd"])
 def handle_forward(self, cmd):
 # Apply gain scheduling
 gains = self.gain_scheduler.get_gains(cmd.speed)

 # Cascade control: Position → Velocity → Thrust
 target_velocity = self.cascade.compute(...)

 # RC ramping: Smooth acceleration
 pwm = self.rc_controller.apply_ramp(...)

 # Check convergence
 if self.convergence_gate.is_settled():
 return CommandComplete()

 RC PWM values

 RCController.send_rc_override()

 Thread-safe PWM update:

 with self._lock: # [DONE] Bug #5 fix
 self._ramped[channel] = pwm_value

 Convert to PWM:
 -100% → 1100 μs
 0% → 1500 μs (neutral)
 +100% → 1900 μs

 @ 20Hz (every 50ms)

 MAVLink RC_CHANNELS_OVERRIDE

 message.target_system = 1 (Pixhawk)
 message.target_component = 1 (Autopilot)
 message.chan1_raw = 1650 (Surge forward)
 message.chan2_raw = 1500 (Sway neutral)
 message.chan3_raw = 1450 (Heave down)
 message.chan4_raw = 1550 (Yaw right)
 message.chan5_raw = 1500 (Roll neutral)
 message.chan6_raw = 1500 (Pitch neutral)
 message.chan7_raw = 0 (Unused)
 message.chan8_raw = 0 (Unused)

 Serial 115200 baud

 ArduSub Firmware
 Receives RC override
 Watchdog: Timeout if no message for 500ms
 Motor mixing: 8 ESC channels

```

---

## Safety & Watchdog Systems

### Multi-Layer Failsafe Architecture

```

 Layer 1: ArduSub Firmware Watchdogs

 GCS Heartbeat Watchdog (Bug #1 fix)
 Requires HEARTBEAT @ 2Hz (every 500ms)
 If missing: Trigger GCS failsafe
 Inspector sends: 2Hz (10x margin)

 RC Override Watchdog (Bug #2 verified)
 Requires RC_CHANNELS_OVERRIDE continuously
 Timeout: 500ms (configurable)
 If missing: Revert to manual control
 Inspector sends: 20Hz (every 50ms, 10x faster)

 Layer 2: Inspector Safety Checks

 Telemetry Watchdog (Bug #27 enhancement)
 Track 5 message timestamps:
 - last_ahrs2_time
 - last_pressure_time
 - last_imu_time
 - last_battery_time
 - last_gps_time
 Check every 1Hz: Warn if >2s stale
 Publish diagnostics to /mavlink/diagnostics

 Parameter Validation (Bug #28 enhancement)
 On startup: Validate all parameters
 PIDs: Kp/Ki/Kd must be >= 0
 PWM: limits within 1000-2000 μs
 Rates: update_rate > 0
 Timeouts: watchdog_timeout > 0
 Fail-fast: Exit with error if invalid

 Arm State Check
 Most commands require armed state
 UNARMED_ALLOWED: mode, arm, disarm, etc.
 Reject commands if disarmed (log warning)

 Layer 3: Control Safety Features

 PID Anti-Windup
 Clamp integral term to prevent saturation
 Max integral: Configurable per PID
 Reset integral on mode change

 PWM Limits (hard-coded safety)
 Absolute min: 1100 μs
 Absolute max: 1900 μs
 Clamp all outputs before sending

 Emergency Neutral (RC watchdog timeout)
 If no RC success for >500ms:
 - Send all channels to 1500 μs (neutral)
 - Log critical warning
 - Attempt reconnection

 Layer 4: Thread Safety (Bug #5 fix)

 RCController Thread Safety
 self._lock = threading.Lock()
 All 6 _ramped dict accesses wrapped:
 1. apply_ramp() read
 2. apply_ramp() write
 3. send_rc_override() read
 4. emergency_neutral() write
 5. stop_all() write
 6. reset() write

```

### Watchdog Timing Verification

| Watchdog | ArduSub Requirement | Inspector Implementation | Safety Margin |
|----------|---------------------|--------------------------|---------------|
| **GCS Heartbeat** | Must receive every ~1s | Send @ 2Hz (500ms) | **2x faster** [DONE] |
| **RC Override** | Must receive every 500ms | Send @ 20Hz (50ms) | **10x faster** [DONE] |
| **Telemetry Stale** | N/A | Warn if >2s | Detect sensor failures |

---

## Convergence & Settling

### Convergence Gate Logic (V2 Phase 1)

Prevents vehicle from advancing to next command while still moving/settling:

```

 ConvergenceGate.is_settled()

 Criteria (all must be true for settling_duration):

 1. Position Error < threshold
 Depth: < 0.1 m
 Yaw: < 5°
 XY position: < 0.2 m

 2. Velocity < threshold
 Surge/sway/heave: < 0.1 m/s
 Yaw rate: < 5°/s

 3. Steady State Duration
 Must maintain above criteria for 0.5s
 Reset timer if criteria violated

Example Timeline:

t=0.0s Command: "forward 50% 3s"
 Start moving forward
 RC ramp up to 50% over ~0.5s

t=3.0s Duration complete
 Check convergence gate
 Still moving at 0.3 m/s (too fast!)
 Gate NOT settled

t=3.5s Active braking engaged
 RC controller applies reverse thrust
 Velocity reducing: 0.2 m/s → 0.1 m/s

t=4.0s Braking complete
 Velocity: 0.05 m/s (below threshold)
 Position error: 0.08 m (below threshold)
 Start settling timer

t=4.5s Settling duration complete (0.5s)
 Gate is_settled() returns True
 Publish CommandComplete
 Advance to next command

```

### Movement Modifiers

| Modifier | Effect | Example |
|----------|--------|---------|
| **wait** | Enable convergence gate | `forward 50% 3s wait` |
| **nowait** | Skip convergence check (default) | `forward 50% 3s nowait` |
| **brake** | Active braking at end | `forward 50% 3s brake` |
| **coast** | Let vehicle coast (no brake) | `forward 50% 3s coast` |

**Default behavior:** No waiting, active braking enabled

---

## Key Technical Details

### Gravity Compensation (Bug #4 Fix)

**Problem:** IMU measures total acceleration (gravity + motion). At 30° pitch, gravity contributes 4.9 m/s² to surge axis → 49 m/s drift over 10s.

**Solution:** Rotate gravity vector using quaternion, subtract from body-frame acceleration:

```python
# Quaternion from AHRS2
w, x, y, z = attitude.quaternion

# Rotate gravity to body frame
g_x = 2 * (x*z - w*y) * 9.81 # Surge component
g_y = 2 * (y*z + w*x) * 9.81 # Sway component
g_z = (w*w - x*x - y*y + z*z) * 9.81 # Heave component

# Subtract gravity from measured acceleration
a_motion_x = a_body_x - g_x
a_motion_y = a_body_y - g_y
a_motion_z = a_body_z - g_z

# Integrate to get velocity
v_x += a_motion_x * dt
v_y += a_motion_y * dt
v_z += a_motion_z * dt
```

**Impact:** Eliminates velocity drift during pitch/roll maneuvers [DONE]

### ZUPT (Zero-Velocity Update) Correction

**Purpose:** Mitigate IMU drift when vehicle is stationary

**Logic:**
```python
# If acceleration is near zero AND velocity is low
if abs(a_x) < 0.5 and abs(v_x) < 0.1:
 # Assume vehicle is stationary
 # Decay velocity toward zero
 v_x *= 0.9 # 10% decay per update
```

**Thresholds (Bug #18 fix):**
- Acceleration threshold: 0.02 → **0.5 m/s²** (less aggressive)
- Velocity threshold: 0.1 m/s
- Decay window: 1.0 s

### Per-DOF Cascade Control (Bug #9 Fix)

**Problem:** Shared integral terms caused cross-DOF contamination (surge command affected sway integral).

**Solution:** Separate integral dictionaries per DOF:

```python
class CascadeController:
 def __init__(self):
 self.position_integrals = {
 'surge': 0.0,
 'sway': 0.0,
 'heave': 0.0
 }
 self.velocity_integrals = {
 'surge': 0.0,
 'sway': 0.0,
 'heave': 0.0
 }

 def compute_thrust(self, dof, target_pos, current_pos, current_vel):
 # Position loop
 pos_error = target_pos - current_pos
 self.position_integrals[dof] += pos_error * dt
 target_vel = Kp * pos_error + Ki * self.position_integrals[dof]

 # Velocity loop
 vel_error = target_vel - current_vel
 self.velocity_integrals[dof] += vel_error * dt
 thrust = Kp * vel_error + Ki * self.velocity_integrals[dof]

 return thrust
```

**Impact:** No cross-axis coupling, independent DOF control [DONE]

---

## See Also

- [V2 Bug Fix Completion Report](../roadmap/bugfix-completion-report.md)
- [Control Stack V2 Design](../design-decisions/control-stack-v2.md)
- [MAVLink Deep Dive](../reference/mavlink-deep-dive.md)
- [Pool Testing Checklist](../guides/pool-testing/next_things_to_check.md)

---

**Last Updated:** April 2026
**Status:** [DONE] Production Ready
**Build:** 10/10 packages, 4.10s, zero errors
