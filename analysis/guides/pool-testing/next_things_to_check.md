# Next Things to Check - Phase 1-5 Testing Guide

> **Purpose**: Systematic testing checklist for Control Redesign V2 features.
> Each feature maps to specific commands and measurable outcomes.

---

## Pre-Test Requirements

### Environment Setup
- [ ] Pool water clear and calm
- [ ] Vehicle battery >80%
- [ ] All sensors connected and verified
- [ ] ROS2 nodes running (`ros2 node list` shows all expected nodes)
- [ ] Baseline mission working (forward 30% 5s, turn 90°)

### Enable Features (One at a Time)
```bash
# Phase 1: Convergence
ros2 param set /mavlink_inspector convergence_enabled true

# Phase 2: Rotate-in-place
ros2 param set /mavlink_inspector rotate_in_place_enabled true

# Phase 3: Cascade control (experimental)
ros2 param set /mavlink_inspector cascade_enabled true

# Phase 4: Gain scheduling
ros2 param set /mavlink_inspector gain_scheduling_enabled true
ros2 param set /mavlink_inspector accel_limiting_enabled true

# Phase 5: DVL/External sensors
ros2 param set /mavlink_inspector dvl_enabled true
ros2 param set /mavlink_inspector external_yaw_enabled true
```

---

## Phase 1: Velocity Estimation & Convergence Gates

### Feature: Stop Detection & Convergence
**Problem Solved**: AUV coasts past target, next command starts while still moving

### Test 1.1: Basic Stop Convergence
**Command**: 
```
forward 30% 5s
```

**What to Check**:
- [ ] Vehicle stops completely before accepting next command
- [ ] Log shows: "Convergence: waiting for velocity < 0.05 m/s"
- [ ] Log shows: "Convergence: achieved in X.XXs"

**Expected Values**:
- Convergence time: 0.3-1.0 seconds
- Final velocity: <0.05 m/s (from logs)

**If Failing**:
- Increase `convergence_velocity_threshold` (more lenient)
- Decrease `convergence_settling_time` (faster pass)
- Check IMU data flowing (log shows SCALED_IMU2 updates)

### Test 1.2: Chain Commands Without Drift
**Commands**:
```
forward 30% 3s
back 30% 3s
forward 30% 3s
```

**What to Check**:
- [ ] Vehicle returns close to starting position
- [ ] Each command waits for previous to stabilize
- [ ] No cumulative drift visible

**Expected Values**:
- Final position: within 0.5m of start
- Each convergence logged

---

## Phase 2: Rotate-in-Place & Precision Yaw

### Feature: Sharp 90° Corners (Not U-turns)
**Problem Solved**: Yaw changes drift like a car making U-turns

### Test 2.1: Basic 90° Turn
**Command**:
```
turn left 90 50%
```

**What to Check**:
- [ ] Vehicle rotates without translating forward
- [ ] Turn looks like rotation on axis (not arc)
- [ ] Log shows three phases: "Stopping translation", "Rotating", "Stabilizing"

**Expected Values**:
- Final heading error: ±3° (measure manually or from telemetry)
- No forward/lateral drift during turn
- Turn time: 2-4 seconds

**If Failing**:
- Increase `yaw_precision_deadband` (more tolerance)
- Decrease turn speed (less aggressive)
- Check translation lockout working (surge/sway should be neutral)

### Test 2.2: Precision Zone Behavior
**Command**:
```
turn_to 180 50%  # Absolute heading
```

**What to Check**:
- [ ] Normal gains at >5° error
- [ ] Reduced gains at 1-5° error (precision zone)
- [ ] Settling timer at <1° error (final zone)
- [ ] Log shows: "Yaw precision zone: normal → precision → final"

### Test 2.3: Square Pattern (Key Test)
**Commands**:
```
forward 30% 4s
turn left 90 50%
forward 30% 4s
turn left 90 50%
forward 30% 4s
turn left 90 50%
forward 30% 4s
turn left 90 50%
```

**What to Check**:
- [ ] 4 sharp 90° corners visible
- [ ] Not 4 U-turn arcs
- [ ] Vehicle returns close to starting position

**Measure** (from overhead video/observation):
- Each corner angle: ±5° of 90°
- Closure error: <1.0m (distance from start to end)

**Success Criteria**:
```
Corner 1: 90° ± 5° ✓
Corner 2: 90° ± 5° ✓
Corner 3: 90° ± 5° ✓
Corner 4: 90° ± 5° ✓
Closure error: <1.0m ✓
```

---

## Phase 3: Cascade Control & Position Estimation

> **Note**: Phase 3 is experimental. Enable only after Phase 1-2 validated.

### Feature: Distance-Based Movement
**Problem Solved**: Time-based commands have ±1m variance

### Test 3.1: Single Distance Movement
**Setup**: Enable cascade control
```bash
ros2 param set /mavlink_inspector cascade_enabled true
```

**Code/Command** (via API):
```python
# Reset origin
node._position_estimator.reset_origin()

# Move forward 2 meters
success = handler.move_distance_cascade('surge', 2.0, max_speed_pct=30)
```

**What to Check**:
- [ ] Vehicle moves approximately 2 meters
- [ ] Log shows position updates at 20 Hz
- [ ] Log shows convergence when target reached

**Expected Values**:
- Actual distance: 2.0m ± 0.3m
- Convergence: position_error < 0.1m

### Test 3.2: Repeatability Test
**Procedure**:
1. Mark starting position
2. Run same distance command 5 times
3. Measure actual distance each time

**Expected Values**:
```
Run 1: 2.05m
Run 2: 1.98m
Run 3: 2.02m
Run 4: 1.95m
Run 5: 2.01m
Mean: 2.00m
StdDev: 0.04m (target: <0.15m)
```

---

## Phase 4: Gain Scheduling & Acceleration Limiting

### Feature: Reliable High-Speed Operation
**Problem Solved**: 70%+ speed causes overshoots and misses

### Test 4.1: Gain Scheduling Transitions
**Command**: 
```
forward 70% 5s  # High speed range
```

**What to Check**:
- [ ] Log shows: "Gain schedule: medium → high"
- [ ] No overshoots or oscillations
- [ ] Yaw stays stable during forward motion

**Then**:
```
forward 20% 3s  # Low speed range
```

**What to Check**:
- [ ] Log shows: "Gain schedule: high → low"
- [ ] Vehicle responsive (not sluggish)

### Test 4.2: Acceleration Ramp
**Command**:
```
forward 80% 5s  # From stop
```

**What to Check**:
- [ ] Vehicle accelerates smoothly (not instant jump)
- [ ] Log shows ramping: 0% → 10% → 20% → ... → 80%
- [ ] Time to reach 80%: ~1.6 seconds (at 50%/sec)

**If Too Slow**:
```bash
ros2 param set /mavlink_inspector max_accel_pct_per_sec 75.0
```

**If Unstable**:
```bash
ros2 param set /mavlink_inspector max_accel_pct_per_sec 30.0
```

### Test 4.3: High-Speed Turn
**Command**:
```
turn left 90 70%  # High speed turn
```

**What to Check**:
- [ ] Gains automatically reduced (high-speed set)
- [ ] Turn completes without overshoot
- [ ] Final heading error: ±5°

**Baseline Comparison**:
- Without gain scheduling: ±15° error
- With gain scheduling: ±5° error (3x improvement)

### Test 4.4: High-Speed Square
**Commands**:
```
forward 70% 3s
turn left 90 70%
forward 70% 3s
turn left 90 70%
forward 70% 3s
turn left 90 70%
forward 70% 3s
```

**What to Check**:
- [ ] All 4 corners hit (within ±5°)
- [ ] Smooth acceleration throughout
- [ ] No visible oscillations

---

## Phase 5: Multi-Source Sensors (DVL + External)

> **Note**: Requires DVL or external compass hardware connected

### Feature: Ground-Truth Velocity from DVL
**Problem Solved**: IMU integration drifts over time

### Test 5.1: DVL Connection
**Setup**:
```bash
ros2 param set /mavlink_inspector dvl_enabled true
ros2 param set /mavlink_inspector dvl_topic '/dvl/velocity'
```

**What to Check**:
- [ ] Log shows: "DVL subscribed to /dvl/velocity"
- [ ] `ros2 topic hz /dvl/velocity` shows data flowing
- [ ] Sensor manager reports: `get_status()['dvl'] = 'healthy'`

### Test 5.2: DVL Velocity vs IMU
**Procedure**:
1. Start with DVL enabled
2. Move vehicle forward
3. Compare DVL velocity to IMU estimate

**What to Check**:
- [ ] DVL reading stable (no drift)
- [ ] IMU estimate drifts over time
- [ ] Manager uses DVL when available

### Test 5.3: Automatic Fallback
**Procedure**:
1. Block DVL (hand over sensor, lift out of water)
2. Observe source switch

**What to Check**:
- [ ] Log shows: "Velocity source: dvl → imu_estimate"
- [ ] Control continues without interruption
- [ ] Manager status shows: `dvl: 'timeout'`

### Test 5.4: External Yaw Source
**Setup**:
```bash
ros2 param set /mavlink_inspector external_yaw_enabled true
ros2 param set /mavlink_inspector external_yaw_topic '/witmotion/yaw'
```

**What to Check**:
- [ ] Log shows: "ExternalYawSource initialized"
- [ ] Yaw readings match Pixhawk (roughly)
- [ ] Priority: External > Pixhawk (when valid)

### Test 5.5: Yaw Source Comparison
**Procedure**:
1. Enable all yaw sources
2. Spin vehicle slowly
3. Log all yaw readings

**Code**:
```python
yaws = sensor_manager.get_all_yaws()
print(f"Pixhawk: {yaws['pixhawk']:.1f}°")
print(f"External: {yaws.get('external', 'N/A')}°")
print(f"DVL IMU: {yaws.get('dvl_imu', 'N/A')}°")
```

**What to Check**:
- [ ] All sources track same heading
- [ ] Offset is consistent (calibration needed)
- [ ] No sudden jumps in any source

---

## Parameter Tuning Reference

### Phase 1 Parameters
| Parameter | Default | Increase If | Decrease If |
|-----------|---------|-------------|-------------|
| convergence_velocity_threshold | 0.05 | Never converges | Converges too easily |
| convergence_settling_time | 0.2 | False positives | Takes too long |
| convergence_timeout | 5.0 | Times out often | Safety margin too high |

### Phase 2 Parameters
| Parameter | Default | Increase If | Decrease If |
|-----------|---------|-------------|-------------|
| yaw_precision_deadband | 5.0 | Oscillates near target | Enters precision too late |
| yaw_final_deadband | 1.0 | Never settles | Settles too easily |
| yaw_settling_time | 0.5 | Declares done early | Takes too long |

### Phase 4 Parameters
| Parameter | Default | Increase If | Decrease If |
|-----------|---------|-------------|-------------|
| yaw_gains_high_kp | 1.2 | Sluggish at high speed | Overshoots |
| max_accel_pct_per_sec | 50 | Acceleration slow | Causes instability |

---

## Common Issues & Solutions

### Issue: Vehicle never converges
**Symptoms**: Convergence gate times out every time
**Solutions**:
1. Check IMU data: `ros2 topic echo /mavlink_inspector/telemetry`
2. Increase threshold: `convergence_velocity_threshold: 0.1`
3. Check ZUPT working: Vehicle should zero velocity when stopped

### Issue: Turns look like U-turns
**Symptoms**: Vehicle arcs instead of rotating in place
**Solutions**:
1. Verify `rotate_in_place_enabled: true`
2. Check translation lockout: Surge/sway should be neutral during turn
3. Reduce turn speed (less momentum)

### Issue: High-speed overshoots
**Symptoms**: Turns miss target by 10-20° at 70%+ speed
**Solutions**:
1. Enable gain scheduling: `gain_scheduling_enabled: true`
2. Reduce high-speed gains: `yaw_gains_high_kp: 0.8`
3. Enable acceleration limiting

### Issue: DVL not detected
**Symptoms**: Status shows 'not_available'
**Solutions**:
1. Check topic exists: `ros2 topic list | grep dvl`
2. Check message type matches config
3. Verify DVL has bottom lock (minimum altitude)

---

## Agent Handoff Instructions

> For AI agents working on this codebase:

### Understanding the Architecture
1. Read `velocity_control.py` - Contains all Phase 1-4 control classes
2. Read `sensor_sources.py` - Contains all Phase 5 sensor abstraction
3. Read `command_handler.py` - Command dispatch and helper methods
4. Read `inspector_node.py` - Main orchestrator, wiring, parameters

### Key Files
```
src/mavlink_inspector/mavlink_inspector/
├── velocity_control.py     # VelocityEstimator, ConvergenceGate, CascadeController, etc.
├── sensor_sources.py       # DVLSource, ExternalYawSource, SensorSourceManager
├── command_handler.py      # Command dispatch, convergence helpers
├── movement_commands.py    # Movement command definitions
├── inspector_node.py       # Main node, parameters, wiring
└── config/defaults.yaml    # All configuration parameters
```

### Testing Changes
1. Enable feature via parameter
2. Run specific test from this document
3. Check logs for expected messages
4. Measure outcome (heading error, distance, etc.)
5. If failing, check "Solutions" section

### Adding New Features
1. Add class to appropriate module (velocity_control.py or sensor_sources.py)
2. Add parameters to defaults.yaml
3. Declare parameters in inspector_node.py
4. Wire in inspector_node.py initialization
5. Add helper method to command_handler.py if needed
6. Update this testing guide

### Disabling Features (If Broken)
```bash
ros2 param set /mavlink_inspector [feature]_enabled false
```
All features have master enable switches - disable individually to isolate issues.

---

## Test Results Log

### Date: ___________
### Tester: ___________

| Test | Result | Notes |
|------|--------|-------|
| 1.1 Stop Convergence | ✓/✗ | |
| 1.2 Chain Commands | ✓/✗ | |
| 2.1 Basic 90° Turn | ✓/✗ | |
| 2.2 Precision Zone | ✓/✗ | |
| 2.3 Square Pattern | ✓/✗ | Closure: ___m |
| 3.1 Distance Movement | ✓/✗ | Accuracy: ±___m |
| 4.1 Gain Scheduling | ✓/✗ | |
| 4.2 Accel Ramp | ✓/✗ | |
| 4.3 High-Speed Turn | ✓/✗ | Error: ±___° |
| 5.1 DVL Connection | ✓/✗ | |
| 5.3 Fallback | ✓/✗ | |

### Parameters After Tuning
```yaml
# Record final tuned values here
convergence_velocity_threshold: 
yaw_precision_deadband:
yaw_gains_high_kp:
max_accel_pct_per_sec:
```
