# Pool Testing Guide - V2 Control Stack

## Overview

This guide provides procedures to validate all V2 control improvements and bug fixes completed in April 2026.

**What's New in V2:**
- [DONE] Gravity-compensated velocity estimation
- [DONE] Cascade position/velocity control
- [DONE] Active braking and smooth ramping
- [DONE] 30 bug fixes (MAVLink, sensors, threading, control)

**Testing Goals:**
1. Verify all V2 improvements work in real-world pool environment
2. Validate bug fixes are effective
3. Tune control parameters for optimal performance
4. Build confidence for competition deployment

## Quick Links to Existing Guides

| Guide | Use When |
|-------|----------|
| [setup-checklist.md](setup-checklist.md) | Arriving at pool |
| [network-config.md](network-config.md) | Network issues |
| [startup-sequence.md](startup-sequence.md) | Starting system |
| [tuning-guide.md](tuning-guide.md) | Adjusting PID/ramp |
| [troubleshooting.md](troubleshooting.md) | Something breaks |
| [test-procedures.md](test-procedures.md) | What to test |

## Prerequisites

### Hardware Setup
- [ ] BlueROV2/AUV fully assembled
- [ ] Tether connected and tested
- [ ] Battery charged (>14.0V recommended)
- [ ] ArduSub firmware updated to compatible version
- [ ] Pixhawk booted and connected via MAVLink
- [ ] All thrusters operational (6-8 thrusters depending on configuration)
- [ ] Cameras functional (if using vision systems)

### Software Setup
```bash
# Build workspace
cd ~/ROS_workspaces/Duburi_ws
colcon build
source install/setup.bash

# Verify connection
ros2 launch mavlink_inspector inspector.launch.py

# In another terminal, check topics
ros2 topic list | grep duburi

# Expected topics:
# /duburi/state
# /duburi/diagnostics
# /duburi/control_command
# /duburi/manual_override
```

### Safety Checks
- [ ] Emergency stop procedure reviewed with all team members
- [ ] Tether length marked and tested (know maximum operating radius)
- [ ] Pool depth measured (minimum 2m recommended for depth tests)
- [ ] Recovery procedures practiced (how to retrieve AUV if stuck)
- [ ] First aid kit available poolside
- [ ] Multiple operators present (never test alone)
- [ ] Communication protocol established (hand signals if needed)

### Calibration Required
```bash
# Verify sensor calibration before testing
ros2 topic echo /duburi/diagnostics

# Check:
# - IMU calibration status
# - Barometer calibration (depth sensor)
# - Compass calibration (magnetometer)
# - Thruster calibration (ESC endpoints)
```

## Testing Phases

### Phase 1: Connection & Telemetry Validation (10 minutes)
Verify MAVLink communication and sensor readings.

**Purpose:** Ensure all systems communicate properly before any movement.

### Phase 2: Basic Maneuvers (20 minutes)
Test fundamental movement commands.

**Purpose:** Validate individual thruster control and basic motion primitives.

### Phase 3: Stability & Holding (15 minutes)
Validate depth hold and yaw hold.

**Purpose:** Test PID controllers and station-keeping capabilities.

### Phase 4: Mission Execution (30 minutes)
Run structured missions with multiple waypoints.

**Purpose:** Validate integrated control stack under realistic mission scenarios.

### Phase 5: Emergency Procedures (10 minutes)
Test failsafes and recovery.

**Purpose:** Ensure safety systems work correctly.

### Phase 6: Advanced Testing (Optional, 20 minutes)
State machine integration, DVL testing, vision-based tasks.

**Purpose:** Test advanced autonomous behaviors.

## Detailed Test Procedures

### Phase 1: Connection & Telemetry Validation

**Test 1.1: Heartbeat and Connection**
```bash
# Launch inspector in one terminal
ros2 launch mavlink_inspector inspector.launch.py

# In another terminal, monitor diagnostics (should show 2Hz heartbeat)
ros2 topic echo /duburi/diagnostics | grep heartbeat

# Expected: heartbeat_rate: 2.0 (NOT 1.0 - this was a V1 bug)
# Expected: heartbeat_health: "HEALTHY"
```

**Success Criteria:**
- Heartbeat rate is 2.0 Hz ± 0.1 Hz
- No timeout warnings in logs
- Connection status shows "HEALTHY"

**Test 1.2: Depth Sensor**
```bash
# Monitor depth (should read 0.0m on surface, or slight negative due to air pressure)
ros2 topic echo /duburi/state | grep depth

# Submerge AUV 0.5m - depth should read ~0.5m
# Submerge AUV 1.0m - depth should read ~1.0m

# Bug fix verification: Now uses SCALED_PRESSURE (accurate), not AHRS2 altitude
```

**Success Criteria:**
- Surface depth reads 0.0 ± 0.1m
- Depth reading tracks physical depth within ±0.05m
- No sudden jumps or outliers in readings

**Test 1.3: IMU and Attitude**
```bash
# Monitor attitude while AUV is on surface
ros2 topic echo /duburi/state | grep -E "pitch|roll|yaw"

# Expected: pitch ≈ 0°, roll ≈ 0° (within ±5° for surface waves)

# Tilt AUV 30° in pitch - pitch should read ~30°
# Rotate AUV 90° in yaw - yaw should change by ~90°

# Bug fix verification: Gravity compensation prevents velocity drift during tilting
```

**Success Criteria:**
- Attitude readings are stable (not oscillating wildly)
- Attitude changes match physical orientation within ±5°
- No drift in attitude when vehicle is stationary

**Test 1.4: RC Override Watchdog**
```bash
# Monitor logs for RC watchdog status
ros2 topic echo /rosout | grep -i "rc.*watchdog"

# Expected: NO timeout warnings (20Hz continuous sending implemented)
# Expected: "RC override watchdog healthy" or similar

# Also check diagnostics
ros2 topic echo /duburi/diagnostics | grep rc_override
```

**Success Criteria:**
- No RC watchdog timeout warnings
- RC override messages sent at 20Hz
- Watchdog timer does not trigger during normal operation

**Test 1.5: Control Mode Verification**
```bash
# Check current control mode
ros2 topic echo /duburi/state | grep mode

# Try switching modes (requires ArduSub compatibility)
# Expected modes: MANUAL, STABILIZE, DEPTH_HOLD, etc.
```

### Phase 2: Basic Maneuvers

**Test 2.1: Forward/Backward Movement**
```bash
# Run pre-made test mission
ros2 run mavlink_runner runner analysis/guides/pool-testing/test-missions/test_basic.txt

# Watch the AUV:
# 1. Should accelerate smoothly (ramping enabled)
# 2. Moves forward for 5 seconds
# 3. Stops with minimal coasting (active braking)
# 4. Moves backward for 5 seconds
# 5. Stops precisely

# Monitor velocity during test
ros2 topic echo /duburi/state | grep -E "vx|vy|vz"
```

**Success Criteria:**
- Smooth acceleration (no jerky starts)
- Movement in expected direction
- Stops within 0.5m of commanded position (active braking works)
- No drift after stop command
- Velocity drops to <0.05 m/s within 2 seconds of stop

**Test 2.2: Lateral Movement**
```bash
# Test lateral thrusters
cat > test_lateral.txt << 'EOF'
arm
mode MANUAL
right 30% 5s
delay 2
left 30% 5s
delay 2
stop
disarm
EOF

ros2 run mavlink_runner runner test_lateral.txt

# Watch for lateral movement (starboard/port)
```

**Success Criteria:**
- AUV moves laterally without significant forward/backward drift
- Symmetrical movement (right and left distances similar)
- Stops without overshoot

**Test 2.3: Vertical Movement**
```bash
# Test depth control thrusters
cat > test_vertical.txt << 'EOF'
arm
mode MANUAL
down 30% 3s
delay 2
up 30% 3s
delay 2
stop
disarm
EOF

ros2 run mavlink_runner runner test_vertical.txt

# Monitor depth
ros2 topic echo /duburi/state | grep depth
```

**Success Criteria:**
- Smooth vertical movement
- Depth reading changes appropriately
- Returns to approximately starting depth

**Test 2.4: Yaw/Heading Changes**
```bash
# Use pre-made yaw test mission
cat > test_yaw.txt << 'EOF'
arm
mode MANUAL
turn right 90 50%
delay 2
turn left 90 50%
delay 2
~turn 180 50% # Relative turn
delay 2
stop
disarm
EOF

ros2 run mavlink_runner runner test_yaw.txt

# Monitor heading
ros2 topic echo /duburi/state | grep yaw
```

**Success Criteria:**
- Sharp 90° turns (no U-turn drift - this was a V1 bug)
- Settles at target heading within ±5°
- Bug fix verification: Convergence gates ensure precision
- Relative turn executes correctly

### Phase 3: Stability & Holding

**Test 3.1: Depth Hold**
```bash
# Run depth hold test
ros2 run mavlink_runner runner analysis/guides/pool-testing/test-missions/test_stability.txt

# In parallel, monitor depth stability
ros2 topic echo /duburi/state | grep depth > depth_log.txt

# Let run for 60 seconds, then analyze
```

**Success Criteria:**
- Depth stays within 1.0 ± 0.05m for 60 seconds
- No oscillations (well-tuned PID)
- Handles surface waves/disturbances
- No steady-state error (integral term working)

**Test 3.2: Yaw Hold**
```bash
# Test yaw stability
cat > test_yaw_hold.txt << 'EOF'
arm
mode MANUAL
turn right 90 50%
delay 1
hold yaw 60s
stop
disarm
EOF

ros2 run mavlink_runner runner test_yaw_hold.txt

# Monitor yaw
ros2 topic echo /duburi/state | grep yaw > yaw_log.txt
```

**Success Criteria:**
- Yaw stays at target ± 2° for 60 seconds
- Compensates for tether pull and water currents
- No drift over time

**Test 3.3: Combined Hold (Depth + Yaw)**
```bash
# Test both controllers simultaneously
cat > test_combined_hold.txt << 'EOF'
arm
mode MANUAL
dive 1.0m 50%
delay 1
turn right 45 50%
delay 1
hold depth 30s
hold yaw 30s
surface
stop
disarm
EOF

ros2 run mavlink_runner runner test_combined_hold.txt
```

**Success Criteria:**
- Both depth and yaw held simultaneously
- Controllers don't interfere with each other
- Stable station-keeping

### Phase 4: Mission Execution

**Test 4.1: Square Pattern**
```bash
# Run square pattern mission
ros2 run mavlink_runner runner analysis/guides/pool-testing/test-missions/test_square.txt

# Place visual marker at starting position
# After mission, measure return accuracy
```

**Success Criteria:**
- Returns to starting position within 0.5m
- Square is approximately square (not parallelogram)
- Bug fix verification: Gravity compensation prevents drift accumulation
- Each side is approximately equal length

**Test 4.2: Depth Profile Mission**
```bash
# Run depth profile test
ros2 run mavlink_runner runner analysis/guides/pool-testing/test-missions/test_depth_profile.txt

# Monitor depth transitions
ros2 bag record /duburi/state /duburi/diagnostics
```

**Success Criteria:**
- Smooth transitions between depth levels
- No overshoot at each depth
- Holds each depth stably for commanded duration
- Total time matches expected mission duration

**Test 4.3: Complex Navigation**
```bash
# Create a more complex mission
cat > test_complex.txt << 'EOF'
arm
mode MANUAL
# Navigate to waypoint 1
forward 50% 10s
turn right 45 50%
delay 1
# Navigate to waypoint 2
forward 50% 8s
dive 1.0m 50%
hold depth 10s
# Navigate to waypoint 3
turn left 90 50%
forward 50% 8s
# Return home
turn right 45 50%
forward 50% 10s
surface
stop
disarm
EOF

ros2 run mavlink_runner runner test_complex.txt
```

**Success Criteria:**
- Completes entire mission without intervention
- Maintains stability during all maneuvers
- Returns to approximately starting position

### Phase 5: Emergency Procedures

**Test 5.1: RC Watchdog Trigger**
```bash
# Start a long mission
ros2 run mavlink_runner runner analysis/guides/pool-testing/test-missions/test_square.txt

# During mission, disconnect MAVLink (unplug Ethernet or kill inspector)
# Watch AUV behavior
```

**Expected Behavior:**
- RC override watchdog triggers after 500ms
- AUV goes to neutral thrust (emergency_neutral behavior)
- Vehicle stops safely
- No runaway condition

**Success Criteria:**
- Watchdog triggers within 500ms
- All thrusters go to neutral (1500 PWM)
- Vehicle stabilizes safely

**Test 5.2: Manual Override**
```bash
# Start mission
ros2 run mavlink_runner runner analysis/guides/pool-testing/test-missions/test_square.txt &

# In another terminal, send manual override during mission
ros2 topic pub /duburi/manual_override std_msgs/Bool "data: true" --once

# Then take manual control via joystick or other interface
```

**Success Criteria:**
- Mission pauses/stops when override triggered
- Manual control is responsive
- Can resume or abort mission safely

**Test 5.3: Emergency Surface**
```bash
# Dive to depth, then trigger emergency surface
cat > test_emergency.txt << 'EOF'
arm
mode MANUAL
dive 1.5m 50%
delay 5
# Simulate emergency - full up thrust
up 100% 10s
stop
disarm
EOF

ros2 run mavlink_runner runner test_emergency.txt
```

**Success Criteria:**
- Vehicle surfaces quickly
- Controlled ascent (not out of control)
- Can stop mid-ascent if needed

### Phase 6: Advanced Testing (Optional)

**Test 6.1: DVL Integration (if available)**
```bash
# Enable DVL in configuration
# Run mission with DVL-based velocity feedback

# Monitor DVL data
ros2 topic echo /dvl/velocity

# Expected: Improved position accuracy with DVL
```

**Test 6.2: State Machine Autonomy**
```bash
# Load state machine mission
# Observe autonomous behavior

# Expected: State transitions work correctly
# Expected: Error handling and recovery functions
```

**Test 6.3: Vision-Based Tasks**
```bash
# Test gate detection, path following, etc.
# If vision pipeline is integrated
```

## Expected Results Summary

| Test | Success Criteria | V2 Improvement | Bug Fixed |
|------|------------------|----------------|-----------|
| Heartbeat | 2Hz rate | Was 1Hz | [DONE] Failsafe timing |
| Depth Reading | Accurate ±0.05m | Was using MSL altitude | [DONE] SCALED_PRESSURE |
| Forward Movement | No drift after stop | Active braking added | [DONE] Coasting eliminated |
| Yaw Turns | Sharp 90° turns | Convergence gates | [DONE] U-turn drift |
| Depth Hold | ±0.05m stability | Improved PID tuning | [DONE] Oscillations |
| Square Mission | Return within 0.5m | Gravity compensation | [DONE] Drift accumulation |
| RC Watchdog | Triggers in 500ms | 20Hz sending | [DONE] Timeout crashes |
| Velocity Estimate | No tilt-induced drift | Gravity compensation | [DONE] False motion |

## Troubleshooting

### AUV Drifts After Stop Command
**Symptoms:** Vehicle continues moving after stop command, coasts significantly.

**Possible Causes:**
- Active braking disabled
- Braking threshold too tight
- Convergence thresholds incorrect

**Solutions:**
1. Check configuration: `braking_enabled: true` in config file
2. Increase braking threshold: `braking_threshold: 0.1` (meters/radians)
3. Tune convergence parameters in defaults.yaml
4. Verify thrust ramping is enabled

### Depth Oscillates or Overshoots
**Symptoms:** AUV bounces up/down around target depth, never settles.

**Possible Causes:**
- PID gains too aggressive (high Kp or low Kd)
- Depth sensor noise
- Thruster deadzone not calibrated

**Solutions:**
1. Check depth PID gains in defaults.yaml:
 ```yaml
 depth_kp: 100.0 # Try reducing to 80.0
 depth_ki: 5.0
 depth_kd: 50.0 # Try increasing to 70.0
 ```
2. Verify SCALED_PRESSURE messages arriving: `ros2 topic hz /mavlink/messages/scaled_pressure`
3. Tune deadzone compensation for vertical thrusters
4. Add low-pass filter to depth readings if noisy

### Yaw Overshoots or Oscillates
**Symptoms:** Vehicle spins past target heading, oscillates back and forth.

**Possible Causes:**
- Yaw PID gains too aggressive
- Convergence threshold too tight
- Magnetic interference affecting compass

**Solutions:**
1. Check yaw PID gains:
 ```yaml
 yaw_kp: 2.0 # Try reducing to 1.5
 yaw_kd: 0.5 # Increase for more damping
 ```
2. Increase convergence threshold: `convergence_yaw_threshold: 5.0` (degrees)
3. Recalibrate compass away from magnetic interference
4. Check for loose wires or metal objects near Pixhawk

### RC Watchdog Triggers During Normal Operation
**Symptoms:** Watchdog triggers unexpectedly, vehicle stops mid-mission.

**Possible Causes:**
- MAVLink connection unstable
- Network congestion
- Inspector node crashing
- RC override send rate too low

**Solutions:**
1. Check MAVLink connection: `ros2 topic hz /mavlink/from`
2. Monitor diagnostics: `ros2 topic echo /duburi/diagnostics | grep connection`
3. Check network latency: `ping <pixhawk_ip>`
4. Verify no CPU throttling: `top` or `htop`
5. Check RC override send rate is 20Hz in code

### Vehicle Drifts During Depth Hold
**Symptoms:** Depth is stable, but vehicle drifts horizontally.

**Possible Causes:**
- No horizontal position control (expected without DVL/vision)
- Tether pull
- Water currents
- Asymmetric thruster output

**Solutions:**
1. This is expected behavior without DVL or visual odometry
2. Add tether management (slack line)
3. Orient vehicle into current
4. Verify all horizontal thrusters producing equal thrust
5. Consider adding DVL for position hold

### Sensors Reading NaN or Invalid Values
**Symptoms:** State topic shows NaN for depth, attitude, or velocity.

**Possible Causes:**
- Sensor not calibrated
- MAVLink message not being received
- Sensor hardware failure
- Message parsing error

**Solutions:**
1. Check sensor calibration in ArduSub
2. Monitor raw MAVLink messages: `ros2 topic list | grep mavlink`
3. Restart Pixhawk and re-establish connection
4. Check for error messages in logs: `ros2 topic echo /rosout | grep -i error`
5. Verify sensor connections and power

### Mission Aborts Unexpectedly
**Symptoms:** Mission stops partway through without completing.

**Possible Causes:**
- Safety limit reached (depth, distance, battery)
- Convergence timeout
- Node crash

**Solutions:**
1. Check safety limits in configuration
2. Review convergence timeout settings
3. Check logs for error messages
4. Verify battery voltage > 12V
5. Monitor node health: `ros2 node list` and `ros2 node info`

## Data Collection

**Log these metrics during testing:**
```bash
# Record all relevant topics
ros2 bag record /duburi/state /duburi/diagnostics /duburi/control_command /rosout -o pool_test_$(date +%Y%m%d_%H%M%S)

# Later, analyze:
ros2 bag info <bagfile>
ros2 bag play <bagfile> --topics /duburi/state
```

**Key metrics to log:**

1. **Depth Accuracy**
 - Target depth vs actual depth
 - Settling time to reach target
 - Steady-state error
 - Variance while holding

2. **Yaw Precision**
 - Target heading vs actual heading
 - Turn completion time
 - Overshoot amount
 - Hold variance

3. **Movement Accuracy**
 - Commanded distance vs actual distance
 - Stopping distance (overshoot)
 - Drift after stop
 - Path deviation

4. **Control Performance**
 - Control loop frequency
 - Thrust command range used
 - Convergence time
 - Number of convergence failures

5. **System Health**
 - MAVLink heartbeat rate
 - RC override send rate
 - Watchdog trigger count
 - Sensor update rates

**Analysis Scripts:**
```bash
# Extract depth data from bag
ros2 bag play <bagfile> --topics /duburi/state | grep depth > depth_analysis.txt

# Calculate statistics
python3 << 'EOF'
import numpy as np
# Load depth_analysis.txt
# Calculate mean, std, min, max
# Plot depth vs time
EOF
```

## Post-Testing Analysis

### 1. Review Bag Files
```bash
# Play back at different speeds
ros2 bag play <bagfile> --rate 0.5 # Half speed
ros2 bag play <bagfile> --rate 2.0 # Double speed

# Extract specific time windows
ros2 bag play <bagfile> --start-offset 10.0 --duration 30.0
```

### 2. Calculate Success Metrics
- **Mission success rate:** X% of missions completed without intervention
- **Position accuracy:** Mean error of Y meters
- **Depth accuracy:** Mean error of Z meters
- **Heading accuracy:** Mean error of W degrees
- **Control stability:** X% of time within convergence thresholds

### 3. Compare to V1 Baseline
If you have V1 test data, compare:
- Drift reduction (should be ~50% better with gravity compensation)
- Stopping accuracy (should be ~80% better with active braking)
- Stability (should have less oscillation with tuned PIDs)

### 4. Document Failures
For any failed tests:
- What was expected?
- What actually happened?
- Logs and data showing failure
- Suspected root cause
- Proposed fix

### 5. Update Configuration
Based on test results:
```yaml
# Example updated defaults.yaml
control:
 depth_kp: 90.0 # Reduced from 100.0 (was oscillating)
 yaw_kd: 0.7 # Increased from 0.5 (needed more damping)

convergence:
 depth_threshold: 0.08 # Relaxed from 0.05 (too tight for pool)
```

## Next Steps After Pool Testing

### Immediate Actions
- [ ] Review all bag files and extract metrics
- [ ] Update PID parameters based on results
- [ ] Fix any bugs discovered during testing
- [ ] Document vehicle-specific configuration
- [ ] Create competition-ready mission files

### Hardware Improvements
- [ ] Replace any failed thrusters or sensors
- [ ] Improve tether management
- [ ] Add DVL if position drift is significant
- [ ] Upgrade cameras if vision tasks needed
- [ ] Balance vehicle if asymmetric behavior observed

### Software Improvements
- [ ] Fine-tune control gains for competition pool
- [ ] Add any missing safety checks
- [ ] Optimize control loop frequency if needed
- [ ] Integrate additional sensors (DVL, vision)
- [ ] Implement state machine for autonomous missions

### Competition Preparation
- [ ] Design competition-specific missions
- [ ] Practice gate traversal
- [ ] Practice path following
- [ ] Practice object manipulation (if required)
- [ ] Develop backup strategies for sensor failures
- [ ] Train all team members on vehicle operation

### Documentation
- [ ] Create vehicle operation manual
- [ ] Document lessons learned
- [ ] Update safety procedures
- [ ] Create troubleshooting quick reference
- [ ] Record competition-ready configuration

## Safety Reminders

**ALWAYS:**
- [DONE] Test with multiple people present
- [DONE] Have emergency stop plan ready
- [DONE] Monitor battery voltage
- [DONE] Keep tether untangled
- [DONE] Know pool depth before diving
- [DONE] Have recovery tools ready (net, pole)

**NEVER:**
- Test alone
- Exceed safe depth limits
- Operate with low battery (<12V)
- Test in thunderstorm
- Leave vehicle unattended in water
- Run thrusters out of water

## Appendix: Quick Reference Commands

### Start Testing
```bash
# Terminal 1: Launch inspector
ros2 launch mavlink_inspector inspector.launch.py

# Terminal 2: Monitor state
ros2 topic echo /duburi/state

# Terminal 3: Run missions
ros2 run mavlink_runner runner <mission_file>
```

### Emergency Stop
```bash
# Kill all nodes
pkill -f ros2

# Or send disarm command
ros2 topic pub /duburi/emergency_stop std_msgs/Bool "data: true" --once
```

### Check System Health
```bash
ros2 topic hz /duburi/state # Should be ~10Hz
ros2 topic hz /mavlink/from # Should be ~50Hz+
ros2 topic echo /duburi/diagnostics # Check all health metrics
```

### Record Everything
```bash
ros2 bag record -a # Record ALL topics (large file)
# or
ros2 bag record /duburi/state /duburi/diagnostics /duburi/control_command
```

---

**Document Version:** 1.0
**Last Updated:** 2026-04-XX
**Author:** Duburi Team
**Status:** Ready for Pool Testing

**Good luck with your pool testing! **
