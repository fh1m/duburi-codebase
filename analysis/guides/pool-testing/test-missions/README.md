# Test Missions Reference

This directory contains pre-built mission files for pool testing. Each mission is designed to validate specific V2 control features.

## Available Missions

### test_basic.txt
**Duration:** ~15 seconds  
**Purpose:** Basic forward/backward movement with active braking validation  
**Tests:**
- Smooth thrust ramping
- Forward movement control
- Backward movement control
- Active braking (minimal coasting)
- Stop precision

**Run:**
```bash
ros2 run mavlink_runner runner analysis/guides/pool-testing/test-missions/test_basic.txt
```

**Success Criteria:**
- Smooth acceleration (no jerks)
- Stops within 0.5m of commanded position
- No drift after stop

---

### test_square.txt
**Duration:** ~40 seconds  
**Purpose:** Integrated navigation test with gravity compensation validation  
**Tests:**
- Four-sided square pattern
- Sharp 90° turns (no U-turn drift)
- Dead reckoning accuracy
- Gravity-compensated velocity estimation

**Run:**
```bash
ros2 run mavlink_runner runner analysis/guides/pool-testing/test-missions/test_square.txt
```

**Success Criteria:**
- Returns to starting position within 0.5m
- Square shape is approximately square (not parallelogram)
- All four sides are roughly equal length
- Turns are sharp (not gradual arcs)

---

### test_depth_profile.txt
**Duration:** ~60 seconds  
**Purpose:** Depth control and stability validation  
**Tests:**
- Depth transitions (0.5m, 1.0m, 1.5m)
- Depth hold stability
- SCALED_PRESSURE sensor accuracy
- Vertical PID control

**Run:**
```bash
ros2 run mavlink_runner runner analysis/guides/pool-testing/test-missions/test_depth_profile.txt
```

**Success Criteria:**
- Smooth depth transitions (no overshoot)
- Depth holds within ±0.05m at each level
- No oscillations during hold
- Total mission time matches expected

**Monitor depth during test:**
```bash
ros2 topic echo /duburi/state | grep depth
```

---

### test_stability.txt
**Duration:** ~90 seconds  
**Purpose:** Extended stability and station-keeping test  
**Tests:**
- 30-second depth hold at 1.0m
- 30-second yaw hold at 90°
- 20-second combined depth + yaw hold
- PID controller stability
- Disturbance rejection

**Run:**
```bash
ros2 run mavlink_runner runner analysis/guides/pool-testing/test-missions/test_stability.txt
```

**Success Criteria:**
- Depth holds within ±0.05m for entire duration
- Yaw holds within ±2° for entire duration
- No steady-state error
- Minimal oscillation

**Record data for analysis:**
```bash
ros2 bag record /duburi/state /duburi/diagnostics -o stability_test
```

---

## Running Missions

### Prerequisites
```bash
# Build workspace
cd ~/ROS_workspaces/Duburi_ws
colcon build
source install/setup.bash

# Start inspector in Terminal 1
ros2 launch mavlink_inspector inspector.launch.py
```

### Basic Usage
```bash
# Terminal 2: Run a mission
ros2 run mavlink_runner runner <path_to_mission_file>

# Example:
ros2 run mavlink_runner runner analysis/guides/pool-testing/test-missions/test_basic.txt
```

### Monitoring During Tests
```bash
# Terminal 3: Monitor state
ros2 topic echo /duburi/state

# Or monitor specific fields
ros2 topic echo /duburi/state | grep -E "depth|yaw|vx|vy"

# Monitor diagnostics
ros2 topic echo /duburi/diagnostics
```

### Recording Data
```bash
# Record all data for later analysis
ros2 bag record /duburi/state /duburi/diagnostics /duburi/control_command /rosout -o test_$(date +%Y%m%d_%H%M%S)
```

## Creating Custom Missions

### Mission Syntax
Missions use a simple text-based format. Each line is a command.

**Available Commands:**
```
arm                        # Arm the vehicle
disarm                     # Disarm the vehicle
mode MANUAL                # Set control mode
forward <power%> <duration>   # Move forward
backward <power%> <duration>  # Move backward
left <power%> <duration>      # Move left
right <power%> <duration>     # Move right
up <power%> <duration>        # Move up
down <power%> <duration>      # Move down
turn left <degrees> <power%>  # Turn left
turn right <degrees> <power%> # Turn right
~turn <degrees> <power%>      # Relative turn
dive <depth_m> <power%>       # Dive to depth
surface                       # Surface to 0m
hold depth <duration>         # Hold current depth
hold yaw <duration>           # Hold current yaw
stop                          # Stop all movement
delay <seconds>               # Wait
```

### Example Custom Mission
```bash
cat > custom_mission.txt << 'EOF'
arm
mode MANUAL

# Navigate to waypoint
forward 50% 8s
turn right 45 50%
delay 1

# Inspect at depth
dive 1.0m 50%
hold depth 15s

# Return
turn right 180 50%
forward 50% 8s
surface

stop
disarm
EOF

ros2 run mavlink_runner runner custom_mission.txt
```

## Troubleshooting

### Mission Doesn't Start
- Check inspector is running
- Verify MAVLink connection: `ros2 topic list | grep duburi`
- Check mission file syntax (no typos)
- Ensure vehicle is in correct mode

### Mission Stops Mid-Execution
- Check battery voltage (>12V required)
- Monitor logs for errors: `ros2 topic echo /rosout | grep -i error`
- Verify convergence timeouts not too short
- Check for RC watchdog triggers

### Commands Not Executing
- Verify ArduSub is armed
- Check control mode is MANUAL
- Ensure no manual override active
- Monitor control commands: `ros2 topic echo /duburi/control_command`

## Testing Checklist

Before running missions:
- [ ] Inspector launched and connected
- [ ] Topics publishing (`ros2 topic list`)
- [ ] Sensors reading correctly (depth, IMU, etc.)
- [ ] Battery charged (>14V)
- [ ] Pool depth measured
- [ ] Emergency stop procedure reviewed
- [ ] Multiple operators present

After each mission:
- [ ] Review actual vs expected behavior
- [ ] Record any anomalies
- [ ] Check bag files if recorded
- [ ] Update PID gains if needed
- [ ] Document lessons learned

## Next Steps

After running all test missions:
1. Analyze recorded data
2. Compare results to success criteria
3. Tune PID gains based on performance
4. Create competition-specific missions
5. Practice, practice, practice!

For detailed testing procedures and analysis, see [README.md](../README.md).
