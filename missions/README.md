# Mission Files Guide

Mission files provide an easy way to script AUV behavior without writing code. Each file contains a sequence of commands that execute in order.

## Mission File Format

Mission files are plain text with one command per line.

**Basic structure:**
```
# Comments start with #
# Blank lines are ignored

# Setup
mode MANUAL
arm

# Mission commands
forward 30% 5s
turn left 90 50%
forward 30% 3s

# Cleanup
stop
disarm
```

**Rules:**
- One command per line
- `#` for comments (can be inline or full line)
- Blank lines ignored
- Same syntax as interactive CLI
- Commands execute sequentially

## Running Missions

### From Command Line

```bash
# Run a mission file
ros2 run mavlink_runner runner missions/gate.txt

# Or specify full path
ros2 run mavlink_runner runner /home/fh1m/ROS_workspaces/Duburi_ws/missions/my_mission.txt
```

### From Interactive CLI

```bash
ros2 run mavlink_runner runner

duburi> run gate.txt
duburi> run my_mission.txt
duburi> list missions
```

### Mission Search Paths

The runner searches for mission files in:
1. Current directory
2. `~/ROS_workspaces/Duburi_ws/missions/`
3. Package share directory

## Example Missions

### Basic Movement

**File:** `basic_movement.txt`
```
# Test basic movement commands
mode MANUAL
arm

# Forward and back
forward 30% 3s
delay 2
back 30% 3s
delay 2

# Strafe left and right
left 30% 3s
delay 2
right 30% 3s

# Stop and disarm
stop
disarm
```

### Square Pattern

**File:** `square.txt`
```
# Swim a square pattern
mode MANUAL
arm

forward 30% 5s
turn left 90 50%
delay 1

forward 30% 5s
turn left 90 50%
delay 1

forward 30% 5s
turn left 90 50%
delay 1

forward 30% 5s
turn left 90 50%

stop
disarm
```

### Depth Profile

**File:** `depth_profile.txt`
```
# Multi-depth mission
mode ALT_HOLD
arm

# Dive to 0.5m
depth 0.5
delay 3

# Move forward at depth
forward 40% 5s
delay 2

# Dive to 1.0m
depth 1.0
delay 3

# Move forward deeper
forward 40% 5s
delay 2

# Return to surface
surface
delay 5

stop
disarm
```

### Gate Passing Sequence

**File:** `gate.txt`
```
# Pass through competition gate
mode MANUAL
arm

# Approach gate
forward 30% 5s
delay 1

# Align with gate (strafe if needed)
left 20% 2s
delay 1

# Pass through
forward 40% 4s
delay 2

# Exit and stop
stop
disarm
```

### Search Pattern

**File:** `search_pattern.txt`
```
# Lawn mower search pattern
mode MANUAL
arm
depth 0.5

# Row 1
forward 40% 8s
delay 1

# Turn and advance
turn right 90 50%
delay 1
forward 30% 3s
turn right 90 50%
delay 1

# Row 2
forward 40% 8s
delay 1

# Turn and advance
turn left 90 50%
delay 1
forward 30% 3s
turn left 90 50%
delay 1

# Row 3
forward 40% 8s

# Complete
stop
disarm
```

## V2 Best Practices

### 1. Use Moderate Speeds for Precision

For missions requiring accuracy (gate passing, object manipulation):

```
# Good - controlled, precise
forward 30% 5s
turn left 90 40%

# Too fast - may overshoot
forward 80% 2s
turn left 90 90%
```

**Speed recommendations:**
- **Gates/obstacles:** 30-40%
- **Open water transit:** 50-70%
- **Fine positioning:** 20-30%
- **Emergency maneuvers:** 80-100%

### 2. Add Delays Between Maneuvers

Give the AUV time to stabilize between commands:

```
forward 30% 5s
delay 1              # Convergence settling time
turn left 90 50%
delay 1              # Stabilize heading
forward 30% 3s
```

**When to use delays:**
- After turns (heading stabilization)
- After depth changes (depth stabilization)
- Before precision maneuvers
- When convergence is disabled

**How long:**
- **After forward/back:** 0.5-1s
- **After turns:** 1-2s
- **After depth changes:** 2-3s
- **Before precision tasks:** 2s

### 3. Enable Convergence for Precision Missions

Set in launch file or at runtime:

```yaml
# In launch file
parameters=[{
    'convergence_enabled': True,
}]
```

```bash
# At runtime
ros2 param set /mavlink_inspector convergence_enabled true
```

**With convergence enabled:**
```
# No delays needed - convergence gates handle it
forward 30% 5s
turn left 90 50%
forward 30% 3s
```

**Without convergence:**
```
# Manual delays required
forward 30% 5s
delay 1
turn left 90 50%
delay 1
forward 30% 3s
```

### 4. Use Mode Switching Strategically

Different modes for different tasks:

```
# Precision horizontal movement
mode MANUAL
forward 30% 5s
turn left 90 50%

# Depth control during movement
mode ALT_HOLD
depth 0.5
forward 40% 10s     # Maintains 0.5m depth automatically

# Free movement (for testing)
mode STABILIZE
forward 30% 5s      # No depth or heading hold
```

**Mode characteristics:**
- **MANUAL:** Full control, no stabilization
- **ALT_HOLD:** Automatic depth hold, manual heading/position
- **STABILIZE:** Automatic leveling, no position hold

### 5. Test in SITL First

Always test missions in simulation before running on hardware:

```bash
# Terminal 1: Start ArduSub SITL
cd ~/ardupilot/ArduSub
python3 sim_vehicle.py -v ArduSub -f vectored_6dof

# Terminal 2: Run mission
ros2 run mavlink_runner runner missions/my_test.txt
```

**SITL benefits:**
- Safe testing environment
- Fast iteration
- Verify command syntax
- Check timing and sequencing
- No risk to hardware

### 6. Include Safety Commands

Always include arming/disarming and stops:

```
# Start of mission
mode MANUAL
arm

# ... mission commands ...

# End of mission
stop                # Emergency stop all thrusters
disarm              # Disarm for safety
```

**Safety best practices:**
- Always disarm at end
- Use `stop` before `disarm`
- Test emergency stop procedure
- Have manual override ready

### 7. Use Comments Liberally

Document your mission intent:

```
# Gate passing mission - Competition 2024
# Approach, align, pass through gate

mode MANUAL
arm

# Phase 1: Approach gate from 5m
forward 40% 6s
delay 1

# Phase 2: Fine alignment
# Strafe left to center on gate
left 20% 2s
delay 2

# Phase 3: Pass through gate
forward 30% 5s
delay 1

# Phase 4: Clear gate and stop
forward 20% 2s
stop
disarm
```

## Advanced Patterns

### Coordinated Cruise

Maintain depth and heading while moving:

```
mode MANUAL
arm

# Cruise forward at 90° heading, 0.5m depth
cruise 0 90 0.5 50% 10s
delay 2

# Change heading mid-cruise
cruise 0 180 0.5 50% 10s

stop
disarm
```

### Vision-Guided Alignment

Use vision to align with targets:

```
mode MANUAL
arm
depth 0.5

# Approach roughly
forward 40% 5s
delay 2

# Vision alignment
lat-align 30% until    # Align laterally until centered
delay 1
dep-align 30% until    # Align depth until centered
delay 1

# Final approach aligned
align-forward 30% 3s

stop
disarm
```

### Relative Turns

Use `~turn` for smooth PID-controlled turns:

```
mode MANUAL
arm

# Sharp bang-bang turn
turn left 90 50%
delay 2

# Smooth PID turn
~turn right 180
delay 2

# Instant turn (no ramp)
just turn left 90 50%

stop
disarm
```

### Depth Profiles

Complex depth changes:

```
mode ALT_HOLD
arm

# Shallow search at 0.5m
depth 0.5
forward 40% 10s
delay 2

# Drop to 1.5m
depth 1.5
delay 3
forward 40% 10s
delay 2

# Return to shallow
depth 0.5
delay 3

surface
stop
disarm
```

## Mission File Organization

### Naming Convention

```
mission-type_description_version.txt

Examples:
gate_simple_v1.txt
search_lawnmower_v2.txt
competition_full_v3.txt
test_forward_only.txt
```

### Directory Structure

```
missions/
├── README.md (this file)
├── test/                    # Simple test missions
│   ├── basic_movement.txt
│   ├── depth_test.txt
│   └── turn_test.txt
├── patterns/                # Reusable patterns
│   ├── square.txt
│   ├── circle.txt
│   └── search_pattern.txt
├── competition/             # Competition-specific
│   ├── gate.txt
│   ├── buoy.txt
│   └── full_course.txt
└── templates/               # Starting points
    ├── simple_mission.txt
    └── complex_mission.txt
```

## Troubleshooting

### Mission Completes Too Fast

**Problem:** Commands execute but AUV doesn't reach targets

**Solutions:**
```
# Enable convergence gates
ros2 param set /mavlink_inspector convergence_enabled true

# Or add manual delays
forward 30% 5s
delay 2              # Add this
turn left 90 50%
```

### Commands Don't Execute

**Problem:** Mission file loads but commands are ignored

**Solutions:**
1. Check syntax (one command per line)
2. Verify command names match CLI commands
3. Check for typos in parameters
4. Ensure file has Unix line endings (not Windows CRLF)

```bash
# Convert line endings if needed
dos2unix missions/my_mission.txt
```

### AUV Overshoots Targets

**Problem:** AUV goes past intended positions

**Solutions:**
```
# Enable active braking
ros2 param set /mavlink_inspector braking_enabled true

# Reduce speed
forward 30% 5s      # Instead of 60%

# Add convergence wait
ros2 param set /mavlink_inspector convergence_enabled true
```

### Turns Are Imprecise

**Problem:** Turn commands don't reach exact heading

**Solutions:**
```
# Use PID turns for precision
~turn left 90       # Instead of: turn left 90

# Enable convergence
ros2 param set /mavlink_inspector convergence_enabled true

# Reduce turn speed
turn left 90 30%    # Instead of 80%
```

### Mission Hangs

**Problem:** Mission stops executing partway through

**Solutions:**
1. Check for convergence timeouts in logs
2. Increase timeout: `ros2 param set /mavlink_inspector convergence_timeout 10.0`
3. Verify thrusters are responding
4. Check for mode changes blocking commands

## Parameter Reference for Missions

Common parameters to tune for mission execution:

```yaml
# Mission execution
convergence_enabled: true         # Wait for stable arrival
convergence_timeout: 5.0          # Max wait time
braking_enabled: true             # Reduce overshoot

# Speed and control
default_speed_percent: 50         # Default if not specified
max_speed_percent: 80             # Safety limit
ramp_rate: 800                    # PWM/s acceleration

# Thresholds
convergence_velocity_threshold: 0.05   # m/s for "stopped"
convergence_settling_time: 0.2         # Stability duration
```

Set at launch:
```bash
ros2 run mavlink_runner runner missions/my_mission.txt \
  --ros-args \
  -p convergence_enabled:=true \
  -p braking_enabled:=true
```

Or at runtime:
```bash
ros2 param set /mavlink_inspector convergence_enabled true
```

## See Also

- [Command Reference](../analysis/reference/command-reference.md) - Complete command documentation
- [Mavlink Runner README](../src/mavlink_runner/README.md) - V2 control features
- [Control Stack V2](../analysis/design-decisions/control-stack-v2.md) - Technical details

## Contributing

When adding new example missions:
1. Test in SITL first
2. Document the mission purpose in comments
3. Follow naming convention
4. Add entry to this README
5. Include V2 best practices
