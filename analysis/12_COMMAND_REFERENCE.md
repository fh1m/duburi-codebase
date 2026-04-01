# Duburi 4.2 — Complete Command Reference

This document covers every command available in the Duburi AUV control system:
the interactive **Runner CLI**, **mission file** syntax, and the **Python API**
(`driver_client.py`) for coded missions.

---

## Table of Contents

1. [Command Syntax Overview](#1-command-syntax-overview)
2. [Movement Commands (Ramped)](#2-movement-commands-ramped)
3. [Instant Fallback Commands (just\_\*)](#3-instant-fallback-commands-just_)
4. [Diagonal / Compound Movement](#4-diagonal--compound-movement)
5. [Body-Frame Vector Movement (move\_at)](#5-body-frame-vector-movement-move_at)
6. [Depth Control](#6-depth-control)
7. [Heading / Yaw Control](#7-heading--yaw-control)
8. [Relative Turn Commands](#8-relative-turn-commands)
9. [Simultaneous Move + Heading (go)](#9-simultaneous-move--heading-go)
10. [Coordinated Cruise](#10-coordinated-cruise)
11. [Surface / Stop / Arm / Disarm / Mode](#11-surface--stop--arm--disarm--mode)
12. [Actuators (Grabber)](#12-actuators-grabber)
13. [Teleop (Joystick)](#13-teleop-joystick)
14. [Chained Commands & Missions](#14-chained-commands--missions)
15. [Mission File Syntax](#15-mission-file-syntax)
16. [Python API (driver_client.py)](#16-python-api-driver_clientpy)
17. [Naming Convention Summary](#17-naming-convention-summary)
18. [Fallback Matrix](#18-fallback-matrix)
19. [ROS Parameters](#19-ros-parameters)

---

## 1. Command Syntax Overview

### Common Parameters

| Parameter | Syntax | Default | Description |
|-----------|--------|---------|-------------|
| **gain/speed** | `<N>%` | `50%` | Thruster power 0–100%. Mapped to PWM offset from 1500. |
| **duration** | `<N>s` | indefinite | How long to hold the movement. `0` = run until next command. |
| **heading** | `<degrees>` | — | Absolute compass heading 0–360°. |
| **depth** | `<metres>` | — | Depth below surface. Positive = below. |

### Prefix Convention

| Prefix | Meaning | Example |
|--------|---------|---------|
| (none) | ArduSub firmware / default ramped | `heading 260`, `forward 50% 3s` |
| `~` | Software PID (smooth, closed-loop) | `~heading 260`, `~depth 0.5` |
| `just` | Instant bang-bang (no ramp, raw PWM) | `just forward 50% 3s` |

### Backward-Compatible Aliases

| Alias | Maps to |
|-------|---------|
| `dive` | `depth` |
| `p_dive` | `~depth` |
| `yaw` | `heading` |
| `p_yaw` | `~heading` |
| `p_turn` | `~turn` |
| `backward` | `back` |

---

## 2. Movement Commands (Ramped)

All movement commands use the PWM ramp system by default. PWM ramps at
`ramp_rate` PWM/second (default 800 — 0.5s from zero to full speed).

### Runner CLI Syntax

```
forward [<gain>%] [<dur>s]
move forward [<gain>%] [<dur>s]
back [<gain>%] [<dur>s]
move back [<gain>%] [<dur>s]
left [<gain>%] [<dur>s]
move left [<gain>%] [<dur>s]
right [<gain>%] [<dur>s]
move right [<gain>%] [<dur>s]
up [<gain>%] [<dur>s]
move up [<gain>%] [<dur>s]
down [<gain>%] [<dur>s]
move down [<gain>%] [<dur>s]
```

### Examples

```
Duburi > forward 50% 3s          # Forward at 50% for 3 seconds
Duburi > move left               # Left at default 50%, indefinite
Duburi > move up 80% 5s          # Ascend at 80% for 5 seconds
Duburi > back 30%                # Backward at 30%, indefinite
```

### Mission File Syntax

```
forward 3 50          # direction duration speed
back 5 60
left 2
up 3 80
move forward 3 50     # 'move' prefix also works
```

### Python API

```python
from mavlink_driver.driver_client import move_forward, move_left, move_up

move_forward(duration=3.0, speed=50)   # Returns DriverCommand
move_left(duration=2.0, speed=60)
move_up(duration=5.0, speed=80)
# Also: move_back, move_right, move_down
```

---

## 3. Instant Fallback Commands (just\_\*)

Bypass the PWM ramp — apply target PWM instantly (bang-bang).
Use when ramping causes issues during testing, or when you need
immediate thruster response without acceleration delay.

### Runner CLI Syntax

Prefix any movement command with `just`:

```
just forward [<gain>%] [<dur>s]
just move left [<gain>%] [<dur>s]
just back 50% 3s
just up 80%
just heading left 50% 3s
just heading right 50% 3s
just forward-right 60% 5s
just go forward 90 60% 5s
just surface
```

### Examples

```
Duburi > just forward 50% 3s     # Instant forward (no ramp)
Duburi > just move left 60%      # Instant left strafe
Duburi > just surface            # Instant throttle up
```

### Mission File Syntax

```
just forward 3 50
just back 5 60
just heading left 3 50
just go forward 90 5 60
just surface
just move forward 3 50
just forward-right 5 60
```

### Python API

```python
from mavlink_driver.driver_client import (
    just_move_forward, just_move_left, just_yaw_left,
    just_move_combo, just_go_combo, just_surface
)

just_move_forward(duration=3.0, speed=50)
just_move_left(duration=2.0, speed=60)
just_yaw_left(duration=3.0, speed=50)
just_move_combo('forward-right', duration=5.0, speed=60)
just_go_combo('forward', angle=90, duration=5.0, speed=60)
just_surface()
# Also: just_move_back, just_move_right, just_move_up, just_move_down,
#        just_yaw_right, just_go_forward, just_go_back, just_go_left, just_go_right
```

---

## 4. Diagonal / Compound Movement

Combine two horizontal axes (forward/back + left/right). Speed is automatically
scaled per axis by 1/√2 so the resultant vector keeps the requested magnitude.

### Runner CLI Syntax

```
forward-right [<gain>%] [<dur>s]
move forward-right [<gain>%] [<dur>s]
forward-left [<gain>%] [<dur>s]
back-right [<gain>%] [<dur>s]
back-left [<gain>%] [<dur>s]
```

### Examples

```
Duburi > forward-right 60% 5s
Duburi > move back-left 50%
Duburi > just forward-right 70%    # Instant diagonal
```

### Python API

```python
from mavlink_driver.driver_client import move_combo, just_move_combo

move_combo('forward-right', duration=5.0, speed=60)
move_combo('back-left', duration=3.0, speed=50)
just_move_combo('forward-right', duration=5.0, speed=60)  # Instant
```

---

## 5. Body-Frame Vector Movement (move\_at)

Move at any bearing relative to the AUV's body. The angle is decomposed into
forward and lateral channels using `cos(θ)` and `sin(θ)`, providing continuous
360° directional control instead of only 8 discrete directions.

**Bearing convention (body-relative):**
- 0° = pure forward
- 90° = pure right (starboard)
- 180° = pure backward
- 270° = pure left (port)
- 45° = forward-right diagonal (equivalent to `forward-right` but via trig)

### Runner CLI Syntax

```
at <angle°> [<gain>%] [<dur>s]         Shorthand
move at <angle°> [<gain>%] [<dur>s]    With 'move' prefix
just at <angle°> [<gain>%] [<dur>s]    Instant (no ramp)
```

### Examples

```
Duburi > at 45 60% 3s           # Forward-right at 60% for 3s
Duburi > at 135 50%             # Back-right at 50% indefinitely
Duburi > move at 270 80% 5s    # Pure left at 80% for 5s
Duburi > just at 0 100%        # Instant full-speed forward
```

### Mission File Syntax

```
at 45 3 60         # bearing=45°, dur=3s, speed=60%
move at 90 5 50    # bearing=90°, dur=5s, speed=50%
just at 180 2 70   # instant, bearing=180°, dur=2s, speed=70%
```

### Python API

```python
from mavlink_driver.driver_client import move_at, just_move_at

move_at(bearing=45.0, duration=3.0, speed=60)   # 45° vector
move_at(bearing=90.0, duration=5.0, speed=50)   # Pure right
just_move_at(bearing=0.0, duration=3.0, speed=100)  # Instant forward
```

### How It Works

```
offset = speed_pwm − 1500
CH_FORWARD = 1500 + offset × cos(bearing)
CH_LATERAL = 1500 + offset × sin(bearing)
```

The resultant vector magnitude equals the requested speed. At cardinal
directions (0°, 90°, 180°, 270°) the behavior is identical to the
corresponding `move_*` command. At 45° it matches `forward-right` with
automatic √2 scaling via the trig decomposition.

---

## 6. Depth Control

Two approaches: firmware ALT_HOLD or software PID.

### Firmware Depth (ALT_HOLD)

Uses ArduSub's built-in depth controller. Auto-switches to ALT_HOLD mode.

```
depth <metres>
dive <metres>              # Alias for depth
move depth <metres>        # Alternate syntax
```

```
Duburi > depth 0.5         # Hold at 0.5m depth (ALT_HOLD mode)
Duburi > depth 1.0
```

### Software PID Depth (~depth)

Uses software PID via RC throttle. Works in any mode (MANUAL, STABILIZE).
Auto-switches to STABILIZE for best stability.

```
~depth                     # PID hold CURRENT depth
~depth <metres>            # PID hold at specific depth
~depth off                 # Disable software depth PID
p_dive                     # Alias for ~depth
p_dive <metres>            # Alias for ~depth <m>
```

```
Duburi > ~depth             # Lock current depth
Duburi > ~depth 0.5         # PID to 0.5m
Duburi > ~depth off         # Release depth hold
```

### Mission File Syntax

```
depth 0.5
~depth 0.5
~depth off
~depth                      # Hold current depth
```

### Python API

```python
from mavlink_driver.driver_client import set_depth, pid_depth, pid_depth_off

set_depth(0.5)       # ALT_HOLD firmware depth
pid_depth(0.5)       # Software PID depth
pid_depth(0.0)       # PID hold current depth
pid_depth_off()      # Disable PID
```

---

## 7. Heading / Yaw Control

### Absolute Heading (Bang-Bang)

Open-loop thrusters: full power in direction of error until within tolerance.

```
heading <degrees> [<gain>%]
yaw <degrees> [<gain>%]          # Alias
```

```
Duburi > heading 260 50%         # Yaw to 260° using bang-bang
Duburi > heading 90              # Yaw to 90° at default speed
```

### Absolute Heading (PID — Smooth)

Software PID: proportional response, smooth approach, no overshoot.

```
~heading <degrees> [<gain>%]
p_yaw <degrees> [<gain>%]       # Alias
```

```
Duburi > ~heading 260            # PID smooth yaw to 260°
Duburi > ~heading 180 70%        # PID yaw at 70% max speed
```

### Open-Loop Yaw (Timed Rotation)

Spin in a direction for a duration. Uses ramped PWM by default.

```
heading left [<gain>%] [<dur>s]
heading right [<gain>%] [<dur>s]
just heading left [<gain>%] [<dur>s]    # Instant (no ramp)
```

```
Duburi > heading left 50% 3s    # Spin left at 50% for 3s
Duburi > heading right 30%      # Spin right indefinitely
```

### Python API

```python
from mavlink_driver.driver_client import (
    yaw_to_heading, pid_yaw,
    yaw_left, yaw_right,
    just_yaw_left, just_yaw_right
)

yaw_to_heading(260, speed=50)   # Bang-bang to 260°
pid_yaw(260, speed=50)          # PID to 260°
yaw_left(duration=3.0, speed=50)    # Spin left (ramped)
just_yaw_left(duration=3.0, speed=50)  # Spin left (instant)
```

---

## 8. Relative Turn Commands

Turn left/right by a number of degrees from current heading.
Requires telemetry (current heading) to compute target.

### Bang-Bang Turn

```
turn left <degrees> [<gain>%]
turn right <degrees> [<gain>%]
```

```
Duburi > turn left 90            # Turn 90° left from current heading
Duburi > turn right 45 60%       # Turn 45° right at 60%
```

### PID Turn (Smooth)

```
~turn left <degrees> [<gain>%]
~turn right <degrees> [<gain>%]
p_turn left <degrees> [<gain>%]    # Alias
```

```
Duburi > ~turn left 90           # PID smooth 90° left turn
Duburi > ~turn right 180 50%     # PID 180° right turn
```

### Python API

```python
from mavlink_driver.driver_client import (
    turn_left, turn_right,
    pid_turn_left, pid_turn_right
)

# All require current_heading from telemetry:
turn_left(current_heading=180.0, angle=90.0, speed=50)
turn_right(current_heading=180.0, angle=45.0, speed=60)
pid_turn_left(current_heading=180.0, angle=90.0, speed=50)
```

---

## 9. Simultaneous Move + Heading (go)

Move in a direction while simultaneously PID-rotating to a target heading.
Movement uses Layer 2 (ramped); yaw uses PID Layer 4 (smooth).

### Runner CLI Syntax

```
go <direction> <heading°> [<gain>%] [<dur>s]
```

Directions: `forward`, `back`, `left`, `right`, and diagonal combos
(`forward-right`, `back-left`, etc.)

### Examples

```
Duburi > go forward 90 60% 5s          # Forward toward 90° for 5s
Duburi > go forward-right 45 70% 3s    # Diagonal NE toward 45°
Duburi > go left 180                   # Strafe left toward 180°
Duburi > just go forward 90 60% 5s     # Same but instant (no ramp)
```

### Mission File Syntax

```
go forward 90 5 60          # direction heading duration speed
go forward-right 45 3 70
just go forward 90 5 60     # Instant fallback
```

### Python API

```python
from mavlink_driver.driver_client import (
    go_forward, go_back, go_left, go_right, go_combo,
    just_go_forward, just_go_combo
)

go_forward(angle=90, duration=5.0, speed=60)
go_combo('forward-right', angle=45, duration=3.0, speed=70)
just_go_forward(angle=90, duration=5.0, speed=60)  # Instant
just_go_combo('forward-right', angle=45, duration=3.0, speed=70)
```

---

## 10. Coordinated Cruise

Simultaneously activates movement at a body-frame bearing, depth PID, and
yaw PID. This is the full coordinated maneuver: the AUV moves in a direction,
holds a target depth, and maintains a heading — all at once.

### Runner CLI Syntax

```
cruise <bearing°> <heading°> <depth_m> [<gain>%] [<dur>s]
just cruise <bearing°> <heading°> <depth_m> [<gain>%] [<dur>s]
```

| Parameter | Description |
|-----------|-------------|
| `bearing°` | Body-frame direction (0°=forward, 90°=right, 180°=back, 270°=left) |
| `heading°` | Target compass heading 0–360° (yaw PID) |
| `depth_m` | Target depth in metres (depth PID) |

### Examples

```
Duburi > cruise 0 90 0.5 60% 10s    # Forward, heading 90°, depth 0.5m
Duburi > cruise 45 180 1.0 50% 5s   # Diagonal, heading 180°, depth 1m
Duburi > just cruise 0 0 0.3 70% 8s # Instant (no ramp on movement)
```

### Mission File Syntax

```
cruise 0 90 0.5 10 60       # bearing heading depth duration speed
cruise 45 180 1.0 5 50
just cruise 0 0 0.3 8 70    # Instant fallback
```

### Python API

```python
from mavlink_driver.driver_client import cruise, just_cruise

cruise(bearing=0, heading=90, depth=0.5, duration=10.0, speed=60)
just_cruise(bearing=0, heading=90, depth=0.5, duration=10.0, speed=60)
```

### DriverCommand Fields

| Field | Usage |
|-------|-------|
| `command` | `cruise` or `just_cruise` |
| `angle` | Bearing (body-frame degrees) |
| `mode` | Target heading (string, degrees) |
| `depth` | Target depth (metres) |
| `speed` | Movement speed (0-100%) |
| `duration` | Duration (seconds) |

---

## 11. Surface / Stop / Arm / Disarm / Mode

### Surface

Ascend to surface. Behavior depends on flight mode:
- **ALT_HOLD**: Commands firmware to -0.1m depth.
- **MANUAL**: Throttle up at 50% for 10 seconds.

```
surface
just surface         # Instant throttle up (no ramp, MANUAL only)
```

### Stop

Emergency halt — instantly sets all channels to neutral. Bypasses ramp.
Also clears depth PID, yaw PID, and all active movements.

```
stop
```

### Arm / Disarm

```
arm
disarm
```

Note: Runner waits 4s after arm (for vehicle to complete arming) and 2s
after disarm before accepting the next command.

### Mode

```
mode MANUAL
mode ALT_HOLD
mode STABILIZE
```

### Python API

```python
from mavlink_driver.driver_client import (
    surface, just_surface, stop, arm, disarm, set_mode
)

surface()
just_surface()
stop()
arm()
disarm()
set_mode('MANUAL')
```

---

## 12. Actuators (Grabber)

Controls servo-based grabber via AUX output.

```
grabber open
grabber close
```

### Python API

```python
from mavlink_driver.driver_client import open_grabber, close_grabber

open_grabber()
close_grabber()
```

---

## 13. Teleop (Joystick)

Used by the teleop driver node (not typed manually). Single command
carries all 4 axes simultaneously via repurposed message fields.

| Field | Axis | Positive | Negative |
|-------|------|----------|----------|
| `speed` | Forward | Forward | Backward |
| `duration` | Lateral | Right | Left |
| `depth` | Throttle | Up | Down |
| `angle` | Yaw | CCW/Left | CW/Right |

```
teleop        # Normal (ramped) — smooths joystick jitter
just_teleop   # Instant (raw PWM from joystick)
teleop_idle   # Joystick centered — clears movement, ramp decelerates naturally
```

---

## 14. Chained Commands & Missions

### Chained Commands (Runner CLI)

Separate commands with `;` to run them sequentially:

```
Duburi > arm; forward 50% 3s; left 50% 2s; stop; disarm
Duburi > mode MANUAL; arm; ~depth 0.5
```

The runner waits for each command's duration before executing the next.

### Running Mission Files

```
Duburi > run gate           # Load and run missions/gate.txt
Duburi > run pool_test      # Run from missions/ directory
Duburi > list missions      # List available mission files
```

---

## 15. Mission File Syntax

Mission files are text files with one command per line. Placed in `missions/`.

### Format

```
# Comments start with #
# Blank lines are ignored

# Same commands as Runner CLI, but with positional args:
#   direction duration speed   (for movement)
#   heading degrees            (for yaw)

arm
sleep 4                      # Wait N seconds
mode MANUAL
~depth 0.5
sleep 5
forward 3 60                # Forward 3 seconds at 60% speed
sleep 4
left 2 50                   # Left 2 seconds at 50%
sleep 3
heading 90                  # Yaw to 90°
go forward 90 5 60           # Move forward + yaw to 90° for 5s
forward-right 3 50           # Diagonal for 3s at 50%
just forward 3 50            # Instant (no ramp) forward
pause                        # Pause until resume signal
stop
~depth off
surface
sleep 5
disarm
```

### Special Commands

| Command | Description |
|---------|-------------|
| `sleep <N>` / `wait <N>` | Wait N seconds (interruptible by Ctrl+C) |
| `pause` | Pause mission until externally resumed |
| `resume` | No-op in file (resume is triggered externally) |
| `#` | Comment line (ignored) |

### Mission Search Paths

1. `./missions/` (current working directory)
2. Package `missions/` directory
3. `~/.duburi/missions/`

---

## 16. Python API (driver_client.py)

All functions return a `DriverCommand` message. Publish it to
`/driver/command` topic, or use `mission_executor.py`'s `_publish()`.

### Complete Function List

| Function | Command String | Category |
|----------|---------------|----------|
| `move_forward(dur, spd)` | `move_forward` | Movement (ramped) |
| `move_back(dur, spd)` | `move_back` | Movement (ramped) |
| `move_left(dur, spd)` | `move_left` | Movement (ramped) |
| `move_right(dur, spd)` | `move_right` | Movement (ramped) |
| `move_up(dur, spd)` | `move_up` | Movement (ramped) |
| `move_down(dur, spd)` | `move_down` | Movement (ramped) |
| `move_combo(dir, dur, spd)` | `move_<d1>_<d2>` | Diagonal (ramped) |
| `move_at(bearing, dur, spd)` | `move_at` | Body-frame vector (ramped) |
| `yaw_left(dur, spd)` | `yaw_left` | Open-loop yaw (ramped) |
| `yaw_right(dur, spd)` | `yaw_right` | Open-loop yaw (ramped) |
| `yaw_to_heading(angle, spd)` | `yaw_to_heading` | Bang-bang yaw (Layer 4) |
| `pid_yaw(angle, spd)` | `pid_yaw_to_heading` | PID yaw (Layer 4) |
| `turn_left(hdg, angle, spd)` | `yaw_to_heading` | Relative bang-bang |
| `turn_right(hdg, angle, spd)` | `yaw_to_heading` | Relative bang-bang |
| `pid_turn_left(hdg, angle, spd)` | `pid_yaw_to_heading` | Relative PID |
| `pid_turn_right(hdg, angle, spd)` | `pid_yaw_to_heading` | Relative PID |
| `set_depth(depth)` | `set_depth` | Firmware depth (ALT_HOLD) |
| `pid_depth(depth)` | `pid_depth` | Software PID depth |
| `pid_depth_off()` | `pid_depth_off` | Disable PID depth |
| `go_forward(angle, dur, spd)` | `go_forward` | Move + heading (ramped) |
| `go_back(angle, dur, spd)` | `go_back` | Move + heading (ramped) |
| `go_left(angle, dur, spd)` | `go_left` | Move + heading (ramped) |
| `go_right(angle, dur, spd)` | `go_right` | Move + heading (ramped) |
| `go_combo(dir, angle, dur, spd)` | `go_<d1>_<d2>` | Diagonal + heading |
| `surface()` | `surface` | Ascend |
| `stop()` | `stop` | Emergency halt |
| `arm()` | `arm` | Arm motors |
| `disarm()` | `disarm` | Disarm motors |
| `set_mode(mode)` | `set_mode` | Change flight mode |
| `open_grabber()` | `open_grabber` | Grabber open |
| `close_grabber()` | `close_grabber` | Grabber close |
| `just_move_forward(dur, spd)` | `just_move_forward` | Instant forward |
| `just_move_back(dur, spd)` | `just_move_back` | Instant back |
| `just_move_left(dur, spd)` | `just_move_left` | Instant left |
| `just_move_right(dur, spd)` | `just_move_right` | Instant right |
| `just_move_up(dur, spd)` | `just_move_up` | Instant up |
| `just_move_down(dur, spd)` | `just_move_down` | Instant down |
| `just_yaw_left(dur, spd)` | `just_yaw_left` | Instant yaw left |
| `just_yaw_right(dur, spd)` | `just_yaw_right` | Instant yaw right |
| `just_move_combo(dir, dur, spd)` | `just_move_<d1>_<d2>` | Instant diagonal |
| `just_move_at(bearing, dur, spd)` | `just_move_at` | Instant vector |
| `just_go_forward(angle, dur, spd)` | `just_go_forward` | Instant go forward |
| `just_go_back(angle, dur, spd)` | `just_go_back` | Instant go back |
| `just_go_left(angle, dur, spd)` | `just_go_left` | Instant go left |
| `just_go_right(angle, dur, spd)` | `just_go_right` | Instant go right |
| `just_go_combo(dir, angle, dur, spd)` | `just_go_<d1>_<d2>` | Instant diagonal+hdg |
| `just_surface()` | `just_surface` | Instant surface |

---

## 17. Naming Convention Summary

```
Layer   CLI Prefix   API Prefix     Behavior
─────   ──────────   ──────────     ────────
Bare    (none)       move_*()       ArduSub firmware / ramped movement
PID     ~            pid_*()        Software PID (smooth, closed-loop)
Instant just         just_*()       Raw bang-bang (no ramp, no PID)
```

### Inspector Command Strings (DriverCommand.command)

```
Movement (Layer 2, ramped):
  move_forward, move_back, move_left, move_right, move_up, move_down
  move_forward_right, move_back_left, etc. (diagonals)
  move_at (body-frame vector, any bearing 0-360°)
  yaw_left, yaw_right (open-loop rotation)
  teleop (joystick input)
  go_forward, go_forward_right, etc. (movement + PID heading)

Instant (Layer 2, bypass_ramp=True):
  just_move_forward, just_forward, just_back, just_left, etc.
  just_yaw_left, just_yaw_right
  just_teleop
  just_surface
  just_go_forward, just_go_forward_right, etc.
  just_move_forward_right, etc. (compound)
  just_move_at (body-frame vector, no ramp)

PID (Layers 3-4, inherently smooth — no ramp applied):
  pid_depth (Layer 3 — CH_THROTTLE)
  pid_yaw_to_heading (Layer 4 — CH_YAW)

ArduSub Firmware (not RC override):
  set_depth (ALT_HOLD)
  yaw_angle (SET_ATTITUDE_TARGET)

Control:
  stop, arm, disarm, set_mode, surface
  pid_depth_off, teleop_idle
  open_grabber, close_grabber
```

---

## 18. Fallback Matrix

Every command has a working fallback for when the primary mechanism fails.

| Primary Command | Fallback | Notes |
|-----------------|----------|-------|
| `forward` (ramped) | `just forward` | Bypasses ramp |
| `move left` (ramped) | `just move left` | Bypasses ramp |
| `forward-right` (ramped) | `just forward-right` | Bypasses ramp |
| `heading left` (ramped yaw) | `just heading left` | Bypasses ramp |
| `go forward 90` (ramped) | `just go forward 90` | Bypasses ramp |
| `surface` (ramped in MANUAL) | `just surface` | Instant throttle up |
| `~depth 0.5` (PID) | `depth 0.5` | Firmware ALT_HOLD |
| `~heading 260` (PID) | `heading 260` | Bang-bang (Layer 4) |
| `~turn left 90` (PID) | `turn left 90` | Bang-bang |
| `cruise 0 90 0.5` (ramped) | `just cruise 0 90 0.5` | Bypasses ramp on movement |
| `stop` | — | Already instant |
| `teleop` (ramped) | `just_teleop` | Inspector-level only |

---

## 19. ROS Parameters

Tunable at launch or runtime via `ros2 param set`.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ramp_rate` | `800` | PWM/second ramp rate. 800 = 0.5s full range. |
| `yaw_kp` | `2.0` | Yaw PID proportional gain |
| `yaw_ki` | `0.05` | Yaw PID integral gain |
| `yaw_kd` | `0.5` | Yaw PID derivative gain |
| `yaw_max_integral` | `50.0` | Yaw PID integral windup limit |
| `depth_kp` | `500.0` | Depth PID proportional gain |
| `depth_ki` | `25.0` | Depth PID integral gain |
| `depth_kd` | `200.0` | Depth PID derivative gain |
| `depth_max_integral` | `0.5` | Depth PID integral windup limit |
| `depth_tolerance` | `0.05` | Depth PID deadband (metres) |
| `pid_max_rate` | `50` | PID output rate limit (PWM units/tick). Prevents thruster hunting. |
| `nominal_voltage` | `0.0` | Battery voltage compensation. 0=disabled. Set to nominal (e.g. 14.8 for 4S LiPo). |
| `ack_timeout` | `3.0` | Command ACK timeout (seconds) |
| `yaw_source` | `attitude` | Yaw source: `attitude`, `ahrs2`, or `both` |

### Runtime Tuning Examples

```bash
# Slower ramp (1s full range)
ros2 param set /mavlink_inspector ramp_rate 400

# Faster ramp (0.25s full range)
ros2 param set /mavlink_inspector ramp_rate 1600

# Reduce depth PID oscillation
ros2 param set /mavlink_inspector depth_kp 300

# Tighter yaw tracking
ros2 param set /mavlink_inspector yaw_kp 3.0
```
