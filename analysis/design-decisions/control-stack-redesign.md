# Control Stack Redesign V1 Design Decision Document

## Executive Summary

A comprehensive architectural overhaul of the Duburi AUV control stack, addressing fundamental design issues while enabling future perception integration.

**Key Outcomes:**
- 72% reduction in parser code (492 → 139 lines)
- Single source of truth for commands (decorator-based registry)
- 6 critical safety fixes implemented
- Clean Python API for perception integration
- Live-tunable parameters via ROS2

---

## Table of Contents

1. [The Problem](#the-problem)
2. [Design Goals](#design-goals)
3. [Architecture Overview](#architecture-overview)
4. [Key Changes](#key-changes)
5. [Safety Fixes (C1-C6)](#safety-fixes-c1-c6)
6. [Command Registry Pattern](#command-registry-pattern)
7. [Message Evolution](#message-evolution)
8. [Movement Control](#movement-control)
9. [Perception Integration](#perception-integration)
10. [Potential Issues](#potential-issues)
11. [Migration Guide](#migration-guide)

---

## The Problem

### Before: Scattered Command Definitions

Adding a new command required changes in 4-5 files:

```mermaid
flowchart TD
 subgraph "Adding a New Command (BEFORE)"
 A[movement_commands.py<br/>Add to MOVEMENTS dict] --> B[command_handler.py<br/>Add dispatch logic]
 B --> C[driver_client.py<br/>Add helper function]
 C --> D[command_parser.py<br/>Add if/elif branch]
 D --> E[mission_parser.py<br/>Add if/elif branch]
 end

 style A fill:#ff6b6b
 style B fill:#ff6b6b
 style C fill:#ff6b6b
 style D fill:#ff6b6b
 style E fill:#ff6b6b
```

**Pain Points:**
1. **492-line if/elif chain** in command_parser.py
2. **219-line if/elif chain** in mission_parser.py
3. **Duplicate aliases** scattered across files
4. **just_commands.py** with 114 lines of boilerplate
5. **Overloaded message fields** (mode field repurposed for heading)
6. **No type safety** for command parameters

### Before: Safety Vulnerabilities

```mermaid
flowchart LR
 subgraph "Safety Issues"
 C1[RC Watchdog<br/> None]
 C2[PID Anti-windup<br/> Wrong timing]
 C3[Yaw Derivative<br/> Unfiltered]
 C4[Ramp Timing<br/> Hardcoded]
 C5[Exceptions<br/> Swallowed]
 C6[Thread Safety<br/> No locks]
 end
```

---

## Design Goals

```mermaid
mindmap
 root((Control Stack<br/>Redesign))
 Single Source of Truth
 Decorator-based registry
 Auto-generated parsers
 Shared vocabulary
 Safety First
 RC watchdog
 Fixed PID bugs
 Thread safety
 Perception Ready
 Clean Python API
 DuburiClient class
 Explicit parameters
 Live Tuning
 ROS2 param callbacks
 Dynamic PID gains
 Runtime brake config
```

---

## Architecture Overview

### After: Unified Command Flow

```mermaid
flowchart TD
 subgraph Sources["Command Sources"]
 CLI[CLI Runner]
 MISSION[Mission Files]
 PLANNER[Planner]
 VISION[Vision/Perception]
 end

 subgraph Registry["Command Registry (NEW)"]
 REG[("command_registry.py<br/>@register decorator<br/>26 commands")]
 end

 subgraph Parser["Parsers (Rewritten)"]
 CP[command_parser.py<br/>139 lines]
 MP[mission_parser.py<br/>100 lines]
 end

 subgraph Handler["Command Handler"]
 CH[command_handler.py<br/>Registry lookup]
 end

 subgraph RC["RC Controller"]
 RC_CTRL[Phase-aware ramp<br/>Active braking]
 end

 subgraph MAV["MAVLink"]
 PIX[Pixhawk<br/>RC_CHANNELS_OVERRIDE]
 end

 Sources --> Parser
 Parser --> |DriverCommand| Handler
 Handler --> |Registry lookup| REG
 REG --> |Handler function| Handler
 Handler --> RC_CTRL
 RC_CTRL --> |20 Hz| PIX

 style REG fill:#4ecdc4,stroke:#333,stroke-width:2px
 style CP fill:#95e1d3
 style MP fill:#95e1d3
```

### Adding a New Command (AFTER)

```mermaid
flowchart TD
 A["Add @register decorator<br/>in movement_commands.py"] --> B["Done! [DONE]"]

 A --> |Auto-propagates to| C[command_parser.py]
 A --> |Auto-propagates to| D[mission_parser.py]
 A --> |Auto-propagates to| E[DuburiClient]
 A --> |Auto-propagates to| F[Help text]

 style A fill:#4ecdc4,stroke:#333,stroke-width:2px
 style B fill:#45b7d1,stroke:#333,stroke-width:2px
```

---

## Key Changes

### File Changes Summary

| File | Before | After | Change |
|------|--------|-------|--------|
| command_parser.py | 492 lines | 139 lines | -72% |
| mission_parser.py | 219 lines | 100 lines | -54% |
| just_commands.py | 114 lines | DELETED | -100% |
| DriverCommand.msg | 7 fields | 15 fields | +8 explicit fields |
| NEW: command_registry.py | | 180 lines | Central registry |
| NEW: duburi_client.py | | 350 lines | Clean API |
| NEW: AuvCommand.srv | | Service | System commands |

### Package Impact

```mermaid
pie title Lines Changed by Package
 "mavlink_inspector" : 800
 "mavlink_driver" : 600
 "mavlink_runner" : 400
 "duburi_common" : 200
 "duburi_interfaces" : 100
```

---

## Safety Fixes (C1-C6)

### C1: RC Watchdog

**Problem:** If RC override send failed, thrusters could be stuck at last PWM.

**Solution:**

```mermaid
sequenceDiagram
 participant Timer as RC Timer (20Hz)
 participant Node as Inspector Node
 participant Pixhawk

 Timer->>Node: _send_rc_override()
 Node->>Pixhawk: RC_CHANNELS_OVERRIDE

 alt Success
 Pixhawk-->>Node: (sent)
 Node->>Node: _last_rc_success = now
 else Failure
 Node->>Node: Check watchdog
 Note over Node: now - _last_rc_success > 0.5s?
 Node->>Node: _emergency_neutral()
 Node->>Pixhawk: All channels = 1500
 end
```

### C2: PID Anti-Windup Fix

**Problem:** Anti-windup checked _previous_ output saturation, not current.

```mermaid
flowchart LR
 subgraph "BEFORE (Bug)"
 A1[Compute P+I+D] --> B1[Check last_output]
 B1 --> |"Saturated?"| C1[Skip integration]
 B1 --> |"Not saturated"| D1[Integrate]
 end

 subgraph "AFTER (Fixed)"
 A2[Compute P+D] --> B2[Compute preliminary]
 B2 --> |"Will saturate?"| C2[Skip integration]
 B2 --> |"Won't saturate"| D2[Integrate]
 end

 style A1 fill:#ff6b6b
 style A2 fill:#4ecdc4
```

### C3-C6 Summary

```mermaid
flowchart TD
 C3[C3: Yaw PID Config] -->|"ema_alpha: 1.0 to 0.3"| C3F[Filtered derivative]
 C3 -->|"anti_windup: false to true"| C3W[Bounded integral]

 C4[C4: Real dt] -->|"0.05 to time.time delta"| C4F[Accurate ramp timing]

 C5[C5: Exception Logging] -->|"except pass to log error"| C5F[Visible errors]

 C6[C6: Thread Safety] -->|"threading.Lock"| C6F[Safe connection access]
```

---

## Command Registry Pattern

### Decorator-Based Registration

```python
@register('move_forward', CommandCategory.TRANSLATION, CommandTransport.TOPIC,
 description='Thrust forward on CH_FORWARD',
 channels=['CH_FORWARD'])
def cmd_move_forward(handler, cmd):
 handler.set_movement(
 {CH_FORWARD: NEUTRAL_PWM + handler.offset},
 'Forward')
```

### Registry Structure

```mermaid
classDiagram
 class CommandSpec {
 +str name
 +CommandCategory category
 +CommandTransport transport
 +Callable handler
 +str description
 +list~str~ channels
 +bool supports_duration
 +bool supports_speed
 +bool supports_angle
 +bool requires_armed
 +bool is_pid
 }

 class CommandCategory {
 <<enumeration>>
 SYSTEM
 TRANSLATION
 HEADING
 DEPTH
 COMPOUND
 ACTUATOR
 VISION
 }

 class CommandTransport {
 <<enumeration>>
 SERVICE
 ACTION
 TOPIC
 }

 CommandSpec --> CommandCategory
 CommandSpec --> CommandTransport
```

---

## Message Evolution

### DriverCommand.msg v2

```mermaid
flowchart LR
 subgraph "OLD Fields (Deprecated)"
 O1[mode: string]
 O2[depth: float32]
 O3[angle: float32]
 O4[speed: int32]
 end

 subgraph "NEW Fields (Explicit)"
 N1[speed_pct: float32]
 N2[target_heading: float32]
 N3[target_depth: float32]
 N4[bearing: float32]
 N5[direction: string]
 N6[flight_mode: string]
 N7[bypass_ramp: bool]
 N8[use_pid: bool]
 end

 O1 -.->|"Backward compat"| N6
 O2 -.->|"Backward compat"| N3
 O3 -.->|"Backward compat"| N2
 O4 -.->|"Backward compat"| N1
```

---

## Movement Control

### Phase-Aware Ramping

```mermaid
stateDiagram-v2
 [*] --> RampingUp: start_movement()
 RampingUp --> Cruising: target reached
 Cruising --> RampingDown: time_remaining < decel_time
 RampingDown --> Braking: time_remaining <= 0
 Braking --> Neutral: brake_duration elapsed
 Neutral --> [*]

 note right of Braking: Reverse thrust at<br/>brake_strength (30%)
```

### PWM Profile Visualization

```
PWM

1900
 CRUISING PHASE

1500

1400 BRAKE
 ___
 time
 ↑ ↑ ↑ ↑
 ramp_up cruise ramp_down brake
```

---

## Perception Integration

### DuburiClient API

```mermaid
classDiagram
 class DuburiClient {
 -Node _node
 -Publisher _pub
 +arm()
 +disarm()
 +stop()
 +move_forward(speed, duration, instant)
 +move_at(bearing, speed, duration)
 +yaw_to(heading, method)
 +pid_depth(meters)
 +cruise(bearing, heading, depth, speed)
 +adjust_from_vision(lateral_err, depth_err, heading_err)
 }

 class PerceptionNode {
 +DuburiClient duburi
 +on_detection(detection)
 }

 PerceptionNode --> DuburiClient: uses
```

### Vision Servoing Integration

```mermaid
sequenceDiagram
 participant Cam as Camera
 participant Det as Detector
 participant Align as Alignment Controller
 participant Duburi as DuburiClient
 participant Ctrl as Inspector

 Cam->>Det: Frame
 Det->>Align: Detection (bearing, distance)
 Align->>Duburi: move_at(bearing, speed)
 Duburi->>Ctrl: DriverCommand
 Ctrl->>Ctrl: RC_CHANNELS_OVERRIDE
```

---

## Potential Issues

### Things to Watch

```mermaid
flowchart TD
 subgraph "Known Limitations"
 L1[Registry populated<br/>at import time]
 L2[Backward compat<br/>adds message size]
 L3[Brake tuning<br/>needed per vehicle]
 end

 subgraph "Future Risks"
 R1[Circular imports<br/>if registry grows]
 R2[Parameter explosion<br/>as features add]
 R3[Action server<br/>not yet implemented]
 end
```

### Mitigation Strategies

| Risk | Mitigation |
|------|------------|
| Registry import order | Import movement_commands before using get_all_commands() |
| Backward compat bloat | Phase B will remove deprecated fields |
| Brake overshoot | Conservative defaults, live-tunable |
| Circular imports | Registry has no internal dependencies |

---

## Migration Guide

### For Mission Files

**No changes needed.** Old commands still work:

```bash
# Old syntax still works
move forward 50% 5s
heading 90

# New syntax also works
~depth 0.5 # PID depth
just forward 50% # Bypass ramp
```

### For Python Code

```python
# OLD (deprecated but works)
from mavlink_driver.driver_client import move_forward, arm
msg = move_forward(duration=3, speed=50)

# NEW (preferred)
from mavlink_driver.duburi_client import DuburiClient
duburi = DuburiClient(node)
duburi.arm()
duburi.move_forward(speed=50, duration=3.0)
```

### For Planner States

```python
# OLD - still works
ctx.send('move_forward', duration=3, speed=50)

# NEW - also available
ctx.duburi.move_forward(speed=50, duration=3)
```

---

## Conclusion

The Control Stack Redesign V1 transforms the codebase from a scattered, hard-to-maintain system into a unified, safe, and extensible architecture ready for perception integration.

```mermaid
flowchart LR
 BEFORE[Scattered<br/>5 files to add command<br/>Safety vulnerabilities<br/>Hardcoded timing] --> REDESIGN((Control<br/>Redesign<br/>V1))
 REDESIGN --> AFTER[Unified<br/>1 decorator to add<br/>6 safety fixes<br/>Live-tunable]

 style BEFORE fill:#ff6b6b
 style REDESIGN fill:#4ecdc4
 style AFTER fill:#45b7d1
```
