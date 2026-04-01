# 19 — Design Decisions Analysis: Comprehensive Pros/Cons

> **Purpose:** In-depth analysis of architectural decisions, design patterns, and
> trade-offs in the BRACU Duburi AUV 4.2 codebase.

---

## Table of Contents

1. [Single MAVLink Connection Owner](#1-single-mavlink-connection-owner)
2. [4-Layer RC Override Architecture](#2-4-layer-rc-override-architecture)
3. [Dispatch Table Pattern](#3-dispatch-table-pattern)
4. [YASMIN FSM for Mission Planning](#4-yasmin-fsm-for-mission-planning)
5. [Separated Vision and Control](#5-separated-vision-and-control)
6. [Kalman Filter for Detection Smoothing](#6-kalman-filter-for-detection-smoothing)
7. [TeleopCommand vs DriverCommand Separation](#7-teleopcommand-vs-drivercommand-separation)
8. [PID Controller Design](#8-pid-controller-design)
9. [ROS 2 Topic-Based Architecture](#9-ros-2-topic-based-architecture)
10. [Modular Package Structure](#10-modular-package-structure)

---

## 1. Single MAVLink Connection Owner

### Decision
Only `mavlink_inspector` opens the serial port to the Pixhawk. All other nodes
communicate via ROS 2 topics, never directly with MAVLink.

### Pros
- **No port conflicts**: Serial ports cannot be shared between processes
- **Centralized error handling**: Connection drops, reconnection logic in one place
- **Consistent HEARTBEAT**: Single source of GCS heartbeat at 1 Hz
- **Atomic state**: Vehicle state (armed, mode, telemetry) maintained in one node
- **Simplified testing**: Can mock the inspector for integration tests

### Cons
- **Single point of failure**: Inspector crash = total loss of control
- **Latency**: Commands go through ROS topics (adds ~1-5ms)
- **Complexity**: Inspector becomes a large, critical component
- **No redundancy**: Cannot have backup connection

### Trade-off Analysis
The serial port constraint makes this decision essentially mandatory. The cons are
mitigated by:
- Watchdog timers for automatic reconnection
- Clean shutdown handlers for graceful degradation
- Modular decomposition (7 files) to manage complexity

### Alternatives Considered
1. **MAVProxy + multiple clients**: Adds dependency, complexity, still single serial
2. **UDP multicast**: Requires network-based MAVLink (not serial)
3. **Shared memory**: Complex, error-prone, not ROS-idiomatic

**Verdict**: Correct decision. Alternatives don't solve the fundamental constraint.

---

## 2. 4-Layer RC Override Architecture

### Decision
RC channel values are computed in 4 additive layers:
1. **Neutral base**: All channels start at 1500
2. **Movement layer**: `RcController` adds forward/lateral/throttle/yaw offsets
3. **Depth PID layer**: Adds throttle correction for depth hold
4. **Yaw PID layer**: Adds yaw correction for heading hold

### Pros
- **Separation of concerns**: Each layer only touches its channels
- **Composable**: Depth hold works during forward movement
- **Debuggable**: Can isolate which layer is causing issues
- **Testable**: Each layer can be unit tested independently
- **Non-blocking**: PIDs run in the same 20 Hz tick, no thread coordination

### Cons
- **Channel conflicts possible**: Two layers could fight over same channel
- **Order dependency**: Layers must be applied in correct sequence
- **Complexity**: Understanding full PWM output requires tracing all layers
- **Saturation handling**: Need to clamp final values, losing layer information

### Trade-off Analysis
The layered approach enables simultaneous depth hold + movement, which is critical
for autonomous missions. The channel conflict risk is managed by explicit channel
ownership documentation.

### Alternatives Considered
1. **Single monolithic controller**: Simpler but can't compose behaviors
2. **Cascaded PID**: More correct for true cascade control, but overkill for AUV
3. **State machine per channel**: Too fine-grained, hard to coordinate

**Verdict**: Good balance of modularity and practicality.

---

## 3. Dispatch Table Pattern

### Decision
Command routing uses dictionary dispatch tables instead of if/elif chains:
- `CommandHandler.system_dispatch`: system commands → handlers
- `MOVEMENTS`: movement commands → movement handlers
- `TelemetryParser._dispatch`: MAVLink message types → parsers

### Pros
- **O(1) lookup**: Dictionary vs O(n) if/elif chain
- **Explicit registration**: All handlers visible in one place
- **Easy extension**: Add new command = add dict entry + handler
- **Self-documenting**: Dict keys show all supported commands
- **Testable**: Can test individual handlers in isolation

### Cons
- **Indirection**: Handler code not inline with dispatch logic
- **Learning curve**: Pattern unfamiliar to some developers
- **Dynamic dispatch**: Harder for static analysis tools
- **No partial matching**: Can't handle prefix patterns directly

### Trade-off Analysis
The ~400-line if/elif chain in the original runner.py was unmaintainable. Dispatch
tables reduced cognitive load significantly and enabled the modularization refactor.

### Alternatives Considered
1. **Keep if/elif**: Rejected due to maintenance burden
2. **Visitor pattern**: Overkill for string → handler mapping
3. **Decorator-based registration**: More magic, same outcome

**Verdict**: Clear win. Pattern is widely used (Django, Flask URL routing).

---

## 4. YASMIN FSM for Mission Planning

### Decision
Use YASMIN (Yet Another State MachINe) library for autonomous mission planning
instead of imperative mission scripts.

### Pros
- **Visual debugging**: YASMIN viewer shows current state, transitions
- **Hierarchical composition**: Sub-state machines for complex tasks
- **Standardized outcomes**: `succeeded`, `failed`, `aborted` convention
- **Blackboard sharing**: Clean context passing between states
- **Reusable states**: `SubmergeState`, `SearchState` work across missions
- **ROS 2 native**: Built for ROS 2, handles lifecycle properly

### Cons
- **Additional dependency**: YASMIN library must be installed
- **Learning curve**: FSM concepts not universally known
- **Overhead**: State machine infrastructure for simple sequences
- **Debugging complexity**: Transitions can be hard to trace
- **State explosion**: Complex behaviors need many states

### Trade-off Analysis
For RoboSub competition, missions have clear state sequences (submerge → search →
align → execute → surface). FSM maps naturally to this domain. The visual debugger
is invaluable during pool testing.

### Alternatives Considered
1. **Imperative scripts**: Simpler but no error handling, no visualization
2. **BehaviorTree.CPP**: More powerful but C++, steeper curve
3. **SMACH (ROS 1)**: Deprecated, not ROS 2 native
4. **Custom FSM**: Reinventing the wheel

**Verdict**: YASMIN is the right choice for ROS 2 FSM needs. Hierarchical states
enable clean mission composition.

---

## 5. Separated Vision and Control

### Decision
Vision pipeline (detection, tracking) is separate from control (alignment servo).
`detector_node` publishes detections; `alignment_controller` subscribes and publishes
teleop commands.

### Pros
- **Frame rate decoupling**: Detection at 10-15 Hz, control at 30 Hz (with prediction)
- **Testability**: Can test detection without running motors
- **Flexibility**: Swap detection models without touching control
- **Recording**: Can record detections for offline analysis
- **Multiple consumers**: Planner and alignment both use detections

### Cons
- **Latency**: ROS topic adds ~5-10ms between detection and control
- **Synchronization**: Detection timestamp vs control action timing
- **Dropped frames**: If detection is slow, control uses stale data
- **Complexity**: Two nodes instead of one monolithic vision+control

### Trade-off Analysis
The latency is acceptable for AUV speeds (~0.5 m/s). Kalman filter prediction
compensates for 1-2 frame delays. The flexibility benefits outweigh the latency cost.

### Alternatives Considered
1. **Monolithic vision+control**: Simpler but tightly coupled
2. **GPU-accelerated pipeline**: Would reduce latency but adds HW dependency
3. **Direct memory sharing**: Non-portable, complex

**Verdict**: Correct separation. ROS 2 topic overhead is negligible at AUV speeds.

---

## 6. Kalman Filter for Detection Smoothing

### Decision
Use a Kalman filter (`KalmanTracker`) to smooth YOLO bounding box detections and
handle temporary dropouts.

### Pros
- **Noise reduction**: Smooths jittery bounding boxes
- **Prediction**: Continues tracking during 1-2 frame dropouts
- **Velocity estimation**: Provides bbox velocity for predictive control
- **Principled approach**: Optimal estimator for linear + Gaussian
- **Configurable**: Q/R matrices tune smoothing vs responsiveness

### Cons
- **Assumes linear motion**: Incorrect for rapid target movement
- **Tuning required**: Q/R matrices need pool calibration
- **Single target**: Current implementation tracks one object
- **Delay introduction**: Smoothing adds ~1 frame latency

### Trade-off Analysis
YOLO detections at 15 Hz can have 10-20 pixel jitter. Without smoothing, this
causes oscillating control commands. Kalman filter reduces jitter to ~3 pixels,
enabling stable visual servo.

### Alternatives Considered
1. **Moving average**: Simpler but no prediction, fixed delay
2. **Exponential smoothing**: No velocity model
3. **Particle filter**: Handles multi-modal but overkill for single target
4. **No smoothing**: Unacceptable control oscillation

**Verdict**: Kalman filter is the standard solution. Works well in practice.

---

## 7. TeleopCommand vs DriverCommand Separation

### Decision
Created separate `TeleopCommand.msg` for continuous teleop instead of overloading
`DriverCommand` fields.

### Pros
- **Type safety**: Fields have correct semantics (linear_x vs duration)
- **Clear intent**: Message type indicates continuous vs discrete command
- **No field overloading**: No confusion about field meanings
- **Better tooling**: ROS introspection shows correct field names
- **Extensible**: Can add teleop-specific fields without affecting commands

### Cons
- **Two message types**: More interfaces to maintain
- **Inspector complexity**: Must handle both message types
- **Migration effort**: Required updating teleop_driver and alignment_controller

### Trade-off Analysis
The original design overloaded `DriverCommand.speed/duration/depth/angle` for
teleop PWM offsets. This caused confusion and documentation burden. Separate
message types are cleaner despite added interface.

### Alternatives Considered
1. **Keep overloading**: Rejected due to confusion
2. **Union type**: ROS 2 doesn't support unions well
3. **Generic command with variant**: Over-engineered

**Verdict**: Correct fix. Type safety is worth the extra message definition.

---

## 8. PID Controller Design

### Decision
Custom `PidController` class with:
- Deadband (output = 0 when |error| < tolerance)
- Anti-windup (conditional integration)
- EMA derivative filtering
- Rate limiting on output

### Pros
- **Deadband**: Prevents oscillation at setpoint
- **Anti-windup**: Prevents integral accumulation during saturation
- **Smooth derivative**: Avoids derivative kick on setpoint change
- **Rate limiting**: Prevents sudden PWM jumps
- **Tunable**: All features are parameterized

### Cons
- **Not using simple-pid**: Reinvented wheel instead of using library
- **Complexity**: 162 lines for a PID controller
- **Testing burden**: Custom code needs custom tests
- **Potential bugs**: Library would be battle-tested

### Trade-off Analysis
The `simple-pid` library was considered but doesn't have:
- Deadband (common need for underwater vehicles)
- Rate limiting (critical for thruster protection)
- Derivative-on-measurement (prevents setpoint kick)

Custom implementation allows these features with full control.

### Alternatives Considered
1. **simple-pid**: Missing required features
2. **ros2_control PID**: Heavy dependency for simple use case
3. **Copy from reference codebase**: Did inform design

**Verdict**: Custom PID justified by specific requirements. Should add unit tests.

---

## 9. ROS 2 Topic-Based Architecture

### Decision
All inter-node communication uses ROS 2 topics (pub/sub), not services or actions.

### Pros
- **Decoupled**: Publishers don't need to know subscribers
- **Asynchronous**: No blocking on communication
- **Multicast**: Multiple subscribers per topic
- **Debugging**: `ros2 topic echo` for any topic
- **Recording**: `rosbag` captures all communication

### Cons
- **No request/response**: Topics are fire-and-forget
- **No feedback**: Can't know if command succeeded without separate topic
- **No timeout handling**: Must implement manually
- **QoS complexity**: Need to match QoS settings

### Trade-off Analysis
For continuous control (20 Hz RC override), topics are ideal. Services would
introduce blocking. The feedback limitation is addressed by:
- `VehicleState` topic for state monitoring
- `AlignmentStatus` topic for visual servo feedback
- `MavlinkEvent` topic for arm/disarm/mode events

### Alternatives Considered
1. **ROS 2 Actions**: Good for long-running tasks, but adds complexity
2. **Services**: Blocking, not suitable for continuous control
3. **Direct function calls**: Defeats ROS 2 architecture benefits

**Verdict**: Topics are correct for this real-time control application.

---

## 10. Modular Package Structure

### Decision
Split functionality into 9 packages instead of one monolithic package:
- `duburi_interfaces`: Message definitions
- `duburi_common`: Shared constants
- `mavlink_inspector`: MAVLink bridge
- `mavlink_driver`: Command API
- `mavlink_runner`: CLI
- `mavlink_logger`: Logging
- `vision`: Detection + alignment
- `vision_inspector`: Camera management
- `duburi_planner`: FSM missions

### Pros
- **Separation of concerns**: Each package has single responsibility
- **Independent compilation**: Change one package, rebuild only it
- **Clear dependencies**: Package.xml shows what depends on what
- **Team scalability**: Different people can own different packages
- **Testability**: Can test packages in isolation

### Cons
- **More boilerplate**: Each package needs setup.py, package.xml, etc.
- **Import complexity**: Must install packages to import between them
- **Version coordination**: Package versions must be compatible
- **Build time**: Full rebuild takes longer than monolith

### Trade-off Analysis
The original inspector_node.py was 1854 lines. The monolithic approach was
unsustainable. 9 packages is the right granularity:
- 3 packages would be too coarse (mixing concerns)
- 20 packages would be too fine (excessive boilerplate)

### Alternatives Considered
1. **Single package with modules**: Can't have separate package.xml deps
2. **Metapackage**: Adds complexity without benefit
3. **More packages**: Diminishing returns below ~500 lines/package

**Verdict**: Current structure balances modularity with practicality.

---

## Summary: Design Quality Assessment

| Decision | Quality | Confidence | Notes |
|----------|---------|------------|-------|
| Single MAVLink owner | Excellent | High | Constraint-driven, no alternative |
| 4-layer RC | Good | High | Enables composition, needs documentation |
| Dispatch tables | Excellent | High | Clear improvement over if/elif |
| YASMIN FSM | Good | Medium | Right tool, needs pool testing |
| Vision/control separation | Good | High | Standard robotics practice |
| Kalman tracking | Good | Medium | Needs tuning, consider alternatives |
| TeleopCommand separation | Excellent | High | Fixed real bug |
| Custom PID | Good | Medium | Justified, needs tests |
| Topic-based arch | Excellent | High | ROS 2 best practice |
| Modular packages | Good | High | Right granularity |

---

## Recommendations for Future Work

1. **Add unit tests** for PidController and KalmanTracker
2. **Document channel ownership** explicitly for 4-layer RC
3. **Profile latency** in vision → control pipeline
4. **Consider multi-target tracking** for complex missions
5. **Add watchdog** for inspector crash recovery
6. **Create integration tests** for full mission sequences

---

*Document created from codebase analysis at commit `d0a48d6`.*
