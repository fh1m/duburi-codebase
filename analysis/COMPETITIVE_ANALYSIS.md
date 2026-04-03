# 13 — Competitive Analysis: Duburi 4.2 vs Top RoboSub Teams

This document benchmarks the BRACU Duburi 4.2 software stack against two reference teams:

- **NUS Bumblebee** (BBAUV 4.5) — RoboSub 2025 TDR, 55-member team, Jetson Orin AGX, Py Trees, UKF, XFeat+YOLO11+DepthAnything
- **Desert WAVE** (Dragon) — RoboSub 2024 TDR, 7-member team, Jetson Xavier NX, waypoint navigation, HSV vision

This analysis is read-only — no code changes. Its purpose is to identify architectural gaps, missing capabilities, and strategic priorities before the next development phase.

---

## 1. Team and Resource Context

| Dimension | Duburi 4.2 | Bumblebee (BBAUV 4.5) | Desert WAVE (Dragon) |
|-----------|-----------|----------------------|---------------------|
| Team size | ~15 | 55 | 7 |
| Compute | Jetson Orin Nano (8 GB) | Jetson Orin AGX (32 GB) + i7 SBC | Jetson Xavier NX |
| Firmware | ArduSub (Pixhawk 2.4.8) | Custom firmware stack + CAN bus | ArduSub (Pixhawk-based) |
| Framework | ROS 2 Humble | ROS 2 Humble | Custom (non-ROS) |
| Localization | IMU + DVL (EKF via Pixhawk) | DVL + FOG + IMU + UKF (custom) | DVL + FOG (dead reckoning) |
| Cameras | USB (1–2 cameras) | FLIR BlackFly S PoE (GigE) | Leopard Imaging MIPI ×2 |
| Pool test hours | ~100 (est.) | 300 | Not quantified |
| Simulation hours | 0 | 100 | Limited |
| Vehicles | 1 | 2 (BBAUV 4.5 + Mini-AUV) | 1 |

**Key takeaway:** Duburi's constraints (small team, Orin Nano, single vehicle) demand a strategy that maximizes software leverage per engineering-hour. Bumblebee can parallelise across sub-teams; we cannot. Desert WAVE shows that a tiny team with disciplined navigation can outperform larger teams on task completion rate.

---

## 2. Software Architecture Comparison

### 2.1 High-Level Architecture

| Layer | Duburi 4.2 | Bumblebee | Desert WAVE |
|-------|-----------|-----------|-------------|
| **Mission planning** | Finite state machine (text scripts + runner CLI) | Py Trees (behaviour tree) | Linear state machine |
| **Control** | PID (software depth + yaw) + ArduSub firmware modes | Feedforward + feedback controllers + trajectory planner + QP thrust allocator | ArduSub firmware PID |
| **Perception** | YOLO11 + Kalman tracking + PID visual servo | YOLO11 + XFeat + PnP + HDBSCAN + Depth Anything + MUSIC acoustics | HSV colour detection |
| **Localization** | Pixhawk EKF (IMU + BAR30 + optional DVL) | Custom UKF (IMU + DVL + FOG + vision-based recalibration) | Dead reckoning (DVL + FOG) |
| **Communication** | pymavlink (serial)  to  RC_CHANNELS_OVERRIDE | Custom CAN protocol + ROS 2 | Custom telemetry between hulls |

### 2.2 Mission Planning — Narrowing the Gap

| Feature | Duburi | Bumblebee | Desert WAVE |
|---------|--------|-----------|-------------|
| Task sequencing | YASMIN HFSM (hierarchical state machine) — each task is a sub-SM with explicit transitions | Behaviour tree (composable, fallbacks) | Linear waypoints |
| Runtime re-planning | Partial — timeout  to  fallback state transitions within each sub-SM | Yes — fallback nodes, condition guards | None |
| Task independence | Yes — each task is an independent YASMIN sub-state-machine | Yes — each task is an independent subtree | No |
| Error recovery | Timeout  to  fallback states + watchdog thread for critical conditions | Automatic fallback to alternative strategy | None |
| Simulation validation | None (Gazebo SITL stack documented, not yet connected) | Heavy Gazebo simulation of full missions | Limited |

**Bumblebee's Py Trees** allow them to:
1. Define each competition task as an independent, testable subtree
2. Compose tasks into missions with priority selectors
3. Add fallback behaviours when perception fails
4. Introspect and debug mission state at runtime

**Status update (2026):** The `duburi_planner` package now implements YASMIN hierarchical state machines, addressing the core gap. Each competition task is a sub-SM with explicit fallback transitions (e.g., vision timeout  to  dead reckoning). The YASMIN Viewer web UI provides runtime state introspection comparable to Py Trees' debugging tools. Two missions are implemented: `gate.py` and `demo_square.py`, with reusable states (`arm`, `submerge`, `drive`, `surface`, `search`, `align`, `wait_feedback`, `send_command`). The remaining gap vs Bumblebee is **breadth** (only 2 of 6 tasks implemented) and **reactivity** (no tick-based preemption — fallbacks are timeout-driven).

### 2.3 Control System

| Feature | Duburi | Bumblebee | Desert WAVE |
|---------|--------|-----------|-------------|
| Controller type | PID (depth, yaw) + open-loop thrust | Feedforward + PID + trajectory planner | Firmware PID only |
| Thrust allocation | Direct channel mapping (DESIGN 5 layering) | Quadratic programming (QP) solver | Direct channel mapping |
| Motion planning | None — instant setpoint jumps | Polynomial trajectory interpolation | Waypoint-to-waypoint |
| Thruster saturation handling | PWM clamping at 1100/1900 | QP solver optimises allocation to avoid saturation | None |
| Velocity ramping | Trapezoidal PWM ramp (software) | Smooth trajectory curves | None |

**Assessment:** Our control system is solid for a Pixhawk-based platform. The RC override layering design (DESIGN 5) is actually quite elegant and allows PID + movement to coexist. What we lack is **trajectory planning** — smooth paths between waypoints rather than bang-bang setpoint changes. For competition tasks requiring precise approach trajectories (torpedoes, bins), this matters.

### 2.4 Perception Pipeline

| Feature | Duburi | Bumblebee | Desert WAVE |
|---------|--------|-----------|-------------|
| Detection | YOLO11n (nano) | YOLO11 (larger variants likely) | HSV colour thresholding |
| Feature matching | None | XFeat (lightweight learned features) | None |
| Pose estimation | None | PnP + monocular depth (DepthAnything) | None |
| Tracking | Single-object Kalman filter | Multi-object (implied by behaviour tree structure) | None |
| Depth estimation | None (2D detections only) | DepthAnything V2 (monocular) | None |
| Acoustics | None | MUSIC algorithm for pinger DOA | Subsonus (not yet competition-tested) |
| Visual servo | PID (lateral + vertical + forward) | Implied but not detailed in TDR | HSV centroid  to  80% approach |
| Fallback strategy | Proportional-only when PID disabled | Multiple: XFeat fails  to  PnP  to  clustering  to  recalibrate | None |

**Assessment:** Our perception stack is in an excellent position for a team our size. YOLO11 + Kalman + PID visual servo gives us detection  to  tracking  to  alignment in a single pipeline. What we're missing:
1. **Pose estimation** — knowing not just *where* the target is in the frame but *how far* and *at what angle*
2. **Multi-object tracking** — our Kalman tracker handles one object per class; competition scenarios often need simultaneous tracking
3. **Monocular depth** — DepthAnything V2 could estimate approach distance without DVL bottom-lock

---

## 3. Feature Gap Matrix

| Capability | Priority | Duburi Status | Bumblebee | Desert WAVE | Effort |
|------------|----------|---------------|-----------|-------------|--------|
| **Behaviour tree / mission planner** | CRITICAL | [DONE] Partial — YASMIN HFSM (`duburi_planner`, 2 missions, 8 reusable states) | Py Trees | Linear SM | Large  to  Medium (foundation done, need more task SMs) |
| **Simulation environment** | HIGH | Missing | Gazebo | Unreal Engine 5 (Duburi TDR) | Large |
| **Trajectory planning** | HIGH | Missing | Polynomial interpolation | Waypoint-to-waypoint | Medium |
| **Pose estimation (PnP/depth)** | HIGH | Missing | XFeat + PnP + DepthAnything | None | Medium |
| **Multi-object tracking** | MEDIUM | Single-object KF | Implied multi-object | None | Small |
| **Acoustic pinger** | MEDIUM | Missing (no hardware) | MUSIC + custom DAQ | Subsonus (limited) | Hardware-dependent |
| **Actuator integration (torpedo, dropper, grabber)** | HIGH | Grabber only (servo) | Full suite | Pneumatics (limited) | Medium |
| **DVL integration** | HIGH | Planned (Nortek Nucleus 1000) | Teledyne Pathfinder | Nortek DVL 1000 | Medium |
| **Telemetry dashboard** | LOW | ANSI CLI status display | Telegram alerts + OCS | Operator camera feed | Small |
| **Error recovery / fallback** | CRITICAL | Partial — YASMIN timeout  to  fallback transitions per sub-SM | Behaviour tree fallbacks | None | Medium (need more fallback states) |
| **Sensor fusion (UKF)** | MEDIUM | Pixhawk EKF only | Custom UKF with vision recalibration | Dead reckoning | Large |
| **Network/comms resilience** | LOW | USB serial + tether | CAN bus + Ethernet + Telegram | Radio (surfaced) | Medium |

---

## 4. What Duburi Does Well (Competitive Advantages)

### 4.1 Clean ROS 2 Architecture
Our 10-package architecture with single MAVLink connection owner, typed messages, and layered RC override is cleaner than many RoboSub teams. The `duburi_interfaces` package with 11 message types gives us a well-defined API surface. Most teams at our level use monolithic scripts.

### 4.2 Interactive Development Workflow
The `duburi_runner` CLI with chained commands, mission files, and real-time status is a genuine productivity advantage. Pool testing time is the most expensive resource — being able to type `arm; ~depth 0.5; go forward 90 60% 5s; stop; disarm` in real-time without deploying code is valuable. Bumblebee uses test scripts; we have an interactive REPL.

### 4.3 Vision Pipeline Performance
Achieving 20-25 FPS on Orin Nano with YOLO11 + Kalman + annotation is competitive. The threaded display, lazy publish, and rate-limiting optimisations show disciplined performance engineering. Bumblebee runs on Orin AGX (4x the GPU) — our per-watt efficiency is likely better.

### 4.4 Comprehensive Documentation
17+ analysis documents covering architecture, design decisions, constraints, and code walkthroughs is unusual for a student robotics team. This institutional knowledge transfer mechanism is exactly what Bumblebee credits as critical for their multi-generation team continuity.

### 4.5 Command Vocabulary and Abstraction
The command system (`move`, `go`, `cruise`, `just`, `at`, `~` prefix) gives operators a rich and intuitive control language. The shared `duburi_common.command_vocabulary` ensures consistency between the runner CLI and mission files. This is more expressive than Desert WAVE's waypoint system and comparable to Bumblebee's ROS 2 action interface.

---

## 5. Strategic Priorities (Ranked)

Based on the competitive analysis, here are the recommended development priorities. Items are ordered by **competition impact per engineering-hour** — critical for our team size.

### Tier 1: Must-Have for Competition Readiness

1. **State Machine Mission Planner (YASMIN)** — [DONE] **IMPLEMENTED.** The `duburi_planner` package provides YASMIN HFSM with 8 reusable states and 2 missions (`gate.py`, `demo_square.py`). **Remaining work:** implement state machines for slalom, torpedoes, bins, and octagon tasks. See `16_PLANNER_DOCUMENTATION.md` for the full implementation guide.

2. **Simulation Environment** — We cannot rely solely on pool time for testing mission logic. A basic Gazebo simulation with gate and bin props would let us iterate 10x faster on behaviour tree development. Our own TDR mentions Unreal Engine 5 simulation capability — this needs to be connected to the ROS 2 stack.

3. **DVL Integration** — The Nortek Nucleus 1000 is already available. Integrating it into the Pixhawk EKF (or a software UKF) transforms our localization from gyro+barometer to actual position estimation. This is the prerequisite for waypoint navigation and approach trajectories.

### Tier 2: High-Impact Improvements

4. **Trajectory Planning** — Even simple polynomial interpolation between waypoints would make our approach to tasks smoother and more reliable. The current bang-bang setpoint jumps cause overshoot and oscillation.

5. **Pose Estimation** — Adding monocular depth estimation (DepthAnything V2 runs on Orin Nano) would let the alignment controller estimate distance to target, not just angular error. This enables proper approach trajectories for torpedoes and bins.

6. **Multi-Object Tracking** — Extend `KalmanObjectTracker` to maintain multiple tracks simultaneously. Competition scenarios (slalom gates, multiple bins) require tracking several objects.

7. **Actuator Suite** — Torpedo and dropper integration. The mechanical hardware exists; the software command path through `DriverCommand` is already designed for extensibility.

### Tier 3: Competitive Edge

8. **Custom UKF** — Replace Pixhawk's EKF with a software UKF that fuses DVL + IMU + barometer + visual odometry. Gives us sensor fusion that we can tune and debug.

9. **Acoustic Pinger Detection** — Hardware-dependent but high-value for specific tasks (Torpedoes, Octagon).

10. **Telemetry Enhancements** — Automated health monitoring, battery alerts, maybe a simple web dashboard instead of terminal ANSI.

---

## 6. Lessons from Each Team

### From Bumblebee (NUS)

1. **"Modular, dynamically reconfigurable execution"** — Their shift from rigid task sequencing to composable mission planners is the single most impactful architectural decision. We follow this path with YASMIN hierarchical state machines, which provide equivalent per-task modularity through sub-state-machines.

2. **Multi-vehicle strategy** — While we won't have two vehicles, the principle of **graceful degradation** applies. Each capability should function independently. If vision fails, we should still be able to navigate by dead reckoning.

3. **Perception pipeline layering** — YOLO  to  XFeat  to  PnP  to  fallback to clustering. Multiple methods for the same problem, with automatic fallback. Our single-method pipelines are fragile.

4. **Testing rigor** — 300 hours in-water, 100 hours simulation, Docker-based ROS1↔ROS2 bridge for regression testing. Their testing infrastructure *is* their competitive advantage.

5. **Telegram telemetry** — Simple but high-value. Pool-side team members get automated alerts. We could do this with a ROS 2 node that forwards critical events to a Telegram bot.

### From Desert WAVE

1. **Dead reckoning works** — With a good DVL + FOG, you don't need vision for navigation between tasks. Their 1-inch waypoint accuracy with a Leica GPS survey is remarkable. We should separate *navigation* (getting to the task area) from *task execution* (using vision to interact with the task).

2. **Simplicity wins** — A 7-person team competing effectively by doing fewer things well. Their choice to skip ML for navigation and focus on reliable waypoints is strategically sound.

3. **Course survey methodology** — Using physical landmarks to triangulate underwater waypoints. This is a testing-day technique we should adopt, regardless of our vision capabilities.

4. **Multiple runs per match** — Dragon completed the course 3 times in 20 minutes because waypoint navigation is fast. Speed through the course multiplies points. Our mission system should support this.

---

## 7. Architectural Patterns Worth Adopting

### 7.1 Hierarchical State Machine Integration (YASMIN, inspired by Bumblebee's modularity)

Bumblebee uses behaviour trees (Py Trees) for composable task execution with automatic fallback. We achieve the same modularity using YASMIN hierarchical state machines (HFSM), where each task is a sub-state-machine with explicit fallback transitions:

```
Top-Level Mission SM (YASMIN)
├── GateTask (sub-SM)
│   ├── SearchGate ──detected── to  AlignGate
│   ├── SearchGate ──timeout─── to  DeadReckonGate (fallback)
│   ├── AlignGate ──aligned─── to  PassThrough
│   ├── AlignGate ──lost────── to  SearchGate (retry)
│   └── PassThrough ──passed── to  [gate_done]
├── SlalomTask (sub-SM)
│   ├── SearchSlalom  to  AlignSlalom  to  PassSlalom
│   └── SearchSlalom ──timeout── to  WaypointSlalom (fallback)
└── ...
```

This pattern makes every task independently testable and gives explicit fallback when perception fails. The fallback is more verbose than a BT Selector node, but equally functional at RoboSub scale (5–6 tasks). See `15_MISSION_PLANNER_ANALYSIS.md` for the full architecture.

### 7.2 Navigation / Task Execution Split (from Desert WAVE)

```
Phase 1: Navigate to task area (DVL + waypoint — no vision)
Phase 2: Task execution (vision + alignment controller)
```

Separating navigation from task execution reduces the burden on the perception system. Vision only needs to work at close range during task execution.

### 7.3 Multi-Stage Perception (from Bumblebee)

```
Stage 1: YOLO detection (coarse — "there's a gate somewhere")
Stage 2: Feature matching (precise — "the gate corners are here")
Stage 3: Pose estimation (3D — "the gate is 2m away at 15° angle")
Fallback: Revert to Stage 1 + proportional approach
```

---

## 8. Hardware-Software Integration Gaps

| Area | Current State | What's Needed | Competition Impact |
|------|--------------|---------------|-------------------|
| DVL | Available hardware (Nucleus 1000) | ROS 2 driver + EKF integration | Transforms localization |
| Bottom camera | USB camera hardware | `camera_manager` already supports multi-cam | Required for bins task |
| Torpedo launcher | Mechanical design done | `DriverCommand` handler + servo/solenoid control | Required for torpedoes task |
| Dropper | Mechanical design done | `DriverCommand` handler + solenoid control | Required for bins task |
| Grabber | Implemented (open/close servo) | Vision-guided grasp planning | Required for octagon task |
| IMU (VN-200) | Connected via Pixhawk | Consider direct ROS 2 driver for higher-rate data | Improves control bandwidth |

---

## 9. Summary: Where We Stand

**Strengths (exploit these):**
- Clean, well-documented ROS 2 architecture (10 packages, 80 Python files)
- YASMIN hierarchical state machine planner with web viewer — proven at RoboSub 2025 (8th place)
- Interactive development workflow (runner CLI)
- Efficient vision pipeline on constrained hardware
- Rich command vocabulary with clean message interfaces
- Strong foundation for extension (message types, command dispatch)

**Weaknesses (address these):**
- ~~No mission planner beyond linear scripts~~  to  [DONE] YASMIN HFSM implemented (`duburi_planner`), but only 2 of 6 competition tasks have state machines
- No simulation environment connected to ROS 2 (Gazebo SITL stack documented but not integrated)
- No trajectory planning (bang-bang only)
- No pose estimation or depth estimation
- Single-object tracking only
- DVL not yet integrated

**Threats:**
- Pool testing time is limited — without simulation, iteration speed is capped
- Competition scenarios are non-linear — without a mission planner, edge cases cause mission failure
- Orin Nano thermal limits may constrain adding more perception stages

**Opportunities:**
- YASMIN is ROS 2-native, apt-installable, proven at RoboSub 2025, and **now implemented** in `duburi_planner`
- DepthAnything V2 and XFeat both have lightweight variants suitable for Orin Nano
- Our `alignment_controller` already provides the visual servo building block that YASMIN `MonitorState` task nodes need
- The `duburi_common` pattern can be extended to share task definitions between planner and executor
