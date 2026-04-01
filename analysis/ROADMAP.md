# ROADMAP — Duburi 4.2 → RoboSub 2026

> **Living document.** Check off items as they're completed. Last updated: 2026-04-02.
>
> **Mission:** Win RoboSub 2026. Theme: *"Restore and Recovery"* — underwater pipeline maintenance scenario.
>
> **Constraint:** Small team (~15), Jetson Orin Nano (8 GB), single vehicle, limited pool time. Every engineering-hour must count.

---

## Table of Contents

1. [Development So Far](#1-development-so-far)
2. [RoboSub 2026 Competition Task Analysis](#2-robosub-2026-competition-task-analysis)
3. [Phase 1 — Refine Controls](#3-phase-1--refine-controls)
4. [Phase 2 — Autonomous Missions](#4-phase-2--autonomous-missions)
5. [Phase 3 — Perception Stack](#5-phase-3--perception-stack)
6. [Phase 4 — Additional Sensors](#6-phase-4--additional-sensors)
7. [Phase 5 — Utils & Infrastructure](#7-phase-5--utils--infrastructure)
8. [Timeline & Dependencies](#8-timeline--dependencies)
9. [Risk Register](#9-risk-register)

---

## 1. Development So Far

### What We Have (April 2026)

The Duburi 4.2 stack is a **10-package, 80-file** ROS 2 Humble codebase with:

| Layer | Status | Details |
|-------|--------|---------|
| **MAVLink Bridge** | ✅ Production-ready | 7-module inspector: serial I/O, command dispatch, PID controllers, telemetry, RC override at 20 Hz |
| **Control System** | ✅ Functional | Software depth PID + yaw PID, trapezoidal PWM ramp, 4-layer RC override, `just_*` instant variants |
| **Command System** | ✅ Rich | 30+ commands: `move`, `go`, `cruise`, `at`, `just_*`, diagonals, PID depth/yaw, `~` prefix convention |
| **Interactive CLI** | ✅ Complete | `duburi_runner` REPL with history, file-based missions, chained commands, status dashboard |
| **Mission Planner** | 🟡 Partial | YASMIN HFSM in `duburi_planner` — 8 reusable states, 2 missions (gate, demo_square). Needs 4 more task SMs |
| **Vision Pipeline** | ✅ Functional | YOLO11n on CUDA (20-25 FPS on Orin Nano), Kalman tracking, PID visual servoing, multi-camera manager |
| **Feedback System** | ✅ Implemented | `/driver/feedback` with `accepted`/`reached`/`completed`/`rejected` status + error magnitude |
| **Teleop** | ✅ Implemented | `TeleopCommand` on `/driver/teleop` with multi-axis support + idle detection |
| **BlueOS Integration** | ✅ Functional | REST API client for system monitoring, parameter management, endpoint health checks |
| **Logging** | ✅ Functional | Session-based CSV/JSON logging with rotation |
| **Documentation** | ✅ Extensive | 26 analysis documents + comprehensive README |
| **Simulation** | 🔴 Not connected | Gazebo SITL stack documented (`17_SIMULATION`) but not integrated with ROS 2 pipeline |
| **DVL** | 🔴 Hardware only | Nortek Nucleus 1000 available, no ROS 2 driver integrated |
| **Actuators** | 🟡 Grabber only | Open/close servo. Torpedo launcher and dropper not integrated |
| **Acoustic Pinger** | 🔴 None | No hardware or software |

### Architecture Diagram (Current)

```
                     ┌─── CONTROLS ────────────────────┐   ┌──── PERCEPTION ──────────┐
                     │                                  │   │                           │
                     │  Pixhawk 2.4.8 (/dev/ttyACM0)   │   │  USB Camera(s)            │
                     │          │                       │   │       │                    │
                     │  mavlink_inspector (7 modules)   │   │  vision_inspector          │
                     │  ├─ connection_manager            │   │  (camera_manager)          │
                     │  ├─ telemetry_parser              │   │       │                    │
                     │  ├─ command_handler               │   │  /camera/<name>/image_raw  │
                     │  ├─ movement_commands             │   │       │                    │
                     │  ├─ rc_controller                 │   │  vision (detector_node)    │
                     │  ├─ pid_controller ×2             │   │       │                    │
                     │  └─ inspector_node                │   │  /vision/detections        │
                     │          │                       │   │       │                    │
                     │  /mavlink/vehicle_state           │   │  alignment_controller      │
                     │  /mavlink/events                  │   │  (PID visual servo)        │
                     │  /mavlink/diagnostics             │   │       │                    │
                     │  /driver/feedback                 │   └───────┼────────────────────┘
                     │          │                       │           │
                     │  /driver/command ◄───────────────┼───────────┘
                     │  /driver/teleop                  │
                     │          │                       │
                     │  ┌───────┴──────────┐            │
                     │  │ mavlink_runner   │            │     ┌──────────────────┐
                     │  │ mission_executor │            │     │ duburi_planner   │
                     │  │ teleop_driver    │            │     │ (YASMIN HFSM)    │
                     │  └──────────────────┘            │     │ → /driver/command│
                     │                                  │     └──────────────────┘
                     │  mavlink_logger → logs/           │
                     │  duburi_blueos → /blueos/*        │
                     └──────────────────────────────────┘
```

---

## 2. RoboSub 2026 Competition Task Analysis

> Source: [RoboNation Task Descriptions](https://robonation.gitbook.io/robosub-resources/section-3-autonomy-challenge/3.2-task-descriptions)
>
> Theme: *"Restore and Recovery"* — the AUV acts as a maintenance robot servicing a damaged underwater pipeline.

### 2.1 Task 1 — Begin Assessment (Gate)

**What:** Pass through a gate (two vertical poles with a horizontal bar). A "coin flip" mechanism determines which side the AUV enters from. After passing, the AUV selects a role (affects subsequent task order).

**Scoring:** Points for passing through; bonus for correct side entry after coin flip.

**First Principles Analysis:**
- **Core challenge:** Detect two vertical poles + crossbar, determine center, drive through.
- **Perception need:** Forward camera, YOLO model trained on gate poles (orange/white), bounding box center → alignment target.
- **Control need:** `go_forward` with heading hold + visual servo lateral correction. Depth hold at gate height.
- **Failure mode:** Gate not detected (sun glare, turbidity) → fallback to dead reckoning on known heading.

**What Duburi Has:**
- ✅ YOLO detection pipeline with Kalman tracking
- ✅ PID visual servoing (lateral + vertical + forward)
- ✅ `go_forward` with heading PID
- ✅ YASMIN gate mission (`gate.py`) with search → align → drive-through
- ❌ No coin-flip detection (requires reading a visual indicator)
- ❌ No custom YOLO model for RoboSub props (using generic COCO model)

**What Other Teams Do:**
- **Bumblebee:** YOLO + XFeat feature matching for precise pole localization, PnP for distance estimate, Selector fallback to dead reckoning.
- **Desert WAVE:** Surveyed GPS waypoints — drives to gate location without vision, HSV color detection as backup.

**Our Approach:**
```
SearchGate ──detected──→ AlignGate ──aligned──→ DriveThrough ──→ [done]
     │                        │
     └──timeout──→ DeadReckonGate ──→ DriveThrough
                        │
                  AlignGate ──lost──→ SearchGate (retry ×3)
```

**Gap Checklist:**
- [ ] Train custom YOLO model on RoboSub gate props (orange poles + crossbar)
- [ ] Implement coin-flip visual detection (or skip — low points vs complexity)
- [ ] Tune alignment PID for gate-width target (approach distance matters)
- [ ] Test dead reckoning fallback with known heading

---

### 2.2 Task 2 — Avoid Debris (Slalom)

**What:** Navigate through 3 sets of RED and WHITE vertical pipes arranged in a slalom pattern. The AUV must pass between each pair, alternating left and right.

**Scoring:** Points per gate cleared; bonus for completing all 3.

**First Principles Analysis:**
- **Core challenge:** Sequential detection of pipe pairs, alternating pass direction. Requires understanding which side to pass on.
- **Perception need:** Detect RED and WHITE pipes separately. Determine pipe pair geometry (gap center, which color is left/right). Multi-object tracking across 3 pairs.
- **Control need:** Series of lateral corrections + forward advance. Heading must be maintained throughout. Depth hold constant.
- **Failure mode:** Confusing pipe pairs (which is current target?), losing track mid-slalom, overcorrecting laterally.

**What Duburi Has:**
- ✅ YOLO detection (can distinguish colors if model is trained)
- ✅ Kalman tracking (currently single-object — needs extension)
- ✅ Lateral + forward visual servo
- ❌ No slalom state machine
- ❌ Single-object tracker won't handle multiple pipes simultaneously
- ❌ No spatial reasoning (which pipe pair am I at?)

**What Other Teams Do:**
- **Bumblebee:** Multi-object tracking + spatial reasoning in behaviour tree. Each pipe pair is a subtask with independent detection.
- **Desert WAVE:** Pre-surveyed waypoints between pipe pairs. No vision needed for navigation, HSV confirms pipe positions.

**Our Approach (Proposed):**
```
For each pipe pair (i = 1, 2, 3):
  SearchPipes_i ──detected──→ IdentifySide_i ──→ AlignPass_i ──→ DriveThrough_i
       │                                               │
       └──timeout──→ DeadReckonToNext_i                └──lost──→ SearchPipes_i
```

**Gap Checklist:**
- [ ] Train YOLO model to detect RED pipe and WHITE pipe as separate classes
- [ ] Extend `KalmanObjectTracker` to multi-object (track both pipes in a pair)
- [ ] Implement slalom state machine with 3 sub-iterations
- [ ] Add spatial logic: determine pass direction based on color arrangement
- [ ] Waypoint fallback: if vision fails, dead-reckon forward to next pair

---

### 2.3 Task 3 — Recon (Bins)

**What:** Locate bins on a 3D pipeline structure. Drop markers into the correct bins. Bins have visual indicators (symbols/colors) that determine which bin to target.

**Scoring:** Points per correct marker drop; penalties for wrong bin.

**First Principles Analysis:**
- **Core challenge:** Downward-looking detection of bins, marker identification, precise positioning above target, actuator release.
- **Perception need:** **Downward camera** (critical — forward camera can't see bins below). Detect bin outlines + symbols. Determine which bin matches the mission target.
- **Control need:** Hover over target bin with < 5cm precision. Depth hold at drop altitude. Zero lateral drift during drop.
- **Failure mode:** Bin symbol misidentification (wrong bin), lateral drift during drop, dropper actuator failure.

**What Duburi Has:**
- ✅ `camera_manager` supports multi-camera (can add downward camera)
- ✅ Depth PID for stable hover
- ❌ No downward camera integrated
- ❌ No dropper actuator
- ❌ No bin detection model
- ❌ No hover-and-drop state machine

**What Other Teams Do:**
- **Bumblebee:** Downward camera + DepthAnything for height estimation + YOLO for bin detection + PnP for precise positioning. Pneumatic dropper.
- **Desert WAVE:** GPS survey of bin locations, drive to waypoint, drop by position.

**Our Approach (Proposed):**
```
NavigateToBins ──arrived──→ SearchBins (downward cam) ──detected──→ IdentifyTarget
     │                            │                                      │
     └──timeout──→ [skip]         └──timeout──→ [skip]           AlignOverBin
                                                                        │
                                                                  DropMarker ──→ [done]
```

**Gap Checklist:**
- [ ] Mount and integrate downward camera (USB, V4L2)
- [ ] Train YOLO model for bin symbols (competition-specific props)
- [ ] Implement dropper actuator (solenoid/servo + `DriverCommand` handler)
- [ ] Implement hover-and-drop state machine
- [ ] Add downward visual servo mode (camera below, not forward)
- [ ] Test drop precision: ±5cm at 0.5m altitude

---

### 2.4 Task 4 — Deploy (Torpedoes)

**What:** Fire torpedoes at designated targets. An acoustic pinger indicates the task location. Targets have visual markers indicating valid strike zones.

**Scoring:** Points per hit; bonus for bullseye.

**First Principles Analysis:**
- **Core challenge:** Navigate to task area (acoustic pinger), detect torpedo targets, align precisely, fire.
- **Perception need:** Acoustic pinger detection for navigation. Forward camera for target detection. Precise alignment (tighter than gate — torpedo travel path must intersect target).
- **Control need:** Stable hover at firing distance. Zero angular drift during fire. Heading + depth + lateral all locked.
- **Failure mode:** Miss due to alignment error at fire moment, pinger localization error, torpedo mechanism jam.

**What Duburi Has:**
- ✅ PID visual servo (lateral + vertical + forward)
- ✅ Stable depth hold
- ❌ No acoustic pinger hardware or software
- ❌ No torpedo launcher actuator
- ❌ No torpedo target detection model
- ❌ No fire-and-verify state machine

**What Other Teams Do:**
- **Bumblebee:** MUSIC algorithm for pinger DOA (direction of arrival) with custom DAQ hardware. XFeat + PnP for target alignment at <1m distance.
- **Desert WAVE:** Subsonus acoustic positioning (not yet competition-tested).

**Our Approach (Proposed):**
```
If pinger available:
  ListenForPinger ──bearing──→ NavigateToPinger ──arrived──→ SearchTargets
Else:
  DeadReckonToArea ──→ SearchTargets

SearchTargets ──detected──→ AlignToTarget ──aligned──→ FireTorpedo ──→ [done]
     │                            │
     └──timeout──→ [skip]         └──lost──→ SearchTargets (retry ×3)
```

**Gap Checklist:**
- [ ] Research acoustic pinger hardware (hydrophones + DAQ)
- [ ] Implement torpedo launcher (mechanical + `DriverCommand` handler)
- [ ] Train YOLO model for torpedo targets
- [ ] Implement torpedo task state machine
- [ ] Tighten alignment PID for torpedo-level precision (±2cm lateral, ±2cm vertical)
- [ ] Test alignment stability at hover (must hold position for 1-2s while firing)

---

### 2.5 Task 5 — Resupply (Octagon)

**What:** Surface inside an octagonal structure. Pick up objects from the octagon. An acoustic pinger indicates the octagon location.

**Scoring:** Points for surfacing inside octagon; additional points per object retrieved.

**First Principles Analysis:**
- **Core challenge:** Navigate to octagon (pinger), detect octagon from below, position inside, surface. Then use grabber to pick up objects.
- **Perception need:** Upward-looking detection of octagon opening (from below). Object detection on surface. Acoustic pinger for navigation.
- **Control need:** Precise positioning under octagon center, controlled ascent, stable surface hold.
- **Failure mode:** Surfacing outside octagon (no points), grabber fails to secure object, pinger localization error.

**What Duburi Has:**
- ✅ `surface` command with controlled ascent
- ✅ Grabber actuator (open/close servo)
- ❌ No acoustic pinger
- ❌ No upward/downward octagon detection
- ❌ No octagon task state machine
- ❌ No object pickup sequence

**What Other Teams Do:**
- **Bumblebee:** Pinger DOA → navigate → upward camera detects octagon → center → surface → pick objects with manipulator.
- **Desert WAVE:** Surveyed octagon location, drive to waypoint, surface.

**Our Approach (Proposed):**
```
If pinger available:
  ListenForPinger ──bearing──→ NavigateToPinger ──arrived──→ SearchOctagon
Else:
  DeadReckonToArea ──→ SearchOctagon

SearchOctagon (upward cam) ──detected──→ CenterUnder ──centered──→ Surface
     │                                        │
     └──timeout──→ SurfaceBlind               └──lost──→ SearchOctagon

Surface ──surfaced──→ SearchObjects ──found──→ GrabObject ──→ [done]
```

**Gap Checklist:**
- [ ] Acoustic pinger (shared with Task 4 — one hardware investment serves both)
- [ ] Upward or downward camera for octagon detection
- [ ] Octagon detection model (geometric shape detection or YOLO)
- [ ] Object detection and grab sequence state machine
- [ ] Test surface-inside-octagon positioning precision
- [ ] Grabber reliability testing under water

---

### 2.6 Task 6 — Return Home (Gate)

**What:** Return through the starting gate from the opposite direction.

**Scoring:** Points for passing back through.

**First Principles Analysis:**
- **Core challenge:** Navigate back to start gate (may be far away), detect gate from reverse direction, pass through.
- **Perception need:** Same as Task 1 (gate detection) but from opposite direction.
- **Control need:** Long-range navigation back to start area + gate alignment + drive-through.
- **Failure mode:** Can't find way back (no absolute position reference), gate looks different from reverse side.

**What Duburi Has:**
- ✅ Gate detection + alignment (reuse Task 1 SM)
- ✅ Heading PID for return navigation
- ❌ No absolute position tracking (no DVL-based odometry)
- ❌ No path memory (no breadcrumb trail)

**Our Approach:**
```
TurnAround (180° from start heading) ──→ NavigateHome (reverse heading, timed)
     │
     └──→ SearchGate (reuse Task 1) ──→ AlignGate ──→ DriveThrough ──→ [done]
```

**Gap Checklist:**
- [ ] DVL odometry for position tracking during mission (enables return navigation)
- [ ] Dead reckoning fallback: reverse heading + timed drive
- [ ] Reuse gate state machine from Task 1

---

### 2.7 Cross-Cutting Competition Elements

**Path Markers (ORANGE, ~4ft × 6in):**
- Placed on pool floor between tasks to guide the AUV.
- Downward camera can detect them → heading correction.
- [ ] Train YOLO model on orange path markers
- [ ] Implement path-marker-follow state (detect → align heading → advance)

**Acoustic Pingers (at Torpedoes + Octagon):**
- 25-40 kHz pulsed signals.
- Require hydrophone array + DSP for direction-of-arrival estimation.
- [ ] Research hydrophone hardware options (cost vs accuracy)
- [ ] Implement pinger DOA algorithm (MUSIC, cross-correlation, or TDOA)

**20-Minute Time Limit:**
- Must prioritize tasks by points-per-time ratio.
- Gate (easy, fast) → Slalom (medium) → Bins (if downward cam ready) → Return Home.
- Torpedoes and Octagon require pinger — skip if hardware not ready.
- [ ] Implement mission timeout in top-level YASMIN SM (skip remaining tasks, go to Return Home)
- [ ] Profile per-task time in simulation

---

## 3. Phase 1 — Refine Controls

> **Goal:** Make the AUV's motion smooth, accurate, and robust. Every subsequent phase depends on reliable control.

### 3.1 Depth PID Tuning

- [ ] Pool-test current PID defaults (kp=500, ki=25, kd=200) at multiple depths (0.3m, 0.5m, 1.0m, 2.0m)
- [ ] Document steady-state error, overshoot, and settling time at each depth
- [ ] Create depth-specific gain schedules if performance varies significantly
- [ ] Test depth hold under forward thrust (coupling effects)
- [ ] Test depth hold during surfacing approach (slow, controlled ascent for octagon task)

### 3.2 Yaw PID Tuning

- [ ] Pool-test yaw PID (kp=2.0, ki=0.05, kd=0.5) for 90°, 180°, and 270° turns
- [ ] Measure heading accuracy after `go_forward` (does it hold heading under thrust?)
- [ ] Test `~turn` precision: command 90° turn, measure actual turn
- [ ] Tune for minimum overshoot — competition tasks need precise heading

### 3.3 Thruster Characterization

- [ ] Measure actual thrust vs PWM curve (nonlinear at extremes)
- [ ] Verify trapezoidal ramp rate is appropriate for pool conditions (too slow = sluggish, too fast = overshoot)
- [ ] Test `just_*` variants for emergency response time
- [ ] Measure battery voltage compensation effectiveness (`nominal_voltage` parameter)

### 3.4 Visual Servo Tuning

- [ ] Tune alignment controller PIDs for gate-sized target at 1-3m range
- [ ] Measure alignment settling time and precision (±pixels → ±cm at target distance)
- [ ] Test alignment stability: can it hold center for 5s without oscillation?
- [ ] Tune `lost_timeout` for competition turbidity (2s may be too short in murky water)
- [ ] Test `just_*` commands in alignment controller (per Issue 3 recommendation)

### 3.5 Motion Smoothness

- [ ] Profile jitter in PID output during steady-state hold (depth + yaw)
- [ ] Evaluate whether `MultiThreadedExecutor` reduces timing jitter (per Issue 7)
- [ ] Test cruise command (simultaneous bearing + heading + depth) for competition approaches
- [ ] Verify diagonal movement scaling (√2 factor) produces straight diagonal motion

---

## 4. Phase 2 — Autonomous Missions

> **Goal:** The AUV performs competition tasks autonomously — first via file-based missions, then via YASMIN state machine planner.

### 4.1 File-Based Mission Testing

- [ ] Write `.txt` mission files for each competition task approach pattern
- [ ] Test gate approach: `arm; ~depth 0.5; go forward <heading> 50% 8s; stop; surface; disarm`
- [ ] Test slalom pattern: series of `go forward-right`, `go forward-left` with heading holds
- [ ] Use file missions as **regression baselines** — if YASMIN produces worse results, debug against file mission

### 4.2 YASMIN State Machine — Complete Task Coverage

**Currently implemented:** `gate.py`, `demo_square.py`

- [ ] **Slalom SM:** 3 pipe-pair iterations with alternating pass direction
- [ ] **Bins SM:** Navigate → search (downward cam) → identify → align → drop
- [ ] **Torpedoes SM:** Navigate (pinger or dead reckon) → search → align → fire
- [ ] **Octagon SM:** Navigate (pinger or dead reckon) → search → center → surface → grab
- [ ] **Return Home SM:** Turn around → navigate → search gate → align → drive through
- [ ] **Top-level Mission SM:** Chain all task SMs with timeout transitions + task skipping

### 4.3 YASMIN Infrastructure

- [ ] Implement `mission_node.py` — ROS 2 node that constructs and runs the top-level SM
- [ ] Implement configurable mission ordering (blackboard parameters for task sequence)
- [ ] Add per-task timeout in top-level SM (skip task if timeout, proceed to next)
- [ ] Implement `watchdog.py` — monitor battery, leak sensor, heartbeat; trigger emergency surface
- [ ] Create `mission.launch.py` — launch planner + vision + inspector together
- [ ] Test YASMIN Viewer web UI during pool sessions (phone/laptop access)

### 4.4 Feedback Loop Integration

- [ ] Wire `wait_feedback` state to actually block on `/driver/feedback` `reached`/`completed` signals
- [ ] Add retry logic: if `rejected`, retry command up to N times before transitioning to fallback
- [ ] Test feedback timing: how long between command publish and `reached` callback?
- [ ] Ensure `mission_executor.py` also uses feedback (for file-based mission reliability)

---

## 5. Phase 3 — Perception Stack

> **Goal:** Forward + downward cameras with custom YOLO models trained on RoboSub competition props.

### 5.1 Multi-Camera Setup

- [ ] Mount downward-facing camera (USB, V4L2 compatible)
- [ ] Configure `camera_manager` for dual camera operation (forward: `duburi_cam`, downward: `duburi_down`)
- [ ] Verify both cameras stream simultaneously without frame drops
- [ ] Test USB bandwidth: two 640×480@30fps cameras + YOLO inference on single Orin Nano USB bus
- [ ] Add camera selection to alignment controller (forward vs downward mode)

### 5.2 Custom YOLO Models

**RoboSub prop classes to detect:**

| Class | Task | Camera | Priority |
|-------|------|--------|----------|
| `gate_pole` | Task 1, 6 | Forward | HIGH |
| `gate_crossbar` | Task 1, 6 | Forward | HIGH |
| `red_pipe` | Task 2 | Forward | HIGH |
| `white_pipe` | Task 2 | Forward | HIGH |
| `bin_symbol_A` ... | Task 3 | Downward | MEDIUM |
| `torpedo_target` | Task 4 | Forward | MEDIUM |
| `octagon_frame` | Task 5 | Upward/Downward | LOW |
| `path_marker` | Cross-cutting | Downward | MEDIUM |

- [ ] Collect training data: photos/video of RoboSub practice props from team pool sessions
- [ ] Augment with synthetic data: render 3D models in various water conditions (turbidity, lighting)
- [ ] Supplement with competition footage from other teams (YouTube, TDRs)
- [ ] Train YOLO11n (nano variant for Orin Nano) with custom classes
- [ ] Validate on held-out test set: target mAP > 0.8 for gate and slalom classes
- [ ] Export to TensorRT for maximum inference speed on Orin Nano
- [ ] Test detection under various water conditions (clear, turbid, backlit)

### 5.3 Multi-Object Tracking

- [ ] Extend `KalmanObjectTracker` to maintain N simultaneous tracks (currently single-object)
- [ ] Track management: birth (new detection), death (disappeared for N frames), association (Hungarian algorithm or IoU matching)
- [ ] Publish `TrackedObjectArray` with track IDs and predicted positions
- [ ] Test with slalom scenario: track both RED and WHITE pipes in a pair simultaneously

### 5.4 Pose Estimation (Stretch Goal)

- [ ] Evaluate DepthAnything V2 (small variant) on Orin Nano: FPS + memory usage
- [ ] If feasible: publish depth map alongside YOLO detections
- [ ] Alignment controller uses monocular depth for approach distance estimation
- [ ] If not feasible: use bounding box size as crude distance proxy (already implicit in forward PID)

### 5.5 Perception Reliability

- [ ] Implement detection confidence filtering per task (gate needs high confidence, fallback to dead reckon)
- [ ] Add temporal consistency check: N detections in M frames before triggering transition
- [ ] Test under competition-like conditions: moving water, other robots, spectators above pool
- [ ] Measure and log detection latency: camera frame → YOLO → published detection (target: <100ms)

---

## 6. Phase 4 — Additional Sensors

> **Goal:** DVL for distance-based navigation, external heading for compass accuracy.

### 6.1 DVL Integration (Nortek Nucleus 1000)

- [ ] Research existing ROS 2 DVL drivers (check for Nortek-specific packages)
- [ ] If no driver exists: write minimal ROS 2 node for Nucleus 1000 serial protocol
- [ ] Publish DVL velocity + position estimate on `/dvl/velocity` and `/dvl/position`
- [ ] Feed DVL into Pixhawk EKF via `VISION_POSITION_DELTA` MAVLink message
- [ ] Alternatively: implement software UKF that fuses IMU + DVL + barometer
- [ ] Implement `NavigateToWaypoint` YASMIN state: drive to (x, y, depth) using DVL position + heading PID
- [ ] Test position accuracy over 10m traverse: target ±20cm (sufficient for task area navigation)
- [ ] Calibrate DVL mounting: forward axis alignment with Pixhawk forward axis

**Why DVL is transformative:**
```
Without DVL:                          With DVL:
  "go forward 50% 5s" (open-loop)     "navigate to (3.0, 1.5, 0.5)" (closed-loop)
  → distance depends on current,       → actual position feedback,
    battery, thrust nonlinearity         corrects for drift in real-time
```

### 6.2 External Heading (Witmotion or similar)

- [ ] Evaluate Witmotion IMU/compass accuracy vs Pixhawk internal compass
- [ ] If Pixhawk compass has interference issues (motors, metal hull): mount external compass
- [ ] Feed external heading into Pixhawk via `GPS_INPUT` or `VISION_POSITION_DELTA`
- [ ] Alternatively: use external heading directly in software yaw PID (bypass Pixhawk heading)
- [ ] Test heading accuracy: command 360° rotation, measure cumulative drift

### 6.3 Acoustic Pinger Detection (Stretch Goal)

- [ ] Research hydrophone options: single element (bearing only) vs array (bearing + elevation)
- [ ] Evaluate MUSIC algorithm complexity vs Orin Nano compute budget
- [ ] If feasible: implement pinger DOA node publishing bearing on `/acoustics/pinger_bearing`
- [ ] Integrate pinger bearing into YASMIN navigation states for torpedoes and octagon tasks

---

## 7. Phase 5 — Utils & Infrastructure

> **Goal:** Dashboard, digital twin, calibration tools, unit tests — everything that makes the team faster.

### 7.1 Dashboard / Digital Twin

- [ ] Web dashboard showing real-time vehicle state (depth, heading, battery, mode, active task)
- [ ] Visualize YASMIN state machine state (integrate with YASMIN Viewer or custom)
- [ ] Camera feed display with detection overlays (already have `/vision/annotated_image`)
- [ ] Mission progress indicator (which task, time remaining, points estimate)
- [ ] Possible tech: Flask/FastAPI + WebSocket + React, or RViz2 custom panels

### 7.2 Calibration Tools

- [ ] Camera intrinsic calibration workflow documented and tested (`camera_calibrate` node exists)
- [ ] Thruster deadband calibration: find minimum PWM that produces motion (per thruster)
- [ ] PID auto-tune script: relay feedback (Ziegler-Nichols) for depth and yaw
- [ ] DVL mounting calibration: measure and correct for mounting angle offset
- [ ] Compass calibration procedure for competition venue (different magnetic environment)

### 7.3 Testing Infrastructure

- [ ] Connect Gazebo SITL to ROS 2 pipeline (documented in `17_SIMULATION` — needs actual integration)
- [ ] Create test world with gate, slalom pipes, and bins
- [ ] Run YASMIN missions in simulation before pool testing
- [ ] Implement `rosbag2` recording in launch files (record all topics during pool sessions)
- [ ] Build replay-and-evaluate workflow: replay bag → run detection → compare results
- [ ] Unit tests for state machine transitions (mock `/driver/feedback` and `/vision/detections`)
- [ ] Integration tests: full mission in simulation with assertions on task completion

### 7.4 Operations Runbook

- [ ] Pre-test checklist: hardware inspection, software version verification, battery charge, ballast check
- [ ] Day-of-competition procedure: course survey, heading calibration, mission parameter loading
- [ ] Emergency procedures: manual override, emergency surface, abort mission
- [ ] Post-test data collection: download logs, rosbag, camera recordings

### 7.5 Code Quality

- [ ] Fix version skew across package.xml and setup.py (Issue 4)
- [ ] Fix `perception.launch.py` default model mismatch (Issue 5)
- [ ] Add `stop` command on runner/executor shutdown (Issue 6)
- [ ] Align all launch files for consistent parameter naming
- [ ] Type hints across all Python files (gradual — prioritize driver_client and states)

---

## 8. Timeline & Dependencies

```
                    Apr 2026         May 2026         Jun 2026         Jul 2026
                    ─────────────────────────────────────────────────────────────
Phase 1 (Controls)  ████████░░░░░░░░░
                         │
Phase 2 (Missions)       ░░░████████████████░░░░░░░░░
                              │           │
Phase 3 (Perception)         ░░░░████████████████░░░░░
                                   │
Phase 4 (Sensors)                  ░░░░░░████████████████
                                              │
Phase 5 (Utils)             ░░░░░░░░░░░░░░████████████████
                    ─────────────────────────────────────────────────────────────
                                                         ▲
                                                    Competition
                                                    (late Jul / Aug)
```

### Critical Dependencies

```
Phase 1 (Controls) ──→ Phase 2 (Missions)    # Can't automate bad controls
Phase 3 (Perception) ──→ Phase 2 (Missions)   # Vision states need detection
Phase 4 (DVL) ──→ Phase 2 (Navigation states) # Waypoint nav needs position
Phase 5 (Simulation) ──→ Phase 2 (Testing)    # Mission testing needs sim
```

### Parallel Tracks

These can proceed independently:
- **Custom YOLO training** (Phase 3.2) — can start immediately with data collection
- **DVL hardware integration** (Phase 4.1) — independent of software missions
- **Simulation setup** (Phase 5.3) — independent of pool testing
- **Dashboard** (Phase 5.1) — independent of everything

---

## 9. Risk Register

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|-----------|
| Orin Nano thermal throttle with dual camera + YOLO + DVL | High | Medium | Profile early; use lazy inference (only activate when approaching task); TensorRT optimization |
| DVL integration delays (driver issues, Pixhawk EKF rejection) | Medium | High | Test with simulated DVL in Gazebo first; have dead-reckoning fallback for all tasks |
| Custom YOLO model underperforms on competition day (different lighting/turbidity) | Medium | High | Augment training data heavily; bring multiple model variants; have proportional-only fallback |
| Pool testing time insufficient for tuning | High | High | Prioritize simulation for logic testing; use pool time exclusively for PID tuning and hardware validation |
| YASMIN state explosion as tasks grow | Low | Medium | HFSM keeps per-level count low; reuse states across tasks |
| Actuator failure (torpedo jam, dropper stuck, grabber slip) | Medium | Medium | Mechanical testing in advance; software retry with timeout; graceful skip in mission SM |
| USB camera disconnect underwater (vibration, water ingress) | Medium | High | Secure connectors; camera health monitoring in `vision_inspector`; dead-reckon fallback |
| Battery depletion mid-mission | Low | Critical | Monitor voltage in watchdog; emergency surface if < threshold; pre-mission battery check |
| Competition rule changes | Low | Medium | Monitor RoboNation updates; keep architecture flexible |

---

## Quick Reference: What to Work on Now

**Immediate (this week):**
1. Pool-test control PIDs (Phase 1.1, 1.2) — foundation for everything
2. Start YOLO training data collection (Phase 3.2) — long lead time
3. Write slalom + return-home YASMIN SMs (Phase 2.2) — low-hanging fruit

**Next 2 weeks:**
4. DVL ROS 2 driver (Phase 4.1) — transforms navigation capability
5. Simulation integration (Phase 5.3) — unblocks rapid mission iteration
6. Multi-object tracking (Phase 3.3) — required for slalom

**Next month:**
7. Remaining YASMIN task SMs (bins, torpedoes, octagon)
8. Downward camera integration
9. Dashboard for competition day operations
