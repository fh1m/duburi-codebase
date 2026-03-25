# 14 — Issues, Design Critique, and Recommendations

Post-refactor analysis of the Duburi 4.2 codebase. Identifies current architectural issues, design debt, and recommends concrete next steps with implementation guidance. Cross-references `13_COMPETITIVE_ANALYSIS.md` for strategic context.

---

## Part A: Current Issues (Code-Level)

### Issue 1: No Watchdog / Health Monitor (Severity: HIGH)

**Problem:** If `mavlink_inspector` crashes or loses serial connection, upstream nodes (runner, executor, teleop, alignment controller) have no mechanism to detect this and take corrective action. The runner shows a "no telemetry" warning after 5 seconds, but mission executors and the alignment controller will continue publishing commands into the void.

**Impact:** During autonomous operation, a connection drop silently degrades to an uncontrolled vehicle. ArduSub will failsafe (disarm) after ~3 seconds of no RC override, but the mission executor doesn't know this happened.

**Recommendation:**
- Add a `/mavlink/heartbeat` topic (simple `std_msgs/Bool` at 1 Hz) from the inspector.
- Consumers check heartbeat liveness. If stale for >2s, alignment controller stops, executor pauses.
- Long-term: implement a `system_monitor` node that aggregates health from all nodes and can trigger emergency surface.

---

### Issue 2: Mission Executor Has No Feedback Loop (Severity: HIGH)

**Problem:** `mission_executor.py` publishes commands and uses `time.sleep()` between them. It does not subscribe to `/driver/feedback` to verify command acceptance or completion. If a command is rejected (e.g., trying to arm when Pixhawk rejects it), the executor blindly continues to the next step.

**Impact:** Missions can silently fail. The executor continues sending movement commands to a disarmed vehicle, wasting mission time.

**Recommendation:**
- Subscribe to `/driver/feedback` in the executor.
- After critical commands (`arm`, `set_mode`, `pid_depth`), wait for `status == 'accepted'` or `'reached'` with a timeout.
- On `'rejected'`, log error and retry or abort based on a configurable retry policy.
- This is a prerequisite for behaviour tree integration — BT leaf nodes need success/failure signals.

---

### Issue 3: Alignment Controller Publishes Raw DriverCommand (Severity: MEDIUM)

**Problem:** `alignment_controller.py` publishes `DriverCommand` with `command='move_left'`, `command='move_forward'`, etc. Each publish triggers a full command dispatch cycle in the inspector. At 10 Hz control rate, this generates 10 DriverCommand messages per second, each going through `CommandHandler.handle()` → movement lookup → ramp reset.

**Impact:** The trapezoidal ramp in `rc_controller` is constantly being reset by new commands. The visual servo effectively bypasses the ramp and creates jittery motion. Additionally, each alignment command overwrites any concurrent movement from the runner or executor.

**Recommendation:**
- Consider a dedicated `/vision/rc_correction` topic with raw PWM offsets that the inspector merges into the RC overlay as a new Layer 5 (vision correction), rather than going through the full command dispatch path.
- Alternatively, add a `source` field to `DriverCommand` so the inspector can handle vision-sourced commands differently (skip ramp, additive rather than overwrite).
- Short-term fix: alignment controller should use `just_*` commands (skip ramp) since it's already rate-limited at 10 Hz.

---

### Issue 4: Version Skew Between setup.py and package.xml (Severity: LOW)

**Problem:** `mavlink_inspector` has version `1.1.0` in `setup.py` but `1.0.0` in `package.xml`. This can cause confusion during packaging and is a hygiene issue.

**Fix:** Align versions across all packages. Consider a single `VERSION` file in the workspace root that all packages reference.

---

### Issue 5: perception.launch.py Uses Stale Default (Severity: LOW)

**Problem:** `vision_inspector/launch/perception.launch.py` defaults to `yolov8n.pt` as the model, while `vision/config.py` defaults to `yolo11n.pt`. Running via the launch file gives different behaviour than running `detector_node` directly.

**Fix:** Both should default to `yolo11n.pt`. Update `perception.launch.py`.

---

### Issue 6: No Graceful Shutdown Protocol (Severity: MEDIUM)

**Problem:** When a node receives SIGINT (Ctrl+C), there's no coordinated shutdown. The inspector stops sending RC overrides, which triggers ArduSub's failsafe (disarm after ~3s). But if only the runner or executor is killed, the inspector keeps sending whatever the last movement was until the movement duration expires.

**Impact:** Killing the runner during active movement doesn't stop the vehicle immediately. The operator must also send `stop` or kill the inspector.

**Recommendation:**
- On runner/executor shutdown, publish `DriverCommand(command='stop')` as a last message before exit.
- Inspector should detect subscriber disconnect on `/driver/command` and auto-stop after a timeout.
- Surface command on inspector shutdown (already partially implemented).

---

### Issue 7: Single-Threaded Serial I/O (Severity: MEDIUM)

**Problem:** `connection_manager.py` reads MAVLink on a timer callback in the single-threaded ROS executor. If MAVLink parsing takes longer than expected (large message burst), it can delay the RC override timer, which runs at 20 Hz.

**Impact:** Occasional missed RC override ticks under heavy MAVLink traffic. ArduSub failsafe is at ~3 seconds, so a few missed ticks are tolerable, but this adds jitter to PID control.

**Recommendation:**
- Move to a `MultiThreadedExecutor` with the serial read on a dedicated callback group.
- Or move MAVLink I/O to a background thread (like the display thread in `detector_node.py`).
- This becomes more critical as we add more subscribers and higher-rate processing.

---

## Part B: Architectural Design Critique

### Critique 1: Absence of a State Machine Layer

**Current state:** Mission logic is expressed as linear command sequences (text files or semicolons in the runner). There is no concept of "states" (IDLE, NAVIGATING, ALIGNING, TASK_EXECUTING, ERROR_RECOVERY) with defined transitions and guards.

**Why this matters:** Every competition task is fundamentally a state machine:

```
SEARCHING → DETECTED → APPROACHING → ALIGNED → EXECUTING → COMPLETED
     ↓           ↓            ↓           ↓          ↓
   TIMEOUT    LOST_TARGET   TOO_CLOSE   MISALIGNED  FAILED
     ↓           ↓            ↓           ↓          ↓
   ROTATE    RE-SEARCH     RETREAT      RE-ALIGN   RETRY/ABORT
```

Our current architecture has no way to express this. The alignment controller implements a micro-state machine internally (SEARCHING, ALIGNING, HOLDING), but this isn't exposed to a higher-level planner.

**Recommendation:** This is addressed by Issue #1 in the strategic priorities (13_COMPETITIVE_ANALYSIS.md) — YASMIN hierarchical state machine integration. The alignment controller's internal states (SEARCHING, ALIGNING, HOLDING) map directly to YASMIN `MonitorState` outcomes, and `/vision/alignment_status` provides the feedback signal that YASMIN states consume. See `15_MISSION_PLANNER_ANALYSIS.md` for the full architecture.

---

### Critique 2: Tight Coupling Between Command Parsing and Execution

**Current state:** `command_parser.py` in the runner and `mission_parser.py` in the driver both convert user text into `DriverCommand` messages. The `duburi_common.command_vocabulary` refactor helped, but the parsing logic is still interleaved with ROS publishing and `time.sleep()`.

**Why this matters:** A behaviour tree needs to issue commands programmatically, not by parsing text strings. The command generation API (`driver_client.py`) exists but is underutilised — the mission executor reimplements much of the same logic.

**Recommendation:**
- Establish `driver_client.py` as the canonical programmatic API. All command generation (runner, executor, YASMIN states, alignment controller) should call `driver_client` functions.
- `command_parser.py` should only translate text → `driver_client` function calls, not construct `DriverCommand` directly.
- This creates a clean `text → API → message` pipeline.

---

### Critique 3: No Separation of Navigation and Task Execution

**Current state:** The vision alignment controller and the runner/executor operate on the same conceptual level. There's no distinction between "navigate to the task area" and "execute the task."

**Why this matters (from Desert WAVE):** Navigation between tasks should use dead reckoning (DVL + IMU) — fast, reliable, and vision-independent. Vision should only activate for close-range task execution. This separation means:
- Vision failures during transit don't abort the mission
- The vehicle moves faster between tasks (no visual servo latency)
- Each phase has different control requirements (navigation: position control; task: visual servo)

**Recommendation:**
- Define a `NavigationMode` enum: `WAYPOINT`, `VISUAL_SERVO`, `MANUAL`
- Navigation controller handles waypoint-to-waypoint using DVL + heading PID
- Alignment controller handles close-range visual servo
- YASMIN state machine manages mode transitions

---

### Critique 4: PID Tuning Is Static

**Current state:** PID gains for depth and yaw are loaded from YAML config at startup and remain constant throughout operation. The alignment controller PIDs are similarly static.

**Why this matters:** Optimal PID gains change with:
- Depth (buoyancy changes, thruster efficiency varies)
- Battery voltage (thrust output degrades over time)
- Payload (grabber holding an object changes dynamics)
- Current/surge (environmental disturbances)

**Recommendation:**
- Short-term: gain scheduling — different PID profiles for different depth ranges (shallow pool vs 4m)
- Medium-term: adaptive gain based on battery voltage (we already have `nominal_voltage` compensation for thrust; extend to PID gains)
- Long-term: online PID auto-tuning (Ziegler-Nichols relay feedback or model reference adaptive control)

---

### Critique 5: No Data Recording / Replay Infrastructure

**Current state:** `mavlink_logger` records events and state to CSV/JSON. `camera_recorder` records frames. But there's no integrated `rosbag2` recording or replay capability.

**Why this matters:** Bumblebee uses recorded data extensively:
- Old recordings validate new perception pipelines
- Sensor data from pool tests can be replayed in simulation
- Regression testing: "does the new YOLO model still detect the gate in recording #47?"

**Recommendation:**
- Integrate `rosbag2` recording into `duburi_control.launch.py` as an optional argument
- Record topics: `/mavlink/vehicle_state`, `/vision/detections`, `/camera/*/image_raw`, `/driver/command`, `/driver/feedback`
- Build a replay-and-evaluate workflow for perception

---

## Part C: Concrete Next Steps (Phased Roadmap)

### Phase 1: YASMIN State Machine Foundation (est. 1–2 weeks)

**Goal:** Replace text-based missions with YASMIN hierarchical state machines.

1. Install YASMIN: `sudo apt install ros-humble-yasmin ros-humble-yasmin-ros ros-humble-yasmin-viewer`
2. Create `duburi_planner` package with:
   - `mission_node.py` — ROS 2 node, constructs & runs top-level SM
   - `mission_builder.py` — `build_gate_task()`, `build_slalom_task()`, etc.
   - `states/` — reusable state classes for each atomic action
   - `watchdog.py` — background thread for safety conditions
3. First sub-SM: a GateTask with fallback:
   ```
   GateTask (sub-SM)
   ├── SearchGate ──detected──→ AlignGate
   ├── SearchGate ──timeout───→ DeadReckonGate (fallback)
   ├── AlignGate ──aligned───→ PassThrough
   ├── AlignGate ──lost──────→ SearchGate (retry)
   └── PassThrough ──passed──→ [gate_done]
   ```
4. Each state wraps `driver_client` functions; `MonitorState` subscribes to `/driver/feedback` and `/vision/alignment_status`
5. Real-time debugging via `yasmin_viewer` web UI at `localhost:5000`

**Deliverables:** Working gate task with explicit fallback from vision to dead reckoning. See `15_MISSION_PLANNER_ANALYSIS.md` for the full architecture sketch.

---

### Phase 2: Simulation and DVL (est. 2–3 weeks)

**Goal:** Enable offline mission testing and position-based navigation.

1. **Gazebo simulation:**
   - Use `ros_gz` (Gazebo Harmonic) with underwater vehicle plugin
   - Model the competition pool with gate and bin props
   - Bridge simulated camera and IMU to ROS 2 topics
   - Run YASMIN state machine missions in simulation

2. **DVL integration:**
   - ROS 2 driver for Nortek Nucleus 1000 (community packages exist)
   - Feed DVL velocity into Pixhawk EKF (VISION_POSITION_DELTA or GPS_INPUT)
   - Add waypoint navigation state: `NavigateToWaypoint(x, y, depth)` using DVL position + heading PID

**Deliverables:** Simulated gate mission running autonomously. DVL providing position estimates.

---

### Phase 3: Advanced Perception (est. 2–3 weeks)

**Goal:** Multi-stage perception for robust task execution.

1. **Monocular depth estimation:**
   - Integrate DepthAnything V2 (small variant for Orin Nano)
   - Publish depth map alongside YOLO detections
   - Alignment controller uses depth estimate for approach distance

2. **Feature matching:**
   - XFeat for precise target localization after YOLO provides ROI
   - PnP pose estimation when target geometry is known (gate dimensions, bin markers)

3. **Multi-object tracking:**
   - Extend `KalmanObjectTracker` to maintain N simultaneous tracks
   - Track management: birth, death, occlusion handling
   - Publish `TrackedObjectArray.msg` with track IDs and predicted positions

**Deliverables:** 3-stage perception pipeline: YOLO → XFeat → PnP with fallback.

---

### Phase 4: Competition Polish (est. 1–2 weeks)

**Goal:** Reliability, testing, and operational readiness.

1. **rosbag2 integration** for recording and replay
2. **Health monitoring node** with Telegram alerts
3. **Regression test suite** using recorded data
4. **Mission library** — YASMIN sub-state-machines for each competition task
5. **Operations runbook** — pre-test checklist, day-of procedures
6. **Gain scheduling** for different depth ranges

---

## Part D: Quick Wins (Can Be Done Immediately)

These require minimal effort and have immediate benefit:

| # | Quick Win | Effort | Impact |
|---|-----------|--------|--------|
| 1 | Fix `perception.launch.py` default model to `yolo11n.pt` | 5 min | Consistency |
| 2 | Align version numbers across all package.xml and setup.py | 15 min | Hygiene |
| 3 | Add `stop` command publish on runner/executor shutdown | 30 min | Safety |
| 4 | Use `just_*` commands in alignment controller | 15 min | Smoother visual servo |
| 5 | Add `/driver/feedback` subscription in mission_executor | 1 hr | Reliability |
| 6 | Publish `DriverCommand('stop')` on alignment controller `vision_stop` | 15 min | Clean shutdown |
| 7 | Add `rosbag2 record` argument to `duburi_control.launch.py` | 30 min | Data collection |

---

## Part E: Risk Assessment

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|-----------|
| YASMIN state machine complexity slows development | Low | Medium | Team already has RoboSub 2025 experience; start with gate task sub-SM, grow incrementally |
| Orin Nano thermal throttling with DepthAnything + YOLO | High | Medium | Profile early; DepthAnything-small + YOLO11n should fit in 8 GB; use lazy inference (only when approaching target) |
| DVL integration delays (firmware/driver issues) | Medium | High | Test with simulated DVL in Gazebo first; have dead-reckoning fallback |
| YASMIN edge-case transitions missed | Low | Medium | YASMIN Viewer web UI shows live state + transitions; test each sub-SM independently |
| Simulation fidelity vs real pool | Medium | Medium | Use simulation for logic testing only; tune parameters in real pool |
| Hardware reliability (hull leaks, connectors) | Medium | High | Pre-test checklist, bench tests before every pool session (follow Bumblebee's approach) |
