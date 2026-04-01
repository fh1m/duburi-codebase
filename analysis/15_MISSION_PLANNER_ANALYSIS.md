# 15 — Mission Planner Analysis: YASMIN (FSM) vs Behaviour Trees

> **✅ IMPLEMENTATION STATUS (2026):** YASMIN has been implemented in the `duburi_planner` package. The analysis below remains valid as the architectural rationale. For the actual implementation details, see `16_PLANNER_DOCUMENTATION.md`.
>
> **What exists now:**
> - 8 reusable states: `arm.py`, `submerge.py`, `drive.py`, `surface.py`, `search.py`, `align.py`, `wait_feedback.py`, `send_command.py`
> - 2 missions: `gate.py`, `demo_square.py`
> - States inherit from `yasmin.State`, use `driver_client` functions, publish `DriverCommand` to `/driver/command`

Comprehensive comparison of **YASMIN** (Yet Another State MachINe) against **Py Trees** (behaviour trees) as the mission planning layer for the Duburi 4.2 ROS 2 stack. Evaluates both paradigms against the concrete codebase, existing integration points, and team context — including the 8th-place RoboSub 2025 finish using YASMIN.

> **Verdict: Use YASMIN.** Team experience, ROS 2-native integration, web viewer, and proven competition performance outweigh the structural elegance of behaviour trees at RoboSub scale.

---

## 1. How YASMIN Maps to the Duburi Codebase

YASMIN integration is clean and natural with the existing architecture. Every YASMIN primitive has a direct counterpart in Duburi's topic/message layer.

```
┌───────────────────── YASMIN State Machine ─────────────────────┐
│                                                                │
│  Mission SM                                                    │
│  ├── GateTask (sub-SM)                                        │
│  │   ├── SearchGateState ──→ driver_client.go_forward()       │
│  │   ├── AlignGateState  ──→ MonitorState /vision/alignment   │
│  │   ├── DeadReckonGate  ──→ driver_client.go_forward()       │
│  │   └── PassThrough     ──→ driver_client.go_forward()       │
│  ├── SlalomTask (sub-SM)                                      │
│  ├── TorpedoTask (sub-SM)                                     │
│  ├── BinTask (sub-SM)                                         │
│  └── OctagonTask (sub-SM)                                     │
│                                                                │
└────────────────── publishes DriverCommand via driver_client ───┘
         │                         │                    │
         ▼                         ▼                    ▼
  /driver/feedback      /vision/alignment_status  /mavlink/vehicle_state
  (DriverCommandFeedback)  (AlignmentStatus)       (VehicleState)
```

### Concrete mapping

| YASMIN Concept | Duburi Counterpart | Notes |
|---|---|---|
| `State.execute(blackboard)` | `driver_client` functions (`arm()`, `pid_depth()`, `go_forward()`) | State publishes `DriverCommand` via the canonical API |
| `MonitorState` (topic subscriber) | `/driver/feedback` (`DriverCommandFeedback`) | Wait for `status == 'reached'` or `'completed'` |
| `MonitorState` | `/vision/alignment_status` (`AlignmentStatus`) | Wait for `fully_aligned == True` |
| `CbState` (callback guard) | `/mavlink/vehicle_state` fields | Check `is_armed`, `depth_reached`, `flight_mode` |
| `Blackboard` | Mission parameters | Target heading, depth, task class name, waypoints, timeouts |
| `Concurrence` | Parallel operations | Hold depth PID while aligning laterally |
| HFSM (sub-state-machine) | Per-task encapsulation | Each competition task is a nested SM |
| `ActionState` | Future ROS 2 action servers | When/if tasks are exposed as actions |
| `PublisherState` | Direct topic publishing | Alternative to `State.execute()` for simple publishes |
| XML factory (YASMIN 4.x) | Mission files (`.txt`) | Non-coders can define mission sequences |

### Key integration files

| File | Role |
|---|---|
| `driver_client.py` | 30+ command builder functions — the canonical programmatic API |
| `DriverCommandFeedback.msg` | `accepted` / `reached` / `rejected` / `completed` status, `error` float |
| `AlignmentStatus.msg` | `fully_aligned`, per-axis flags, `target_detected`, `kalman_predicted` |
| `VehicleState` (topic) | `armed`, `depth`, `yaw`, `flight_mode` — published by `inspector_node` |

---

## 2. FSM vs Behaviour Tree — Structural Comparison

### 2.1 Finite State Machine (YASMIN)

```
States: INIT → SUBMERGE → SEARCH_GATE → ALIGN_GATE → PASS_THROUGH → NEXT_TASK
Transitions: explicit edges between states
Data flow: shared Blackboard
```

- Each state owns its logic and returns an **outcome string** (e.g., `"detected"`, `"timeout"`, `"aligned"`)
- Transitions are **explicit**: you define exactly which outcome goes to which next state
- Hierarchical: states can be entire sub-state-machines (HFSM)
- Concurrence: YASMIN 4.x supports `Concurrence` (parallel states with outcome maps)
- States block in `execute()` — no need to manage RUNNING across ticks

### 2.2 Behaviour Tree (Py Trees)

```
Root (Sequence)
├── Selector (fallback)
│   ├── VisionGate (Sequence: detect → align → pass)
│   └── DeadReckoningGate (Sequence: navigate → drive)
├── NextTask...
```

- Nodes are **tick-driven**: root ticks all children every cycle
- Nodes return `SUCCESS`, `FAILURE`, `RUNNING`
- **Implicit fallback**: Selector tries children in order until one succeeds
- **Reactive**: higher-priority branches can preempt lower ones mid-execution
- Long-running actions return `RUNNING` and manage internal state across ticks

### 2.3 Execution Model Difference

| Aspect | YASMIN (FSM) | Py Trees (BT) |
|---|---|---|
| Execution | States block in `execute()` | Tick-driven, nodes must be non-blocking |
| Control flow | Explicit transitions (`outcome → state`) | Implicit via tree structure (Selector/Sequence) |
| Fallback | Explicit timeout → fallback state | Structural: Selector auto-tries next child |
| Parallelism | `Concurrence` with outcome maps | Parallel composite with policies |
| Preemption | Manual: check conditions in each state | Structural: higher-priority branch preempts |
| Statefulness | Natural: each state is a blocking function | Must manage `RUNNING` state across ticks |
| Debuggability | YASMIN Viewer (web, real-time) | `render_dot_tree()` (static), py_trees_ros viewer |

---

## 3. Pros and Cons for Duburi Specifically

### YASMIN (FSM) — Pros

1. **Team already knows it.** 8th place at RoboSub 2025 is hard evidence. Learning curve is zero. For a small team, this alone is worth weeks of development time that would otherwise be spent learning BT execution semantics.

2. **Direct ROS 2 integration.** Built-in `MonitorState` (subscribe to topic and wait for a condition), `ActionState` (wrap a ROS 2 action), `ServiceState`, `PublisherState` — all map directly to Duburi's topic-based architecture without adapter code.

3. **Web viewer for pool-day debugging.** `yasmin_viewer` provides real-time FSM visualisation at `localhost:5000`. You can see which state the mission is in, what transitions are available, and what the current blackboard contains — from a phone or laptop poolside.

4. **XML factory.** YASMIN 4.x supports loading state machines from XML files, enabling non-coders to define mission sequences. This is conceptually similar to the current `.txt` mission files but with state logic, guards, and transitions.

5. **Simpler mental model.** "I'm in state X, I see outcome Y, I go to state Z" is easier to reason about under pool-day pressure than tick-based BT execution semantics with `SUCCESS`/`FAILURE`/`RUNNING` propagation.

6. **Concurrence support.** YASMIN `Concurrence` handles parallel states (e.g., hold depth + align laterally) with explicit outcome maps that determine when the concurrence resolves.

7. **Blackboard key remapping.** States can remap blackboard keys at construction time, enabling reuse of the same state class with different parameters for different tasks.

8. **Apt-installable.** `sudo apt install ros-humble-yasmin ros-humble-yasmin-*` — no source build needed. Active development: v4.2.4 (Jan 2026), ROS 2 Humble supported.

### YASMIN (FSM) — Cons

1. **No implicit fallback/reactivity.** If vision fails mid-state, the FSM doesn't automatically try an alternative. You must explicitly code timeout transitions and fallback states. In a BT, a Selector node handles this structurally.

2. **State explosion for complex tasks.** If a task has many possible failure modes, each needs an explicit transition edge. For N failure modes across M states, the transition graph grows combinatorially.

3. **No tick-based preemption.** A BT can preempt a running child when a higher-priority condition changes (e.g., "battery critical" interrupts any task). In YASMIN, you'd need to check conditions inside each state's execute loop or run a watchdog.

4. **Less composable for large mission trees.** Adding a new task to a BT is adding a subtree under a Sequence. Adding a new task to an FSM requires wiring transitions to/from all relevant states at the top level.

### Py Trees (BT) — Pros

1. **Structural fallback.** Selector nodes try alternatives automatically — "vision gate failed? try dead reckoning" is one tree node.

2. **Reactive preemption.** Condition nodes can abort running behaviours when circumstances change (battery low, obstacle detected).

3. **Better for large, complex missions.** Top-10 teams (Bumblebee, etc.) use BTs because they scale to 20+ tasks with cross-cutting concerns.

4. **More academic/industry adoption.** BTs are the dominant paradigm in game AI and increasingly in robotics. More documentation, papers, and community support.

### Py Trees (BT) — Cons

1. **Team has no experience.** Learning BT execution semantics (tick cycle, `SUCCESS`/`FAILURE`/`RUNNING`, blackboard scoping) takes real development time.

2. **No built-in web viewer.** `py_trees` has `render_dot_tree()` for static visualisation but no live web dashboard like YASMIN Viewer. `py_trees_ros` has a tree watcher but it's less polished.

3. **Tick-based execution is non-trivial.** A state that takes 5 seconds must return `RUNNING` on every tick until done, managing internal state across ticks. YASMIN states simply block in `execute()`.

4. **GPL-3.0 vs MIT licensing.** Py Trees is MIT. YASMIN is GPL-3.0 — not an issue for a non-commercial competition team, but worth noting.

---

## 4. Are There Major Setbacks with FSMs?

**For RoboSub specifically: No.** Here is why:

### 4.1 Task count is bounded

RoboSub has 5–6 tasks. The state graph is manageable. State explosion becomes a real problem at 20+ tasks with many cross-cutting concerns — well beyond RoboSub's scope.

### 4.2 Missions are largely sequential

Gate → Slalom → Torpedoes → Bins → Octagon. Even with fallbacks, the mission flow is mostly linear with per-task retry logic. FSMs handle linear-with-branches well.

### 4.3 YASMIN HFSM handles complexity

Each competition task becomes a sub-state-machine. The top-level FSM just sequences between task sub-machines. This keeps the per-level state count low and the transition graph readable.

### 4.4 Fallback is easy to add manually

For any state, add a `"timeout"` outcome that transitions to a fallback state. It's more explicit than a BT Selector, but equally functional for 5–6 tasks.

### 4.5 The one real setback

If you later need **reactive abort** (e.g., "obstacle detected, abandon current task immediately regardless of state"), this requires either:

- (a) Checking conditions in every state's execute loop, or
- (b) A watchdog thread that cancels the current state machine

BTs handle this more elegantly with high-priority condition nodes. For now, the existing `stop` + `surface` commands cover emergency abort, and a simple watchdog thread monitoring `/mavlink/vehicle_state` for critical conditions (battery low, leak detected) can trigger FSM cancellation.

---

## 5. Concrete Architecture with YASMIN

### 5.1 Top-Level Mission State Machine

```
                        ┌──────────┐
                        │   Init   │
                        └────┬─────┘
                  armed_and_submerged
                             │
                        ┌────▼─────┐     gate_timeout
                        │ GateTask ├──────────────┐
                        └────┬─────┘              │
                       gate_done                  │
                             ├────────────────────┘
                        ┌────▼──────┐   slalom_timeout
                        │SlalomTask ├──────────────┐
                        └────┬──────┘              │
                      slalom_done                  │
                             ├─────────────────────┘
                       ┌─────▼───────┐
                       │TorpedoTask  │
                       └─────┬───────┘
                       torpedo_done
                             │
                        ┌────▼────┐
                        │ BinTask │
                        └────┬────┘
                         bin_done
                             │
                      ┌──────▼──────┐
                      │ OctagonTask │
                      └──────┬──────┘
                       octagon_done
                             │
                      ┌──────▼──────┐
                      │ ReturnHome  │
                      └──────┬──────┘
                         surfaced
                             │
                          [done]
```

Timeout transitions allow the mission to skip a failed task and proceed to the next, maximising points within the 20-minute window.

### 5.2 Per-Task Sub-State-Machine (GateTask Example)

```
                 ┌────────────┐
                 │ SearchGate │
                 └──┬─────┬──┘
            detected│     │search_timeout
                    │     │
             ┌──────▼──┐  │  ┌───────────────┐
             │AlignGate │  └──▶DeadReckonGate │
             └──┬────┬──┘     └──────┬────────┘
         aligned│    │lost_target    │waypoint_reached
                │    │               │
                │  ┌─▼──────────┐    │
                │  │ SearchGate │    │
                │  └────────────┘    │
                │                    │
             ┌──▼────────────────────▼──┐
             │      PassThrough         │
             └──────────┬───────────────┘
                      passed
                        │
                     [done]
```

### 5.3 Implementation Sketch

```python
class SearchGateState(State):
    """Move forward slowly, monitor /vision/detections for gate class."""

    def __init__(self):
        super().__init__(["detected", "search_timeout"])

    def execute(self, blackboard):
        node = blackboard["node"]
        pub = blackboard["cmd_pub"]
        pub.publish(driver_client.go_forward(
            angle=blackboard["gate_heading"], duration=0, speed=30))

        deadline = node.get_clock().now() + Duration(seconds=15)
        while node.get_clock().now() < deadline:
            if blackboard.get("gate_detected"):
                return "detected"
            time.sleep(0.2)

        return "search_timeout"


class AlignGateState(MonitorState):
    """Monitor /vision/alignment_status until gate is aligned."""

    def __init__(self):
        super().__init__(
            AlignmentStatus,
            "/vision/alignment_status",
            ["aligned", "lost_target", "timeout"],
            self._handler,
            qos=10,
            msg_queue=10,
            timeout=15,
        )

    def _handler(self, blackboard, msg):
        if msg.fully_aligned:
            blackboard["gate_heading"] = msg.target_center_x
            return "aligned"
        if not msg.target_detected and not msg.kalman_predicted:
            return "lost_target"


class PassThroughState(State):
    """Drive forward through the gate for a fixed duration."""

    def __init__(self):
        super().__init__(["passed"])

    def execute(self, blackboard):
        pub = blackboard["cmd_pub"]
        pub.publish(driver_client.go_forward(
            angle=blackboard["gate_heading"], duration=5, speed=60))
        time.sleep(6)
        return "passed"
```

### 5.4 Wiring the GateTask Sub-Machine

```python
from yasmin import StateMachine

def build_gate_task():
    sm = StateMachine(outcomes=["gate_done", "gate_timeout"])

    sm.add_state("SEARCH", SearchGateState(),
                 transitions={
                     "detected": "ALIGN",
                     "search_timeout": "DEAD_RECKON",
                 })
    sm.add_state("ALIGN", AlignGateState(),
                 transitions={
                     "aligned": "PASS_THROUGH",
                     "lost_target": "SEARCH",
                     "timeout": "DEAD_RECKON",
                 })
    sm.add_state("DEAD_RECKON", DeadReckonGateState(),
                 transitions={
                     "waypoint_reached": "PASS_THROUGH",
                 })
    sm.add_state("PASS_THROUGH", PassThroughState(),
                 transitions={
                     "passed": "gate_done",
                 })

    return sm
```

### 5.5 Package Structure

**Proposed (from this analysis)** vs **Actual (as implemented):**

```
src/duburi_planner/                      # ✅ Package exists
├── package.xml                          # ✅
├── setup.py                             # ✅
├── duburi_planner/
│   ├── __init__.py                      # ✅
│   ├── mission_node.py                  # ❌ Not implemented (proposed)
│   ├── mission_builder.py               # ❌ Not implemented (proposed)
│   ├── states/                          # ✅ Exists with different file names
│   │   ├── __init__.py                  # ✅
│   │   ├── arm.py                       # ✅ (was proposed as init_state.py)
│   │   ├── submerge.py                  # ✅ (was proposed as init_state.py)
│   │   ├── search.py                    # ✅ (was proposed as search_state.py)
│   │   ├── align.py                     # ✅ (was proposed as align_state.py)
│   │   ├── drive.py                     # ✅ (was proposed as drive_state.py)
│   │   ├── surface.py                   # ✅ (was proposed as surface_state.py)
│   │   ├── wait_feedback.py             # ✅ (new — waits for /driver/feedback)
│   │   └── send_command.py              # ✅ (new — publishes DriverCommand)
│   ├── missions/                        # ✅ Exists (was proposed as mission_builder.py)
│   │   ├── __init__.py                  # ✅
│   │   ├── gate.py                      # ✅ Gate task sub-SM
│   │   └── demo_square.py              # ✅ Test pattern mission
│   └── watchdog.py                      # ❌ Not implemented yet
└── launch/
    └── mission.launch.py                # ❌ Not implemented yet
```

**Key differences from proposal:** States use individual files (one per state) rather than grouped files. Missions are in a separate `missions/` subdirectory rather than a single `mission_builder.py`. Two additional states (`wait_feedback`, `send_command`) were added beyond the original proposal. No `mission_node.py` orchestrator yet — missions are run directly.

---

## 6. YASMIN Feature Summary (v4.2.4)

| Feature | Description | Duburi Usage |
|---|---|---|
| `State` | Base class, `execute(blackboard) → outcome` | Wrap `driver_client` calls |
| `StateMachine` | Compose states with transitions | Per-task sub-machines |
| `MonitorState` | Subscribe to topic, callback returns outcome | `/driver/feedback`, `/vision/alignment_status` |
| `ActionState` | Wrap a ROS 2 action client | Future action-based tasks |
| `ServiceState` | Call a ROS 2 service | Future service-based queries |
| `PublisherState` | Publish a message as a state | Simple command publishes |
| `CbState` | Callback-based state (no subclass needed) | Quick guards and checks |
| `Concurrence` | Run states in parallel, outcome map | Depth hold + lateral align |
| HFSM | Nested state machines | Task sub-machines |
| `Blackboard` | Shared key-value store with remapping | Mission parameters, inter-state data |
| XML Factory | Load SM from XML files | Mission definitions without code changes |
| `yasmin_viewer` | Web visualisation at `:5000` | Pool-day debugging |

---

## 7. Addressing the Previous BT Recommendation

`13_COMPETITIVE_ANALYSIS.md` (Section 5, Tier 1, Item 1) recommended Py Trees behaviour trees based on Bumblebee's architecture. That recommendation was sound in the abstract — BTs are the dominant mission planning paradigm in top-tier teams. However, it did not account for:

1. **Team's existing YASMIN experience** — proven at competition level
2. **YASMIN's ROS 2-native tooling** — `MonitorState`, `ActionState`, web viewer
3. **RoboSub's bounded task count** — FSM drawbacks (state explosion, reactivity) don't materialise at 5–6 tasks
4. **Development velocity** — zero ramp-up time vs weeks learning BT semantics

The revised recommendation is: **use YASMIN** for the mission planner. If future competition requirements grow beyond FSM's sweet spot (20+ tasks, complex cross-cutting concerns), a BT migration can be considered. The `duburi_planner` package structure is designed so that state implementations (wrapping `driver_client` + monitoring feedback) are reusable regardless of the orchestration framework.

---

## 8. Migration Path from Current Mission System

The current mission system (`mission_executor.py` + `.txt` files) can coexist with YASMIN during the transition:

| Phase | Action | Current System | YASMIN | Status |
|---|---|---|---|---|
| 0 | Add `duburi_planner` package with YASMIN dependency | Fully operational | Empty scaffold | ✅ Done |
| 1 | Implement init states (arm + submerge + mode) | Still used for all missions | Can run init sequence | ✅ Done (`arm.py`, `submerge.py`) |
| 2 | Implement GateTask sub-SM | Used for non-gate tasks | Handles gate task | ✅ Done (`gate.py` with search → align → drive) |
| 3 | Add remaining task sub-SMs (slalom, torpedoes, bins, octagon) | Gradually retired | Handles all tasks | ⬜ Not started |
| 4 | Remove `mission_executor.py` | Deprecated | Primary mission system | ⬜ Blocked by Phase 3 |

At every phase, the runner CLI remains available for manual testing and ad-hoc commands. YASMIN handles autonomous missions only.

---

## 9. Risk Assessment (YASMIN-Specific)

| Risk | Probability | Impact | Mitigation |
|---|---|---|---|
| GPL-3.0 license conflict | Very Low | Low | Non-commercial competition use; no distribution |
| YASMIN development stalls | Low | Medium | Library is mature; our usage is basic; fork if needed |
| Reactive abort needed | Medium | Medium | Watchdog thread + FSM cancellation API (`StateMachine.cancel_state()`) |
| State explosion as tasks grow | Low | Medium | HFSM keeps per-level count low; RoboSub caps at ~6 tasks |
| Blackboard concurrency issues | Low | Low | Single-threaded executor; blackboard access is sequential |
| XML factory complexity | Low | Low | Optional feature; Python API is primary |

---

## 10. Summary

**YASMIN is the right choice for Duburi 4.2.** The rationale:

- **Proven** — 8th place finish at RoboSub 2025
- **Zero ramp-up** — team already knows the tool and its patterns
- **Clean integration** — `MonitorState` maps perfectly to `/driver/feedback` and `/vision/alignment_status`
- **Web viewer** — critical for pool-day debugging where terminal access is limited
- **ROS 2 Humble native** — apt-installable, actively maintained (v4.2.4, Jan 2026)
- **Sufficient for RoboSub scale** — 5–6 tasks with per-task HFSM is well within FSM's sweet spot
- **Gradual migration** — coexists with current mission executor during transition

BTs remain architecturally superior for very large systems, but for a team doing RoboSub with bounded tasks, YASMIN's practical advantages (experience, tooling, simplicity) outweigh BT's structural elegance.
