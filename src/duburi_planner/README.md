# duburi_planner — YASMIN FSM Mission Planner

**The central mission planning package for the BRACU Duburi AUV.**

Uses [YASMIN (Yet Another State MachINe)](https://github.com/uleroboticsgroup/yasmin) to
orchestrate RoboSub competition tasks as hierarchical finite state machines.
Every decision the AUV makes is a state transition — visible, logged, and
debuggable through the YASMIN web viewer.

---

## Quick Start

```bash
# 1. Build
cd ~/workspaces/duburi_ws
colcon build --packages-select duburi_planner

# 2. Source
source install/setup.bash

# 3a. Run the desk demo (safe — no arm needed)
ros2 run duburi_planner demo_node

# 3b. Run the full mission (requires armed vehicle + vision pipeline)
ros2 launch duburi_planner planner.launch.py

# 4. Open the YASMIN viewer in a browser
#    http://localhost:5000/
#    Filter: DUBURI_DEMO_SQUARE  or  DUBURI_MISSION
```

---

## Package Structure

```
duburi_planner/
├── package.xml                  # ROS 2 package manifest
├── setup.py / setup.cfg         # Python build config
├── README.md                    # ← you are here
│
├── config/
│   └── planner.yaml             # All tunables — edit between pool runs
│
├── launch/
│   └── planner.launch.py        # Starts mission_node + YASMIN viewer
│
└── duburi_planner/              # Python package
    ├── __init__.py
    ├── bb_utils.py              # Safe Blackboard access (YASMIN compat)
    ├── planner_config.py        # Typed dataclass config from ROS params
    ├── planner_context.py       # Shared ROS bridge (pubs/subs/caches)
    ├── mission_node.py          # Full mission entry point
    ├── demo_node.py             # Demo square entry point
    │
    ├── states/                  # ── Reusable atomic states ──
    │   ├── __init__.py          # Exports all states
    │   ├── arm.py               # Arm + set MANUAL mode (no dive)
    │   ├── submerge.py          # Arm → set mode → PID dive → stabilize
    │   ├── search.py            # Yaw-sweep scan for a YOLO class
    │   ├── align.py             # Visual servo via alignment_controller
    │   ├── drive.py             # Timed movement (with optional heading hold)
    │   ├── surface.py           # Ascend + disarm
    │   ├── wait_feedback.py     # Block until DriverCommandFeedback
    │   └── send_command.py      # One-shot command injection
    │
    └── missions/                # ── Task sub-state-machines ──
        ├── __init__.py
        ├── gate.py              # Task 1: Begin Assessment (gate pass)
        └── demo_square.py       # Desk test: forward + 90° turn × 4
```

---

## Architecture

### How It All Fits Together

```
┌─────────────────────────────────────────────────────────────────────┐
│                        mission_node.py                              │
│   Top-level StateMachine:                                           │
│                                                                     │
│   SUBMERGE ──→ GATE_TASK ──→ [future tasks] ──→ SURFACE            │
│      │             │                                │               │
│      └─failed──→ ABORT                   ←──failed──┘               │
│                                                                     │
│   Each task (GATE_TASK, etc.) is a nested sub-StateMachine          │
│   composed of reusable states from states/                          │
├─────────────────────────────────────────────────────────────────────┤
│                     PlannerContext                                   │
│   Shared object on Blackboard["ctx"]:                               │
│   • ROS node (single node for entire planner)                       │
│   • Publisher:  /driver/command                                     │
│   • Subscribers: /driver/feedback, /mavlink/vehicle_state,          │
│                  /vision/alignment_status, /vision/detections        │
│   • Thread-safe caches for latest messages                          │
│   • Helper: ctx.send('move_forward', duration=3, speed=50)          │
├─────────────────────────────────────────────────────────────────────┤
│                     PlannerConfig                                    │
│   Immutable snapshot of all ROS parameters.                         │
│   Source: config/planner.yaml                                       │
│   Per-task configs: cfg.tasks['gate'], cfg.tasks['slalom'], etc.    │
└─────────────────────────────────────────────────────────────────────┘
```

### YASMIN Concepts Used

| YASMIN Concept | How Duburi Uses It |
|---|---|
| `State` | Each atomic action (submerge, search, align, drive, surface) |
| `StateMachine` | Top-level mission + per-task sub-SMs |
| `CbState` | Lightweight setup/glue callbacks (parameter injection) |
| `Blackboard` | Shared data store — `ctx`, task parameters, heading snapshots |
| `YasminViewerPub` | Real-time FSM visualization in browser |

### The Blackboard

All states communicate through the YASMIN Blackboard. The most important key:

| Key | Type | Set By | Used By |
|---|---|---|---|
| `ctx` | `PlannerContext` | `mission_node` | Every state |
| `target_class` | `str` | Task SETUP state | SearchState, AlignState |
| `drive_duration` | `float` | Task SETUP state | DriveState |
| `drive_heading` | `float` | Lock heading callback | DriveState |
| `dive_depth` | `float` | Top-level or default | SubmergeState |

**Important:** YASMIN's `Blackboard.get(key)` does NOT accept a default value.
Always use `bb_get(blackboard, key, default)` from `bb_utils.py`.

---

## States Reference

### ArmState — `states/arm.py`

Arms the vehicle and sets MANUAL mode. Waits for telemetry confirmation
but proceeds gracefully if no Pixhawk is connected (desk testing).

| Outcome | Meaning |
|---|---|
| `armed` | Vehicle armed (confirmed or assumed after settle time) |
| `failed` | Arm explicitly rejected |

| Blackboard Key | Required | Default |
|---|---|---|
| `ctx` | Yes | — |

### SubmergeState — `states/submerge.py`

Arms the vehicle, sets MANUAL mode, activates PID depth hold, and waits
until target depth is reached.

| Outcome | Meaning |
|---|---|
| `submerged` | Vehicle armed and at target depth |
| `failed` | Arm rejected or depth timeout |

| Blackboard Key | Required | Default |
|---|---|---|
| `ctx` | Yes | — |
| `dive_depth` | No | `cfg.dive_depth` |

### SearchState — `states/search.py`

Rotates in place (yaw sweep) looking for a YOLO detection class.
Zigzags direction after a full 360° sweep.

| Outcome | Meaning |
|---|---|
| `found` | Target class detected |
| `timeout` | Search timeout expired |

| Blackboard Key | Required | Default |
|---|---|---|
| `ctx` | Yes | — |
| `target_class` | Yes | — |
| `search_yaw_step` | No | `30.0` degrees |
| `search_speed` | No | `cfg.default_speed` |
| `search_timeout` | No | `cfg.search_timeout` |

### AlignState — `states/align.py`

Activates visual servo alignment (delegates to `alignment_controller`)
and monitors `/vision/alignment_status` until fully aligned.

| Outcome | Meaning |
|---|---|
| `aligned` | All axes aligned |
| `lost` | Target disappeared |
| `timeout` | Alignment timeout |

| Blackboard Key | Required | Default |
|---|---|---|
| `ctx` | Yes | — |
| `target_class` | Yes | — |
| `alignment_mode` | No | `"pid_align"` |
| `alignment_timeout` | No | `cfg.alignment_timeout` |

### DriveState — `states/drive.py`

Timed open-loop movement. Optionally holds a heading via `go_forward`.

Set `stop_after=False` when the next state will immediately send another
movement command — the RC ramp handles the smooth transition without
jerking thrusters to zero. Set `stop_after=True` (default) when this is
the final movement before the vehicle should halt.

| Outcome | Meaning |
|---|---|
| `done` | Duration elapsed |

| Blackboard Key | Required | Default |
|---|---|---|
| `ctx` | Yes | — |
| `drive_command` | No | `"move_forward"` |
| `drive_duration` | No | `cfg.drive_through_time` |
| `drive_speed` | No | `cfg.default_speed` |
| `drive_heading` | No | `None` (no heading hold) |
| `stop_after` | No | `True` (send stop when done) |

### SurfaceState — `states/surface.py`

Ascends to surface and disarms.

| Outcome | Meaning |
|---|---|
| `surfaced` | At surface, disarmed |

### WaitFeedbackState — `states/wait_feedback.py`

Blocks until a `DriverCommandFeedback` message arrives on `/driver/feedback`.

| Outcome | Meaning |
|---|---|
| `reached` | Target achieved |
| `completed` | Duration command finished |
| `rejected` | Command rejected |
| `timeout` | No feedback in time |

### SendCommandState — `states/send_command.py`

Publishes a single `DriverCommand` and immediately transitions. Glue state
for injecting one-shot commands (stop, set_mode, etc.).

| Outcome | Meaning |
|---|---|
| `done` | Command published |

---

## Missions Reference

### Gate Task — `missions/gate.py`

RoboSub Task 1: Begin Assessment. Pass through the gate.

```
SETUP → SEARCH_GATE → ALIGN_GATE → LOCK_HEADING → DRIVE_THROUGH
                ↑          │
                └──lost─────┘    (re-search if target lost)
```

| Outcome | Meaning |
|---|---|
| `gate_passed` | Successfully drove through |
| `gate_failed` | Search or alignment failed |

Config keys (in `planner.yaml` under `gate:`):
- `target_class` — YOLO class name (default: `"gate"`)
- `search_yaw_step` — degrees per sweep step
- `search_speed` — rotation speed during search
- `approach_speed` — speed when driving through
- `drive_through_time` — seconds to drive after alignment
- `alignment_timeout` — max alignment time
- `search_timeout` — max search time

### Demo Square — `missions/demo_square.py`

Desk test: drive forward + PID turn 90° right, repeated 4 times.
Transitions between legs and turns are **smooth** — no stop command
between moves. The RC ramp handles accel/decel. Only the final turn
sends stop.

```
SETUP → LEG_1 → TURN_1 → LEG_2 → TURN_2 → LEG_3 → TURN_3 → LEG_4 → TURN_4
```

| Outcome | Meaning |
|---|---|
| `square_done` | All 4 legs completed |

---

## Configuration — `config/planner.yaml`

Every tunable parameter lives here. **Edit between pool runs — no rebuild needed.**

```yaml
mission_node:
  ros__parameters:
    default_speed: 50          # Global default PWM offset
    dive_depth: 0.6            # metres below surface
    arm_settle_time: 4.0       # seconds after arming
    dive_settle_time: 3.0      # seconds after reaching depth
    feedback_timeout: 10.0     # max wait for DriverCommandFeedback
    alignment_timeout: 15.0    # max visual alignment time
    search_timeout: 30.0       # max search scan time
    drive_through_time: 4.0    # default drive-through duration

    gate:                      # Per-task overrides
      target_class: "gate"
      search_timeout: 45.0
      alignment_timeout: 20.0
      drive_through_time: 5.0
      # ... etc.

    enable_viewer: true        # Start YASMIN web viewer
    viewer_name: "DUBURI_MISSION"
```

---

## Adding a New Task

1. **Create the mission file:** `duburi_planner/missions/my_task.py`

```python
from yasmin import StateMachine, CbState, Blackboard
from ..states.search import SearchState, FOUND, TIMEOUT
from ..states.align import AlignState, ALIGNED, LOST, TIMEOUT as ALIGN_TIMEOUT
from ..states.drive import DriveState, DONE

MY_TASK_DONE = "my_task_done"
MY_TASK_FAILED = "my_task_failed"

def _setup(blackboard: Blackboard) -> str:
    ctx = blackboard["ctx"]
    task_cfg = ctx.cfg.tasks['my_task']
    blackboard["target_class"] = task_cfg.target_class
    blackboard["search_timeout"] = task_cfg.search_timeout
    # ... set other parameters ...
    return "configured"

def build_my_task() -> StateMachine:
    sm = StateMachine(outcomes=[MY_TASK_DONE, MY_TASK_FAILED])
    sm.add_state("SETUP", CbState(["configured"], _setup),
                 transitions={"configured": "SEARCH"})
    sm.add_state("SEARCH", SearchState(),
                 transitions={"found": "ALIGN", "timeout": MY_TASK_FAILED})
    sm.add_state("ALIGN", AlignState(),
                 transitions={"aligned": "DRIVE", "lost": "SEARCH",
                              "timeout": MY_TASK_FAILED})
    sm.add_state("DRIVE", DriveState(),
                 transitions={"done": MY_TASK_DONE})
    return sm
```

2. **Add config** to `config/planner.yaml` under the task name.

3. **Register** in `planner_config.py` — add a `TaskConfig` entry.

4. **Wire** into `mission_node.py`'s `build_mission()`:

```python
sm.add_state("MY_TASK", build_my_task(),
             transitions={MY_TASK_DONE: "NEXT_TASK",
                          MY_TASK_FAILED: "SURFACE_ABORT"})
```

---

## YASMIN Web Viewer

The viewer is the **primary debugging tool** during pool tests. It shows:
- Which state is currently executing (highlighted)
- State transition history
- The full hierarchical structure of nested SMs

### How to use

```bash
# Started automatically by the launch file, or manually:
ros2 run yasmin_viewer yasmin_viewer_node

# Open in browser:
# http://localhost:5000/
# Use the dropdown to filter: DUBURI_MISSION or DUBURI_DEMO_SQUARE
```

When the AUV acts unexpectedly, the viewer shows exactly which state
it's in and which transition it took — no guessing required.

---

## Feedback System

The planner has full visibility into what the lower stack is doing:

1. **Command logging:** Every command published by the planner is logged
   with its parameters (command name, speed, duration, angle).

2. **DriverCommandFeedback:** The `WaitFeedbackState` listens for
   acknowledgements from `mavlink_inspector`. Feedback statuses:
   - `accepted` — command received and executing
   - `reached` — target achieved (depth, heading)
   - `completed` — duration-based command finished
   - `rejected` — invalid or disarmed
   - `timeout` — didn't reach target in time

3. **AlignmentStatus:** The `AlignState` monitors visual servo alignment
   in real time — error values, alignment flags, PID outputs.

4. **VehicleState:** The `PlannerContext` caches current depth, heading,
   armed status — any state can check `ctx.heading`, `ctx.depth`, etc.

---

## Entry Points

| Command | What It Does |
|---|---|
| `ros2 run duburi_planner mission_node` | Full mission (submerge → gate → surface) |
| `ros2 run duburi_planner demo_node` | Demo square (fwd + 90° turn × 4, desk-safe) |
| `ros2 launch duburi_planner planner.launch.py` | Mission node + YASMIN viewer |

---

## ROS 2 Topics

### Published
| Topic | Type | Purpose |
|---|---|---|
| `/driver/command` | `DriverCommand` | Movement commands to the vehicle |

### Subscribed
| Topic | Type | Purpose |
|---|---|---|
| `/driver/feedback` | `DriverCommandFeedback` | Command acknowledgements |
| `/mavlink/vehicle_state` | `VehicleState` | Depth, heading, armed status |
| `/vision/alignment_status` | `AlignmentStatus` | Visual servo alignment state |
| `/vision/detections` | `DetectionArray` | YOLO object detections |

---

## Design Principles

1. **Every state is reusable.** `SearchState` works for any YOLO class.
   `DriveState` works for any direction/duration. Parameters come from
   the Blackboard, not hardcoded.

2. **Configuration over code.** Pool-day tuning means editing
   `planner.yaml`, not Python files.

3. **Transparency.** Every command, transition, and decision is logged.
   The YASMIN viewer shows real-time state. No mystery why the AUV
   did something.

4. **Fail gracefully.** Every state has timeout/failure outcomes.
   Failed tasks surface the vehicle instead of hanging.

5. **Hierarchical composition.** Each RoboSub task is a sub-SM.
   The top-level mission chains them: Task1 → Task2 → ... → Surface.
   Add or remove tasks by editing `build_mission()`.
