# 16 — duburi_planner: Complete Theory, Implementation & Usage Guide

The central mission planning package for the BRACU Duburi AUV. This document covers **everything** — from "what is a state machine?" to "how do I create a new competition mission?" — so that a brand-new team member on day one of pool testing can understand, configure, and extend the planner.

---

## Table of Contents

1. [Why Does an AUV Need a Mission Planner?](#1-why-does-an-auv-need-a-mission-planner)
2. [What Is a Finite State Machine?](#2-what-is-a-finite-state-machine)
3. [What Is YASMIN?](#3-what-is-yasmin)
4. [How the Duburi Planner Works](#4-how-the-duburi-planner-works)
5. [Architecture — The Big Picture](#5-architecture--the-big-picture)
6. [Every Component Explained](#6-every-component-explained)
7. [The Blackboard — How States Share Data](#7-the-blackboard--how-states-share-data)
8. [Smooth Thruster Transitions (No Jerk)](#8-smooth-thruster-transitions-no-jerk)
9. [Running Existing Missions](#9-running-existing-missions)
10. [Configuring Missions (planner.yaml)](#10-configuring-missions-planneryaml)
11. [Creating Your Own Mission — Step by Step](#11-creating-your-own-mission--step-by-step)
12. [The YASMIN Web Viewer](#12-the-yasmin-web-viewer)
13. [Feedback & Acknowledgement System](#13-feedback--acknowledgement-system)
14. [State Reference (Complete)](#14-state-reference-complete)
15. [Mission Reference (Complete)](#15-mission-reference-complete)
16. [ROS 2 Topics & Interfaces](#16-ros-2-topics--interfaces)
17. [File-by-File Walkthrough](#17-file-by-file-walkthrough)
18. [Common Patterns & Recipes](#18-common-patterns--recipes)
19. [Troubleshooting](#19-troubleshooting)
20. [Quick Reference Cheat Sheet](#20-quick-reference-cheat-sheet)
21. [Demo Mission — Complete Deep Dive](#21-demo-mission--complete-deep-dive)

---

## 1. Why Does an AUV Need a Mission Planner?

### The Problem

An autonomous underwater vehicle must complete a sequence of tasks without human input. At RoboSub, those tasks are things like:

- Pass through a gate
- Navigate around obstacles (slalom)
- Fire torpedoes at targets
- Drop markers into bins
- Surface inside a specific area

Each task involves multiple steps: dive to depth, search for the target, align with it, perform an action, move to the next target. Some steps might fail — the camera doesn't see the gate, the vehicle drifts off course, a command times out.

Without a planner, you'd write one giant script with nested if/else statements, sleep calls, and retry loops. That code becomes unmaintainable within days of pool testing, and debugging "why did the AUV turn left instead of going forward?" becomes a nightmare.

### The Solution

A **mission planner** organises all these steps into a structured, visual, debuggable system. Instead of spaghetti code, you get:

- **Named states** — "SEARCH_GATE", "ALIGN_GATE", "DRIVE_THROUGH" — each doing exactly one thing
- **Explicit transitions** — "if alignment succeeded, go to DRIVE_THROUGH; if target lost, go back to SEARCH_GATE"
- **Real-time visibility** — a web dashboard shows which state is executing right now
- **Easy tuning** — change timeout from 10s to 15s in a YAML file, no code changes

This is what `duburi_planner` provides.

---

## 2. What Is a Finite State Machine?

### The Concept (From First Principles)

A **finite state machine** (FSM) is a mathematical model for systems that can be in exactly **one state** at a time, and that transition between states based on **events** or **conditions**.

Think of a traffic light:

```
     ┌─────┐  timer_expired  ┌────────┐  timer_expired  ┌─────┐
     │ RED │────────────────→│ GREEN  │────────────────→│YELLOW│
     └─────┘                 └────────┘                 └──┬───┘
        ↑                                                  │
        └──────────────────timer_expired───────────────────┘
```

At any moment, the light is in exactly ONE state (RED, GREEN, or YELLOW). When a timer expires, it transitions to the next state. The rules are explicit: GREEN always goes to YELLOW, never directly to RED.

### FSM Terminology

| Term | Meaning | Duburi Example |
|---|---|---|
| **State** | A named condition the system is in | `SEARCH_GATE` — rotating looking for the gate |
| **Transition** | A directed edge from one state to another | `found → ALIGN_GATE` |
| **Outcome** | The result of executing a state | `"found"`, `"timeout"`, `"aligned"` |
| **Event** | What triggers a transition | Detection received, timer expired, alignment achieved |
| **Initial state** | Where the FSM starts | `SUBMERGE` |
| **Terminal state** | Where the FSM ends (no outgoing transitions) | `mission_success`, `mission_aborted` |

### Hierarchical FSM (HFSM)

When FSMs get complex, you can **nest** them. A single state can itself be an entire sub-state-machine. This is called a **hierarchical FSM**.

In Duburi:
- The **top-level** FSM has states like `SUBMERGE`, `GATE_TASK`, `SURFACE`
- `GATE_TASK` is itself a sub-FSM with states `SETUP`, `SEARCH_GATE`, `ALIGN_GATE`, `LOCK_HEADING`, `DRIVE_THROUGH`

From the top level's perspective, `GATE_TASK` is just a single state with outcomes `gate_passed` and `gate_failed`. The internal complexity is hidden.

```
Top Level:   SUBMERGE → [GATE_TASK] → SURFACE
                            │
                    (internally):
             SETUP → SEARCH → ALIGN → LOCK → DRIVE
```

### Why FSM and Not Something Else?

There are other approaches — **behaviour trees** (BTs) are popular in game AI and large-scale robotics. For context, see `analysis/15_MISSION_PLANNER_ANALYSIS.md`. The short version:

- FSMs are **simpler to understand** — "I'm in state X, outcome Y, go to state Z"
- FSMs are **easier to debug** — you can point to exactly which state the robot is in
- FSMs work well when the **number of tasks is bounded** (RoboSub has ~6 tasks)
- The Duburi team has **proven experience** with FSMs (8th place at RoboSub 2025)

BTs become advantageous when you have 20+ tasks with complex cross-cutting concerns. RoboSub doesn't reach that threshold.

---

## 3. What Is YASMIN?

**YASMIN** = **Y**et **A**nother **S**tate **M**ach**IN**e

YASMIN is an open-source Python/C++ library for building state machines in ROS 2. It provides:

| Feature | What It Does |
|---|---|
| `State` | Base class — you subclass it, implement `execute()`, return an outcome string |
| `StateMachine` | Composes states with transition maps |
| `CbState` | Lightweight state from a plain function (no subclass needed) |
| `Blackboard` | Shared key-value store that states use to communicate |
| `MonitorState` | Built-in state that subscribes to a ROS topic and triggers on a condition |
| `ActionState` | Wraps a ROS 2 action client as a state |
| `Concurrence` | Runs multiple states in parallel with outcome policies |
| `YasminViewerPub` | Publishes FSM structure + current state to a web dashboard |

### How a YASMIN State Works

Every state has:
1. A list of **outcomes** (strings) — the possible results of executing this state
2. An `execute(blackboard)` method — the actual logic
3. The method **blocks** until it's done, then returns one outcome string

```python
from yasmin import State, Blackboard

class MyState(State):
    def __init__(self):
        super().__init__(outcomes=["success", "failure"])

    def execute(self, blackboard: Blackboard) -> str:
        # Do something...
        if everything_ok:
            return "success"
        return "failure"
```

### How a YASMIN StateMachine Works

A `StateMachine` is also a `State` — this is what enables nesting (HFSM). You add states to it with transition maps:

```python
from yasmin import StateMachine

sm = StateMachine(outcomes=["done", "failed"])

sm.add_state("STEP_ONE", MyState(),
             transitions={"success": "STEP_TWO", "failure": "failed"})

sm.add_state("STEP_TWO", AnotherState(),
             transitions={"success": "done", "failure": "failed"})

# Run it:
blackboard = Blackboard()
outcome = sm(blackboard)  # Returns "done" or "failed"
```

When the SM executes:
1. It starts at the first state added (`STEP_ONE`)
2. `STEP_ONE.execute(blackboard)` runs, returns e.g. `"success"`
3. The SM looks up the transition: `"success" → "STEP_TWO"`
4. `STEP_TWO.execute(blackboard)` runs, returns e.g. `"success"`
5. The SM looks up: `"success" → "done"` — `"done"` is a terminal outcome
6. The SM itself returns `"done"`

### Installing YASMIN

YASMIN is apt-installable for ROS 2 Humble:

```bash
sudo apt install ros-humble-yasmin ros-humble-yasmin-ros \
                 ros-humble-yasmin-viewer
```

Verify installation:

```bash
python3 -c "import yasmin; print(yasmin.__version__)"
ros2 run yasmin_viewer yasmin_viewer_node  # Should start on port 5000
```

---

## 4. How the Duburi Planner Works

### The Core Idea

`duburi_planner` sits between the **decision layer** ("what should the AUV do next?") and the **execution layer** (`mavlink_inspector` which actually moves thrusters).

```
┌──────────────────────────────────────────────────────┐
│                 duburi_planner                        │
│                                                      │
│  "The Brain" — decides WHAT to do and WHEN           │
│  Uses YASMIN FSM to orchestrate tasks                │
│  Publishes DriverCommand messages                    │
│                                                      │
│       ▼ publishes /driver/command                    │
├──────────────────────────────────────────────────────┤
│               mavlink_inspector                      │
│                                                      │
│  "The Muscles" — executes commands physically        │
│  Receives DriverCommand, sends RC_CHANNELS_OVERRIDE  │
│  Manages PID controllers (depth, yaw)                │
│  Sends DriverCommandFeedback back                    │
│                                                      │
│       ▼ sends RC PWM to Pixhawk                      │
├──────────────────────────────────────────────────────┤
│               Pixhawk (ArduSub)                      │
│                                                      │
│  "The Body" — ESCs, thrusters, sensors               │
└──────────────────────────────────────────────────────┘
```

The planner **never** talks to hardware directly. It only publishes `DriverCommand` messages. The inspector translates those into thruster movements. This separation means you can test the planner on a desk with no Pixhawk connected — watch the commands on `ros2 topic echo /driver/command`.

### Data Flow During a Mission

```
duburi_planner                  mavlink_inspector          vision pipeline
      │                               │                         │
      │── DriverCommand("arm") ──────→│                         │
      │                               │── arms Pixhawk ────→    │
      │←── VehicleState(armed=true) ──│                         │
      │                               │                         │
      │── DriverCommand("pid_depth") →│                         │
      │                               │── PID depth hold ──→    │
      │←── VehicleState(depth=0.6) ──│                         │
      │                               │                         │
      │   (SearchState rotating...)   │                         │
      │── DriverCommand("pid_yaw") ──→│                         │
      │                               │                         │
      │←─────────────────── DetectionArray("gate") ─────────────│
      │   (SearchState → "found")     │                         │
      │                               │                         │
      │── DriverCommand("pid_align") →│                         │
      │                               │                    (alignment_controller
      │←─────── AlignmentStatus(fully_aligned=true) ────────────│
      │                               │                     runs PID)
      │── DriverCommand("go_forward")→│                         │
      │                               │── RC forward ─────→     │
      │                               │                         │
      │── DriverCommand("stop") ─────→│                         │
      │── DriverCommand("disarm") ───→│                         │
```

---

## 5. Architecture — The Big Picture

### Package Location

```
~/workspaces/duburi_ws/src/duburi_planner/
├── package.xml                  # ROS 2 package manifest
├── setup.py / setup.cfg         # Python build config
├── README.md                    # Package-level README
│
├── config/
│   └── planner.yaml             # All tunables — edit between pool runs
│
├── launch/
│   └── planner.launch.py        # Starts mission_node + YASMIN viewer
│
└── duburi_planner/              # Python package
    ├── __init__.py
    ├── bb_utils.py              # Safe Blackboard access helper
    ├── planner_config.py        # Typed dataclass config from ROS params
    ├── planner_context.py       # Shared ROS bridge (pubs/subs/caches)
    ├── mission_node.py          # Full mission entry point
    ├── demo_node.py             # Demo square entry point
    │
    ├── states/                  # ── Reusable atomic states ──
    │   ├── __init__.py
    │   ├── arm.py               # Arm + set MANUAL mode (no dive)
    │   ├── submerge.py          # Arm + mode + PID dive + stabilize
    │   ├── search.py            # Yaw-sweep scan for a YOLO class
    │   ├── align.py             # Visual servo via alignment_controller
    │   ├── drive.py             # Timed movement (with optional heading hold)
    │   ├── surface.py           # Ascend + disarm
    │   ├── wait_feedback.py     # Block until DriverCommandFeedback
    │   └── send_command.py      # One-shot command injection
    │
    └── missions/                # ── Task sub-state-machines ──
        ├── __init__.py
        ├── gate.py              # Task 1: pass through the gate
        └── demo_square.py       # Desk test: forward + 90° turn × 4
```

### Layered Architecture

```
┌───────────────────────────────────────────────────────────────────┐
│                      Entry Points (Nodes)                        │
│   mission_node.py        demo_node.py                            │
│   Builds top-level SM    Builds demo SM                          │
│   Initialises ROS,       Same pattern,                           │
│   runs SM on blackboard  smaller mission                         │
├───────────────────────────────────────────────────────────────────┤
│                    Missions (Sub-State-Machines)                  │
│   gate.py              demo_square.py        (future tasks)      │
│   SEARCH→ALIGN→DRIVE   LEG→TURN→LEG→TURN    slalom, bins, etc.  │
│   Each is a StateMachine with named states and transitions       │
├───────────────────────────────────────────────────────────────────┤
│                    Reusable States (states/)                      │
│   ArmState   SubmergeState   SearchState   AlignState            │
│   DriveState  SurfaceState  WaitFeedbackState  SendCommandState  │
│   Each is a YASMIN State subclass with execute() → outcome       │
│   All parameterised via Blackboard — zero hardcoded values       │
├───────────────────────────────────────────────────────────────────┤
│                    Infrastructure                                 │
│   PlannerContext    PlannerConfig    bb_utils                     │
│   ROS node, pubs,  Typed config     Safe Blackboard              │
│   subs, caches     from YAML        access helper                │
├───────────────────────────────────────────────────────────────────┤
│                    External (other packages)                      │
│   duburi_interfaces   mavlink_inspector   vision pipeline        │
│   DriverCommand.msg   Executes commands   DetectionArray,        │
│   VehicleState.msg    PID controllers     AlignmentStatus        │
└───────────────────────────────────────────────────────────────────┘
```

### Execution Flow

When you run `ros2 run duburi_planner mission_node`:

1. **Init ROS** — creates a ROS 2 node
2. **Load config** — reads `planner.yaml` → `PlannerConfig` dataclass
3. **Create context** — `PlannerContext` sets up publisher and subscribers
4. **Build SM** — `build_mission()` wires states and transitions
5. **Start viewer** — `YasminViewerPub` publishes FSM to web dashboard
6. **Create Blackboard** — puts `ctx` (PlannerContext) on it
7. **Start executor** — background thread spins the ROS node (receives messages)
8. **Run SM** — `sm(blackboard)` executes blocking on the main thread
9. **Cleanup** — stop, disarm, shut down ROS

---

## 6. Every Component Explained

### 6.1 PlannerContext (`planner_context.py`)

**What it is:** The shared ROS bridge. Every state accesses ROS through this single object.

**Why it exists:** YASMIN states execute sequentially on a blocking thread. Creating separate ROS nodes or subscribers in each state would be wasteful and complex. Instead, one `PlannerContext` object manages everything and is stored on the Blackboard under the key `"ctx"`.

**What it provides:**

| Method/Property | What It Does |
|---|---|
| `ctx.send('move_forward', duration=3, speed=50)` | Build a `DriverCommand` and publish it to `/driver/command` |
| `ctx.heading` | Latest heading (yaw) from vehicle telemetry |
| `ctx.depth` | Latest depth from vehicle telemetry |
| `ctx.armed` | Whether the vehicle is armed |
| `ctx.vehicle_state` | Full latest `VehicleState` message (or `None`) |
| `ctx.alignment_status` | Full latest `AlignmentStatus` message (or `None`) |
| `ctx.detections` | Latest `DetectionArray` from the vision pipeline |
| `ctx.has_detection("gate")` | True if the latest detections contain the class "gate" |
| `ctx.wait_for_feedback(timeout=10)` | Block until a `DriverCommandFeedback` arrives |
| `ctx.log("message")` | Log via YASMIN (appears in terminal and viewer) |
| `ctx.warn("message")` | Log a warning |
| `ctx.sleep(seconds)` | Non-busy sleep |
| `ctx.cfg` | Access to `PlannerConfig` for tunable parameters |

**Thread safety:** The context uses a `threading.Lock` for all subscriber caches. The ROS executor runs in a background thread (receives messages), while the YASMIN FSM runs on the main thread (reads caches). The lock ensures no torn reads.

### 6.2 PlannerConfig (`planner_config.py`)

**What it is:** An immutable snapshot of all ROS parameters loaded from `planner.yaml`.

**Why it exists:** States need parameters (timeout values, speeds, depth targets). Instead of passing raw numbers or calling `node.get_parameter()` everywhere, we load everything once into a frozen dataclass. This means:
- Type safety (IDE autocomplete works)
- Single source of truth (change `planner.yaml`, everything updates)
- No parameter-fetching latency during mission execution

**Structure:**

```python
@dataclass(frozen=True)
class PlannerConfig:
    default_speed: int = 50        # Global PWM offset
    dive_depth: float = 0.6        # Mission start depth (metres)
    arm_settle_time: float = 4.0   # Seconds to wait after arming
    dive_settle_time: float = 3.0  # Seconds to stabilise at depth
    feedback_timeout: float = 10.0 # Max wait for feedback
    # ... etc.

    tasks: dict[str, TaskConfig]   # Per-task configs: tasks['gate'], tasks['slalom']

@dataclass(frozen=True)
class TaskConfig:
    target_class: str = "gate"     # YOLO class name
    search_yaw_step: float = 30.0  # Degrees per search rotation
    search_speed: int = 40         # Speed during search
    # ... etc.
```

Access in a state:

```python
def execute(self, blackboard):
    ctx = blackboard["ctx"]
    depth = ctx.cfg.dive_depth              # 0.6
    gate_cfg = ctx.cfg.tasks['gate']
    timeout = gate_cfg.search_timeout       # 45.0
```

### 6.3 bb_utils (`bb_utils.py`)

**What it is:** A one-function helper for safe Blackboard access.

**Why it exists:** YASMIN's `Blackboard.get(key)` does NOT accept a default value argument (unlike Python's `dict.get(key, default)`). Calling `blackboard.get(key, default)` raises a `RuntimeError`. Our helper:

```python
def bb_get(blackboard, key, default=None):
    if key in blackboard:
        return blackboard[key]
    return default
```

**Rule:** Always use `bb_get(blackboard, key, default)` instead of `blackboard.get(key, default)` for optional parameters. For required keys, `blackboard["key"]` is fine (it will raise a clear error if missing).

### 6.4 Entry Points

**`mission_node.py`** — Full competition mission:

```
SUBMERGE → GATE_TASK → [future tasks] → SURFACE
    │           │
    └─failed──→ SURFACE_ABORT ←──failed──┘
```

Initialises ROS, loads config, builds the full SM, starts the YASMIN viewer, and runs. Used during actual competition or full pool tests.

**`demo_node.py`** — Demo square mission:

```
ARM → SQUARE (LEG_1→TURN_1→LEG_2→TURN_2→LEG_3→TURN_3→LEG_4→TURN_4) → DISARM
```

Simplified entry point for desk testing and verifying the planner stack. Arms the vehicle, traces a square, disarms. Safe to run without a Pixhawk — commands are still published, but nothing physical happens.

---

## 7. The Blackboard — How States Share Data

The Blackboard is YASMIN's built-in key-value store. It's passed to every `execute()` call. States read configuration from it and write results back.

### Lifetime of a Blackboard Key

```
mission_node.py:
    blackboard["ctx"] = ctx          # Set once at startup

gate.py (_setup_gate_blackboard):
    blackboard["target_class"] = "gate"
    blackboard["search_timeout"] = 45.0
    blackboard["drive_heading"] = None

SearchState.execute():
    target = blackboard["target_class"]    # reads "gate"
    # ... searches ... finds gate ...

_record_heading():
    blackboard["drive_heading"] = 127.3    # writes heading after alignment

DriveState.execute():
    heading = bb_get(blackboard, "drive_heading", None)  # reads 127.3
```

### Key Conventions

| Key | Type | Set By | Used By |
|---|---|---|---|
| `ctx` | `PlannerContext` | Entry point node | Every state |
| `target_class` | `str` | Task SETUP callback | SearchState, AlignState |
| `search_timeout` | `float` | Task SETUP callback | SearchState |
| `search_yaw_step` | `float` | Task SETUP callback | SearchState |
| `search_speed` | `int` | Task SETUP callback | SearchState |
| `alignment_mode` | `str` | Task SETUP callback | AlignState |
| `alignment_timeout` | `float` | Task SETUP callback | AlignState |
| `drive_command` | `str` | Task SETUP callback | DriveState |
| `drive_duration` | `float` | Task SETUP callback | DriveState |
| `drive_speed` | `int` | Task SETUP callback | DriveState |
| `drive_heading` | `float` | Heading-lock callback | DriveState |
| `stop_after` | `bool` | Task SETUP or state | DriveState |
| `dive_depth` | `float` | Top-level or SETUP | SubmergeState |
| `feedback_timeout` | `float` | Task SETUP callback | WaitFeedbackState |
| `expected_command` | `str` | Task logic | WaitFeedbackState |
| `cmd_name` | `str` | Pre-command setup | SendCommandState |
| `cmd_kwargs` | `dict` | Pre-command setup | SendCommandState |
| `leg_duration` | `float` | Demo SETUP | DemoLegState |
| `turn_angle` | `float` | Demo SETUP | DemoTurnState |
| `demo_speed` | `int` | Demo SETUP | DemoLegState, DemoTurnState |
| `turn_settle` | `float` | Demo SETUP | DemoTurnState |

### Required vs Optional

- **Required keys** (accessed via `blackboard["key"]`): If missing, you get a `RuntimeError` — this is intentional, it means the mission is misconfigured.
- **Optional keys** (accessed via `bb_get(blackboard, "key", default)`): If missing, the default is used. This lets states be reusable without requiring every possible parameter to be set.

---

## 8. Smooth Thruster Transitions (No Jerk)

### The Problem

When the planner sends `move_forward` followed by `stop` followed by `pid_yaw_to_heading`, the thrusters:
1. Ramp up to forward speed
2. **Slam to zero** (stop)
3. Ramp up to yaw speed

That slam-to-zero causes a **physical jerk** — the AUV lurches, loses momentum, and wastes time. It also stresses the thrusters and ESCs.

### The Solution: The RC Velocity Ramp

The `mavlink_inspector` has a **trapezoidal velocity ramp** (`rc_controller.py`). Instead of instantly setting thruster PWM values, it smoothly transitions between them:

```
PWM
 ▲
 │          ┌────────────────────┐     ← target (e.g. forward at speed 50)
 │         ╱                      ╲
 │        ╱                        ╲   ← ramp_rate controls slope
 │       ╱                          ╲
 │──────╱                            ╲──────── neutral (1500)
 └──────────────────────────────────────────→ time
        ↑ command sent               ↑ new command sent
```

The ramp rate is 800 PWM/second. At 20 Hz (50ms per tick), that's 40 PWM per tick. A full range change (400 PWM offset) takes 0.5 seconds — smooth enough to feel natural.

### How the Planner Uses This

**Wrong (causes jerk):**
```python
ctx.send('move_forward', speed=50)
ctx.sleep(3)
ctx.send('stop')         # ← JERK: thrusters slam to zero
ctx.sleep(0.5)
ctx.send('pid_yaw_to_heading', angle=90)
```

**Correct (smooth transition):**
```python
ctx.send('move_forward', speed=50)
ctx.sleep(3)
# Do NOT send stop — the next command replaces the previous one
ctx.send('pid_yaw_to_heading', angle=90)  # ramp handles transition
```

The ramp sees that forward channel needs to go from speed-50 to neutral, and yaw channel needs to go from neutral to turn-speed. It ramps both simultaneously over ~0.5s. The result is a smooth arc rather than a stop-and-go.

### In Practice

The `DemoLegState` does NOT send `stop` at the end:

```python
class DemoLegState(State):
    def execute(self, blackboard):
        ctx.send('move_forward', speed=40)
        ctx.sleep(3.0)
        return "leg_done"           # NO stop — next state sends its own command
```

The `DemoTurnState` does NOT send `stop` either — unless it's the **final** turn:

```python
class DemoTurnState(State):
    def __init__(self, turn_number, is_final=False):
        ...
        self._is_final = is_final

    def execute(self, blackboard):
        ctx.send('pid_yaw_to_heading', angle=target)
        ctx.sleep(settle)
        if self._is_final:
            ctx.send('stop')        # Only at the very end
        return "turn_done"
```

### The `stop_after` Parameter in DriveState

`DriveState` has a `stop_after` Blackboard parameter:

```python
blackboard["stop_after"] = False    # Don't stop — another move follows
# or
blackboard["stop_after"] = True     # Do stop — this is the last movement
```

Default is `True` (stop after drive). Set it to `False` when the next state will immediately send another movement command.

### The Only Exception: `stop` Command

The `stop` command bypasses the ramp entirely — it's a **safety halt**. RC channels snap to neutral (1500) instantly. Use it:
- At the end of a mission
- In emergency situations
- When you genuinely want all thrusters to stop immediately

For normal state transitions, let the ramp handle it.

---

## 9. Running Existing Missions

### Prerequisites

```bash
# 1. Build the workspace
cd ~/workspaces/duburi_ws
colcon build --packages-select duburi_planner
source install/setup.bash

# 2. Verify YASMIN is installed
python3 -c "import yasmin; print('YASMIN OK')"
```

### Demo Square (Desk Testing)

```bash
# Terminal 1: Run the demo
ros2 run duburi_planner demo_node

# Terminal 2: Watch commands being published
ros2 topic echo /driver/command

# Terminal 3 (optional): Start the YASMIN web viewer
ros2 run yasmin_viewer yasmin_viewer_node
# Open http://localhost:5000/ → filter "DUBURI_DEMO_SQUARE"
```

**What happens:**
1. `ARM` state sends `arm` + `set_mode MANUAL`
2. `SQUARE` sub-SM executes 4 legs and 4 turns
3. `DISARM` state sends `surface` + `disarm`

No Pixhawk needed — commands are published to `/driver/command` but nothing physical moves without `mavlink_inspector` running and connected.

### Full Mission (Pool Testing)

```bash
# Terminal 1: Start the inspector (connects to Pixhawk)
ros2 run mavlink_inspector inspector

# Terminal 2: Start the vision pipeline
ros2 launch vision vision.launch.py

# Terminal 3: Launch the planner with YASMIN viewer
ros2 launch duburi_planner planner.launch.py

# Terminal 4 (laptop/phone): Open the YASMIN viewer
# http://<jetson-ip>:5000/ → filter "DUBURI_MISSION"
```

**What happens:**
1. `SUBMERGE` arms, sets MANUAL mode, dives to 0.6m via PID
2. `GATE_TASK` searches for the gate, aligns, locks heading, drives through
3. `SURFACE` ascends and disarms

### Override Parameters at Launch

```bash
# Deeper dive, longer search
ros2 launch duburi_planner planner.launch.py \
    dive_depth:=1.0 \
    gate.search_timeout:=60.0

# Or edit config/planner.yaml directly (no rebuild needed — just re-launch)
```

---

## 10. Configuring Missions (`planner.yaml`)

The file `config/planner.yaml` contains every tunable parameter. **Edit between pool runs — no rebuild needed.** Just re-launch.

### Full Configuration Reference

```yaml
mission_node:
  ros__parameters:

    # ═══ Global defaults ═══
    default_speed: 50           # PWM offset (0-100). Higher = faster.
                                # 50 is moderate. 30 is slow. 70 is fast.

    dive_depth: 0.6             # Target depth in metres below surface.
                                # 0.6 is safe for shallow pools.

    surface_depth: 0.0          # Depth to surface to (always 0).

    # ═══ Timing ═══
    arm_settle_time: 4.0        # Seconds to wait after arming.
                                # Gives the vehicle time to stabilise.
                                # Increase if arming is slow.

    dive_settle_time: 3.0       # Seconds to hold at target depth before
                                # starting the first task. Lets oscillations
                                # from the PID controller settle.

    feedback_timeout: 10.0      # Max seconds to wait for a
                                # DriverCommandFeedback message.
                                # If no feedback in this time, state times out.

    alignment_timeout: 15.0     # Max seconds for visual alignment.
                                # Vision PID servoing has this long to
                                # centre the target in frame.

    search_timeout: 30.0        # Max seconds for search rotation.
                                # If the target isn't found, search fails.

    drive_through_time: 4.0     # Default seconds to drive forward
                                # after alignment. Should be enough
                                # to pass through the target.

    # ═══ Gate Task (Task 1: Begin Assessment) ═══
    gate:
      target_class: "gate"      # YOLO class name. Must match the
                                # class name in your YOLO model.

      search_yaw_step: 30.0     # Degrees to rotate per search sweep.
                                # 30° × 12 steps = full 360° scan.

      search_speed: 40          # Rotation speed during search.

      approach_speed: 50        # Speed when driving through gate.

      drive_through_time: 5.0   # Seconds to drive through after alignment.

      alignment_timeout: 20.0   # Gate-specific override (wider than default).

      search_timeout: 45.0      # Gate-specific override (longer search).

    # ═══ Future tasks (placeholders) ═══
    slalom:
      target_class: "pipe_red"
      search_timeout: 30.0

    bins:
      target_class: "bin"
      search_timeout: 30.0

    # ═══ YASMIN Viewer ═══
    enable_viewer: true         # Set to false to disable web viewer.
    viewer_name: "DUBURI_MISSION"  # Name shown in viewer dropdown.
```

### Pool-Day Tuning Workflow

1. Run the mission, observe the AUV behaviour
2. If the gate search times out too early → increase `gate.search_timeout`
3. If the AUV doesn't drive far enough through the gate → increase `gate.drive_through_time`
4. If alignment is slow → increase `gate.alignment_timeout`
5. If the AUV is too fast → decrease `gate.approach_speed`
6. Edit `config/planner.yaml`, re-launch. No rebuild needed.

---

## 11. Creating Your Own Mission — Step by Step

This section walks through creating a complete new task from scratch. We'll use the **Slalom** task (Task 2: Avoid Debris) as an example.

### Step 1: Understand the Task

RoboSub Slalom: Navigate between coloured pipe markers without touching them. The AUV must detect red and green pipes, navigate around them.

Plan:
```
SETUP → SEARCH_PIPE → ALIGN_PIPE → NAVIGATE_AROUND → CHECK_MORE_PIPES → (done/failed)
```

### Step 2: Create the Mission File

Create `duburi_planner/missions/slalom.py`:

```python
"""
Slalom mission — RoboSub Task 2: Avoid Debris.

Sub-state-machine:
    SETUP → SEARCH_PIPE → ALIGN_PIPE → NAVIGATE_AROUND → (done/failed)

Outcomes:
    "slalom_passed"  — navigated through all pipes
    "slalom_failed"  — search or alignment failed
"""

from __future__ import annotations

from yasmin import StateMachine, CbState, Blackboard

from ..states.search import SearchState, FOUND, TIMEOUT as SEARCH_TIMEOUT
from ..states.align import AlignState, ALIGNED, LOST, TIMEOUT as ALIGN_TIMEOUT
from ..states.drive import DriveState, DONE as DRIVE_DONE

SLALOM_PASSED = "slalom_passed"
SLALOM_FAILED = "slalom_failed"


def _setup(blackboard: Blackboard) -> str:
    ctx = blackboard["ctx"]
    cfg = ctx.cfg.tasks['slalom']

    blackboard["target_class"] = cfg.target_class
    blackboard["search_timeout"] = cfg.search_timeout
    blackboard["alignment_mode"] = "pid_align"
    blackboard["drive_command"] = "move_forward"
    blackboard["drive_duration"] = 3.0
    blackboard["stop_after"] = True

    ctx.log("SLALOM — parameters loaded")
    return "configured"


def build_slalom_task() -> StateMachine:
    sm = StateMachine(outcomes=[SLALOM_PASSED, SLALOM_FAILED])

    sm.add_state("SETUP", CbState(["configured"], _setup),
                 transitions={"configured": "SEARCH_PIPE"})

    sm.add_state("SEARCH_PIPE", SearchState(),
                 transitions={FOUND: "ALIGN_PIPE",
                              SEARCH_TIMEOUT: SLALOM_FAILED})

    sm.add_state("ALIGN_PIPE", AlignState(),
                 transitions={ALIGNED: "NAVIGATE",
                              LOST: "SEARCH_PIPE",
                              ALIGN_TIMEOUT: SLALOM_FAILED})

    sm.add_state("NAVIGATE", DriveState(),
                 transitions={DRIVE_DONE: SLALOM_PASSED})

    return sm
```

### Step 3: Add Config to planner.yaml

```yaml
    slalom:
      target_class: "pipe_red"     # YOLO class for slalom pipes
      search_timeout: 30.0
      search_speed: 35
      approach_speed: 40
      drive_through_time: 3.0
      alignment_timeout: 15.0
```

### Step 4: Register in planner_config.py

In `load_config()`, the slalom config is already loaded. If you added new fields, update `TaskConfig` and the loading code:

```python
slalom_cfg = TaskConfig(
    target_class=_p('slalom.target_class', 'pipe_red'),
    search_timeout=_p('slalom.search_timeout', 30.0),
    search_speed=_p('slalom.search_speed', 35),
    approach_speed=_p('slalom.approach_speed', 40),
    drive_through_time=_p('slalom.drive_through_time', 3.0),
    alignment_timeout=_p('slalom.alignment_timeout', 15.0),
)
```

### Step 5: Wire Into mission_node.py

```python
from .missions.slalom import build_slalom_task, SLALOM_PASSED, SLALOM_FAILED

def build_mission(ctx):
    sm = StateMachine(outcomes=[MISSION_SUCCESS, MISSION_ABORTED], ...)

    sm.add_state("SUBMERGE", SubmergeState(), ...)

    sm.add_state("GATE_TASK", build_gate_task(),
                 transitions={GATE_PASSED: "SLALOM_TASK",    # ← chain to slalom
                              GATE_FAILED: "SURFACE_ABORT"})

    sm.add_state("SLALOM_TASK", build_slalom_task(),
                 transitions={SLALOM_PASSED: "SURFACE",       # ← continue to surface
                              SLALOM_FAILED: "SURFACE_ABORT"})

    sm.add_state("SURFACE", SurfaceState(), ...)
    sm.add_state("SURFACE_ABORT", SurfaceState(), ...)

    return sm
```

### Step 6: Export from missions/__init__.py

```python
from .slalom import build_slalom_task, SLALOM_PASSED, SLALOM_FAILED
```

### Step 7: Build and Test

```bash
colcon build --packages-select duburi_planner
source install/setup.bash

# Desk test — watch command sequence
ros2 run duburi_planner mission_node &
ros2 topic echo /driver/command

# Or run with the YASMIN viewer
ros2 launch duburi_planner planner.launch.py
# Open http://localhost:5000/ — you'll see SLALOM_TASK in the hierarchy
```

### Step 8: Pool-Day Tuning

At the pool, just edit `planner.yaml`:
```yaml
    slalom:
      search_timeout: 45.0    # Give more time in murky water
      approach_speed: 35      # Slow down for precision
```

Re-launch. No rebuild needed.

---

## 12. The YASMIN Web Viewer

### What It Is

The YASMIN web viewer (`yasmin_viewer`) is a Flask-based web UI that visualises your FSM in real time. It shows:

- The full state machine hierarchy (nested sub-SMs are expandable)
- Which state is **currently executing** (highlighted in green)
- Available transitions from each state
- The outcome of each completed state

### Why It Matters

During pool testing, you can't stare at a terminal. The viewer runs on port 5000 and is accessible from any device on the same network — your phone, a laptop on the pool deck, a tablet taped to the pool wall.

When the AUV does something unexpected, you open the viewer and see exactly:
- "Oh, it's stuck in SEARCH_GATE — timeout hasn't been reached yet"
- "It went from ALIGN_GATE to SEARCH_GATE — that means it lost the target"
- "It's in SURFACE_ABORT — the gate search failed entirely"

No guessing. No parsing log files. Just a visual.

### How to Use

```bash
# Option A: Started automatically by the launch file
ros2 launch duburi_planner planner.launch.py
# The viewer is started alongside mission_node

# Option B: Start manually
ros2 run yasmin_viewer yasmin_viewer_node
```

Open in a browser: `http://<jetson-ip>:5000/`

Use the dropdown at the top to filter:
- `DUBURI_MISSION` — the full competition mission
- `DUBURI_DEMO_SQUARE` — the demo square mission

### Network Access

If the Jetson is on WiFi at `192.168.1.42`:
```
http://192.168.1.42:5000/
```

From the Jetson itself:
```
http://localhost:5000/
```

### Disabling the Viewer

If you don't need it (saves a tiny bit of CPU):

```bash
ros2 launch duburi_planner planner.launch.py enable_viewer:=false
```

Or in `planner.yaml`:
```yaml
    enable_viewer: false
```

---

## 13. Feedback & Acknowledgement System

### The Problem

If the planner sends `pid_depth 0.6` and then immediately moves to the next state, how does it know the vehicle actually reached 0.6m? What if arming failed? What if the command was rejected because the vehicle was already disarmed?

### The Solution: Multi-Layer Feedback

The planner has **three feedback channels**:

#### 1. DriverCommandFeedback (`/driver/feedback`)

The `mavlink_inspector` publishes feedback after processing each command:

| Status | Meaning |
|---|---|
| `accepted` | Command received and executing |
| `reached` | Target achieved (e.g., depth reached, heading reached) |
| `completed` | Duration-based command finished |
| `rejected` | Invalid command or vehicle not armed |
| `timeout` | Didn't reach target in time |

The `WaitFeedbackState` blocks until feedback arrives:

```python
# In a mission:
sm.add_state("DIVE", SubmergeState(), ...)
sm.add_state("WAIT_DEPTH", WaitFeedbackState(),
             transitions={"reached": "SEARCH",
                          "timeout": "ABORT",
                          "rejected": "ABORT"})
```

#### 2. VehicleState (`/mavlink/vehicle_state`)

Real-time telemetry from the Pixhawk via the inspector:

```python
ctx.armed        # Is the vehicle armed?
ctx.heading      # Current yaw in degrees (0-360)
ctx.depth        # Current depth in metres
```

States use this for confirmation polling:

```python
# In SubmergeState:
while time.monotonic() < deadline:
    if abs(ctx.depth - target) < 0.15:
        return SUBMERGED
    ctx.sleep(0.5)
```

#### 3. AlignmentStatus (`/vision/alignment_status`)

Real-time visual servo feedback from the vision pipeline:

```python
status = ctx.alignment_status
status.fully_aligned     # Are all axes aligned?
status.error_x           # Horizontal error (-1 to +1)
status.error_y           # Vertical error (-1 to +1)
status.target_detected   # Is the target in frame?
```

The `AlignState` monitors this until `fully_aligned == True`.

#### 4. YASMIN Logging

Every state logs its actions via YASMIN's logging system:

```
[planner] ARM — sending arm + MANUAL mode
[planner] ARM — confirmed armed via telemetry
[planner] ARM — ready (MANUAL mode set)
[planner] SEARCH — scanning for 'gate' (timeout=45.0s)
[planner] SEARCH — 'gate' detected!
[planner] ALIGN — 'pid_align_forward' on 'gate' (timeout=20.0s)
[planner] ALIGN — fully aligned to 'gate' (err_x=0.012 err_y=0.008)
[planner] GATE — locked heading 127.3° for drive-through
[planner] DRIVE — go_forward heading=127.3° dur=5.0s spd=50
[planner] DRIVE — complete
```

These appear in the terminal AND in ROS logs. Combined with the YASMIN viewer, you have complete visibility into every decision.

---

## 14. State Reference (Complete)

### ArmState (`states/arm.py`)

**Purpose:** Arm the vehicle and set MANUAL mode.

**Behaviour:**
1. Sends `arm` command
2. Polls `ctx.armed` for confirmation (up to `arm_settle_time`)
3. If no telemetry (desk testing), warns and continues
4. Sends `set_mode MANUAL`
5. Returns `armed`

| Outcome | Meaning |
|---|---|
| `armed` | Vehicle armed (confirmed or assumed) |
| `failed` | Arm explicitly rejected |

| Blackboard Key | Required | Default |
|---|---|---|
| `ctx` | Yes | — |

---

### SubmergeState (`states/submerge.py`)

**Purpose:** Arm, set mode, dive to target depth, wait for stabilisation.

**Behaviour:**
1. Sends `arm` → polls for confirmation
2. Sends `set_mode MANUAL`
3. Sends `pid_depth` with target depth
4. Polls `ctx.depth` until within 0.15m of target (or timeout)
5. Waits `dive_settle_time` for oscillation to damp
6. Returns `submerged`

| Outcome | Meaning |
|---|---|
| `submerged` | At target depth (or settle elapsed) |
| `failed` | Arm explicitly rejected |

| Blackboard Key | Required | Default |
|---|---|---|
| `ctx` | Yes | — |
| `dive_depth` | No | `cfg.dive_depth` (0.6m) |

---

### SearchState (`states/search.py`)

**Purpose:** Rotate in place looking for a YOLO detection class.

**Behaviour:**
1. Reads `target_class`, `search_yaw_step`, `search_speed`, `search_timeout`
2. Loops: check `ctx.has_detection(target)`, if found → return `found`
3. If not found, rotate by `yaw_step` degrees using `pid_yaw_to_heading`
4. After 360° without detection, reverse direction (zigzag)
5. If timeout expires, return `timeout`

| Outcome | Meaning |
|---|---|
| `found` | Target class detected in `/vision/detections` |
| `timeout` | Search time expired |

| Blackboard Key | Required | Default |
|---|---|---|
| `ctx` | Yes | — |
| `target_class` | Yes | — |
| `search_yaw_step` | No | `30.0°` |
| `search_speed` | No | `cfg.default_speed` |
| `search_timeout` | No | `cfg.search_timeout` |

---

### AlignState (`states/align.py`)

**Purpose:** Activate visual servo alignment and monitor until target is centred.

**Behaviour:**
1. Sends alignment command (e.g., `pid_align`, `pid_align_forward`)
2. Polls `ctx.alignment_status` every 0.3s
3. If `fully_aligned == True` → return `aligned`
4. If target disappears for too long → return `lost`
5. If timeout → return `timeout`
6. Sends `vision_stop` when done

| Outcome | Meaning |
|---|---|
| `aligned` | Target centred in all axes |
| `lost` | Target disappeared during alignment |
| `timeout` | Alignment time expired |

| Blackboard Key | Required | Default |
|---|---|---|
| `ctx` | Yes | — |
| `target_class` | Yes | — |
| `alignment_mode` | No | `"pid_align"` |
| `alignment_timeout` | No | `cfg.alignment_timeout` |

---

### DriveState (`states/drive.py`)

**Purpose:** Timed open-loop movement in a specified direction.

**Behaviour:**
1. Reads command, duration, speed, heading, stop_after
2. If `drive_heading` is set, uses `go_forward` (heading hold)
3. Otherwise, sends generic command (`move_forward`, `move_left`, etc.)
4. Sleeps for duration
5. If `stop_after` is True, sends `stop`
6. Returns `done`

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
| `stop_after` | No | `True` |

---

### SurfaceState (`states/surface.py`)

**Purpose:** Ascend to surface and disarm.

**Behaviour:**
1. Sends `surface`
2. Waits 5 seconds
3. Sends `disarm`
4. Waits 2 seconds
5. Returns `surfaced`

| Outcome | Meaning |
|---|---|
| `surfaced` | At surface, disarmed |

| Blackboard Key | Required | Default |
|---|---|---|
| `ctx` | Yes | — |

---

### WaitFeedbackState (`states/wait_feedback.py`)

**Purpose:** Block until a `DriverCommandFeedback` message arrives.

**Behaviour:**
1. Clears any pending feedback
2. Waits for `ctx.wait_for_feedback(timeout)`
3. If `expected_command` is set, verifies the feedback matches
4. Returns the feedback status as the outcome

| Outcome | Meaning |
|---|---|
| `reached` | Target achieved |
| `completed` | Duration command finished |
| `rejected` | Command rejected |
| `timeout` | No feedback in time |

| Blackboard Key | Required | Default |
|---|---|---|
| `ctx` | Yes | — |
| `feedback_timeout` | No | `cfg.feedback_timeout` |
| `expected_command` | No | `None` (match any) |

---

### SendCommandState (`states/send_command.py`)

**Purpose:** Publish a single `DriverCommand` and immediately transition. Glue state for injecting one-shot commands into a state machine.

**Behaviour:**
1. Reads `cmd_name` and `cmd_kwargs`
2. Calls `ctx.send(name, **kwargs)`
3. Returns `done` immediately

| Outcome | Meaning |
|---|---|
| `done` | Command published |

| Blackboard Key | Required | Default |
|---|---|---|
| `ctx` | Yes | — |
| `cmd_name` | Yes | — |
| `cmd_kwargs` | No | `{}` |

---

## 15. Mission Reference (Complete)

### Gate Task (`missions/gate.py`)

RoboSub Task 1: Begin Assessment — pass through the gate.

```
SETUP → SEARCH_GATE → ALIGN_GATE → LOCK_HEADING → DRIVE_THROUGH → (gate_passed)
                 ↑          │
                 └──lost─────┘    (re-search if target lost during alignment)
```

**SETUP:** Injects gate-specific parameters from `planner.yaml`:
- `target_class = "gate"`
- `search_yaw_step`, `search_speed`, `search_timeout`
- `alignment_mode = "pid_align_forward"`
- `drive_command = "move_forward"`, `drive_duration`, `drive_speed`

**SEARCH_GATE:** Rotates in place until the gate is detected.

**ALIGN_GATE:** Visual servo centres the gate in the camera frame.

**LOCK_HEADING:** Snapshots the current heading after alignment. This heading is used by the drive-through state to maintain a straight line.

**DRIVE_THROUGH:** Drives forward for `drive_through_time` seconds at `approach_speed`, holding the locked heading via `go_forward`.

**Failure modes:**
- Search timeout → `gate_failed` → top-level aborts to `SURFACE_ABORT`
- Alignment timeout → `gate_failed`
- Target lost during alignment → re-search (`SEARCH_GATE`)

---

### Demo Square (`missions/demo_square.py`)

Desk-testing mission: forward + 90° PID turn × 4 to trace a square.

```
SETUP → LEG_1 → TURN_1 → LEG_2 → TURN_2 → LEG_3 → TURN_3 → LEG_4 → TURN_4 → (square_done)
```

**SETUP:** Injects default demo parameters (overridable via Blackboard):
- `leg_duration = 3.0s`
- `turn_angle = 90.0°`
- `demo_speed = 40`
- `turn_settle = 2.0s`

**LEG_N:** Sends `move_forward` and sleeps. Does NOT send `stop` — the next turn command transitions smoothly via the RC ramp.

**TURN_N:** Sends `pid_yaw_to_heading` to current heading + 90°. Does NOT send `stop` unless it's the final turn (TURN_4).

**Customisation (set on Blackboard before running):**
```python
blackboard["leg_duration"] = 5.0    # Longer legs
blackboard["turn_angle"] = 45.0     # Octagon instead of square
blackboard["demo_speed"] = 60       # Faster
blackboard["turn_settle"] = 3.0     # More time to complete turn
```

---

## 16. ROS 2 Topics & Interfaces

### Published by duburi_planner

| Topic | Message Type | Rate | Purpose |
|---|---|---|---|
| `/driver/command` | `duburi_interfaces/DriverCommand` | On-demand | Movement commands to the vehicle |

### Subscribed by duburi_planner

| Topic | Message Type | QoS | Purpose |
|---|---|---|---|
| `/driver/feedback` | `duburi_interfaces/DriverCommandFeedback` | RELIABLE | Command acknowledgements from inspector |
| `/mavlink/vehicle_state` | `duburi_interfaces/VehicleState` | BEST_EFFORT | Depth, heading, armed status |
| `/vision/alignment_status` | `duburi_interfaces/AlignmentStatus` | BEST_EFFORT | Visual servo alignment state |
| `/vision/detections` | `duburi_interfaces/DetectionArray` | BEST_EFFORT | YOLO object detections |

### Key Message Fields

**DriverCommand:**
```
string command       # "move_forward", "pid_depth", "arm", etc.
float32 depth        # For depth commands
float32 angle        # For heading commands
float32 duration     # For timed commands
int32 speed          # PWM offset (0-100)
string mode          # For set_mode commands
```

**VehicleState (relevant fields):**
```
bool armed
float32 depth
float32 yaw          # heading 0-360
string flight_mode
```

**AlignmentStatus (relevant fields):**
```
string target_class
bool target_detected
bool fully_aligned
float32 error_x      # -1.0 to +1.0
float32 error_y      # -1.0 to +1.0
```

---

## 17. File-by-File Walkthrough

### `mission_node.py` — Full Mission Entry Point

```
Line 1-17:   Module docstring (usage instructions)
Line 19-35:  Imports — rclpy, yasmin, local modules
Line 37-38:  Outcome constants (MISSION_SUCCESS, MISSION_ABORTED)
Line 41-84:  build_mission() — constructs top-level SM
                SUBMERGE → GATE_TASK → SURFACE / SURFACE_ABORT
Line 87-138: main() — init ROS, load config, create context,
                build SM, start viewer, create blackboard,
                start executor in background thread, run SM,
                cleanup in finally block
```

### `demo_node.py` — Demo Square Entry Point

```
Line 1-28:   Module docstring (smooth transitions explained)
Line 30-42:  Imports
Line 44-45:  Outcome constants
Line 48-76:  build_demo_mission() — ARM → SQUARE → DISARM
Line 79-131: main() — same pattern as mission_node
```

### `planner_context.py` — Shared ROS Bridge

```
Line 1-13:   Module docstring
Line 15-38:  Imports, QoS profiles
Line 41-52:  Class definition, __slots__
Line 53-81:  __init__ — creates publisher, 4 subscribers
Line 83-97:  publish_command() and send() — command publishing
Line 99-116: Subscription callbacks — thread-safe cache updates
Line 118-158: Properties (vehicle_state, alignment_status, etc.)
                wait_for_feedback(), has_detection()
Line 160-183: heading, depth, armed properties, log(), warn(), sleep()
```

### `planner_config.py` — Typed Configuration

```
Line 1-13:   Module docstring
Line 15-25:  TaskConfig dataclass (per-task tunables)
Line 27-45:  PlannerConfig dataclass (global tunables + tasks dict)
Line 48-91:  load_config() — declares ROS params, builds configs
```

### `states/arm.py` — Arm State

```
Line 1-15:   Docstring (outcomes, blackboard keys)
Line 17-24:  Imports, outcome constants
Line 29-60:  ArmState class
                execute(): send arm, poll ctx.armed,
                handle desk mode gracefully, set_mode MANUAL
```

### `states/submerge.py` — Submerge State

```
Line 1-15:   Docstring
Line 17-26:  Imports, outcome constants
Line 29-83:  SubmergeState class
                execute(): arm (same pattern as ArmState),
                set mode, pid_depth, poll depth until within 0.15m,
                handle no-telemetry gracefully, settle time
```

### `states/search.py` — Search State

```
Line 1-17:   Docstring
Line 19-26:  Imports, outcome constants
Line 31-69:  SearchState class
                execute(): yaw-sweep loop, zigzag on full rotation,
                check ctx.has_detection() each step,
                timeout handling
```

### `states/align.py` — Align State

```
Line 1-17:   Docstring
Line 19-29:  Imports, outcome constants
Line 33-83:  AlignState class
                execute(): send alignment command,
                poll ctx.alignment_status every 0.3s,
                track lost_streak, vision_stop on exit
```

### `states/drive.py` — Drive State

```
Line 1-22:   Docstring (smooth transition explanation)
Line 24-30:  Imports, outcome constants
Line 33-58:  DriveState class
                execute(): choose go_forward vs generic,
                sleep for duration, optional stop_after
```

### `states/surface.py` — Surface State

```
Line 1-6:    Docstring
Line 8-29:   SurfaceState class — surface + disarm + wait
```

### `states/wait_feedback.py` — Wait Feedback State

```
Line 1-18:   Docstring
Line 20-29:  Imports, outcome constants
Line 32-66:  WaitFeedbackState class
                execute(): clear feedback, wait, match command,
                map status to outcome
```

### `states/send_command.py` — Send Command State

```
Line 1-14:   Docstring
Line 16-22:  Imports, outcome constants
Line 25-36:  SendCommandState class — ctx.send(name, **kwargs)
```

### `missions/gate.py` — Gate Task Mission

```
Line 1-15:   Docstring (sub-SM diagram)
Line 17-27:  Imports, outcome constants
Line 30-47:  _setup_gate_blackboard() — injects all gate params
Line 50-56:  _record_heading() — snapshots heading after alignment
Line 59-100: build_gate_task() — wires SETUP, SEARCH, ALIGN,
                LOCK_HEADING, DRIVE_THROUGH with transitions
```

### `missions/demo_square.py` — Demo Square Mission

```
Line 1-59:   Module docstring (extensive YASMIN customisation guide)
Line 61-64:  Imports, outcome constants
Line 67-84:  DemoLegState class — no stop between legs
Line 87-115: DemoTurnState class — is_final flag for last turn
Line 118-132: _setup_demo() — injects demo params
Line 135-160: build_demo_square() — wires legs and turns in loop
```

---

## 18. Common Patterns & Recipes

### Pattern 1: Add a Pause Between States

```python
from yasmin import CbState

def pause_2s(blackboard):
    blackboard["ctx"].sleep(2.0)
    return "done"

sm.add_state("PAUSE", CbState(["done"], pause_2s),
             transitions={"done": "NEXT_STATE"})
```

### Pattern 2: Conditional Branch (Check Heading Drift)

```python
class CheckDriftState(State):
    def __init__(self):
        super().__init__(outcomes=["ok", "drifted"])

    def execute(self, blackboard):
        ctx = blackboard["ctx"]
        expected = bb_get(blackboard, "expected_heading", 0)
        if abs(ctx.heading - expected) > 20:
            ctx.warn("Heading drifted too far!")
            return "drifted"
        return "ok"
```

### Pattern 3: Retry a State N Times

```python
class RetryState(State):
    def __init__(self, inner_state, max_retries=3):
        super().__init__(outcomes=["success", "give_up"])
        self._inner = inner_state
        self._max = max_retries

    def execute(self, blackboard):
        for attempt in range(self._max):
            result = self._inner.execute(blackboard)
            if result == "found":
                return "success"
        return "give_up"
```

### Pattern 4: Skip a Task (Comment Out in build_mission)

To remove a task during pool testing, just change the transition:

```python
# Before: GATE → SLALOM
sm.add_state("GATE_TASK", ...,
             transitions={GATE_PASSED: "SLALOM_TASK", ...})

# After: GATE → SURFACE (skip slalom)
sm.add_state("GATE_TASK", ...,
             transitions={GATE_PASSED: "SURFACE", ...})
```

### Pattern 5: Smooth Multi-Leg Drive (No Jerk)

```python
# Set stop_after=False for intermediate drives
blackboard["stop_after"] = False
sm.add_state("DRIVE_PART_1", DriveState(), transitions={"done": "DRIVE_PART_2"})

# Set stop_after=True for the final drive
blackboard["stop_after"] = True
sm.add_state("DRIVE_PART_2", DriveState(), transitions={"done": "NEXT"})
```

Or, if using custom leg states (like in demo_square), just don't send `stop` between moves.

### Pattern 6: Change Demo Parameters at Runtime

In `demo_node.py`, before calling `sm(blackboard)`:

```python
blackboard["ctx"] = ctx
blackboard["leg_duration"] = 5.0    # Override defaults
blackboard["turn_angle"] = 45.0     # Make an octagon
blackboard["demo_speed"] = 60
```

---

## 19. Troubleshooting

### "Nothing happens when I run the demo"

**Symptom:** Demo runs, commands are published, but vehicle doesn't move.

**Cause:** `mavlink_inspector` is not running or not connected to Pixhawk.

**Fix:** In a separate terminal:
```bash
ros2 run mavlink_inspector inspector
```
Check that it connects to Pixhawk. The demo sends commands to `/driver/command` — the inspector must be running to act on them.

### "Thrusters jerk between moves"

**Symptom:** Thrusters slam to zero and restart between legs/turns.

**Cause:** A `stop` command is being sent between moves.

**Fix:** Remove `ctx.send('stop')` between consecutive movement commands. The RC ramp handles smooth transitions. See [Section 8](#8-smooth-thruster-transitions-no-jerk).

### "RuntimeError: Element 'X' does not exist in the blackboard"

**Symptom:** State crashes accessing a Blackboard key.

**Cause:** The key wasn't set by a previous state or SETUP callback.

**Fix:** Either:
- Add the key to the SETUP callback for that mission
- Use `bb_get(blackboard, "key", default)` instead of `blackboard["key"]`

### "incompatible function arguments" from blackboard.get()

**Symptom:** `RuntimeError: get(): incompatible function arguments`

**Cause:** YASMIN's `Blackboard.get()` doesn't accept a default argument.

**Fix:** Use `bb_get()` from `bb_utils.py`:
```python
from ..bb_utils import bb_get
value = bb_get(blackboard, "key", default_value)
```

### "No Pixhawk found" but demo still works

**Expected behaviour.** The planner is designed for desk testing. `ArmState` and `SubmergeState` warn about missing telemetry but continue anyway. Commands are published (visible on `ros2 topic echo /driver/command`) even without hardware.

### "RCLError: publisher's context is invalid" at shutdown

**Cosmetic issue.** Happens when the `finally` block tries to send `stop`/`disarm` after ROS context has already shut down. The `try...except` in the finally block catches this. Not harmful.

### "YASMIN viewer shows nothing"

**Cause:** Viewer node isn't running, or the SM hasn't started yet.

**Fix:**
1. Ensure `yasmin_viewer_node` is running
2. Refresh the browser after the mission node starts
3. Check the dropdown for the correct viewer name (`DUBURI_MISSION` or `DUBURI_DEMO_SQUARE`)

### "Search always times out"

**Possible causes:**
- Vision pipeline not running (`ros2 launch vision vision.launch.py`)
- YOLO model doesn't have the target class (check model classes)
- `target_class` in `planner.yaml` doesn't match YOLO class name exactly
- Search timeout too short — increase `search_timeout`

### "Alignment keeps timing out"

**Possible causes:**
- `alignment_controller` not active — check that `alignment_mode` command is valid
- Target too small/far — approach closer before aligning
- PID gains need tuning — see `alignment_controller` config
- `alignment_timeout` too short — increase it

---

## 20. Quick Reference Cheat Sheet

### Essential Commands

```bash
# Build
colcon build --packages-select duburi_planner && source install/setup.bash

# Demo (desk-safe)
ros2 run duburi_planner demo_node

# Full mission
ros2 launch duburi_planner planner.launch.py

# Watch commands
ros2 topic echo /driver/command

# Watch feedback
ros2 topic echo /driver/feedback

# Watch vehicle state
ros2 topic echo /mavlink/vehicle_state

# YASMIN viewer
ros2 run yasmin_viewer yasmin_viewer_node
# → http://localhost:5000/
```

### State Outcomes (Quick Lookup)

| State | Outcomes |
|---|---|
| ArmState | `armed`, `failed` |
| SubmergeState | `submerged`, `failed` |
| SearchState | `found`, `timeout` |
| AlignState | `aligned`, `lost`, `timeout` |
| DriveState | `done` |
| SurfaceState | `surfaced` |
| WaitFeedbackState | `reached`, `completed`, `rejected`, `timeout` |
| SendCommandState | `done` |

### Blackboard Quick Reference

| To Do This... | Set This Key... |
|---|---|
| Change dive depth | `blackboard["dive_depth"] = 1.0` |
| Set search target | `blackboard["target_class"] = "gate"` |
| Change search timeout | `blackboard["search_timeout"] = 60.0` |
| Change drive direction | `blackboard["drive_command"] = "move_left"` |
| Hold heading during drive | `blackboard["drive_heading"] = 127.3` |
| Don't stop after drive | `blackboard["stop_after"] = False` |
| Change drive duration | `blackboard["drive_duration"] = 5.0` |
| Change drive speed | `blackboard["drive_speed"] = 60` |
| Inject a one-shot command | `blackboard["cmd_name"] = "stop"` |

### Adding a Task in 60 Seconds

1. Copy `missions/gate.py` → `missions/my_task.py`
2. Change class names, target, transitions
3. Add config under task name in `planner.yaml`
4. Wire into `mission_node.py`'s `build_mission()`
5. Rebuild: `colcon build --packages-select duburi_planner`

---

## 21. Demo Mission — Complete Deep Dive

This section dissects the demo square mission **line by line**, explaining every decision, every parameter, and what would happen if you changed each one. Use this as the reference template when building more complex missions.

### 21.1 What the Demo Does (Plain English)

The demo makes the AUV trace a square in the water:

```
         Start
           │
           ▼
    ┌──────────────┐
    │   LEG 1      │  Drive forward 3 seconds
    │   (forward)  │
    └──────┬───────┘
           │
    ┌──────▼───────┐
    │   TURN 1     │  Rotate 90° right (PID controlled)
    │   (+90°)     │
    └──────┬───────┘
           │
    ┌──────▼───────┐
    │   LEG 2      │  Drive forward 3 seconds
    │   (forward)  │
    └──────┬───────┘
           │
    ┌──────▼───────┐
    │   TURN 2     │  Rotate 90° right
    │   (+90°)     │
    └──────┬───────┘
           │
    ┌──────▼───────┐
    │   LEG 3      │  Drive forward 3 seconds
    │   (forward)  │
    └──────┬───────┘
           │
    ┌──────▼───────┐
    │   TURN 3     │  Rotate 90° right
    │   (+90°)     │
    └──────┬───────┘
           │
    ┌──────▼───────┐
    │   LEG 4      │  Drive forward 3 seconds
    │   (forward)  │
    └──────┬───────┘
           │
    ┌──────▼───────┐
    │   TURN 4     │  Rotate 90° right + STOP thrusters
    │   (final)    │
    └──────────────┘
```

But before the square starts, the vehicle must be **armed** (thrusters enabled). And after the square, it **disarms** (thrusters disabled). So the full flow is:

```
ARM → SQUARE (4 legs + 4 turns) → DISARM
```

### 21.2 The Two Files Involved

The demo is split into two files:

| File | Responsibility |
|---|---|
| `demo_node.py` | The **entry point**. Initialises ROS, creates the top-level state machine (ARM → SQUARE → DISARM), starts the YASMIN viewer, and runs the mission. |
| `missions/demo_square.py` | The **square pattern logic**. Defines the SETUP, LEG, and TURN states, and wires them into a sub-state-machine. |

This separation exists because `demo_square.py` is a **reusable sub-machine**. You could embed it inside a competition mission (`mission_node.py`) just by adding one line:

```python
sm.add_state("SQUARE", build_demo_square(), transitions={...})
```

### 21.3 demo_node.py — Line by Line

#### Startup (lines 84-112)

```python
def main() -> None:
    rclpy.init()                          # 1. Start ROS 2 runtime
    set_ros_loggers()                     # 2. Route YASMIN logs through ROS

    node = rclpy.create_node('demo_node') # 3. Create a ROS node named 'demo_node'
    cfg = load_config(node)               # 4. Load planner.yaml → PlannerConfig
    ctx = PlannerContext(node, cfg)        # 5. Create shared ROS bridge
```

**What's happening:** The node is created, config is loaded, and the `PlannerContext` is built. The context creates one publisher (`/driver/command`) and four subscribers (`/driver/feedback`, `/mavlink/vehicle_state`, `/vision/alignment_status`, `/vision/detections`).

**If you change `'demo_node'`** to another name, the ROS node appears with that name in `ros2 node list`. No functional difference.

```python
    sm = build_demo_mission(ctx)          # 6. Build: ARM → SQUARE → DISARM

    viewer_name = "DUBURI_DEMO_SQUARE"
    YasminViewerPub(sm, viewer_name)      # 7. Publish FSM to YASMIN viewer
```

**What's happening:** The state machine is constructed (all states and transitions defined). Then `YasminViewerPub` begins publishing the FSM structure to port 5000 so the web viewer can display it.

**If you change `viewer_name`**, the filter dropdown in the web viewer shows the new name. Purely cosmetic.

```python
    blackboard = Blackboard()
    blackboard["ctx"] = ctx               # 8. Inject context into blackboard

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()                   # 9. Start ROS in background thread
```

**What's happening:** The Blackboard is created with `ctx` as its first entry. A background thread starts spinning the ROS executor — this is what receives incoming messages (vehicle state, feedback, detections) while the FSM runs on the main thread.

**Why a background thread?** YASMIN states block (`time.sleep`, polling loops). If the ROS executor ran on the same thread, it couldn't process incoming messages while a state was sleeping.

```python
    try:
        outcome = sm(blackboard)          # 10. RUN THE MISSION (blocking)
```

**This is where the action happens.** `sm(blackboard)` starts executing the first state (`ARM`). The call blocks until the entire state machine finishes and returns a terminal outcome (`demo_success` or `demo_failed`).

#### Cleanup (lines 124-132)

```python
    finally:
        try:
            ctx.send('stop')              # Safety: halt thrusters
        except Exception:
            pass
        executor.shutdown()               # Stop ROS executor
        node.destroy_node()               # Clean up node
        if rclpy.ok():
            rclpy.shutdown()              # Shut down ROS 2
```

**Why the `try...except`?** If the user hits Ctrl-C or the ROS context is already shut down, `ctx.send('stop')` would throw an `RCLError`. The `except` catches it silently — it's a cleanup step, not critical.

#### The Top-Level State Machine (lines 53-81)

```python
def build_demo_mission(ctx: PlannerContext) -> StateMachine:
    sm = StateMachine(
        outcomes=[DEMO_SUCCESS, DEMO_FAILED],   # Terminal outcomes
        handle_sigint=True,                       # Ctrl-C triggers clean exit
    )
```

`outcomes` defines the possible end states of this machine. When the SM returns one of these strings, it's done. `handle_sigint=True` means YASMIN will catch Ctrl-C and shut down the SM gracefully.

```python
    sm.add_state("ARM", ArmState(),
        transitions={ARMED: "SQUARE", ARM_FAILED: DEMO_FAILED})
```

**Reading this:** "Add a state called `ARM` that runs `ArmState()`. When it returns `"armed"`, go to the state called `SQUARE`. When it returns `"failed"`, the entire SM returns `demo_failed`."

```python
    sm.add_state("SQUARE", build_demo_square(),
        transitions={SQUARE_DONE: "DISARM"})
```

**Reading this:** "Add a state called `SQUARE` that is itself a sub-state-machine (the square pattern). When it returns `"square_done"`, go to `DISARM`."

This is **hierarchical composition** — from the top level's perspective, `SQUARE` is just a single box with one outcome. Inside, it's 9 states.

```python
    sm.add_state("DISARM", SurfaceState(),
        transitions={SURFACED: DEMO_SUCCESS})
```

**Reading this:** "Add `DISARM` running `SurfaceState`. When it returns `"surfaced"`, the whole SM returns `demo_success`."

### 21.4 ArmState — What Happens Step by Step

When the SM starts, `ARM` is the first state. Here's exactly what `ArmState.execute()` does:

```
1. ctx.wait_for_ready(timeout=8.0)
   │
   │  The planner just started. Its publisher (/driver/command)
   │  needs to find the inspector's subscriber. DDS discovery
   │  takes 1-3 seconds. This method blocks until at least one
   │  subscriber is found. Without this, the first commands
   │  would be silently dropped.
   │
   │  If no subscriber after 8s: warns but continues (desk mode).
   │
2. ctx.send('set_mode', mode='MANUAL')
   │
   │  Sets the Pixhawk to MANUAL mode. In MANUAL, the Pixhawk
   │  passes RC channel values directly to ESCs. This is required
   │  before any thruster commands work.
   │
   │  Sleep 1.0s to let mode change confirm.
   │
3. ctx.send('arm')
   │
   │  Tells the Pixhawk to enable thrusters. Without arming,
   │  all movement commands are rejected.
   │
4. Poll loop (4 seconds):
   │
   │  Every 0.5s: check ctx.armed (from /mavlink/vehicle_state).
   │  If armed=true: log success, break out.
   │  Every 1.5s: resend 'arm' (in case the first was dropped).
   │
   │  If 4s pass without confirmation:
   │    - No telemetry at all? → "desk mode, continuing anyway"
   │    - Telemetry but not armed? → "continuing anyway" (warn)
   │
5. Return "armed"
```

**Why retry?** Even with `wait_for_ready`, there's a tiny window where a message could be lost. Retrying every 1.5s makes it bulletproof.

**Why set_mode before arm?** ArduSub sometimes rejects arm if not in a compatible mode. MANUAL is always valid.

### 21.5 demo_square.py — The Square Pattern

#### The SETUP State

```python
def _setup_demo(blackboard: Blackboard) -> str:
    ctx = blackboard["ctx"]
    if "leg_duration" not in blackboard:
        blackboard["leg_duration"] = 3.0
    if "turn_angle" not in blackboard:
        blackboard["turn_angle"] = 90.0
    if "demo_speed" not in blackboard:
        blackboard["demo_speed"] = 40
    if "turn_settle" not in blackboard:
        blackboard["turn_settle"] = 2.0

    ctx.log("DEMO SQUARE — starting 4-leg square pattern")
    return "ready"
```

This is a `CbState` — a lightweight state made from a plain function. It **injects default parameters** into the Blackboard. The `if key not in blackboard` pattern means you can override any parameter before the mission starts and SETUP won't overwrite it.

**Parameters and their effects:**

| Parameter | Default | What It Controls | Effect of Increasing | Effect of Decreasing |
|---|---|---|---|---|
| `leg_duration` | `3.0` sec | How long the AUV drives forward per side | Longer sides → bigger square | Shorter sides → smaller square |
| `turn_angle` | `90.0°` | How many degrees to turn between legs | >90° = overlap, <90° = opens up | 45° = octagon, 60° = hexagon |
| `demo_speed` | `40` | PWM offset (0-100) for both forward and turn | Faster movement (riskier) | Slower, more controlled |
| `turn_settle` | `2.0` sec | How long to wait after sending a yaw command | More time to complete turn | Might not finish turning |

**Changing these creates different shapes:**

| `turn_angle` | `leg_duration` | Shape |
|---|---|---|
| `90.0` | `3.0` | Square (default) |
| `45.0` | `3.0` | Octagon |
| `60.0` | `3.0` | Hexagon |
| `120.0` | `3.0` | Triangle |
| `90.0` | `1.0` | Tiny square |
| `90.0` | `8.0` | Big square |

#### DemoLegState — One Side of the Square

```python
class DemoLegState(State):
    def __init__(self, leg_number: int) -> None:
        super().__init__(outcomes=["leg_done"])
        self._leg = leg_number

    def execute(self, blackboard: Blackboard) -> str:
        ctx = blackboard["ctx"]
        duration = bb_get(blackboard, "leg_duration", 3.0)
        speed = bb_get(blackboard, "demo_speed", 40)

        ctx.log(f"DEMO LEG {self._leg} — move_forward dur={duration}s spd={speed}")
        ctx.send('move_forward', duration=duration, speed=speed)
        ctx.sleep(duration)
        return "leg_done"
```

**What happens physically:**

1. `ctx.send('move_forward', duration=3.0, speed=40)` — publishes a `DriverCommand` to `/driver/command`. The inspector receives it and sets the forward RC channel to 1500 + (40/100 × 400) = **1660 PWM**. The trapezoidal ramp smoothly increases from whatever the current value is to 1660.

2. `ctx.sleep(duration)` — waits 3 seconds. During this time, the inspector continues sending RC overrides at 20 Hz (every 50ms), holding the forward channel at 1660. The AUV moves forward.

3. Returns `"leg_done"` — **no stop command**. The thrusters are still running at 1660 PWM. The next state (a turn) will send a yaw command, and the ramp will smoothly transition forward→neutral while ramping yaw.

**Why no stop?** If we sent `stop` here:
- Forward channel would snap to 1500 (neutral) instantly
- The AUV would lurch/decelerate abruptly
- Then the turn command would start yaw from zero
- The AUV would jerk again

Without stop, the ramp smoothly blends: forward decreases while yaw increases over ~0.5s. The transition feels natural.

#### DemoTurnState — A Corner of the Square

```python
class DemoTurnState(State):
    def __init__(self, turn_number: int, is_final: bool = False) -> None:
        super().__init__(outcomes=["turn_done"])
        self._turn = turn_number
        self._is_final = is_final

    def execute(self, blackboard: Blackboard) -> str:
        ctx = blackboard["ctx"]
        turn_angle = bb_get(blackboard, "turn_angle", 90.0)
        speed = bb_get(blackboard, "demo_speed", 40)
        settle = bb_get(blackboard, "turn_settle", 2.0)

        current = ctx.heading
        target = (current + turn_angle) % 360

        ctx.send('pid_yaw_to_heading', angle=target, speed=speed)
        ctx.sleep(settle)

        if self._is_final:
            ctx.send('stop')

        return "turn_done"
```

**What happens physically:**

1. `ctx.heading` — reads the latest heading from `/mavlink/vehicle_state`. If the AUV is pointing at 0° (north) and `turn_angle` is 90°, then `target = (0 + 90) % 360 = 90°` (east).

2. `ctx.send('pid_yaw_to_heading', angle=90, speed=40)` — the inspector activates its **PID yaw controller**. The PID reads the current heading from the compass, calculates the error (target − current), and adjusts the yaw RC channel. The AUV rotates smoothly towards 90°.

3. `ctx.sleep(settle)` — waits 2 seconds for the PID to finish. During this time, the PID is running at 20 Hz, adjusting yaw channel every 50ms. The AUV rotates.

4. **If final turn:** `ctx.send('stop')` — all channels snap to neutral. Thrusters stop. The square is done.

5. **If not final:** returns immediately. The next leg state sends `move_forward`, and the ramp transitions from yaw→forward smoothly.

**Why is `turn_settle` important?** If `turn_settle` is too short (e.g. 0.5s), the PID won't have time to finish the turn. The next leg starts while the AUV is still mid-rotation, so it drives forward at an angle. If `turn_settle` is too long (e.g. 10s), the AUV just sits there after finishing the turn, wasting mission time.

**The `% 360` modular arithmetic:** Handles wrapping. If heading is 350° and turn_angle is 90°, target = (350 + 90) % 360 = 80°. The PID controller knows to turn right through 0° (north).

#### How build_demo_square() Wires It All Together

```python
def build_demo_square() -> StateMachine:
    sm = StateMachine(outcomes=[SQUARE_DONE])

    sm.add_state("SETUP", CbState(["ready"], _setup_demo),
                 transitions={"ready": "LEG_1"})

    for i in range(1, 5):
        sm.add_state(f"LEG_{i}", DemoLegState(i),
                     transitions={"leg_done": f"TURN_{i}"})

        is_final = (i == 4)
        after_turn = SQUARE_DONE if is_final else f"LEG_{i + 1}"
        sm.add_state(f"TURN_{i}", DemoTurnState(i, is_final=is_final),
                     transitions={"turn_done": after_turn})

    return sm
```

The `for i in range(1, 5)` loop creates 4 legs and 4 turns. Here's the transition map it builds:

| State | Outcome | Goes To |
|---|---|---|
| `SETUP` | `ready` | `LEG_1` |
| `LEG_1` | `leg_done` | `TURN_1` |
| `TURN_1` | `turn_done` | `LEG_2` |
| `LEG_2` | `leg_done` | `TURN_2` |
| `TURN_2` | `turn_done` | `LEG_3` |
| `LEG_3` | `leg_done` | `TURN_3` |
| `TURN_3` | `turn_done` | `LEG_4` |
| `LEG_4` | `leg_done` | `TURN_4` |
| `TURN_4` | `turn_done` | `square_done` (terminal) |

The last turn (`i == 4`) is marked `is_final=True` so it sends `stop`. Its transition leads to `SQUARE_DONE`, which is a terminal outcome — the sub-SM returns `"square_done"` to the parent.

### 21.6 SurfaceState — Cleanup

After the square, `DISARM` runs `SurfaceState`:

```python
def execute(self, blackboard):
    ctx = blackboard["ctx"]
    ctx.send('surface')       # Ascend command
    ctx.sleep(5.0)            # Wait 5s for ascent
    ctx.send('disarm')        # Disable thrusters
    ctx.sleep(2.0)            # Wait for disarm
    return "surfaced"
```

This sends `surface` (which the inspector translates to upward throttle), waits, then disarms. The 5-second wait is generous — at the surface, the AUV might already be there.

### 21.7 Complete Command Timeline

Here is the exact sequence of commands published to `/driver/command` with approximate timing:

```
Time (s)  Command                         What Happens Physically
────────  ──────────────────────────────  ─────────────────────────────────
 0.0      (node starts)                   ROS 2 initialises
 0.0-3.0  (DDS discovery)                 wait_for_ready() polls for subscriber
 ~3.0     set_mode MANUAL                 Pixhawk enters MANUAL mode
 ~4.0     arm                             Thrusters enabled
 ~4.5     arm (retry if needed)           Backup in case first was dropped
 ~8.0     (ARM state done)               Confirmed armed or desk-mode timeout

 ~8.0     move_forward (3s, spd 40)       Forward thrust ramps up → 1660 PWM
 ~11.0    pid_yaw_to_heading (+90°)       Forward ramps down, yaw ramps up
 ~13.0    move_forward (3s, spd 40)       Yaw ramps down, forward ramps up
 ~16.0    pid_yaw_to_heading (+90°)       Smooth transition to yaw
 ~18.0    move_forward (3s, spd 40)       Back to forward
 ~21.0    pid_yaw_to_heading (+90°)       Turning
 ~23.0    move_forward (3s, spd 40)       Last forward leg
 ~26.0    pid_yaw_to_heading (+90°)       Last turn
 ~28.0    stop                            All channels → 1500 (neutral)

 ~28.0    surface                         Upward throttle
 ~33.0    disarm                          Thrusters disabled
 ~35.0    (mission complete)              demo_success returned
```

Total runtime: ~35 seconds. The YASMIN viewer shows each state lighting up in real time.

### 21.8 What Each Parameter Change Does

#### Changing `leg_duration`

```python
blackboard["leg_duration"] = 5.0   # Before: 3.0
```

| Aspect | Before (3.0s) | After (5.0s) |
|---|---|---|
| Side length | ~1.5m at speed 40 | ~2.5m at speed 40 |
| Total pattern time | ~20s (excl. arm/disarm) | ~28s |
| Square size | Small (good for tight spaces) | Larger |
| Risk | Lower | Higher (more distance = more drift) |

#### Changing `demo_speed`

```python
blackboard["demo_speed"] = 70     # Before: 40
```

| Aspect | Before (40) | After (70) |
|---|---|---|
| PWM offset | 1500 + 160 = 1660 | 1500 + 280 = 1780 |
| Physical speed | Slow/moderate | Fast |
| Ramp transition time | ~0.2s | ~0.35s |
| PID turn speed | Moderate yaw rate | Aggressive yaw rate |
| Risk | Low (easy to control) | Higher (overshooting turns) |

#### Changing `turn_angle`

```python
blackboard["turn_angle"] = 45.0    # Before: 90.0
```

| Aspect | Before (90°) | After (45°) |
|---|---|---|
| Shape | Square (4 turns × 90° = 360°) | Octagon (4 turns × 45° = 180°, only half done!) |
| To complete a full loop | 4 turns | Need 8 turns (360/45) |

**Important:** The demo always does exactly 4 legs and 4 turns. If you set `turn_angle=45`, you get a half-octagon, not a full one. To make a full octagon, you'd need to modify `build_demo_square()` to use `range(1, 9)` instead of `range(1, 5)`.

#### Changing `turn_settle`

```python
blackboard["turn_settle"] = 4.0    # Before: 2.0
```

| Aspect | Before (2.0s) | After (4.0s) |
|---|---|---|
| Turn accuracy | May undershoot if PID is slow | More time to converge |
| Total time | Faster overall | 8s extra (4 turns × 2s extra) |
| Use case | Quick desk test | Slow PID or heavy vehicle |

### 21.9 How to Modify the Demo for Your Own Missions

#### Example 1: Make a Triangle

```python
def build_demo_triangle() -> StateMachine:
    sm = StateMachine(outcomes=["triangle_done"])
    sm.add_state("SETUP", CbState(["ready"], _setup_triangle),
                 transitions={"ready": "LEG_1"})

    for i in range(1, 4):                             # 3 legs, not 4
        sm.add_state(f"LEG_{i}", DemoLegState(i),
                     transitions={"leg_done": f"TURN_{i}"})
        is_final = (i == 3)                            # 3rd turn is final
        after = "triangle_done" if is_final else f"LEG_{i + 1}"
        sm.add_state(f"TURN_{i}",
                     DemoTurnState(i, is_final=is_final),
                     transitions={"turn_done": after})
    return sm

def _setup_triangle(blackboard):
    blackboard["turn_angle"] = 120.0      # 360° / 3 = 120° per corner
    blackboard["leg_duration"] = 4.0
    blackboard["demo_speed"] = 40
    blackboard["turn_settle"] = 2.5
    blackboard["ctx"].log("TRIANGLE — starting 3-leg triangle")
    return "ready"
```

#### Example 2: Add a Depth Hold During the Square

```python
from yasmin import CbState

def _dive_before_square(blackboard):
    ctx = blackboard["ctx"]
    ctx.send('pid_depth', depth=0.5)      # Hold at 0.5m while doing square
    ctx.sleep(3.0)                         # Wait to reach depth
    return "ready"

def build_underwater_square():
    sm = StateMachine(outcomes=["done"])
    sm.add_state("DIVE", CbState(["ready"], _dive_before_square),
                 transitions={"ready": "SQUARE"})
    sm.add_state("SQUARE", build_demo_square(),
                 transitions={"square_done": "done"})
    return sm
```

#### Example 3: Add Heading Logging Between Moves

```python
class LogHeadingState(State):
    def __init__(self, label):
        super().__init__(outcomes=["logged"])
        self._label = label

    def execute(self, blackboard):
        ctx = blackboard["ctx"]
        ctx.log(f"CHECKPOINT {self._label}: heading={ctx.heading:.1f}° "
                f"depth={ctx.depth:.2f}m")
        return "logged"
```

Insert it between a turn and the next leg:

```python
sm.add_state("CHECK_1", LogHeadingState("after turn 1"),
             transitions={"logged": "LEG_2"})
# Change TURN_1's transition: {"turn_done": "CHECK_1"} instead of "LEG_2"
```

### 21.10 How the Demo Maps to a Real Competition Mission

The demo is structurally identical to a real mission — the only differences are what the states do and how many there are:

| Demo Concept | Competition Equivalent |
|---|---|
| `ArmState` | Same — always needed |
| `SETUP` (inject params) | `_setup_gate_blackboard` — loads task-specific config |
| `DemoLegState` (drive forward) | `DriveState` — same idea, more options |
| `DemoTurnState` (PID turn) | `SearchState` (rotates looking for targets) |
| No vision | `AlignState` (uses camera to centre on target) |
| Fixed pattern | Dynamic: search → align → drive based on outcomes |
| `SurfaceState` | Same — always needed at the end |

The key difference is that the demo is **open-loop** (fixed timing, no sensor feedback) while competition missions are **closed-loop** (react to detections, alignment status, depth readings). But the FSM structure — states, outcomes, transitions, Blackboard — is identical.

### 21.11 What Can Go Wrong (and Why)

| Symptom | Cause | Fix |
|---|---|---|
| "vehicle not armed" on all commands | `arm` command dropped during DDS discovery | Fixed in latest version with `wait_for_ready()` + retry |
| AUV drifts during legs | No heading hold — `move_forward` doesn't hold compass heading | Use `go_forward` instead (holds heading via PID) |
| Turns overshoot | `turn_settle` too short, PID hasn't converged | Increase `turn_settle` or tune PID gains in inspector |
| Square is lopsided | Current in the water pushes AUV during forward legs | Use `go_forward` with heading hold, or add drift correction |
| Thrusters jerk between moves | A `stop` command was accidentally added between states | Remove all `stop` between consecutive moves — let the ramp handle it |
| Mission runs but nothing moves | `mavlink_inspector` not running or not connected to Pixhawk | Start inspector first, verify it connects |
| "No telemetry" warning but vehicle arms | Inspector is running but `VehicleState` hasn't been received yet | Increase `arm_settle_time` in `planner.yaml` |
| Square completed but vehicle still moving | `is_final` not set on last turn | Ensure `DemoTurnState(4, is_final=True)` |

### 21.12 From Demo to Competition: The Upgrade Path

Once you've verified the demo works, building a competition mission follows the same pattern but with sensor-driven states:

```
DEMO:          ARM → [LEG → TURN] × 4 → DISARM
                      (open-loop)

COMPETITION:   SUBMERGE → [SEARCH → ALIGN → DRIVE] per task → SURFACE
                           (closed-loop)
```

The `SEARCH` state replaces fixed legs — instead of driving forward for a set time, it rotates looking for a YOLO detection. The `ALIGN` state replaces fixed turns — instead of turning a fixed angle, it uses PID visual servoing to centre the target in the camera frame. The `DRIVE` state is the same concept as a demo leg but with optional heading hold.

Every concept from the demo carries forward:
- States return outcome strings
- Transitions route outcomes to next states
- Blackboard carries parameters
- The ramp handles smooth thruster transitions
- The YASMIN viewer shows real-time state

---

*This document is part of the Duburi 4.2 analysis suite. For competitive analysis see `13_COMPETITIVE_ANALYSIS.md`. For the YASMIN vs BT analysis see `15_MISSION_PLANNER_ANALYSIS.md`. For package-level docs see `src/duburi_planner/README.md`.*
