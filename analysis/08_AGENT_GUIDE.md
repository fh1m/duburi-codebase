# Agent Quick Reference

For AI agents (Cursor, etc.) working on this codebase. Read this first, then dive into specific analysis files as needed.

---

## Critical Rules (Do Not Break)

1. **RC override must be sent at 20+ Hz** when controlling the vehicle. See `07_ARDUSUB_CONSTRAINTS.md`.
2. **Idle = send neutral**, not silence. Otherwise ArduSub disarms.
3. **Arm/disarm in a thread** — never block the executor with motors_armed_wait.
4. **Mission wait logic:** Always `time.sleep(wait_sec)` after commands with duration. Never skip wait when `wait_sec > 0`.
5. **HEARTBEAT filtering:** Only process HEARTBEATs from system_id=1 (vehicle). Ignore GCS echoes (system_id=255).

---

## File Map

| File | Purpose |
|------|---------|
| `00_OVERVIEW.md` | High-level philosophy, project context |
| `01_ARCHITECTURE.md` | Data flow, package roles, topic diagram |
| `02_DESIGN_DECISIONS.md` | Why we chose X over Y |
| `03_INSPECTOR_LINE_BY_LINE.md` | inspector_node.py explained (pre-refactor note) |
| `04_RUNNER_LINE_BY_LINE.md` | runner.py explained |
| `05_DRIVER_LINE_BY_LINE.md` | driver_client, mission_parser, mission_executor, just_commands, teleop |
| `06_INTERFACES.md` | Message definitions, field semantics |
| `07_ARDUSUB_CONSTRAINTS.md` | ArduSub requirements that drive design |
| `08_AGENT_GUIDE.md` | This file |
| `09_KNOWN_ISSUES_AND_GOTCHAS.md` | Known issues, edge cases, and applied fixes |
| `10_DESIGN_ISSUES.md` | Post-refactor architectural concerns |
| `11_DESK_TESTING_GUIDE.md` | Step-by-step desk testing procedures |
| `11_REFACTORING_PLAN.md` | Refactoring plan (Phase 1 largely complete) |
| `12_CODE_REFERENCE.md` | Complete module map (80 files, 10 packages) |
| `12_COMMAND_REFERENCE.md` | Complete command reference with field encoding |
| `13_COMPETITIVE_ANALYSIS.md` | Deep comparison vs Bumblebee (NUS) and Desert WAVE |
| `14_ISSUES_AND_RECOMMENDATIONS.md` | Gap analysis, design critique, and phased roadmap |
| `15_MISSION_PLANNER_ANALYSIS.md` | YASMIN vs Behaviour Trees — comparison & verdict |
| `16_PLANNER_DOCUMENTATION.md` | duburi_planner: theory, implementation & usage |
| `17_SIMULATION_GAZEBO_ARUDSUB_SITL.md` | Gazebo + ArduSub SITL stack |
| `18_BLUEOS_JETSON_NETWORK_BRINGUP.md` | Pi BlueOS + Jetson production setup |
| `19_DESIGN_DECISIONS_ANALYSIS.md` | In-depth pros/cons analysis |
| `20_ARDUSUB_MAVLINK_DEEP_DIVE.md` | Library learnings from pymavlink, ArduSub |
| `21_DUBURI_BLUEOS_PACKAGE_ANALYSIS.md` | duburi_blueos package deep dive |
| `VISION_PERFORMANCE_ANALYSIS.md` | Vision pipeline FPS optimisation (5→25 FPS) |
| `ROADMAP.md` | **RoboSub 2026 development roadmap** |

---

## Package Quick Reference

| Package | Key Files | Purpose |
|---------|-----------|---------|
| `mavlink_inspector` | `inspector_node.py`, `command_handler.py`, `rc_controller.py` | MAVLink ↔ ROS bridge |
| `mavlink_driver` | `driver_client.py`, `mission_parser.py`, `mission_executor.py`, `just_commands.py` | Command factory, mission runner |
| `mavlink_runner` | `runner.py`, `command_parser.py` | Interactive CLI |
| `mavlink_logger` | `logger_node.py` | Topic logging to CSV/JSON |
| `vision` | `detector_node.py`, `alignment_controller.py`, `kalman_tracker.py` | YOLO + visual servo |
| `vision_inspector` | `camera_manager_node.py`, `frame_publisher.py`, `camera_device.py` | Camera management |
| `duburi_planner` | `mission_node.py`, `planner_context.py`, `states/*`, `missions/*` | YASMIN FSM planner |
| `duburi_blueos` | `blueos_monitor_node.py`, `blueos_client.py`, `health_checker.py` | BlueOS REST API client |
| `duburi_common` | `constants.py`, `command_vocabulary.py` | Shared utilities |

---

## Common Tasks

### Add a new movement command

1. Add handler in `mavlink_inspector/movement_commands.py` and register in `MOVEMENTS` dict.
2. In `mavlink_driver/driver_client.py`: add factory function.
3. In `mavlink_runner/command_parser.py`: add parsing branch.
4. Update `duburi_common/command_vocabulary.py` if aliases needed.

### Add a new mission file

1. Create `missions/<name>.txt` with one command per line.
2. Lines starting with `#` are ignored.
3. Use same syntax as CLI: `move forward 50% 5s`, `arm`, etc.
4. **First line should be `mode MANUAL`** before `arm` for RC override to work.

### Add a new YASMIN mission

1. Create `duburi_planner/duburi_planner/missions/<name>.py` with a `create_<name>_fsm()` function.
2. Import and wire the new mission in `mission_node.py` (missions are loaded by name in `__init__`).
3. Compose states from `states/` (arm, submerge, drive, surface, search, align, wait_feedback, send_command) or create new states as needed.
4. Test with `ros2 run duburi_planner mission_planner --ros-args -p mission_name:=<name>`.

> **Note:** There is no `mission_registry.py`. Existing missions are `gate.py` and `demo_square.py` in `duburi_planner/missions/`.

### Add a new FSM state

1. Create `duburi_planner/duburi_planner/states/<name>.py` inheriting from `yasmin.State`.
2. Implement `execute(blackboard)` method returning an outcome string.
3. Define outcomes in `__init__`: `super().__init__(outcomes=['succeeded', 'failed', 'aborted'])`.
4. Use `bb_utils.get_context(blackboard)` to access `PlannerContext`.
5. Existing states for reference: `arm.py`, `submerge.py`, `drive.py`, `surface.py`, `search.py`, `align.py`, `wait_feedback.py`, `send_command.py`.

### Fix thruster not moving

- Check: Is RC being sent? Inspector must run `_rc_override_tick` at 20 Hz.
- Check: Is `RcController.is_active()`? Check `_current_channels`.
- Check: Is vehicle armed and in MANUAL?
- Check: Are PID controllers interfering? Check depth/yaw PID states.

### Fix disarm after movement

- Likely cause: Stopped sending RC after movement ended. Inspector must send neutral when idle. See `_rc_override_tick` 4-layer builder.

### Fix mission commands overwriting each other

- Cause: Not waiting for duration. In mission executor, ensure proper sleep between commands.

### Fix armed/mode flapping

- Likely cause: Processing GCS HEARTBEAT echoes. Ensure `telemetry_parser.py` filters `system_id != 1`.

---

## Key Variables

| Variable | Location | Meaning |
|----------|----------|---------|
| `RcController._current_channels` | inspector | 8-channel PWM state list |
| `RcController._target_channels` | inspector | Target PWM values (for ramping) |
| `PidController._setpoint` | inspector | Depth or yaw target |
| `CommandHandler.system_dispatch` | inspector | Command → handler mapping |
| `MOVEMENTS` | movement_commands | Movement command registry |
| `PlannerContext.vehicle_state` | planner | Cached VehicleState |
| `PlannerContext.detections` | planner | Cached DetectionArray |
| `DEFAULT_SPEED` | duburi_common | Default gain 0–100 (50) |
| `MISSION_PATHS` | duburi_common | Where to find mission files |
| `PWM_RANGE` | rc_controller | 400 (1100–1900) |
| `NEUTRAL_PWM` | rc_controller | 1500 |

---

## Testing Checklist

- [ ] `arm` → vehicle arms, no block
- [ ] `move forward 50% 10s` → moves 10 s, then stops, stays armed
- [ ] `run gate` → executes full mission, thrusters move
- [ ] `yaw 90 50%` → rotates to 90° using thrusters
- [ ] `move depth 0.2` → accepts float depth
- [ ] Ctrl+C → stops thrusters, clean exit (no RCLError)
- [ ] YASMIN planner: `ros2 run duburi_planner mission_planner -p mission_name:=gate`
- [ ] Visual alignment: target detection → alignment_controller → teleop commands
- [ ] Kalman tracker: smooth bounding boxes during dropout
