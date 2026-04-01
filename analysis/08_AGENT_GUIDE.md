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
| `03_INSPECTOR_LINE_BY_LINE.md` | inspector_node.py explained |
| `04_RUNNER_LINE_BY_LINE.md` | runner.py explained |
| `05_DRIVER_LINE_BY_LINE.md` | driver_client, mission_executor, teleop |
| `06_INTERFACES.md` | Message definitions, field semantics |
| `07_ARDUSUB_CONSTRAINTS.md` | ArduSub requirements that drive design |
| `08_AGENT_GUIDE.md` | This file |
| `09_VISION_SYSTEM.md` | YOLO detection, Kalman tracking, alignment |
| `10_DESIGN_ISSUES.md` | Known issues and their status |
| `11_REFACTORING_PLAN.md` | Planned improvements |
| `12_CODE_REFERENCE.md` | Complete module map (72 files, 9 packages) |
| `19_DESIGN_DECISIONS_ANALYSIS.md` | In-depth pros/cons analysis |
| `20_ARDUSUB_MAVLINK_DEEP_DIVE.md` | Library learnings from pymavlink, ArduSub |

---

## Package Quick Reference

| Package | Key Files | Purpose |
|---------|-----------|---------|
| `mavlink_inspector` | `inspector_node.py`, `command_handler.py`, `rc_controller.py` | MAVLink ↔ ROS bridge |
| `mavlink_driver` | `driver_client.py`, `mission_executor.py` | Command factory, mission runner |
| `mavlink_runner` | `runner.py`, `command_parser.py` | Interactive CLI |
| `vision` | `detector_node.py`, `alignment_controller.py`, `kalman_tracker.py` | YOLO + visual servo |
| `vision_inspector` | `camera_node.py`, `camera_recorder.py` | Camera management |
| `duburi_planner` | `mission_node.py`, `states/*`, `missions/*` | YASMIN FSM planner |
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

1. Create `duburi_planner/missions/<name>_mission.py` with `create_<name>_fsm()` function.
2. Register in `mission_registry.py` `MISSIONS` dict.
3. Compose states from `states/` or create new states as needed.
4. Test with `ros2 run duburi_planner mission_planner --ros-args -p mission_name:=<name>`.

### Add a new FSM state

1. Create `duburi_planner/states/<name>_state.py` inheriting from `BaseState`.
2. Implement `execute()` method returning outcome string.
3. Define outcomes in `__init__`: `super().__init__(outcomes=['succeeded', 'failed', 'aborted'])`.
4. Use `bb_utils.get_context(blackboard)` to access `PlannerContext`.

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
