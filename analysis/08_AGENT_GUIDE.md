# Agent Quick Reference

For AI agents (Cursor, etc.) working on this codebase. Read this first, then dive into specific analysis files as needed.

---

## Critical Rules (Do Not Break)

1. **RC override must be sent at 20+ Hz** when controlling the vehicle. See `07_ARDUSUB_CONSTRAINTS.md`.
2. **Idle = send neutral**, not silence. Otherwise ArduSub disarms.
3. **Arm/disarm in a thread** — never block the executor with motors_armed_wait.
4. **Mission wait logic:** Always `time.sleep(wait_sec)` after commands with duration. Never skip wait when `wait_sec > 0`.

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

---

## Common Tasks

### Add a new movement command

1. Add to `DriverCommand.msg` comments.
2. In `inspector_node.py` `_on_driver_command`: add `elif c == 'new_cmd'` with `set_movement(...)` or custom logic.
3. In `runner.py` `_parse_one`: add parsing branch.
4. In `driver_client.py`: add helper if missions need it.

### Add a new mission file

1. Create `missions/<name>.txt` with one command per line.
2. Lines starting with `#` are ignored.
3. Use same syntax as CLI: `move forward 50% 5s`, `arm`, etc.
4. **First line should be `mode MANUAL`** before `arm` for RC override to work.

### Fix thruster not moving

- Check: Is RC being sent? Inspector must run `_send_rc_override` at 20 Hz.
- Check: Is `_current_movement` set? Or `_yaw_to_heading`?
- Check: Is vehicle armed and in MANUAL?

### Fix disarm after movement

- Likely cause: Stopped sending RC after movement ended. Inspector must send neutral when idle. See `_send_rc_override` else branch.

### Fix mission commands overwriting each other

- Cause: Not waiting for duration. In `_execute_chain`, ensure `if wait_sec > 0: time.sleep(wait_sec)` runs for every command, not only when `i < len(parts) - 1`.

---

## Key Variables

| Variable | Location | Meaning |
|----------|----------|---------|
| `_current_movement` | inspector | `{channels: {ch: pwm}, end_time}` or None |
| `_yaw_to_heading` | inspector | `{target_deg, gain_offset, tolerance_deg}` or None |
| `_default_speed` | runner | Default gain 0–100 (50) |
| `MISSION_PATHS` | runner | Where to find mission files |
| `PWM_RANGE` | inspector | 400 (1100–1900) |
| `NEUTRAL_PWM` | inspector | 1500 |

---

## Testing Checklist

- [ ] `arm` → vehicle arms, no block
- [ ] `move forward 50% 10s` → moves 10 s, then stops, stays armed
- [ ] `run gate` → executes full mission, thrusters move
- [ ] `yaw 90 50%` → rotates to 90° using thrusters
- [ ] `move depth 0.2` → accepts float depth
- [ ] Ctrl+C → stops thrusters, clean exit (no RCLError)
