# mavlink_runner – Line-by-Line Analysis

**File:** `src/mavlink_runner/mavlink_runner/runner.py`

CLI node that parses user input and publishes `DriverCommand` to `/driver/command`.

---

## readline Import

```python
try:
    import readline  # Enables up/down history, left/right cursor (Unix)
except ImportError:
    pass
```
**Why:** On Unix, importing readline enables history and line editing for `input()`. No-op on Windows; code still runs.

---

## MISSION_PATHS

> **Note (updated):** `MISSION_PATHS` is now defined centrally in `duburi_common.constants` and re-exported by `mavlink_runner.constants`. The runner no longer defines this locally — it imports from the shared module to avoid duplication across runner and mission_executor.

```python
# duburi_common/constants.py (canonical definition)
MISSION_PATHS = [
    Path.cwd() / 'missions',
    Path(__file__).resolve().parent.parent / 'missions',
    Path.home() / '.duburi' / 'missions',
]

# mavlink_runner/constants.py (re-export)
from duburi_common.constants import MISSION_PATHS, HISTORY_FILE
```
**Why:** Search order: current dir, package `missions/`, user `~/.duburi/missions/`. Lets users add missions without changing code. Centralised in `duburi_common` so both `mavlink_runner` and `mavlink_driver.mission_executor` use the same paths.

---

## _parse_one() Return Type

```python
def _parse_one(self, line: str) -> tuple[bool, float]:
    """Returns (continue, wait_sec)."""
```
**Why:** `continue` = keep running (False for quit). `wait_sec` = how long to sleep before the next command (for missions and chained commands).

---

## Parameter Extraction Order

```python
for m in re.finditer(r'(\d+(?:\.\d+)?)\s*s\b', line):
    duration = float(m.group(1))
for m in re.finditer(r'(\d+(?:\.\d+)?)\s*%', line):
    gain = min(100, max(0, float(m.group(1))))
line = re.sub(r'\d+(?:\.\d+)?\s*s\b', '', line)
line = re.sub(r'\d+(?:\.\d+)?\s*%', '', line)
```
**Why:** Parse duration (e.g. `10s`) and gain (e.g. `50%`) first, then remove them so the rest of the line is the command. Order of `10s` and `50%` does not matter.

---

## arm/disarm wait_sec

```python
return True, 4.0  # Wait for vehicle to arm before next command
return True, 2.0  # Wait for disarm to complete
```
**Why:** Arming takes a few seconds. Mission runner must wait before sending movement. 4 s for arm, 2 s for disarm.

---

## yaw_to_heading vs yaw_angle

```python
self._publish(DriverCommand(command='yaw_to_heading', angle=angle, speed=int(gain)))
return True, 0.0  # Inspector handles until reached
```
**Why:** `yaw_to_heading` uses thrusters and runs until heading is reached. No fixed duration, so `wait_sec = 0`. Inspector controls completion.

---

## Depth Parsing – rstrip('m')

```python
depth_str = args[1].rstrip('m')
depth = float(depth_str)
```
**Why:** Supports both `move depth 0.2` and `move depth 0.2m`.

---

## _execute_chain – Wait Logic

```python
for part in parts:
    cont, wait_sec = self._parse_one(part)
    if not cont:
        return False
    if wait_sec > 0:
        time.sleep(wait_sec)
```
**Why:** Always sleep when `wait_sec > 0`. Previously we only waited when `i < len(parts) - 1`, so single-command lines (e.g. in mission files) never waited and commands overwrote each other.

---

## _run_mission – Line Processing

```python
if not line or line.startswith('#'):
    continue
```
**Why:** Skip empty lines and comments.

```python
if not self._execute_chain(line):
    return False
```
**Why:** Each line can have multiple commands (`;`). `execute_chain` returns False on quit.

---

## KeyboardInterrupt Handling

```python
except KeyboardInterrupt:
    self._publish(DriverCommand(command='stop'))
    print('\nStopped. Use "quit" to exit.')
```
**Why:** Ctrl+C stops thrusters immediately. Node keeps running; user can quit with `quit`.

---

## readline History

```python
readline.read_history_file(str(HISTORY_FILE))
# ... in finally:
readline.write_history_file(str(HISTORY_FILE))
```
**Why:** Persist command history across sessions. `HISTORY_FILE = Path.home() / '.duburi_history'`.

---

## _on_event – Arm/Disarm Feedback

```python
print(f'\r  [{label}]')
```
**Why:** `\r` overwrites the current line so "Arming..." is replaced by "Armed." without extra newlines. Non-blocking: we do not wait for ack.
