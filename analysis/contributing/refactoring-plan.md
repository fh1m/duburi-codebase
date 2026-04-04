# 11 All-Package Refactoring Plan

> Generated after completing the mavlink_inspector refactoring.
> **Status**: Phase 1 largely complete, Phase 2 partially complete. See [DONE] markers below.

---

## Scope

| Package | Files | Lines | Severity |
|---------|-------|-------|----------|
| mavlink_runner | 1 main | 869 | HIGH |
| mavlink_driver | 3 main | 998 | HIGH |
| mavlink_logger | 1 main | 233 | LOW |
| vision | 2 main | 489 | MEDIUM |
| vision_inspector | 4 main | 811 | MEDIUM |
| **Total** | **11** | **3,400** | |

---

## Phase 1 Low Risk, High Impact

These are safe, mechanical changes. No behavioral difference.

### 1.1 Extract shared constants [DONE] Done (duburi_common.constants)
- `DEFAULT_SPEED = 50` (used in ~40 signatures in driver_client.py, references in runner.py)
- `MISSION_PATHS` dict (duplicated in runner.py L35-39 and mission_executor.py L75-79)
- `_DIRECTION_MAP` (duplicated in runner.py L537-542 and L613-618)
- Topic names (`/driver/command`, `/mavlink/events`, `/camera/image_raw`) scattered across files

### 1.2 Hoist hardcoded wait times to named constants [DONE] Done (duburi_common.constants)
In runner.py:
- `4.0` arm wait (L361), `2.0` disarm wait (L366)
- `2.0` surface wait (L424), `1.0` depth wait (L413)
- `0.5` PID depth wait (L404)
- `16.8`/`14.0` battery voltage bounds (L230)
- `3.0` health timer interval (L161), `5.0` stale telemetry threshold (L204)

### 1.3 Extract `resolve_aliases()` shared function [DONE] Done (duburi_common.command_vocabulary)
The `dive→depth`, `p_dive→~depth`, `yaw→heading`, `p_yaw→~heading`, `p_turn→~turn`
alias table is copied verbatim in:
- runner.py L320-339
- mission_executor.py L340-358

Extract to one function, import in both.

### 1.4 Generate `just_*` variants programmatically in driver_client.py Not yet implemented
~25 one-liner `just_*` functions (L260-355) are mechanical copies of their base
counterparts. Replace with:
```python
def _make_just(base_func, just_name):
 def wrapper(*a, **kw):...
 return wrapper
```
Eliminates ~100 lines of boilerplate.

### 1.5 Extract `V4L2Camera` helper class Not yet implemented
Camera open/configure/read logic is duplicated 4 times across 2 packages:
- camera_manager_node.py / camera_device.py (camera open/configure logic)
- camera_tester.py `run_test()` (L50-62)
- camera_calibrator.py `run_calibration()` (L79-89)
- detector_standalone.py `run()` (L68-77)

Extract to a shared class with `open()`, `read()`, `release()`, `actual_resolution`.

### 1.6 Extract image conversion utilities [DONE] Done
`_ros_image_to_cv2` and `_cv2_to_ros_image` exist in:
- detector_node.py (L258-301)
- camera_node.py (L139-152, inline)

These are `cv_bridge` replacements. Move to a shared `image_utils.py`.

### 1.7 Fix `_DZ` hardcode in teleop_driver.py Not yet implemented
Dead-zone `_DZ = 0.1` at L25 should be a ROS parameter.

### 1.8 Fix `hasattr` control flow in detector_node.py [DONE] Done
`self._class_names_filter` (L91-97) initialize explicitly in `__init__` as `None`,
check with `is not None` instead of `hasattr()`.

---

## Phase 2 Medium Risk, High Impact

These change dispatch architecture. Behavioral testing required after each.

### 2.1 Unify command parsing (HIGHEST LEVERAGE) [DONE] Done
**The #1 cross-cutting issue was resolved.** Two independent parsers existed:
- runner.py `_parse_one()` (L287-693): regex-based, ~400 lines if/elif
- mission_executor.py `_parse_file_command()` (L318-478): positional args, ~150 lines if/elif

Both support the same command vocabulary with independently maintained logic.
Every new command requires dual edits.

**Resolution:** Created `command_parser.py` in `mavlink_runner` and `mission_parser.py` in `mavlink_driver`, both sharing `duburi_common.command_vocabulary`:
```
command_parser.py
 resolve_aliases(cmd) → normalized cmd
 parse_runner_line(text) → (command, gain, duration)
 parse_file_args(cmd, args) → DriverCommand
 COMMAND_DISPATCH: dict[str, Callable]
```

### 2.2 Convert runner `_parse_one()` to dispatch table [DONE] Done
The runner's `_parse_one()` now delegates to `command_parser.parse_command()` which uses the shared vocabulary and dispatch pattern.

### 2.3 Convert executor `_parse_file_command()` to dispatch table [DONE] Done
Extracted to `mission_parser.parse_file_command()` which uses `duburi_common.command_vocabulary` for shared alias/prefix resolution.

### 2.4 Extract `_print_status()` rendering from business logic [DONE] Done
Extracted to `mavlink_runner/status_display.py` (75 lines). Clean separation of status data model from ANSI rendering.

### 2.5 Move `pool_test` hardcoded mission to file
mission_executor.py `_mission_pool_test()` (L229-262) has an inline mission.
Convert to `missions/pool_test.txt`.

### 2.6 Extract `_print_movement()` helper
5+ occurrences of the same 6-8 line feedback printing template in runner.py.
Extract to one function.

---

## Phase 3 Larger Structural Changes

### 3.1 Extract `YoloInference` helper class
detector_node.py (L131-190) and detector_standalone.py (L100-142) share the same
YOLO model loading, inference, box extraction, and annotation drawing.
Extract to a reusable `YoloInference` class.

### 3.2 Extract `FrameDisplayLoop` utility
camera_tester.py (L67-131) and detector_standalone.py (L83-151) share the same
read→FPS→overlay→keyboard→display loop. Extract to a callback-based helper.

### 3.3 Add `TeleopCommand.msg` to duburi_interfaces [DONE] RESOLVED
DriverCommand field overloading for teleop (Issue 7 from design issues):
`speed`, `duration`, `depth`, `angle` were repurposed to carry PWM offsets.
**Resolution:** Added dedicated `TeleopCommand.msg` with proper fields:
`linear_x`, `linear_y`, `linear_z`, `angular_z`, `speed`, `idle`.
See `teleop_driver.py` and `alignment_controller.py` for usage.

### 3.4 Add logger error resilience
Wrap file I/O in logger_node.py (`write()`/`flush()` at L162, L168) in
try/except with throttled warning. Prevents disk-full crashes.

### 3.5 Add diagnostics CSV to logger
Currently only logs `armed,mode,depth,yaw,pitch,roll,voltage,current`.
Pressure, temperature, CPU load, servo outputs are available but not captured.

### 3.6 Move `HELP_TEXT` out of runner.py source
86-line string literal (L44-129) should be an external file or generated function.

---

## Dependency Graph

```
Phase 1 items are independent can be done in any order.

Phase 2 dependencies:
 1.3 (aliases) 2.1 (unified parsing)
 1.1, 1.2 2.2, 2.3 (dispatch tables use named constants)

Phase 3 dependencies:
 2.1 (parsing) unit tests can cover the unified parser
 1.6 (img utils) 3.1 (YoloInference uses shared conversion)
 1.5 (V4L2Cam) 3.2 (FrameDisplayLoop uses shared camera)
```

---

## Recommended Execution Order

1. Phase 1 items (1.1 through 1.8) one commit per logical group
2. Phase 2.1 → 2.2 → 2.3 unified parsing, one commit
3. Phase 2.4 → 2.6 runner cleanup, one commit
4. Phase 2.5 pool_test migration, one commit
5. Phase 3 items one commit each
6. Unit tests after all structural changes stabilize

---

## Risk Assessment

| Risk | Mitigation |
|------|-----------|
| Command parsing unification breaks behavior | Behavioral diff test: run all mission files through old vs new parser |
| `just_*` generation breaks existing API | grep all consumers, verify method signatures match |
| Shared camera class changes timing | Test with actual hardware before merging |
| Interface change (TeleopCommand) breaks comms | Version bump, update all consumers atomically |

---

## What We Already Fixed (inspector)

For reference, the inspector refactoring addressed:
- [DONE] Issue 1: God Object decomposition (1854 → 6 modules)
- [DONE] Issue 3: Hardcoded timing → ROS params
- [DONE] Issue 4: if/elif command dispatch → dispatch table
- [DONE] Issue 5: if/elif telemetry → dispatch table
- [DONE] Movement extraction to standalone file with MOVEMENTS registry

Remaining known items:
- Issue 2: Unit tests (deferred)
- Issue 6: RC idle optimization (deferred too risky before testing)
- Issue 7: DriverCommand overloading [DONE] RESOLVED (TeleopCommand.msg added)
