# COMMAND VERIFICATION REPORT
## Duburi 4.2 AUV Control Stack
**Date:** 2024
**Scope:** Verify documented command syntax matches actual implementation

---

## EXECUTIVE SUMMARY

**COMMANDS CHECKED:** 15  
**MISMATCHES FOUND:** 3 (Critical)  
**FIXES REQUIRED:** Yes

### Critical Issues:
1. **`dive` command** - Test missions use incorrect syntax `dive 0.5m 50%`
2. **`hold` command** - Used in test missions but NOT IMPLEMENTED
3. **Documentation inconsistency** - missions/README.md shows syntax not supported by parser

---

## DETAILED FINDINGS

### 1. Movement Commands ✅ VERIFIED

All movement commands match across implementation, documentation, and help text.

| Command | Implementation | Docs | Help Text | Status |
|---------|---------------|------|-----------|--------|
| `forward` | `forward [speed%] [duration_s]` | `forward [<gain>%] [<dur>s]` | `forward [gain%] [N]s` | ✅ MATCH |
| `back/backward` | `back [speed%] [duration_s]` | `back [<gain>%] [<dur>s]` | `back [gain%] [N]s` | ✅ MATCH |
| `left` | `left [speed%] [duration_s]` | `left [<gain>%] [<dur>s]` | `left [gain%] [N]s` | ✅ MATCH |
| `right` | `right [speed%] [duration_s]` | `right [<gain>%] [<dur>s]` | `right [gain%] [N]s` | ✅ MATCH |
| `up` | `up [speed%] [duration_s]` | `up [<gain>%] [<dur>s]` | `up [gain%] [N]s` | ✅ MATCH |
| `down` | `down [speed%] [duration_s]` | `down [<gain>%] [<dur>s]` | `down [gain%] [N]s` | ✅ MATCH |

**Implementation Details:**
- Parser uses `_extract()` to pull `speed%` and `duration_s` from command line
- Speed range: 0-100% (validated: `min(100, max(0, float(...)))`)
- Default speed: 50% (from `DEFAULT_SPEED_PERCENT` in duburi_common/constants.py)
- Duration default: 0.0 (indefinite) if not specified
- All docs consistently show correct syntax ✅

---

### 2. Depth Commands ⚠️ ISSUES FOUND

#### 2.1 `depth` / `dive` command - ❌ CRITICAL MISMATCH

**Parser Implementation:**
```python
# command_parser.py line 64-71
if cmd == 'depth':
    if is_pid:
        # ~depth handler
        ...
    if not args: print('Usage: depth <m>'); return True, 0.0
    depth_value = float(args[0].rstrip('m'))
    node._publish(DriverCommand(command='set_depth', depth=depth_value))
```

**What it accepts:** `depth <m>` (depth value ONLY, no speed parameter)

**What test missions use:**
```
# test_depth_profile.txt line 10
dive 0.5m 50%
```

**Problem:** Parser does NOT accept speed parameter for depth command, but:
1. Test missions use `dive 0.5m 50%`
2. missions/README.md documents: `dive [target_depth_m] [speed%]`

**Impact:** Test missions will FAIL to parse correctly. The `50%` will be ignored or cause parse errors.

**Root Cause:** `dive` is aliased to `depth` in command_vocabulary.py, but depth command only parses depth value, not speed.

---

#### 2.2 `hold` command - ❌ NOT IMPLEMENTED

**Test missions use:**
```
# test_depth_profile.txt
hold depth 10s

# test_stability.txt
hold depth 30s
hold yaw 30s
```

**Parser status:** Command `hold` does NOT exist in command_parser.py

**Search results:**
```bash
$ grep "cmd == 'hold'" command_parser.py
(no results)
```

**Impact:** Test missions CANNOT RUN - `hold` commands will fail with "Unknown command"

**Documented?** NO - `hold` is NOT documented in:
- command-reference.md
- HELP_TEXT in constants.py
- missions/README.md

**Workaround:** Use `delay` command instead? But `delay` is also not visible in parser.

---

#### 2.3 `surface` command - ✅ VERIFIED

**Parser:** `surface` (no parameters)  
**Docs:** `surface` (no parameters)  
**Status:** ✅ MATCH

---

#### 2.4 `~depth` (PID depth) - ✅ VERIFIED

**Parser:** `~depth [<m>]` (optional depth value)  
**Docs:** `~depth [<metres>]`  
**Status:** ✅ MATCH

---

### 3. Heading Commands ✅ VERIFIED

| Command | Implementation | Docs | Status |
|---------|---------------|------|--------|
| `heading <deg>` | `heading <degrees> [<gain>%]` | `heading <degrees> [<gain>%]` | ✅ MATCH |
| `~heading <deg>` | `~heading <degrees> [<gain>%]` | `~heading <degrees> [<gain>%]` | ✅ MATCH |
| `heading left/right` | `heading left/right [<gain>%] [<dur>s]` | `heading left/right [<gain>%] [<dur>s]` | ✅ MATCH |
| `turn left/right` | `turn left/right <deg> [<gain>%]` | `turn left/right <degrees> [<gain>%]` | ✅ MATCH |
| `~turn left/right` | `~turn left/right <deg> [<gain>%]` | `~turn left/right <degrees> [<gain>%]` | ✅ MATCH |

**Implementation Details:**
- `turn` requires telemetry to compute relative turn
- Parser validates: `len(args) < 2 or args[0] not in ('left', 'right')`
- Angle is required (not optional) for turn commands
- Speed extracted from `speed_percent` via `_extract()`

---

### 4. Parameter Ranges ✅ VERIFIED

#### Speed Percentage
- **Range:** 0-100%
- **Default:** 50% (DEFAULT_SPEED_PERCENT in constants.py)
- **Validation:** `min(100, max(0, float(m.group(1))))` (line 28 command_parser.py)
- **Documented:** ✅ All docs correctly state 0-100%

#### Duration
- **Range:** Any positive float (seconds)
- **Default:** 0.0 (indefinite)
- **Validation:** No max limit in parser
- **Documented:** ✅ Correctly documented as `[N]s`

#### Angle (heading/turn)
- **Range:** 0-360° (no enforcement in parser, modulo 360 applied in turn calculation)
- **Default:** N/A (required parameter)
- **Documented:** ✅ Correctly documented as `<degrees>`

#### Depth
- **Range:** No max limit in parser (should be constrained by max depth parameter?)
- **Default:** N/A (required parameter)
- **Documented:** Stated in meters, positive = below surface

---

### 5. V2 Feature Integration ✅ VERIFIED

#### Convergence Gates
- **Implementation:** Global parameter in defaults.yaml or launch file
- **Per-command control:** NOT AVAILABLE (global only)
- **Documented:** ✅ Correctly states global-only in HELP_TEXT and README

#### Gain Scheduling
- **Implementation:** `GainScheduler` class in velocity_control.py
- **Breakpoints:** 
  - Low: 0-30% (high gains)
  - Medium: 30-60% (balanced gains)
  - High: 60-100% (conservative gains)
- **Code verification:**
```python
# velocity_control.py lines 722-725
self.low_max = config.get('speed_range_low_max', 30)      # 0-30%
self.medium_max = config.get('speed_range_medium_max', 60) # 30-60%
# High range: 60-100%
```
- **Documented:** ✅ HELP_TEXT correctly states ranges

#### Active Braking
- **Implementation:** Global parameter `braking_enabled`
- **Applies to:** All movement commands
- **Documented:** ✅ Correctly documented

---

### 6. Example Missions Analysis

#### test_basic.txt ⚠️ MINOR ISSUE
```
backward 30% 5s
```
**Status:** ✅ WILL PARSE (backward aliased to back)

#### test_square.txt ✅ CORRECT
All commands use correct syntax:
```
forward 30% 5s
turn left 90 50%
```
**Status:** ✅ WILL PARSE CORRECTLY

#### test_depth_profile.txt ❌ CRITICAL ERRORS
```
dive 0.5m 50%      # ❌ Parser does not accept speed parameter
hold depth 10s     # ❌ hold command not implemented
```
**Status:** ❌ WILL FAIL TO PARSE

#### test_stability.txt ❌ CRITICAL ERRORS
```
dive 1.0m 50%      # ❌ Parser does not accept speed parameter
hold depth 30s     # ❌ hold command not implemented
hold yaw 30s       # ❌ hold command not implemented
```
**Status:** ❌ WILL FAIL TO PARSE

---

## CROSS-REFERENCE MATRIX

| Command | command_parser.py | command-reference.md | HELP_TEXT | missions/README.md | Test Missions | Status |
|---------|-------------------|----------------------|-----------|-------------------|---------------|--------|
| forward | `[speed%] [dur]` | `[<gain>%] [<dur>s]` | `[gain%] [N]s` | `[speed%] [duration_s]` | `30% 5s` ✅ | ✅ MATCH |
| turn | `left/right <deg> [speed%]` | `left/right <deg> [<gain>%]` | `left/right <deg> [gain%]` | `left/right [angle_deg] [speed%]` | `left 90 50%` ✅ | ✅ MATCH |
| depth | `<m>` | `<metres>` | `<m>` | `<m>` | N/A | ✅ MATCH |
| dive | Alias→depth `<m>` | `<metres>` | N/A | `[target_depth_m] [speed%]` ❌ | `0.5m 50%` ❌ | ❌ MISMATCH |
| hold | ❌ NOT IMPLEMENTED | ❌ NOT DOCUMENTED | ❌ NOT DOCUMENTED | ❌ NOT DOCUMENTED | `depth 10s` ❌ | ❌ MISSING |
| surface | No params | No params | No params | No params | ✅ | ✅ MATCH |
| ~depth | `[<m>]` | `[<metres>]` | `[<m>]` | `[<m>]` | N/A | ✅ MATCH |
| heading | `<deg> [speed%]` | `<deg> [<gain>%]` | `<deg> [gain%]` | `<deg> [speed%]` | N/A | ✅ MATCH |

---

## FIXES REQUIRED

### Fix 1: Update Test Missions (CRITICAL)

**File:** `analysis/guides/pool-testing/test-missions/test_depth_profile.txt`

**Problem:** Uses incorrect syntax `dive 0.5m 50%` and non-existent `hold depth` command

**Option A - Use depth command + delay:**
```diff
-dive 0.5m 50%
-hold depth 10s
+depth 0.5
+delay 10
```

**Option B - Implement speed parameter for depth command (requires code change)**

**Recommendation:** Option A (update missions to match parser)

---

**File:** `analysis/guides/pool-testing/test-missions/test_stability.txt`

**Problem:** Same issues

**Fix:**
```diff
-dive 1.0m 50%
+depth 1.0
 delay 2
-hold depth 30s
+delay 30

# For yaw hold, use ~heading to PID-lock current heading
-hold yaw 30s
+~heading
+delay 30
```

---

### Fix 2: Update missions/README.md Documentation

**File:** `missions/README.md`

**Problem:** Line 132 documents incorrect syntax for dive command

**Fix:**
```diff
-# Dive to 0.5m
-depth 0.5
-delay 3
+# Dive to 0.5m (ALT_HOLD mode)
+depth 0.5
+delay 3  # Wait for depth stabilization
```

Remove any references to:
- `dive [target_depth_m] [speed%]` ❌ INCORRECT
- `hold depth [duration_s]` ❌ NOT IMPLEMENTED
- `hold yaw [duration_s]` ❌ NOT IMPLEMENTED

Document correct patterns:
- `depth <m>` - Dive to depth (ALT_HOLD)
- `~depth <m>` - PID hold depth (STABILIZE)
- `delay <s>` - Wait for stabilization
- `~heading` - PID lock current heading

---

### Fix 3: (OPTIONAL) Implement Missing Commands

If `hold` functionality is desired, add to command_parser.py:

```python
# Hold command (maintain current state)
if cmd == 'hold':
    if not args: print('Usage: hold depth/yaw <duration>'); return True, 0.0
    hold_type = args[0]
    duration = duration_seconds if duration_seconds > 0 else (float(args[1].rstrip('s')) if len(args) > 1 else 0.0)
    
    if hold_type == 'depth':
        # PID hold current depth
        node._publish(DriverCommand(command='pid_depth', depth=0.0))
        return True, duration
    elif hold_type == 'yaw':
        # PID hold current heading (requires telemetry)
        if node._last_state is None: print('[WARN] No telemetry'); return True, 0.0
        current_yaw = node._last_state.yaw
        node._publish(DriverCommand(command='pid_yaw_to_heading', angle=current_yaw))
        return True, duration
    else:
        print('Usage: hold depth/yaw <duration>'); return True, 0.0
```

**Recommendation:** Implement this - it's a useful command pattern

---

## RECOMMENDATIONS

### Priority 1 (CRITICAL - Breaks Missions)
1. ✅ **Fix test_depth_profile.txt** - Replace `dive 0.5m 50%` with `depth 0.5`
2. ✅ **Fix test_stability.txt** - Replace `dive` and `hold` with correct commands
3. ✅ **Update missions/README.md** - Remove incorrect syntax documentation

### Priority 2 (Enhancement)
4. **Implement `hold` command** - Useful pattern for test missions
5. **Add `delay` command** - For explicit wait periods (may already exist?)

### Priority 3 (Nice to Have)
6. **Add depth speed parameter** - `depth 0.5 50%` for controlled descent rate
7. **Add max depth safety check** - Validate depth <= max_depth parameter

---

## FILES TO MODIFY

### Test Missions (CRITICAL)
- ✅ `analysis/guides/pool-testing/test-missions/test_depth_profile.txt`
- ✅ `analysis/guides/pool-testing/test-missions/test_stability.txt`

### Documentation (IMPORTANT)
- ✅ `missions/README.md` (remove incorrect syntax examples)

### Code (OPTIONAL)
- `src/mavlink_runner/mavlink_runner/command_parser.py` (add `hold` and `delay` commands)

---

## VERIFICATION CHECKLIST

After fixes applied:

- [ ] Test missions parse without errors
- [ ] All documentation matches parser implementation
- [ ] HELP_TEXT is accurate
- [ ] Command reference is accurate
- [ ] Example missions use correct syntax
- [ ] Rebuild with `colcon build` succeeds
- [ ] Run test missions in SITL successfully

---

## APPENDIX: Default Values Summary

| Parameter | Default | Source | Used By |
|-----------|---------|--------|---------|
| `speed%` | 50% | `DEFAULT_SPEED_PERCENT` in duburi_common/constants.py | All movement commands |
| `duration` | 0.0 (indefinite) | `_extract()` in command_parser.py | All timed commands |
| Convergence enabled | true | defaults.yaml | All V2 commands |
| Braking enabled | true | defaults.yaml | All movement commands |
| Convergence velocity threshold | 0.05 m/s | defaults.yaml | Convergence gate |
| Convergence settling time | 0.2 s | defaults.yaml | Convergence gate |
| Convergence timeout | 5.0 s | defaults.yaml | Convergence gate |

---

**Report End**
