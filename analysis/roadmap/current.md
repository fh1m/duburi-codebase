# Current Development

> Active work in progress (April 2026)

---

## MAJOR MILESTONE: V2 Control Stack Complete + All Bugs Fixed! [DONE]

**Status:** **READY FOR TESTING**

The V2 control redesign and comprehensive bug fix effort are **100% COMPLETE**:
- [DONE] **V2 Control Stack**: All 5 phases implemented and verified
- [DONE] **Bug Fixes**: All 30 issues resolved (3 CRITICAL, 7 HIGH, 10 MEDIUM, 6 LOW, 4 INFO)
- [DONE] **Build Status**: PASSING (4.10s, all 10 packages)

**See detailed reports:**
- [Bug Fix Completion Report](bugfix-completion-report.md)
- [Detailed Bug Tracking](bugfixes-2026-04.md)
- [DONE] [Completed Features](completed.md)

---

## Active Focus Areas

### 1. SITL Validation

**Status:** NEXT PRIORITY

Before pool testing, validate control stack in ArduSub SITL simulator.

**Tasks:**
- [ ] Connect Gazebo SITL to ROS 2 pipeline
- [ ] Test V2 features (velocity estimation, convergence gates, active braking)
- [ ] Run square, circle, and complex maneuver missions
- [ ] Validate cascade position/velocity control
- [ ] Test gain scheduling at different speeds
- [ ] Verify mission reliability improvements

**Why This Matters:**
- Catch integration issues before expensive pool time
- Tune parameters in simulation first
- Validate V2 improvements quantitatively

---

### 2. Pool Testing Preparation

**Status:** Scheduled (Post-SITL)

Pool time is limited every session must count.

**Pre-Pool Checklist:**
- [ ] Complete SITL validation
- [ ] Verify Pixhawk connection and telemetry
- [ ] Check thruster response to all commands
- [ ] Validate camera feed quality underwater
- [ ] Prepare logging and data collection
- [ ] Load tuned parameters from SITL

**Testing Priorities:**
1. **Shallow Water Tests** 0.5m depth, stability validation
2. **Depth Holding** Test at 0.3m, 0.5m, 1.0m, 2.0m (hold 5 minutes)
3. **Yaw Control** 90°, 180°, 270° turns with precision
4. **Square Mission** Test turn accuracy and position tracking
5. **Emergency Procedures** RC override, surface command, kill switch

---

### 3. Parameter Tuning

**Status:** Ready for SITL

V2 control stack has new parameters to tune:

| Parameter Category | Components | Status |
|-------------------|------------|--------|
| Velocity Estimator | Alpha filter, gravity comp | ⏳ Needs SITL tuning |
| Convergence Gates | Position/velocity thresholds | ⏳ Needs SITL tuning |
| Active Braking | Decel threshold, brake gain | ⏳ Needs SITL tuning |
| Cascade Controller | Inner velocity, outer position PIDs | ⏳ Needs SITL tuning |
| Gain Scheduling | Speed-dependent gain curves | ⏳ Needs SITL tuning |

**Tuning Methodology:**
1. Start with SITL simulation
2. Record step response at multiple speeds
3. Measure overshoot, settling time, steady-state error
4. Adjust gains using Ziegler-Nichols or manual tuning
5. Validate in pool
6. Document final values for repeatability

---

## V2 Control Stack Features (All Implemented) [DONE]

| Feature | Status | Description |
|---------|--------|-------------|
| **Velocity Estimator** | [DONE] COMPLETE | IMU acceleration integration with gravity compensation via quaternion rotation |
| **Convergence Gates** | [DONE] COMPLETE | Position & velocity thresholds for mission reliability |
| **Active Braking** | [DONE] COMPLETE | Automatic deceleration near waypoints to reduce overshoot |
| **Cascade Control** | [DONE] COMPLETE | Position controller → Velocity controller → Thrust output |
| **Gain Scheduling** | [DONE] COMPLETE | Speed-adaptive gains for stable control across velocities |

**Documentation:** See [V2 Features Guide](../guides/v2-features.md)

---

## Bug Fixes - Complete [DONE]

> **See detailed tracking:** [bugfixes-2026-04.md](bugfixes-2026-04.md)

### **ALL BUGS FIXED! (30/30 - 100%)** [DONE]

#### CRITICAL CRITICAL (3/3 - 100%) [DONE]
| # | Issue | Status |
|---|-------|--------|
| [DONE] 1 | GCS Heartbeat Rate 1Hz → 2Hz | **FIXED** |
| [DONE] 2 | RC Override Not Continuous | **ALREADY IMPLEMENTED** |
| [DONE] 3 | Depth from AHRS2 (MSL altitude) | **FIXED** |

#### WARNING HIGH Priority (7/7 - 100%) [DONE]
| # | Issue | Status |
|---|-------|--------|
| [DONE] 4 | IMU velocity integration gravity | **FIXED** |
| [DONE] 5 | Thread safety: _ramped dict | **FIXED** |
| [DONE] 6 | CH_THROTTLE neutral depth oscillation | **VERIFIED CORRECT** |
| [DONE] 7 | PID derivative kick | **VERIFIED CORRECT** |
| [DONE] 8 | MAV_FRAME_GLOBAL_INT wrong | **FIXED** |
| [DONE] 9 | CascadeController integral state | **FIXED** |
| [DONE] 10 | IMU gravity fallback | **FIXED** |

#### [INFO] MEDIUM Priority (10/10 - 100%) [DONE]
| # | Issue | Status |
|---|-------|--------|
| [DONE] 11 | DVL bottom_lock detection | **FIXED** |
| [DONE] 12 | MAVLink source_system | **VERIFIED CORRECT** |
| [DONE] 13 | Blocking wait in callback | **VERIFIED CORRECT** |
| [DONE] 14 | V2 features documentation | **FIXED** |
| [DONE] 15 | wait_for_ready blocks | **FIXED** |
| [DONE] 16 | Trailing slash inconsistent | **FIXED** |
| [DONE] 17 | Fixed dt=0.05s in yaw | **FIXED** |
| [DONE] 18 | ZUPT threshold too low | **FIXED** |
| [DONE] 19 | Ramp decel fights braking | **FIXED** |
| [DONE] 20 | SET_ATTITUDE_TARGET | **VERIFIED CORRECT** |

#### [COMPLETE] LOW Priority (6/6 - 100%) [DONE]
| # | Issue | Status |
|---|-------|--------|
| [DONE] 21 | Variable names unclear | **FIXED** |
| [DONE] 22 | CommandSpec design | **VERIFIED CORRECT** |
| [DONE] 23 | Speed param docs missing | **FIXED** |
| [DONE] 24 | State log interval hardcoded | **FIXED** |
| [DONE] 25 | DEFAULT_SPEED unclear units | **FIXED** |
| [DONE] 26 | DuburiClient API | **VERIFIED CORRECT** |

#### [INFO] INFO/Enhancements (4/4 - 100%) [DONE]
| # | Issue | Status |
|---|-------|--------|
| [DONE] 27 | MAVLink message watchdog | **IMPLEMENTED** |
| [DONE] 28 | Parameter validation | **IMPLEMENTED** |
| [DONE] 29 | MAV_CMD_DO_SET_HOME | **IMPLEMENTED** |
| [DONE] 30 | Simulation time support | **IMPLEMENTED** |

**Build Status:** [DONE] PASSING (4.10s, all 10 packages)
**Final Progress:** 30/30 issues resolved (100%)
**Ready for:** SITL validation, pool testing, DVL integration

---

## Next Steps (Immediate Priority)

### 1. SITL Validation (This Week)
- Connect Gazebo ArduSub simulator
- Test all V2 features in simulation
- Tune parameters before pool testing
- Validate mission reliability improvements

### 2. Pool Testing (After SITL)
- Shallow water stability tests
- Depth and yaw holding (5 minute tests)
- Square mission validation
- Emergency procedure verification

### 3. DVL Integration (Next Major Phase)
- Nortek Nucleus 1000 MAVLink driver
- Velocity fusion with IMU velocity estimator
- Bottom lock detection and handling
- Waypoint navigation with position feedback

---

## Known Issues (Legacy - Low Priority)

| Issue | Severity | Status | Notes |
|-------|----------|--------|-------|
| Version skew in package.xml/setup.py | Low | Open | Issue #4 - cosmetic |
| perception.launch.py default model mismatch | Medium | Open | Issue #5 - needs fix |
| No `stop` on runner/executor shutdown | Medium | Open | Issue #6 - cleanup needed |

---

## This Week's Goals

| Priority | Task | ETA |
|----------|------|-----|
| [IN PROGRESS] CRITICAL | SITL validation and V2 testing | Apr 5 |
| [IN PROGRESS] HIGH | Parameter tuning in simulation | Apr 6 |
| [IN PROGRESS] HIGH | Schedule pool session | Apr 7 |
| [MEDIUM] MEDIUM | Document V2 tuning guide | Apr 8 |
| [MEDIUM] MEDIUM | DVL integration planning | Apr 10 |

---

## Blocked Items

| Item | Blocked By | Resolution Path |
|------|------------|-----------------|
| Pool testing | SITL validation incomplete | Complete SITL tests first |
| DVL integration | Hardware driver not written | Research ROS 2 DVL drivers |
| Full mission testing | Simulation not connected | Phase 5.3 - Gazebo integration |
| Waypoint navigation | No DVL integration | Complete Phase 4.1 |

---

## Development Velocity Metrics

| Metric | Value |
|--------|-------|
| V2 Control Stack Phases | 5/5 (100%) [DONE] |
| Bug Fixes Resolved | 30/30 (100%) [DONE] |
| Critical Issues | 0 remaining [DONE] |
| High Priority Issues | 0 remaining [DONE] |
| Build Status | PASSING [DONE] |
| Ready for Testing | YES [DONE] |

**Last Major Update:** April 3, 2026 - All V2 features and bug fixes completed

---
