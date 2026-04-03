# Current Development

> Active work in progress (April 2026)

---

## Active Focus Areas

### 1. Action Server Implementation 🔧

**Status:** In Progress

Converting command execution to ROS 2 action servers for long-running command support.

**What's Being Done:**
- Implementing action servers for movement commands
- Adding preemption support for interrupted commands
- Feedback streaming during command execution
- Goal cancellation for emergency stops

**Why It Matters:**
- Current `/driver/command` is fire-and-forget
- Action servers enable: progress monitoring, cancellation, multi-step sequences
- Required for reliable YASMIN state transitions

---

### 2. Pool Testing Preparation 🏊

**Status:** Scheduled

Pool time is limited — every session must count.

**Pre-Pool Checklist:**
- [ ] Verify Pixhawk connection and telemetry
- [ ] Check thruster response to all commands
- [ ] Validate camera feed quality underwater
- [ ] Prepare logging and data collection

**Testing Priorities:**
1. **Depth PID** — Test at 0.3m, 0.5m, 1.0m, 2.0m
2. **Yaw PID** — 90°, 180°, 270° turns
3. **Visual servo** — Gate alignment and approach
4. **Battery compensation** — Thrust consistency across voltage range

---

### 3. Parameter Tuning 🎛️

**Status:** Ongoing

Current defaults need pool validation:

| Parameter | Default | Target | Status |
|-----------|---------|--------|--------|
| Depth PID (kp, ki, kd) | 500, 25, 200 | TBD | ⏳ Needs testing |
| Yaw PID (kp, ki, kd) | 2.0, 0.05, 0.5 | TBD | ⏳ Needs testing |
| Visual servo lateral | (configured) | ±5px settling | ⏳ Needs testing |
| Trapezoidal ramp rate | (configured) | Smooth but responsive | ⏳ Needs testing |

**Tuning Methodology:**
1. Record step response at multiple operating points
2. Measure overshoot, settling time, steady-state error
3. Adjust gains using Ziegler-Nichols or manual tuning
4. Document results for repeatability

---

## Active Issues & Bugs

### Known Issues

| Issue | Severity | Status | Notes |
|-------|----------|--------|-------|
| Version skew in package.xml/setup.py | Low | Open | Issue #4 |
| perception.launch.py default model mismatch | Medium | Open | Issue #5 |
| No `stop` on runner/executor shutdown | Medium | Open | Issue #6 |
| Single-threaded executor timing jitter | Low | Investigating | Issue #7 |

### Under Investigation

- **Visual servo `lost_timeout`**: 2s may be too short in turbid water
- **Alignment oscillation**: Need to test hold stability for 5s
- **Diagonal scaling**: Verify √2 factor produces straight diagonal motion

---

## This Week's Goals

| Priority | Task | Assignee | ETA |
|----------|------|----------|-----|
| 🔴 HIGH | Complete action server prototype | — | Apr 5 |
| 🔴 HIGH | Schedule pool session | — | Apr 7 |
| 🟡 MEDIUM | Start YOLO training data collection | — | Apr 7 |
| 🟡 MEDIUM | Write slalom state machine | — | Apr 10 |
| 🟢 LOW | Fix Issue #5 (launch file) | — | Apr 10 |

---

## Blocked Items

| Item | Blocked By | Resolution Path |
|------|------------|-----------------|
| Full mission testing | Simulation not connected | Phase 5.3 - Gazebo integration |
| Multi-object tracking | Kalman tracker is single-object | Phase 3.3 - Extend tracker |
| Waypoint navigation | No DVL integration | Phase 4.1 - DVL driver |

---

## Notes for Next Standup

- Need to confirm pool availability for April 8-10
- YOLO training data collection should start ASAP (long lead time)
- Consider simulation work in parallel with pool prep
