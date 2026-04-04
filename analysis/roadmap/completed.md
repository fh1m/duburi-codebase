# Completed Development

> What Duburi has accomplished through April 2026.

---

## Overview

The Duburi 4.2 stack is a **10-package, 80-file** ROS 2 Humble codebase with production-ready control and perception systems.

**Major Milestones:**
- ✅ V1 Control Stack (March 2026)
- ✅ V2 Control Redesign (April 2026)
- ✅ Complete Bug Fix Sprint (30/30 issues, April 2026)
- ✅ Vision Pipeline (February-March 2026)
- ✅ Interactive CLI & Mission System (January-March 2026)

---

## Control Stack Redesign V2 (April 2026) ✅

### What Was Built

The V2 control redesign introduced advanced features for mission reliability and precision:

| Component | Status | Description |
|-----------|--------|-------------|
| **Velocity Estimator** | ✅ COMPLETE | IMU acceleration integration with gravity compensation via quaternion rotation |
| **Convergence Gates** | ✅ COMPLETE | Position & velocity thresholds for reliable mission waypoint detection |
| **Active Braking** | ✅ COMPLETE | Automatic deceleration near waypoints to reduce overshoot |
| **Cascade Control** | ✅ COMPLETE | Position controller → Velocity controller → Thrust for improved tracking |
| **Gain Scheduling** | ✅ COMPLETE | Speed-adaptive gains for stable control across different velocities |

### Key Features

1. **Velocity Estimator** — Integrates IMU acceleration with proper gravity compensation
   - Quaternion-based gravity vector rotation
   - Alpha filter for noise reduction
   - Fallback when no external velocity available

2. **Convergence Gates** — Mission reliability improvements
   - Position threshold for waypoint detection
   - Velocity threshold for "truly stopped" detection
   - Prevents premature mission progression

3. **Active Braking** — Reduces overshoot
   - Detects when approaching waypoint
   - Applies counter-thrust to decelerate
   - Configurable brake gain

4. **Cascade Position/Velocity Control** — Better tracking
   - Outer loop: Position error → Velocity setpoint
   - Inner loop: Velocity error → Thrust output
   - Per-DOF integral state management

5. **Gain Scheduling** — Speed adaptation
   - Gains scale with current velocity
   - Maintains stability at high speeds
   - Prevents oscillation at low speeds

### Documentation

- [V2 Features Guide](../guides/v2-features.md)
- [V2 Architecture](../architecture/v2-control-architecture.md)
- [Configuration Guide](../guides/configuration.md)

---

## Bug Fixes - Complete Sprint (April 2026) ✅

### Summary

**All 30 issues resolved (100% completion)**
- 🚨 3 CRITICAL issues FIXED
- ⚠️ 7 HIGH priority issues FIXED
- 🔵 10 MEDIUM priority issues FIXED
- 🟢 6 LOW priority issues FIXED
- ℹ️ 4 INFO/enhancement issues IMPLEMENTED

### Critical Fixes (3/3) ✅

1. **GCS Heartbeat Rate** — Increased from 1Hz to 2Hz for better GCS connection reliability
2. **RC Override Watchdog** — Verified existing 1-second watchdog implementation
3. **Depth from SCALED_PRESSURE** — Fixed depth reading to use pressure sensor (not AHRS2 MSL altitude)

### High Priority Fixes (7/7) ✅

4. **IMU Gravity Compensation** — Proper quaternion rotation of gravity vector before integration
5. **Thread Safety** — Added locks to `_ramped` dict in rc_controller
6. **CH_THROTTLE Neutral** — Verified derivative-on-measurement prevents oscillation
7. **PID Derivative Kick** — Verified correct implementation (derivative-on-measurement)
8. **MAV_FRAME Correction** — Changed from GLOBAL_INT to BODY_NED for local movements
9. **Cascade Controller Integrals** — Per-DOF integral state management
10. **IMU Gravity Fallback** — Proper gravity handling when DVL unavailable

### Medium Priority Fixes (10/10) ✅

11. **DVL Bottom Lock** — Detection and handling of bottom lock loss
12-20. **Documentation, threading, parameter validation, timing fixes**

### Low Priority & Info (10/10) ✅

21-30. **Code clarity, parameter docs, MAVLink watchdog, simulation support**

**Detailed Reports:**
- [Bug Fix Completion Report](../roadmap/bugfix-completion-report.md)
- [Detailed Bug Tracking](../roadmap/bugfixes-2026-04.md)

---

## Control Stack Redesign V1 (March-April 2026)

### What Was Built

The control stack underwent a major redesign to support autonomous mission execution:

| Component | Status | Description |
|-----------|--------|-------------|
| **MAVLink Bridge** | [DONE] Production | 7-module inspector: serial I/O, command dispatch, PID controllers, telemetry, RC override at 20 Hz |
| **Control System** | [DONE] Functional | Software depth PID + yaw PID, trapezoidal PWM ramp, 4-layer RC override, `just_*` instant variants |
| **Command System** | [DONE] Rich | 30+ commands: `move`, `go`, `cruise`, `at`, `just_*`, diagonals, PID depth/yaw, `~` prefix convention |
| **Feedback System** | [DONE] Implemented | `/driver/feedback` with `accepted`/`reached`/`completed`/`rejected` status + error magnitude |

### Key Decisions Made

1. **Separate PID loops for depth and yaw** — Decoupled control for better tuning
2. **Trapezoidal PWM ramping** — Smooth acceleration prevents mechanical stress
3. **4-layer RC override** — Safety at hardware, firmware, software, and application levels
4. **`just_*` instant variants** — Emergency response bypasses normal ramping
5. **`~` prefix convention** — Quick prefix for PID commands (e.g., `~depth 0.5`)

### Problems Solved

- **Thruster coupling**: Diagonal movements scaled by √2 factor for straight motion
- **Battery compensation**: `nominal_voltage` parameter adjusts thrust for battery state
- **Command timing**: Feedback system with `accepted`/`reached`/`completed` states

---

## Vision Pipeline (February-March 2026)

### What Was Built

| Component | Status | Description |
|-----------|--------|-------------|
| **YOLO11 Detection** | [DONE] Functional | YOLO11n on CUDA (20-25 FPS on Orin Nano) |
| **Kalman Tracking** | [DONE] Implemented | Single-object tracking with prediction |
| **Visual Servoing** | [DONE] Functional | PID-based lateral + vertical + forward alignment |
| **Camera Manager** | [DONE] Complete | Multi-camera support with V4L2 backend |

### Key Decisions Made

1. **YOLO11 nano variant** — Optimized for Jetson Orin Nano's 8GB memory
2. **Kalman filter tracking** — Smooths detection jitter, predicts during occlusion
3. **PID visual servo** — Three-axis alignment controller for target centering
4. **Multi-camera architecture** — `camera_manager` supports forward, downward, upward cameras

---

## Interactive CLI & Mission System (January-March 2026)

### What Was Built

| Component | Status | Description |
|-----------|--------|-------------|
| **Interactive CLI** | [DONE] Complete | `duburi_runner` REPL with history, file-based missions, chained commands, status dashboard |
| **Mission Planner** | 🟡 Partial | YASMIN HFSM in `duburi_planner` — 8 reusable states, 2 missions (gate, demo_square) |
| **Teleop** | [DONE] Implemented | `TeleopCommand` on `/driver/teleop` with multi-axis support + idle detection |

### Key Decisions Made

1. **YASMIN HFSM** — Hierarchical finite state machine for readable, composable missions
2. **File-based missions** — `.txt` mission files for quick iteration without code changes
3. **Chained commands** — Single-line command sequences for complex maneuvers

---

## Infrastructure (Ongoing)

### What Was Built

| Component | Status | Description |
|-----------|--------|-------------|
| **BlueOS Integration** | [DONE] Functional | REST API client for system monitoring, parameter management |
| **Logging** | [DONE] Functional | Session-based CSV/JSON logging with rotation |
| **Documentation** | [DONE] Extensive | 26 analysis documents + comprehensive README |

---

## Architecture Diagram (Current State)

```
                     ┌─── CONTROLS ────────────────────┐   ┌──── PERCEPTION ──────────┐
                     │                                  │   │                           │
                     │  Pixhawk 2.4.8 (/dev/ttyACM0)   │   │  USB Camera(s)            │
                     │          │                       │   │       │                    │
                     │  mavlink_inspector (7 modules)   │   │  vision_inspector          │
                     │  ├─ connection_manager            │   │  (camera_manager)          │
                     │  ├─ telemetry_parser              │   │       │                    │
                     │  ├─ command_handler               │   │  /camera/<name>/image_raw  │
                     │  ├─ movement_commands             │   │       │                    │
                     │  ├─ rc_controller                 │   │  vision (detector_node)    │
                     │  ├─ pid_controller ×2             │   │       │                    │
                     │  └─ inspector_node                │   │  /vision/detections        │
                     │          │                       │   │       │                    │
                     │  /mavlink/vehicle_state           │   │  alignment_controller      │
                     │  /mavlink/events                  │   │  (PID visual servo)        │
                     │  /mavlink/diagnostics             │   │       │                    │
                     │  /driver/feedback                 │   └───────┼────────────────────┘
                     │          │                       │           │
                     │  /driver/command ◄───────────────┼───────────┘
                     │  /driver/teleop                  │
                     │          │                       │
                     │  ┌───────┴──────────┐            │
                     │  │ mavlink_runner   │            │     ┌──────────────────┐
                     │  │ mission_executor │            │     │ duburi_planner   │
                     │  │ teleop_driver    │            │     │ (YASMIN HFSM)    │
                     │  └──────────────────┘            │     │ → /driver/command│
                     │                                  │     └──────────────────┘
                     │  mavlink_logger → logs/           │
                     │  duburi_blueos → /blueos/*        │
                     └──────────────────────────────────┘
```

---

## Summary Statistics

| Metric | Value |
|--------|-------|
| ROS 2 Packages | 10 |
| Source Files | ~80 |
| Commands Supported | 30+ |
| YASMIN States | 8 reusable |
| YASMIN Missions | 2 (gate, demo_square) |
| Analysis Documents | 26 |
| Detection FPS | 20-25 (Orin Nano) |
