# Completed Development

> What Duburi has accomplished through April 2026.

---

## Overview

The Duburi 4.2 stack is a **10-package, 80-file** ROS 2 Humble codebase with production-ready control and perception systems.

---

## Control Stack Redesign V1 (March-April 2026)

### What Was Built

The control stack underwent a major redesign to support autonomous mission execution:

| Component | Status | Description |
|-----------|--------|-------------|
| **MAVLink Bridge** | ✅ Production | 7-module inspector: serial I/O, command dispatch, PID controllers, telemetry, RC override at 20 Hz |
| **Control System** | ✅ Functional | Software depth PID + yaw PID, trapezoidal PWM ramp, 4-layer RC override, `just_*` instant variants |
| **Command System** | ✅ Rich | 30+ commands: `move`, `go`, `cruise`, `at`, `just_*`, diagonals, PID depth/yaw, `~` prefix convention |
| **Feedback System** | ✅ Implemented | `/driver/feedback` with `accepted`/`reached`/`completed`/`rejected` status + error magnitude |

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
| **YOLO11 Detection** | ✅ Functional | YOLO11n on CUDA (20-25 FPS on Orin Nano) |
| **Kalman Tracking** | ✅ Implemented | Single-object tracking with prediction |
| **Visual Servoing** | ✅ Functional | PID-based lateral + vertical + forward alignment |
| **Camera Manager** | ✅ Complete | Multi-camera support with V4L2 backend |

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
| **Interactive CLI** | ✅ Complete | `duburi_runner` REPL with history, file-based missions, chained commands, status dashboard |
| **Mission Planner** | 🟡 Partial | YASMIN HFSM in `duburi_planner` — 8 reusable states, 2 missions (gate, demo_square) |
| **Teleop** | ✅ Implemented | `TeleopCommand` on `/driver/teleop` with multi-axis support + idle detection |

### Key Decisions Made

1. **YASMIN HFSM** — Hierarchical finite state machine for readable, composable missions
2. **File-based missions** — `.txt` mission files for quick iteration without code changes
3. **Chained commands** — Single-line command sequences for complex maneuvers

---

## Infrastructure (Ongoing)

### What Was Built

| Component | Status | Description |
|-----------|--------|-------------|
| **BlueOS Integration** | ✅ Functional | REST API client for system monitoring, parameter management |
| **Logging** | ✅ Functional | Session-based CSV/JSON logging with rotation |
| **Documentation** | ✅ Extensive | 26 analysis documents + comprehensive README |

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
