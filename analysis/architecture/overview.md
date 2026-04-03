# BRACU Duburi 4.2 – Codebase Overview

## Purpose

This ROS 2 workspace controls the **BRACU Duburi AUV 4.2** (RoboSub vehicle). It connects to a Pixhawk 2.4.8 running ArduSub via MAVLink over serial (`/dev/ttyACM0`), and provides:

- A single point of MAVLink connection (no multiple nodes opening the same port)
- High-level movement commands (forward, left, depth, yaw, cruise, body-frame vector, etc.)
- Interactive CLI for pool testing
- File-based missions and YASMIN FSM autonomous missions
- Teleop via Twist and visual servo alignment
- YOLO-based object detection with Kalman filtering
- Logging for debugging
- Command feedback for mission coordination (DriverCommandFeedback)
- PWM velocity ramp for smooth thruster control
- Battery voltage compensation for consistent thrust

## Design Philosophy

1. **Single connection owner**: Only `mavlink_inspector` opens the serial port. All control flows through ROS topics.
2. **ArduSub constraints first**: ArduSub requires constant RC override and heartbeat. The design is driven by these requirements.
3. **Command abstraction**: High-level commands (e.g. `move left 50% 10s`) are translated to RC_CHANNELS_OVERRIDE by the inspector.
4. **Separation of concerns (DESIGN 5)**: Each command only sets the channels it owns. PID layers independently control depth (throttle) and heading (yaw).
5. **Non-blocking UX**: Arm/disarm and event handling avoid blocking the CLI.
6. **Mission flexibility**: Missions can be run from the CLI (`run gate`) or from custom Python nodes using `driver_client` or from the YASMIN FSM planner.
7. **Vision-control separation**: Detection runs independently; alignment controller closes the loop at camera frame rate.

## Hardware Context

- **Pixhawk 2.4.8** – flight controller
- **ArduSub** – firmware
- **Channels**: 1=Pitch, 2=Roll, 3=Throttle (depth), 4=Yaw, 5=Forward, 6=Lateral
- **PWM**: 1100–1900, neutral 1500
- **Connection**: Serial 115200 baud
- **Cameras**: USB cameras via V4L2, typically 640x480 @ 30fps

## Reference Codebases

This design draws from:

- `/home/duburi/old_stuff/auv/src/mishu/mishu` – RoboSub-tested control (control.py, basic.py, control_utility.py)
- ArduSub pymavlink docs (ardusub.com, darksleep.com)
- ArduPilot Sub documentation
- YASMIN (Yet Another State MachINe) for ROS 2

## File Layout

```
duburi_ws/
├── src/
│   ├── duburi_interfaces/     # 11 custom messages (DriverCommand, TeleopCommand,
│   │                          #   VehicleState, VehicleDiagnostics, MavlinkEvent,
│   │                          #   DriverCommandFeedback, AlignmentStatus, Detection,
│   │                          #   DetectionArray, CameraStatus, CameraStatusArray)
│   ├── duburi_common/         # Shared constants and command vocabulary
│   ├── mavlink_inspector/     # Pixhawk connection, command execution, PID, ramp
│   │   ├── config/            #   defaults.yaml, competition.yaml, pool_test.yaml
│   │   └── launch/            #   duburi_control.launch.py
│   ├── mavlink_driver/        # driver_client, mission_executor, teleop_driver, just_commands
│   ├── mavlink_runner/        # Duburi > CLI
│   ├── mavlink_logger/        # Session logs under logs/
│   ├── vision/                # YOLO11 detection, Kalman tracking, PID visual servo
│   ├── vision_inspector/      # Multi-camera management, calibration, recording, playback
│   ├── duburi_planner/        # YASMIN FSM mission planner with hierarchical states
│   │   ├── config/            #   planner.yaml
│   │   └── launch/            #   planner.launch.py
│   └── duburi_blueos/         # BlueOS REST API client for RPi4B companion integration
├── missions/                  # Mission .txt files (run gate, etc.)
└── analysis/                  # This documentation (26 documents) + ROADMAP
```

## Codebase Statistics

| Metric | Value |
|--------|-------|
| Python source files | 80 |
| Total lines of code | ~10,500 |
| ROS 2 packages | 10 |
| Custom message types | 11 |
| ROS nodes | 15+ |
| Config files (YAML) | 6 (defaults, competition, pool_test, planner, cameras, blueos_config) |
| Launch files | 6 (duburi_control, planner, vision, perception, camera, duburi_launch) |
