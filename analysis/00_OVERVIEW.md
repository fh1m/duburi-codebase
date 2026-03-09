# BRACU Duburi 4.2 – Codebase Overview

## Purpose

This ROS 2 workspace controls the **BRACU Duburi AUV 4.2** (RoboSub vehicle). It connects to a Pixhawk 2.4.8 running ArduSub via MAVLink over serial (`/dev/ttyACM0`), and provides:

- A single point of MAVLink connection (no multiple nodes opening the same port)
- High-level movement commands (forward, left, depth, yaw, cruise, body-frame vector, etc.)
- Interactive CLI for pool testing
- File-based missions
- Teleop via Twist
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
6. **Mission flexibility**: Missions can be run from the CLI (`run gate`) or from custom Python nodes using `driver_client`.

## Hardware Context

- **Pixhawk 2.4.8** – flight controller
- **ArduSub** – firmware
- **Channels**: 1=Pitch, 2=Roll, 3=Throttle (depth), 4=Yaw, 5=Forward, 6=Lateral
- **PWM**: 1100–1900, neutral 1500
- **Connection**: Serial 115200 baud

## Reference Codebases

This design draws from:

- `/home/duburi/old_stuff/auv/src/mishu/mishu` – RoboSub-tested control (control.py, basic.py, control_utility.py)
- ArduSub pymavlink docs (ardusub.com, darksleep.com)
- ArduPilot Sub documentation

## File Layout

```
duburi_ws/
├── src/
│   ├── duburi_interfaces/     # Messages (DriverCommand, MavlinkEvent, VehicleState,
│   │                          #   DriverCommandFeedback, VehicleDiagnostics, ...)
│   ├── mavlink_inspector/     # Pixhawk connection, command execution, PID, ramp
│   ├── mavlink_driver/        # driver_client, mission_executor, teleop_driver
│   ├── mavlink_runner/        # Duburi > CLI
│   ├── mavlink_logger/        # Logs to auv_logs/
│   ├── vision/                # Camera perception (gate detection, etc.)
│   └── vision_manager/        # Vision pipeline manager
├── missions/                  # Mission .txt files (run gate, etc.)
└── analysis/                  # This documentation
```
