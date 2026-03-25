# Architecture

## Data Flow

```
                    ┌─────────────────────────────────────────┐
                    │           mavlink_inspector             │
                    │  (owns /dev/ttyACM0, single connection) │
                    └─────────────────────────────────────────┘
                                      │
         ┌────────────────────────────┼────────────────────────────┐
         │                            │                            │
         ▼                            ▼                            ▼
  /driver/command              /mavlink/events            /mavlink/vehicle_state
  (DriverCommand)              (MavlinkEvent)              (VehicleState)
  /driver/teleop
  (TeleopCommand)
         │                            │                            │
    ┌────┴────┬───────────────┬──────┴──────┬───────────────┐     │
    ▼         ▼               ▼             ▼               ▼     ▼
  runner   mission_executor  teleop_driver  logger         ...   logger
  (CLI)    (Python mission)   (/cmd_vel →   (events)              (state)
                                /driver/teleop)
```

## Package Roles

| Package | Role | Key Files |
|---------|------|-----------|
| `duburi_interfaces` | Shared message definitions | `msg/DriverCommand.msg`, `TeleopCommand.msg`, `MavlinkEvent.msg`, `VehicleState.msg` |
| `duburi_common` | Shared constants and command vocabulary | `command_vocabulary.py`, `constants.py` |
| `mavlink_inspector` | Owns Pixhawk connection, executes commands, publishes state/events | `inspector_node.py` |
| `mavlink_driver` | Helpers and alternative command sources | `driver_client.py`, `mission_executor.py`, `teleop_driver.py` |
| `mavlink_runner` | Interactive CLI (`Duburi >`) | `runner.py` |
| `mavlink_logger` | Logs events, state, commands to `logs/` | `logger_node.py` |

## Topic Summary

| Topic | Type | Publisher | Subscribers |
|-------|------|-----------|-------------|
| `/driver/command` | DriverCommand | runner, mission_executor | inspector, logger |
| `/driver/teleop` | TeleopCommand | teleop_driver | inspector |
| `/mavlink/events` | MavlinkEvent | inspector | runner, logger |
| `/mavlink/vehicle_state` | VehicleState | inspector | logger, status tools |
| `/cmd_vel` | Twist | teleop/joystick nodes | teleop_driver |

## Execution Order

1. Start `mavlink_inspector` first (it opens the serial port).
2. Start `mavlink_runner` or `mission_executor` or `teleop_driver` as needed.
3. Optional: start `mavlink_logger` for logging.

## Why Single Connection?

- Serial ports cannot be shared by multiple processes.
- pymavlink `recv_match` and `rc_channels_override_send` use the same connection.
- Centralizing in the inspector avoids port conflicts and keeps MAVLink logic in one place.
