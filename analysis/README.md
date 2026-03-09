# Duburi 4.2 Codebase Analysis

This folder contains comprehensive documentation for the BRACU Duburi AUV 4.2 ROS 2 control stack. It is intended for **AI agents** and developers who need to understand, modify, or extend the codebase.

## Reading Order for New Agents

1. **00_OVERVIEW.md** – Start here. High-level purpose, philosophy, and constraints.
2. **08_AGENT_GUIDE.md** – Quick reference: where things live, common tasks, pitfalls.
3. **02_DESIGN_DECISIONS.md** – Why the code is structured this way.
4. **07_ARDUSUB_CONSTRAINTS.md** – Hardware/firmware requirements that drive many design choices.
5. **01_ARCHITECTURE.md** – Package layout, topics, data flow.
6. **06_INTERFACES.md** – Message definitions and usage.
7. **03–05** – Line-by-line explanations of each package (use when modifying specific files).

## File Index

| File | Purpose |
|------|---------|
| `00_OVERVIEW.md` | Project context, philosophy, key constraints |
| `01_ARCHITECTURE.md` | System architecture, packages, topics, data flow |
| `02_DESIGN_DECISIONS.md` | Design choices and rationale |
| `03_INSPECTOR_LINE_BY_LINE.md` | mavlink_inspector – every choice explained |
| `04_RUNNER_LINE_BY_LINE.md` | mavlink_runner – CLI and mission execution |
| `05_DRIVER_LINE_BY_LINE.md` | mavlink_driver – driver_client, mission_executor, teleop |
| `06_INTERFACES.md` | duburi_interfaces – messages and fields |
| `07_ARDUSUB_CONSTRAINTS.md` | ArduSub requirements that shape the design |
| `08_AGENT_GUIDE.md` | Quick reference for agents |
| `09_KNOWN_ISSUES_AND_GOTCHAS.md` | Known issues, edge cases, and applied fixes |
| `10_DESIGN_ISSUES.md` | 7 architectural concerns with severity/effort ratings |
| `11_DESK_TESTING_GUIDE.md` | Step-by-step desk testing procedures |
| `12_COMMAND_REFERENCE.md` | Complete command reference with field encoding details |

## Key Invariants (Do Not Violate)

- **Single MAVLink connection**: Only `mavlink_inspector` connects to Pixhawk. All control flows through `/driver/command`.
- **RC override must be continuous**: ArduSub failsafes (~3s) if RC_CHANNELS_OVERRIDE stops. The inspector sends at 20 Hz.
- **Idle = neutral RC**: When no movement is active, the inspector still sends neutral (1500) on all channels to prevent disarm.
- **Arm/disarm in thread**: Blocking `motors_armed_wait()` runs in a daemon thread to avoid blocking the ROS executor.
