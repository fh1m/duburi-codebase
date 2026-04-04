# Architecture Documentation

System design and high-level architecture for Duburi AUV.

| Document | Description |
|----------|-------------|
| [overview.md](overview.md) | High-level system overview |
| [system-architecture.md](system-architecture.md) | Detailed architecture |
| [control-flow-v2.md](control-flow-v2.md) | **V2 Control flow & execution pipeline** |
| [ros-interfaces.md](ros-interfaces.md) | Messages, services, topics |
| [design-issues.md](design-issues.md) | Known issues |

---

## V2 Control Architecture

See [control-flow-v2.md](control-flow-v2.md) for comprehensive documentation of:

- Mission execution flow (Runner → Inspector → ArduSub)
- Control loop architecture (5 layers: sensor fusion → estimation → control → limiting → output)
- Telemetry processing pipeline (AHRS2, SCALED_PRESSURE, SCALED_IMU2)
- Command dispatch flow (user input → motor commands)
- Safety & watchdog systems (4 layers of protection)
- Convergence & settling logic

**Status:** ✅ Production ready — All 30 bug fixes complete, build passing
