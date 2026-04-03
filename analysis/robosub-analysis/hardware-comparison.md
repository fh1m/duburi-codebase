# Hardware Comparison

Comparison of hardware approaches across RoboSub teams.

---

## Compute Platforms

| Team | Primary Compute | GPU | Memory | Notes |
|------|-----------------|-----|--------|-------|
| Bumblebee | Jetson Orin AGX | 2048 CUDA | 32 GB | + i7 SBC for non-GPU tasks |
| Desert WAVE | Jetson Xavier NX | 384 CUDA | 8 GB | Sufficient for HSV + waypoints |
| **Duburi** | Jetson Orin Nano | 1024 CUDA | 8 GB | Good GPU, limited RAM |
| Harbin | Custom PC | RTX GPU | 16+ GB | Higher power budget |

### Implications for Duburi
- **RAM constraint** — 8 GB limits concurrent model loading (YOLO + DepthAnything)
- **GPU competitive** — 1024 CUDA cores handles YOLO11n at 20-25 FPS
- **Power efficient** — 15W TDP allows longer runtime

---

## Flight Controllers

| Team | Controller | Firmware | Communication |
|------|------------|----------|---------------|
| Bumblebee | Custom | Custom stack | CAN bus |
| Desert WAVE | Pixhawk-based | ArduSub | MAVLink |
| **Duburi** | Pixhawk 2.4.8 | ArduSub | MAVLink (serial) |
| Advanced teams | Custom + Pixhawk | Hybrid | Mixed |

### Implications for Duburi
- **ArduSub proven** — Same as Desert WAVE, sufficient for competition
- **MAVLink limitation** — 20 Hz RC override cap; acceptable for our use case
- **EKF in firmware** — Pixhawk EKF handles IMU + barometer fusion

---

## Thrusters

| Team | Thruster Type | Count | Configuration | Allocation |
|------|---------------|-------|---------------|------------|
| Bumblebee | Blue Robotics T200 | 8 | Vectored | QP solver |
| Desert WAVE | Blue Robotics T200 | 6 | Standard | Direct |
| **Duburi** | Blue Robotics T200 | 6 | Vectored (assumed) | Direct channel |
| Harbin | Custom | 8 | Full holonomic | Advanced |

### Common Configurations

**6-thruster (Duburi, Desert WAVE):**
```
Top view:
    [1]       [2]
       \     /
        \   /
         [_]
        /   \
       /     \
    [3]       [4]

Side view:
    [5]───────[6]  (vertical)
```

**8-thruster (Bumblebee):**
- Additional vertical thrusters for roll/pitch control
- Enables QP-based thrust allocation for optimal distribution

### Implications for Duburi
- **6 thrusters sufficient** — Desert WAVE competed successfully with same count
- **Direct channel mapping** — Works, but no saturation optimization
- **Future: QP solver** — Medium priority improvement for smoother control

---

## Cameras

| Team | Camera Type | Count | Interface | Resolution |
|------|-------------|-------|-----------|------------|
| Bumblebee | FLIR BlackFly S | 3+ | GigE PoE | 1920×1200 |
| Desert WAVE | Leopard Imaging | 2 | MIPI | 1920×1080 |
| **Duburi** | USB webcam | 1-2 | USB (V4L2) | 1920×1080 |
| Harbin | Industrial | 4+ | GigE | High |

### Camera Positions

**Bumblebee (3+ cameras):**
- Forward (primary)
- Downward (bins)
- Upward (octagon)

**Duburi (1-2 cameras):**
- Forward ✅ Integrated
- Downward ❌ Needs integration
- Upward ❌ Not planned

### Implications for Duburi
- **Downward camera critical** — Required for bins task
- **USB adequate** — FLIR is overkill for our resolution needs
- **Bandwidth consideration** — Multiple USB cameras may compete for bandwidth

---

## Localization Sensors

| Team | DVL | IMU | Depth | Acoustic | Other |
|------|-----|-----|-------|----------|-------|
| Bumblebee | Teledyne Pathfinder | FOG + MEMS | BAR30 | Custom DAQ | Vision recal |
| Desert WAVE | Nortek DVL 1000 | FOG + MEMS | BAR30 | Subsonus | GPS survey |
| **Duburi** | Nortek Nucleus 1000 | VN-200 | BAR30 | ❌ None | — |
| Harbin | Premium | Premium | Custom | Custom | Multi-sensor |

### DVL Comparison

| DVL | Range | Accuracy | Interface | Used By |
|-----|-------|----------|-----------|---------|
| Nortek Nucleus 1000 | 0.3-50m | ±0.2% | Serial/Ethernet | Duburi |
| Nortek DVL 1000 | 0.3-50m | ±0.1% | Serial/Ethernet | Desert WAVE |
| Teledyne Pathfinder | 0.5-200m | ±0.1% | Serial | Bumblebee |

### Implications for Duburi
- **DVL available** — Nortek Nucleus 1000 ready, needs ROS 2 driver
- **IMU adequate** — VN-200 is quality MEMS IMU
- **Acoustic gap** — No pinger hardware = skip pinger-dependent tasks

---

## Actuators

| Team | Torpedoes | Dropper | Grabber | Other |
|------|-----------|---------|---------|-------|
| Bumblebee | Pneumatic | Pneumatic | Manipulator | Full suite |
| Desert WAVE | Pneumatic | Solenoid | None | Limited |
| **Duburi** | ❌ Not integrated | ❌ Not integrated | ✅ Servo | Grabber only |
| Harbin | Custom | Custom | Custom | Advanced |

### Actuator Requirements by Task

| Task | Actuator Needed | Duburi Status |
|------|-----------------|---------------|
| Gate | None | ✅ N/A |
| Slalom | None | ✅ N/A |
| Bins | Dropper | ❌ Needs integration |
| Torpedoes | Launcher | ❌ Needs integration |
| Octagon | Grabber | ✅ Implemented |
| Return Home | None | ✅ N/A |

### Implications for Duburi
- **Grabber works** — Octagon task actuator ready
- **Priority: Dropper** — Bins is higher-value than torpedoes
- **Torpedo complexity** — Mechanical design done, software path exists

---

## Power Systems

| Team | Battery | Voltage | Runtime | Notes |
|------|---------|---------|---------|-------|
| Bumblebee | LiPo | 24V | 2+ hours | Redundant cells |
| Desert WAVE | LiPo | 14.8V | 1+ hour | Standard |
| **Duburi** | LiPo | 14.8V (assumed) | 1+ hour | ESC protected |
| Harbin | Custom | High | Extended | Advanced BMS |

---

## Weight/Size Comparison

| Team | Approx Weight | Dimensions | Notes |
|------|---------------|------------|-------|
| Bumblebee | 35+ kg | Large | Two vehicles |
| Desert WAVE | 25 kg | Medium | Compact |
| **Duburi** | ~25 kg (est) | Medium | Single vehicle |
| Harbin | Heavy | Large | Full capability |

---

## Summary: Duburi Hardware Gaps

### Critical Gaps
1. **DVL integration** — Hardware available (Nucleus 1000), needs ROS 2 driver
2. **Downward camera** — Required for bins task
3. **Dropper actuator** — Required for bins task

### Medium Priority Gaps
1. **Torpedo launcher** — Mechanical ready, needs software
2. **Acoustic pinger** — Enables torpedoes/octagon navigation

### Lower Priority
1. **Upward camera** — Octagon task optimization
2. **Additional IMU** — Redundancy

### Strengths
- Jetson Orin Nano — Good GPU/watt ratio for YOLO
- T200 thrusters — Proven, same as top teams
- ArduSub firmware — Reliable, same as Desert WAVE
- Grabber — Octagon actuator ready
