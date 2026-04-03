# Simulation: Gazebo Harmonic + ArduSub SITL + Duburi

This document describes the **working software-in-the-loop (SITL) stack** used to test Duburi **before pool runs**, the **rationale** for transport and port choices, a **tuning roadmap** for stability, a **phased plan** for improving fidelity, and **viability** of other simulators across **lab GPU tiers** (including RTX 2060 and RTX 3080).

**Companion (commands & install checklist):** root [README.md](../README.md) section *Simulation (Gazebo + ArduSub SITL)*.

---

## 1. Goals

| Goal | How this stack supports it |
|------|----------------------------|
| **Autonomy regression** | Same ROS 2 graph: `mavlink_inspector`, `mavlink_runner`, missions, optional `vision` |
| **Repeatable bring-up** | Fixed order: Gazebo → SITL → bridge → inspector |
| **Pool correlation** | ArduSub firmware + RC override path identical to Pixhawk; tune params separately for sim vs pool |
| **Robustness testing** | After baseline stability: ArduPilot `SIM_*` noise, sensor degradation, mission stress |

---

## 2. Architecture (data flow)

```
┌─────────────────────────────────────────────────────────────────────────┐
│  Gazebo Sim 8 (Harmonic)                                                 │
│  bluerov2_underwater.world + ArduPilot JSON plugin                       │
│       │ dynamics / sensors (gz topics)                                   │
└───────┼─────────────────────────────────────────────────────────────────┘
        │  JSON / physics coupling (ArduPilot SITL internal)
        ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  ArduPilot ArduSub SITL + MAVProxy                                       │
│  sim_vehicle.py  →  primary link + duplicate stream                      │
│       │                                                                  │
│       ├── TCP :5760 (typical primary — MAVProxy, optional QGC)           │
│       └── --out=udp:127.0.0.1:5760  →  Duburi mavlink_inspector          │
└───────┼─────────────────────────────────────────────────────────────────┘
        │  MAVLink (UDP)
        ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  mavlink_inspector  (pymavlink udpin:127.0.0.1:5760)                       │
│       → /mavlink/vehicle_state, /mavlink/events, …                        │
└───────┼─────────────────────────────────────────────────────────────────┘
        │  ROS 2
        ▼
  mavlink_runner, mission_executor, vision, logger, …

Parallel (not MAVLink):
  ros_gz_bridge  :  /camera … , /clock …  (topic names depend on world SDF)
```

**Single MAVLink client on the UDP duplicate:** `mavlink_inspector` binds **`udpin:127.0.0.1:5760`**. MAVProxy sends **to** that address:port. No second process should bind the same UDP port.

---

## 3. Version matrix (reference)

These versions are **known-good** for the documented workflow; others may work with adjustments.

| Component | Reference |
|-----------|-----------|
| OS | Ubuntu **22.04** (Jammy) |
| ROS 2 | **Humble** |
| Gazebo | **Gazebo Sim 8.x** (Harmonic), CLI `gz sim` |
| ROS ↔ Gz | **`ros-humble-ros-gzharmonic-bridge`** |
| pymavlink | e.g. **2.4.x** (user or system install) |
| ArduPilot | Recent **master** or team-pinned commit with **ArduSub** + **JSON** Gazebo support |

**Avoid mixing sim generations:** if both `gz` (8.x) and `ign` (6.x) are installed, use **`gz sim`** for Harmonic worlds. Legacy **`ign gazebo`** targets an older line and can confuse debugging.

---

## 4. World and model assets (not in Duburi_ws)

The Duburi repository does **not** ship `bluerov2_underwater.world`. That asset typically lives in:

- **[ArduPilot/ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo)** or a team fork, **or**
- Stacks such as [markusbuchholz/gazebosim_bluerov2_ardupilot_sitl](https://github.com/markusbuchholz/gazebosim_bluerov2_ardupilot_sitl) (reference for `vectored_6dof`, Docker, and launch patterns).

You must set **resource paths** so `gz sim` resolves models and plugins (see upstream `README` for `GZ_SIM_RESOURCE_PATH`, `GZ_SIM_SYSTEM_PLUGIN_PATH`, and plugin build instructions).

---

## 5. Transport: why `udpin` and not `tcp`

MAVProxy’s **`--out=udp:127.0.0.1:PORT`** sends **UDP datagrams** to localhost. The receiving end must **listen on UDP** (`udpin:` in pymavlink).

**TCP** `tcp:127.0.0.1:5760` connects to whatever **listens on TCP 5760** (often the **primary SITL MAVLink server**). That is a **different protocol and socket** from UDP 5760. Linux allows **TCP and UDP on the same port number** simultaneously—they do not share traffic.

**Failure mode:** TCP connect succeeds; **one** heartbeat may appear during `wait_heartbeat`; then **no sustained stream** on that socket → `mavlink_inspector` reports **heartbeat loss** and reconnects. The fix is **not** heartbeat code—it is **matching UDP out to UDP in**.

**Implementation note:** `connection_manager.py` treats `udpin:`, `udp:`, `tcp:`, etc. as network URLs; `inspector_node` advances link health on **any** received MAVLink message when connected (not only `HEARTBEAT`), but **if zero packets arrive**, timeouts still trigger.

---

## 6. Bring-up order and rationale

| Step | Component | Why this order |
|------|-----------|----------------|
| 1 | **`gz sim -r <world>`** | Gazebo loads the **ArduPilot plugin** and environment; JSON SITL expects the sim side to exist. **`-r`** runs unpaused. |
| 2 | **`sim_vehicle.py` … `--model JSON` `--out=udp:…`** | Starts ArduSub SITL and **forwards** a MAVLink copy to Duburi’s UDP port. |
| 3 | **`ros_gz_bridge parameter_bridge …`** | Exposes gz sensors/clock to ROS 2 for perception / time sync (topics **must match your SDF**). |
| 4 | **`mavlink_inspector` with `udpin:`** | Opens the **UDP listener**; receives the `--out` stream. |
| 5 | **`mavlink_runner`** (optional) | Same CLI and missions as hardware. |

If Gazebo is **paused** or the plugin is **misconfigured** (e.g. joint names), SITL may hang waiting for sensor/JSON data—always confirm the **world is running** and **plugin logs** are clean.

---

## 7. SITL frame vs Gazebo model

The team has used **`-f vectored`** with **`--model JSON`**. Some reference stacks use **`vectored_6dof`** for closer coupling to a 6-DOF Gazebo BlueROV. If **heading/depth feel wrong** after physics tuning, **compare** frame name, thruster mapping, and plugin configuration against:

- ArduPilot `SIM_JSON` / frame docs for your branch  
- [gazebosim_bluerov2_ardupilot_sitl](https://github.com/markusbuchholz/gazebosim_bluerov2_ardupilot_sitl) (`vectored_6dof`, launch order)

---

## 8. Tuning roadmap (stability → realism)

### 8.1 Gazebo / physics

- **`max_step_size` / fixed step:** Smaller steps (e.g. **1–2 ms** if CPU allows) reduce numerical jitter; trade off against real-time factor.
- **`real_time_factor`:** Keep **1.0** while tuning control.
- **Physics engine:** Stick to **one** engine (e.g. DART vs Bullet) per project; re-tune if you switch.
- **Buoyancy:** Neutral trim (mass vs displaced volume) — poor trim causes **depth hunting**.
- **Thruster limits:** Match **T200**-class limits so sim is not **over-torqued** vs pool.
- **Hydrodynamics / damping:** If available in your model, increase damping to kill unrealistic oscillation.

### 8.2 ArduSub parameters

- **Mag / yaw:** Sim compass is often noisy; consider reducing compass influence or modes suited to **gyro-heavy** yaw in sim.
- **EKF / INS filters:** Balance smoothing vs pool fidelity.
- **Depth source:** Align what the **JSON/SITL** provides with what ArduSub **expects** (baro vs external depth).
- **Control PIDs:** Tune for sim; export a **separate param file** for pool.

### 8.3 Duburi / ROS

- **`yaw_source`** (`attitude` vs `ahrs2`): Try both if yaw is jumpy.
- **Control rates:** Inspector RC override **20 Hz**; ensure telemetry rates are sufficient for closed-loop software PIDs.
- **`use_sim_time`:** Use on nodes that must follow **`/clock`** from gz when recording or planning in sim time.

### 8.4 Robustness phase

- ArduPilot **`SIM_*`** noise parameters  
- Camera noise / lowered resolution in gz or in `vision` pipeline  
- Longer missions and **ros2 bag** replay for regression

---

## 9. Current plan (phased)

| Phase | Focus | Success criteria |
|-------|--------|------------------|
| **A — Baseline (done)** | Documented bring-up, **UDP** MAVLink path, Harmonic + Humble + `ros_gzharmonic` | Connect, telemetry, runner commands, no false “heartbeat lost” from transport mismatch |
| **B — Fidelity** | World physics step, buoyancy, thruster curves, ArduSub param sets for sim | Depth/heading transients **qualitatively** similar to a short pool log |
| **C — Regression** | Scripted smoke test + optional CI headless gz | Repeatable pass/fail before merges |
| **D — Perception stress** | Bridge cameras; degrade images; optional second sim for datasets | Vision robustness without blocking controls work |
| **E — Optional integrations** | DAVE assets (sensors/currents) **or** Stonefish for dynamics reference | Extra environments or validation—not required for core Duburi loop |

---

## 10. GPU tiers and optional simulators

Duburi’s **primary** sim path is **CPU-friendly** (Gazebo + SITL + pymavlink). **GPU** matters for **vision training**, **Unreal/Isaac-class** sims, and **heavy** rendering.

### 10.1 Reference hardware

| Machine role | GPU (examples) | VRAM | Implication |
|--------------|----------------|------|-------------|
| Developer laptop | GTX **1660 SUPER** | **6 GB** | Comfortable for **Harmonic** + moderate CUDA vision; **tight** for UE5 editor / Isaac Sim default configs |
| Robotics lab | **RTX 2060** | **6 GB** (or 12 GB on some SKUs) | Same band as 1660S for **heavy** sims; **12 GB** variant much better for Isaac / large scenes |
| Robotics lab | **RTX 3080** | **10 GB** / **12 GB** | **Viable** for **HoloOcean-class** Unreal sims, **Isaac Sim / OceanSim-style** workloads at **reasonable** quality; preferred station for **perception SDG** |

### 10.2 Option viability matrix

| Simulator | Primary win | Integration with ArduSub + Duburi | RTX 2060 (6 GB) | RTX 3080 |
|-----------|-------------|-----------------------------------|-----------------|----------|
| **Gazebo Harmonic + SITL (current)** | Full stack, same MAVLink/ROS | **Native** | **Excellent** | **Excellent** |
| **Project DAVE** | Marine sensors, currents, bathymetry, worlds | **Gazebo-side** assets; ROS 2 + Harmonic porting effort on **Humble** may vary vs **Jazzy** docs | **Good** | **Good** |
| **Stonefish + stonefish_ros2** | Hydrodynamics from geometry, marine rendering | **Parallel** sim; custom bridge to MAVLink if full stack needed | **Good** | **Good** |
| **HoloOcean + holoocean-ros** | Sonar, UE visuals, ROS 2 examples | **Parallel**; perception / mission testing | **Moderate** (lighter scenes / settings) | **Strong** |
| **UNav-Sim (UE5)** | Visual realism, synthetic data | **Heavy** integration; MAVLink/ROS paths exist but not drop-in ArduSub JSON | **Weak** | **Moderate** |
| **OceanSim (Isaac Sim)** | GPU perception, digital twins | **Extension** workflow; not ArduSub-in-loop by default | **Weak** | **Moderate–strong** |
| **MuJoCo / OpenMAUVe** | Dynamics studies / equation models | **Offline** or side validation | **N/A** (mostly CPU) | **N/A** |

**Recommendation:** Keep **Gazebo + ArduSub** as the **authority** for **full-stack** Duburi testing. Use **lab 3080** (then **2060 12 GB** if available, then **2060 6 GB**) for **HoloOcean / Isaac-class** experiments **alongside**, not as a replacement until the team invests integration time.

---

## 11. Related Duburi code and config

| Item | Location |
|------|----------|
| Network URL handling (`tcp`, `udpin`, …) | `mavlink_inspector/connection_manager.py` — `_is_network_mavlink_url`, `connect`, `read_messages` |
| Link health / heartbeat timing | `mavlink_inspector/inspector_node.py` — `_read_mavlink` updates `last_heartbeat` on any message; `send_heartbeat` in `connection_manager.py` |
| `connection_port` description | `mavlink_inspector/inspector_node.py` — `ParameterDescriptor` for SITL UDP pairing |
| YAML hint | `mavlink_inspector/config/defaults.yaml` — comment on `udpin` vs `tcp` |

---

## 12. References (external)

- [ArduPilot SITL overview](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)  
- [ArduPilot Gazebo](https://github.com/ArduPilot/ardupilot_gazebo)  
- [Gazebo Harmonic + underwater vehicle systems (Fossen-style)](https://gazebosim.org/api/gazebo/6/underwater_vehicles.html) (concepts; API generation may differ from Sim 8—map to current gz docs)  
- [Project DAVE](https://field-robotics-lab.github.io/dave.doc/)  
- [Stonefish](https://stonefish.readthedocs.io/) · [stonefish_ros2](https://stonefish-ros2.readthedocs.io/)  
- [HoloOcean](https://byu-holoocean.github.io/holoocean-docs/) · [holoocean-ros](https://github.com/byu-holoocean/holoocean-ros)  

---

## Document history

- **2025-03-27:** Initial version — working stack, UDP/TCP rationale, tuning roadmap, GPU tier planning (1660 SUPER dev, RTX 2060 / RTX 3080 lab).
