# 18 — BlueOS + Jetson MAVLink Bring-up Guide (First Principles to Production)

This document is the canonical bring-up and troubleshooting guide for the
Duburi networked MAVLink setup where:

- Pixhawk is connected to Raspberry Pi (BlueOS companion)
- Jetson runs the Duburi ROS 2 stack (`mavlink_inspector`, `runner`, `planner`)
- Pi and Jetson are connected over Ethernet (often through a switch)

It captures both theory and practical lessons from integration fixes in this
repository, including heartbeat-loss loops and armed/disarmed flapping.



## 0) Quick TL;DR (for pool-day operators)

If you only remember 5 lines, remember these:

1. On Pi (BlueOS), set MAVLink endpoint to **UDP Client** -> `JETSON_IP:14550`.
2. On Jetson, run inspector with `connection_port:=udpin:0.0.0.0:14550`.
3. Never use `127.0.0.1` for Pi->Jetson traffic.
4. If runner says "not armed", check `/mavlink/vehicle_state` first.
5. If `armed/disarmed` flips every ~1s, heartbeat source filtering is broken.

ASCII mnemonic:

```text
BlueOS (Pi)  --UDP-->  Jetson inspector (udpin)
```

---

## 1) First Principles (Layman + Engineering)

### 1.1 What MAVLink is doing in this setup

MAVLink is just structured messages over a transport (serial, UDP, TCP). In our
setup:

1. Pixhawk emits MAVLink frames over serial (USB/UART) into Pi
2. BlueOS routes those frames over network to Jetson
3. Jetson `mavlink_inspector` reads MAVLink and publishes ROS topics
4. Other ROS nodes (`runner`, `planner`, `vision`) consume those ROS topics

Control goes the other way:

1. `runner` / `planner` publish `/driver/command`
2. `mavlink_inspector` converts to MAVLink/RC overrides
3. Pixhawk executes

### 1.2 Why heartbeat matters

ArduSub expects communication to stay alive:

- Autopilot should keep receiving valid link traffic
- Control stream (RC override/manual control) should be continuous

If traffic stops, failsafes can trigger. In our code:

- `mavlink_inspector` considers link healthy when **any MAVLink telemetry** is
  arriving
- if telemetry stalls for `heartbeat_timeout`, it marks link unhealthy and
  reconnects

### 1.3 Why routing confusion causes most failures

Most failures are not “bad code,” but wrong socket topology:

- Loopback (`127.0.0.1`) only works for local same-machine traffic
- Pi -> Jetson traffic must target Jetson LAN IP, not loopback
- Ports must match between sender and receiver
- Client/server direction in BlueOS endpoint config must match what Jetson
  expects (`udpin` vs `udpout`)



### 1.1.1 Complete data/control loop (ASCII)

```text
Telemetry path:

  Pixhawk
    |  (serial MAVLink)
    v
  Pi (BlueOS router)
    |  (UDP/TCP MAVLink)
    v
  Jetson: mavlink_inspector
    |  (ROS2 state topics)
    +--> /mavlink/vehicle_state
    +--> /mavlink/events
    +--> /mavlink/diagnostics

Control path:

  runner / planner / vision
    |  (ROS2 DriverCommand)
    v
  /driver/command
    |
    v
  mavlink_inspector
    |  (MAVLink command + RC override)
    v
  Pixhawk -> ESC -> Thrusters
```

### 1.1.2 Why this architecture exists

- Keeps one owner of MAVLink link (`mavlink_inspector`) to avoid race/conflict.
- Makes mission/control code independent of transport (serial vs UDP).
- Gives deterministic place for safety checks (arming gate, PID safety, stop).

### 1.2.3 Two different "success" checks you must not confuse

When debugging, separate these:

1. **Transport success**
   - "connected" and continuous telemetry arriving.
2. **Command acceptance success**
   - COMMAND_ACK says ACCEPTED.
3. **State truth success**
   - `/mavlink/vehicle_state` reflects actual armed/mode/depth state.

All three must be good for stable operation.

```text
connected != armed
ACK accepted != state updated
```

### 1.3.1 The `127.0.0.1` trap explained simply

`127.0.0.1` means "this same computer".

So on Jetson:

```text
udpin:127.0.0.1:14550
```

means "listen only for packets generated on Jetson itself".
Pi packets will never arrive there, even if Pi is physically connected.

Use:

```text
udpin:0.0.0.0:14550
```

which means "listen on all network interfaces".

References:
- [ArduSub pymavlink docs](https://www.ardusub.com/developers/pymavlink.html)
- [BlueOS overview](https://blueos.cloud/docs/stable/usage/overview/)

---

## 2) Duburi Architecture Constraints (Do Not Break)

1. **Single MAVLink ownership**
   - `mavlink_inspector` is the only Duburi process that should directly connect
     to MAVLink stream.

2. **ROS-facing control API**
   - All movement/control should pass through `/driver/command`.

3. **Armed state source of truth**
   - Runner/planner rely on `/mavlink/vehicle_state` (`VehicleState.armed`).
   - If heartbeat parsing is wrong, all higher-level nodes misbehave.

4. **Network mode and serial mode must be swappable**
   - Same codebase, different `connection_port` parameter.

---

## 3) Endpoint Modes: BlueOS vs pymavlink Mapping

BlueOS shows endpoint types: UDP Server/Client, TCP Server/Client, Serial, etc.
Duburi uses pymavlink style strings (`udpin:...`, `udpout:...`, `tcp:...`).

### 3.1 Recommended default for Pi -> Jetson

Use:

- BlueOS: **UDP Client**
- Destination: `JETSON_IP:14550`
- Jetson inspector: `connection_port:=udpin:0.0.0.0:14550`

Why:

- Jetson passively listens on all interfaces
- Pi actively forwards MAVLink to Jetson static IP
- Easy to reason about and robust in switched LAN

### 3.2 Mapping table

| BlueOS Endpoint Type | Typical intent | Jetson `connection_port` |
|---|---|---|
| UDP Client | Pi pushes to Jetson | `udpin:0.0.0.0:<port>` |
| UDP Server | Pi listens, Jetson dials | `udpout:<PI_IP>:<port>` |
| TCP Client | Pi dials Jetson | `tcpin:0.0.0.0:<port>` (if Jetson server) |
| TCP Server | Pi listens TCP | `tcp:<PI_IP>:<port>` |
| Serial | Direct serial on machine | `/dev/ttyACM0` etc. |



### 3.2.1 Endpoint direction diagrams (ASCII)

Pattern A (recommended for Duburi):

```text
[Pi BlueOS UDP Client]  ---- send ---->  [Jetson udpin:0.0.0.0:14550]
```

Pattern B (valid but less common for this project):

```text
[Pi BlueOS UDP Server]  <---- send ----  [Jetson udpout:PI_IP:PORT]
```

Do not mix these patterns at the same time during bring-up.

### 3.2.2 "Server" vs "Client" from first principles

- **Server** = waits/listens for incoming packets on a bound address.
- **Client** = actively sends packets to a destination address.

BlueOS labels endpoints from the BlueOS side.
Pymavlink labels are from the script side (`udpin` listens, `udpout` sends).

### 3.4 What each BlueOS endpoint option means for Duburi

| BlueOS option | Should you use for Duburi MAVLink? | Notes |
|---|---|---|
| UDP Client | Yes (default recommendation) | Easiest Pi->Jetson path |
| UDP Server | Optional | Requires Jetson `udpout` |
| TCP Client/Server | Optional advanced | More overhead/complexity than needed on local switch |
| Serial | Not for Pi->Jetson bridge | Use only for local serial devices |
| Zenoh | Not used by this codebase transport path | Out of scope for current inspector/runner/planner |

### 3.3 Common anti-patterns

1. `udpin:127.0.0.1:5760` on Jetson for Pi traffic
   - Wrong: Pi cannot send into Jetson loopback.

2. Mixed port assumptions (`5760` vs `14550`) without checking router endpoint
   - Always verify actual sender destination and receiver bind port.

3. Multiple overlapping endpoints to same consumer
   - Can cause duplicate messages or confusing state transitions.

---

## 4) IP Planning and Network Bring-up (New System Checklist)

Use this for fresh bring-up on a new Pi + Jetson pair.

### 4.1 Physical topology

- Pixhawk -> Pi (USB/UART)
- Pi Ethernet -> switch
- Jetson Ethernet -> switch
- Optional laptop -> switch for diagnostics

### 4.2 Static IP recommendation

Pick a dedicated subnet for robot local LAN, for example:

- Pi (BlueOS): `192.168.2.2`
- Jetson: `192.168.2.10`
- Netmask: `255.255.255.0`

Keep these fixed so endpoint configs survive reboot.

### 4.3 Connectivity sanity tests

From Jetson:

```bash
ping -c 3 192.168.2.2
```

From Pi:

```bash
ping -c 3 192.168.2.10
```

If ping fails, do not continue to MAVLink debugging yet.


### 4.4 New-system preflight checklist (before touching ROS)

```text
[ ] Pi and Jetson link lights active on switch
[ ] Static IPs assigned and persisted
[ ] Pi can ping Jetson
[ ] Jetson can ping Pi
[ ] BlueOS endpoint enabled
[ ] No duplicate conflicting MAVLink endpoints
```

If any box is unchecked, fix it first.

---

## 5) BlueOS Configuration Steps

## 5.1 Configure endpoint on Pi

In BlueOS MAVLink endpoints:

1. Create endpoint name e.g. `Inspector Endpoint`
2. Type: `UDP Client`
3. Target host: `JETSON_IP` (e.g. `192.168.2.10`)
4. Target port: `14550`
5. Keep “start endpoint enabled” checked

## 5.2 Ensure only one active endpoint path to Jetson

- Disable duplicate endpoints that send same MAVLink stream to same or
  conflicting ports while testing.

## 5.3 Router backend note


### 5.4 BlueOS UI operator walkthrough (step-by-step)

1. Open BlueOS web UI on Pi.
2. Go to MAVLink Endpoints page.
3. Click create endpoint.
4. Name: `Inspector Endpoint`.
5. Type: `UDP Client`.
6. Host/IP: Jetson static IP (e.g. `192.168.2.10`).
7. Port: `14550`.
8. Keep "start enabled" checked.
9. Save.
10. Confirm endpoint appears active and counters/traffic are non-zero.

If counters stay zero, verify IP/port and that inspector is listening.


BlueOS can run different MAVLink routing backends depending on version/config.
Behavior can differ (message fanout, endpoint semantics), so always verify
actual traffic at Jetson socket instead of assuming defaults.

---

## 6) Jetson Configuration Steps

Run inspector with network endpoint:

```bash
ros2 run mavlink_inspector inspector --ros-args -p connection_port:=udpin:0.0.0.0:14550
```

Expected startup:

- `Connecting to SITL/network at udpin:0.0.0.0:14550...`
- `[connected] Pixhawk connected ...`
- no heartbeat-loss reconnect loop

Optional serial fallback (direct Pixhawk to Jetson):

```bash
ros2 run mavlink_inspector inspector --ros-args -p connection_port:=/dev/ttyACM0
```


### 6.3 Recommended launch presets

For easy switching across environments:

```text
Serial lab mode:
  connection_port=/dev/ttyACM0

Pi+Jetson network mode:
  connection_port=udpin:0.0.0.0:14550

Local SITL mode:
  connection_port=udpin:127.0.0.1:<sitl_port>
```

Treat each mode as separate profile; avoid ad-hoc edits during pool sessions.

---

## 7) Verification Flow (Functional)

### 7.1 Telemetry validity

```bash
ros2 topic echo /mavlink/vehicle_state
```

Check:

- `armed` updates correctly
- `flight_mode` is stable/expected
- depth/yaw values are plausible

### 7.2 Runner gate behavior

```bash
ros2 run mavlink_runner runner
```

In REPL:

```text
Duburi > arm
Duburi > forward 40% 2s
Duburi > disarm
```

Expected:

- Movement command accepted only when armed
- no false “Vehicle not armed! Arm first.” after successful arm



### 7.4 Low-level network verification (optional but powerful)

On Jetson, verify UDP listener exists:

```bash
ss -ulpn | rg 14550
```

If needed, capture packets briefly:

```bash
sudo tcpdump -ni any udp port 14550 -c 20
```

If packets are visible here but inspector still fails, problem is parser/state.
If packets are not visible, problem is BlueOS endpoint/network.

### 7.3 Planner behavior

```bash
ros2 run duburi_planner demo_node
```

Expected:

- arm confirmation is visible
- movement states execute
- disarm at end

---

## 8) Known Failure Modes and Fixes

## 8.1 Heartbeat loss loop after initial connect

Symptom:

- connects
- then `heartbeat_lost` and reconnect loop every few seconds

Cause:

- Wrong endpoint IP/port topology (usually loopback or mismatched port)

Fix:

- use Pi `UDP Client -> JETSON_IP:14550`
- Jetson `udpin:0.0.0.0:14550`

## 8.2 Armed/disarmed flapping every ~1s

Symptom:

- alternating `armed` and `disarmed` events at about heartbeat timer cadence

Cause:

- network router looped GCS heartbeat into inbound stream; parser treated all
  heartbeats as vehicle truth

Fix now in codebase:

- `TelemetryParser` ignores `HEARTBEAT` with
  `autopilot == MAV_AUTOPILOT_INVALID`
- only autopilot-origin heartbeats update armed/mode

File:

- `src/mavlink_inspector/mavlink_inspector/telemetry_parser.py`



### 8.4 Diagnostic matrix (symptom -> likely layer)

| Symptom | Likely layer | First check |
|---|---|---|
| Not connected at all | Network/endpoint | BlueOS endpoint host+port, ping, listener socket |
| Connect then heartbeat_lost | Routing topology | Direction mismatch, wrong IP/port, duplicate endpoints |
| ACK accepted but runner says unarmed | State interpretation | `/mavlink/vehicle_state`, heartbeat parsing |
| Planner says armed but no motion | Command path/control mode | inspector command rejection logs, mode, RC output |
| Vision alignment prints HOLD(not armed) forever | Telemetry state propagation | vehicle_state.armed updates |

## 8.3 Runner says “not armed” though arm command accepted

Symptom:

- inspector ACK says arm accepted
- runner still blocks movement

Cause:

- `VehicleState.armed` not updating due to heartbeat interpretation issue

Fix:

- same heartbeat parser fix above
- verify `/mavlink/vehicle_state` shows `armed: true`

---

## 9) Codebase Impact Audit (What this fix touches)

After heartbeat-source filtering fix, these areas are directly affected:

1. `mavlink_inspector`
   - heartbeat parsing
   - emitted events (`armed/disarmed`, `mode_change`)
   - published `VehicleState.armed`

2. `mavlink_runner`
   - `_on_state` updates internal `_armed`
   - `_publish` gating for movement commands

3. `duburi_planner`
   - `PlannerContext.armed` property and state logic that waits for arming
   - command flow depends on inspector rejecting/accepting based on armed state

4. `vision/alignment_controller`
   - checks vehicle armed flag to suppress command spam while disarmed

No interface/schema changes were required; behavior corrected via parser logic.

---

## 10) Reproducible Bring-up Procedure (Copy-Paste)

### Step A: Start inspector (Jetson)

```bash
ros2 run mavlink_inspector inspector --ros-args -p connection_port:=udpin:0.0.0.0:14550
```

### Step B: Confirm telemetry

```bash
ros2 topic echo /mavlink/vehicle_state
```

### Step C: Test runner

```bash
ros2 run mavlink_runner runner
```

Commands:

```text
arm
forward 40% 2s
left 40% 2s
disarm
```

### Step D: Test planner demo

```bash
ros2 run duburi_planner demo_node
```

### Step E: If failure, collect these logs

1. inspector terminal output
2. `ros2 topic echo /mavlink/events`
3. `ros2 topic echo /mavlink/vehicle_state`
4. BlueOS endpoint screenshot + host/port values

---

## 11) Configuration Profiles to Add/Use

For future operations, keep these profile concepts:

1. **serial_direct**
   - `connection_port: /dev/ttyACM0`
2. **network_blueos_udp**
   - `connection_port: udpin:0.0.0.0:14550`
3. **sitl_local**
   - local-only SITL socket config

Switch by passing different `params_file` or `connection_port` override.

---

## 12) Practical Lessons Learned (from this integration)

1. A successful `wait_heartbeat()` does not guarantee stable ongoing stream.
   Always verify sustained telemetry.

2. In routed MAVLink systems, source filtering matters. Do not assume all
   received `HEARTBEAT` messages are from the flight controller.

3. “Runner not armed” bugs are often telemetry-state bugs, not runner parser
   bugs.

4. Keep endpoint topology simple first (single UDP path), then add extra
   tooling endpoints after base control is stable.

---

## 13) References

- [ArduSub pymavlink docs](https://www.ardusub.com/developers/pymavlink.html)
- [BlueOS overview](https://blueos.cloud/docs/stable/usage/overview/)



---



## 14) Appendices

### 14.1 Message classes used in this stack

```text
HEARTBEAT          -> armed/mode truth (autopilot only)
ATTITUDE/AHRS2     -> orientation/depth estimates
SYS_STATUS         -> voltage/current/load
COMMAND_ACK        -> command acceptance/rejection feedback
RC_CHANNELS        -> low-level input/debug channels
```

### 14.2 Bring-up handover template (for new team members)

Copy this into your pool-day notes:

```text
Robot name:
Pi IP:
Jetson IP:
BlueOS endpoint type:
BlueOS endpoint target:
Inspector connection_port:
heartbeat_timeout:
Last successful test time:
Operator:
Notes:
```

### 14.3 Safety reminders

- Always test `stop` and `disarm` before movement tests.
- Keep thruster area clear during network bring-up.
- Verify mode + armed state before sending sustained movement commands.
- If in doubt, kill high-level nodes and keep only inspector until telemetry is stable.
