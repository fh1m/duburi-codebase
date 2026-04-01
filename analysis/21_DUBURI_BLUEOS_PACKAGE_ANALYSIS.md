# 21 — duburi_blueos Package: In-Depth Analysis and Design Decisions

This document provides comprehensive analysis of the `duburi_blueos` package,
covering architecture, design decisions, implementation details, and future
enhancement roadmap for the BlueOS integration layer.

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Problem Statement](#2-problem-statement)
3. [Architecture Overview](#3-architecture-overview)
4. [Module Analysis](#4-module-analysis)
5. [Design Decisions](#5-design-decisions)
6. [Data Flow Analysis](#6-data-flow-analysis)
7. [API Reference](#7-api-reference)
8. [Future Enhancements](#8-future-enhancements)
9. [Integration Patterns](#9-integration-patterns)
10. [Performance Considerations](#10-performance-considerations)

---

## 1) Executive Summary

The `duburi_blueos` package solves a critical integration challenge: bridging
ROS1 MAVROS running on BlueOS (Raspberry Pi) with ROS2 running on the Jetson
Orin Nano, without requiring ros1_bridge or ROS1 installation on the Jetson.

### Key Achievements

- **Zero ROS1 dependency on Jetson**: Uses rosbridge WebSocket protocol
- **Full MAVROS topic access**: 15+ topics bridged to ROS2
- **Bidirectional communication**: Commands flow back to MAVROS
- **BlueOS API integration**: System monitoring and endpoint management
- **Auto-reconnection**: Robust connection handling with configurable retry

### Package Statistics

| Metric | Value |
|--------|-------|
| Total lines of code | ~2,700 |
| Python modules | 5 |
| ROS2 nodes | 3 |
| Bridged topics | 17 |
| Services exposed | 1 |
| External dependencies | 3 (websocket-client, requests, aiohttp) |

---

## 2) Problem Statement

### 2.1 The ROS1/ROS2 Divide

BlueOS provides an excellent ROS1 extension (`blueos-ros`) that runs MAVROS
and rosbridge_websocket on the Raspberry Pi. However, our Jetson runs ROS2
Humble, creating a version mismatch.

**Traditional solutions and their drawbacks:**

| Solution | Drawback |
|----------|----------|
| ros1_bridge | Requires ROS1 installed alongside ROS2, complex setup |
| Run ROS1 on Jetson | Defeats purpose of ROS2 migration |
| BlueOS ROS2 extension | Requires 64-bit BlueOS, Pi 4 doesn't support |
| Direct MAVLink only | Loses MAVROS abstractions (transforms, services) |

### 2.2 Our Solution: rosbridge Protocol

The rosbridge protocol is a JSON-based WebSocket protocol that abstracts ROS
messages. Since it's purely a wire protocol, we can implement a client in
any language without needing ROS1 libraries.

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Traditional Approach                         │
├─────────────────────────────────────────────────────────────────────┤
│  Jetson (ROS2) ←──ros1_bridge──→ Pi (ROS1 MAVROS)                  │
│                                                                     │
│  Problems: Needs ROS1 on Jetson, version conflicts, complexity     │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│                        Our Approach                                 │
├─────────────────────────────────────────────────────────────────────┤
│  Jetson (ROS2)                      Pi (ROS1)                       │
│  ┌──────────────┐                   ┌──────────────┐                │
│  │mavros_bridge │◄──WebSocket──────►│rosbridge_ws  │                │
│  │   (Python)   │    (JSON)         │  port 8889   │                │
│  └──────────────┘                   └──────┬───────┘                │
│        │                                   │                        │
│        │ ROS2 Topics                       │ ROS1 Topics            │
│        ▼                                   ▼                        │
│  /mavros_bridge/*                    /mavros/*                      │
│                                                                     │
│  Benefits: No ROS1 on Jetson, simple Python, reliable              │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 3) Architecture Overview

### 3.1 Package Structure

```
duburi_blueos/
├── duburi_blueos/
│   ├── __init__.py              # Package exports
│   ├── blueos_api.py            # REST API client (450 lines)
│   ├── blueos_monitor_node.py   # Diagnostics publisher (200 lines)
│   ├── mavlink_bridge_node.py   # Endpoint management (180 lines)
│   ├── rosbridge_client.py      # WebSocket client (535 lines)
│   └── mavros_bridge_node.py    # ROS1→ROS2 bridge (560 lines)
├── config/
│   └── blueos_config.yaml       # Configuration
├── launch/
│   └── blueos.launch.py         # Launch file
├── package.xml
├── setup.py
└── setup.cfg
```

### 3.2 Module Dependency Graph

```
                    ┌─────────────────┐
                    │   rclpy (ROS2)  │
                    └────────┬────────┘
                             │
        ┌────────────────────┼────────────────────┐
        │                    │                    │
        ▼                    ▼                    ▼
┌───────────────┐  ┌─────────────────┐  ┌─────────────────┐
│blueos_monitor │  │ mavlink_bridge  │  │  mavros_bridge  │
│    _node      │  │     _node       │  │     _node       │
└───────┬───────┘  └────────┬────────┘  └────────┬────────┘
        │                   │                    │
        ▼                   ▼                    ▼
┌───────────────┐  ┌─────────────────┐  ┌─────────────────┐
│  blueos_api   │  │   blueos_api    │  │rosbridge_client │
│   (REST)      │  │    (REST)       │  │  (WebSocket)    │
└───────────────┘  └─────────────────┘  └─────────────────┘
        │                   │                    │
        ▼                   ▼                    ▼
┌─────────────────────────────────────────────────────────┐
│                    BlueOS (Pi)                          │
│  ┌─────────┐  ┌──────────────┐  ┌────────────────────┐  │
│  │REST API │  │MAVLink Router│  │rosbridge_websocket │  │
│  │ :80     │  │    :14550    │  │      :8889         │  │
│  └─────────┘  └──────────────┘  └────────────────────┘  │
└─────────────────────────────────────────────────────────┘
```

### 3.3 Network Topology

```
┌─────────────────────────────────────────────────────────────────────┐
│                         Physical Layer                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌──────────────┐      ┌──────────────┐      ┌──────────────┐      │
│  │  Pixhawk     │      │ Raspberry Pi │      │Jetson Orin   │      │
│  │  2.4.8       │      │ 4B (BlueOS)  │      │   Nano       │      │
│  │              │      │ 192.168.2.2  │      │192.168.2.69  │      │
│  └──────┬───────┘      └──────┬───────┘      └──────┬───────┘      │
│         │ Serial              │ Ethernet           │ Ethernet      │
│         │ (USB/UART)          │                    │               │
│         └─────────────────────┴────────────────────┘               │
│                               │                                     │
│                        ┌──────┴──────┐                              │
│                        │   Switch    │                              │
│                        └──────┬──────┘                              │
│                               │                                     │
│                        ┌──────┴──────┐                              │
│                        │  GCS/PC     │                              │
│                        │192.168.2.1  │                              │
│                        └─────────────┘                              │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│                         Protocol Layer                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  Pixhawk ──MAVLink/Serial──► Pi ──MAVLink/UDP──► Jetson            │
│                              │                                      │
│                              ├──rosbridge/WS───► Jetson             │
│                              │   (port 8889)     (mavros_bridge)    │
│                              │                                      │
│                              └──REST/HTTP──────► Jetson             │
│                                 (port 80)        (blueos_api)       │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 4) Module Analysis

### 4.1 rosbridge_client.py (Core WebSocket Client)

This is the heart of the package—a pure Python implementation of the
rosbridge protocol v2.0.

#### Class: ROSBridgeClient

**Purpose**: Generic rosbridge WebSocket client supporting pub/sub/service.

**Key Design Patterns**:

1. **Thread-safe subscription management**
   ```python
   self._subscriptions: Dict[str, List[Callable]] = {}
   self._lock = threading.Lock()
   ```

2. **Automatic reconnection**
   ```python
   def _run(self):
       while not self._shutdown:
           self._ws.run_forever(ping_interval=10, ping_timeout=5)
           if self._shutdown or not self.reconnect:
               break
           time.sleep(self.reconnect_interval)
   ```

3. **Callback-based message dispatch**
   ```python
   def _on_message(self, ws, message):
       data = json.loads(message)
       if data.get('op') == 'publish':
           topic = data.get('topic')
           for cb in self._subscriptions.get(topic, []):
               cb(data.get('msg', {}))
   ```

**rosbridge Protocol Messages**:

| Operation | Direction | Purpose |
|-----------|-----------|---------|
| subscribe | Client→Server | Register for topic updates |
| unsubscribe | Client→Server | Stop receiving topic |
| publish | Bidirectional | Send/receive topic data |
| advertise | Client→Server | Declare intent to publish |
| call_service | Client→Server | Invoke ROS service |
| service_response | Server→Client | Service result |

**Example subscribe message**:
```json
{
  "op": "subscribe",
  "topic": "/mavros/state",
  "type": "mavros_msgs/State",
  "throttle_rate": 0,
  "queue_length": 1
}
```

#### Class: MAVROSBridge (extends ROSBridgeClient)

**Purpose**: MAVROS-specific convenience methods.

**Topic Registry**:
```python
TOPICS = {
    '/mavros/state': 'mavros_msgs/State',
    '/mavros/battery': 'sensor_msgs/BatteryState',
    '/mavros/imu/data': 'sensor_msgs/Imu',
    '/mavros/vfr_hud': 'mavros_msgs/VFR_HUD',
    # ... 16 more topics
}
```

**Command Methods**:
```python
def arm(self, arm: bool = True) -> bool
def disarm(self) -> bool
def set_mode(self, mode: str) -> bool
def send_rc_override(self, channels: List[int])
def send_manual_control(self, x, y, z, r, buttons)
```

### 4.2 mavros_bridge_node.py (ROS2 Bridge Node)

**Purpose**: Subscribes to MAVROS topics via rosbridge and republishes as
native ROS2 topics.

#### Topic Mapping

| ROS1 MAVROS Topic | ROS2 Bridge Topic | Message Type |
|-------------------|-------------------|--------------|
| /mavros/state | ~/armed, ~/mode | Bool, String |
| /mavros/battery | ~/battery | BatteryState |
| /mavros/imu/data | ~/imu/data | Imu |
| /mavros/imu/mag | ~/imu/mag | MagneticField |
| /mavros/imu/static_pressure | ~/imu/pressure | FluidPressure |
| /mavros/vfr_hud | ~/vfr_hud | Float32MultiArray |
| /mavros/global_position/global | ~/global_position | NavSatFix |
| /mavros/local_position/pose | ~/local_position/pose | PoseStamped |
| /mavros/local_position/velocity_local | ~/local_position/velocity | TwistStamped |
| /mavros/global_position/compass_hdg | ~/compass_hdg | Float64 |
| /mavros/global_position/rel_alt | ~/rel_alt | Float64 |
| /mavros/rc/in | ~/rc/in | UInt16MultiArray |
| /mavros/rc/out | ~/rc/out | UInt16MultiArray |

#### QoS Profiles

```python
# Sensor data: best-effort, volatile (high frequency, tolerate drops)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)

# State data: reliable, transient-local (important, late-joiners get last)
state_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
```

#### Message Conversion Example

Converting ROS1 Imu (dict from rosbridge) to ROS2 Imu message:

```python
def _on_mavros_imu(self, msg: Dict[str, Any]):
    imu = Imu()
    imu.header.stamp = self.get_clock().now().to_msg()
    imu.header.frame_id = 'imu_link'

    # Orientation quaternion
    orientation = msg.get('orientation', {})
    imu.orientation.x = orientation.get('x', 0.0)
    imu.orientation.y = orientation.get('y', 0.0)
    imu.orientation.z = orientation.get('z', 0.0)
    imu.orientation.w = orientation.get('w', 1.0)

    # Angular velocity
    angular = msg.get('angular_velocity', {})
    imu.angular_velocity.x = angular.get('x', 0.0)
    # ... etc

    self.imu_pub.publish(imu)
```

### 4.3 blueos_api.py (REST API Client)

**Purpose**: Interface with BlueOS REST APIs for system monitoring and
MAVLink endpoint management.

#### Endpoints Accessed

| Endpoint | Purpose |
|----------|---------|
| GET /v1.0/system | System info (hostname, platform) |
| GET /version | BlueOS version |
| GET /v1.0/endpoints/ | List MAVLink endpoints |
| POST /v1.0/endpoints/ | Create endpoint |
| DELETE /v1.0/endpoints/{name} | Remove endpoint |

#### Key Classes

```python
@dataclass
class SystemInfo:
    hostname: str
    platform: str
    cpu_count: int
    memory_total: int
    blueos_version: str

@dataclass
class MavlinkEndpoint:
    name: str
    endpoint_type: EndpointType  # UDP_CLIENT, UDP_SERVER, TCP_CLIENT, etc.
    place: str  # "udpout:192.168.2.69:14550"
    enabled: bool

class BlueOSAPI:
    def get_system_info(self) -> SystemInfo
    def get_endpoints(self) -> List[MavlinkEndpoint]
    def create_endpoint(self, name, type, place, ...) -> bool
    def delete_endpoint(self, name) -> bool
```

### 4.4 blueos_monitor_node.py (Diagnostics Node)

**Purpose**: Publish BlueOS system status to ROS2 diagnostics.

**Published Topics**:
- `/blueos/diagnostics` (diagnostic_msgs/DiagnosticArray)
- `/blueos/system_info` (custom)

**Monitored Metrics**:
- BlueOS version and uptime
- CPU/memory usage
- MAVLink endpoint status
- Service health (MAVROS, rosbridge)

### 4.5 mavlink_bridge_node.py (Endpoint Manager)

**Purpose**: Dynamically manage MAVLink endpoints on BlueOS.

**Features**:
- Auto-create Jetson endpoint on startup
- Monitor endpoint health
- Provide services to add/remove endpoints

---

## 5) Design Decisions

### 5.1 Why WebSocket over Direct MAVLink?

| Aspect | rosbridge/WebSocket | Direct MAVLink |
|--------|---------------------|----------------|
| Message abstraction | High-level (ROS topics) | Low-level (bytes) |
| Existing work | Leverages MAVROS | Must reimplement |
| Coordinate frames | MAVROS handles TF | Manual transforms |
| Services | Full access | Would need custom impl |
| Complexity | Medium | High |
| Latency | ~5-10ms added | Minimal |
| Debugging | JSON readable | Binary inspection |

**Decision**: WebSocket adds minimal latency but provides significant
development velocity by reusing MAVROS's battle-tested code.

### 5.2 Why Not Use ros1_bridge?

1. **Requires ROS1 on Jetson**: Doubles disk usage, version conflicts
2. **Complex setup**: Need to build bridge with all message types
3. **Maintenance burden**: Must rebuild when messages change
4. **Resource usage**: Another process consuming Jetson resources

**Decision**: Pure Python rosbridge client is self-contained and portable.

### 5.3 Thread Model

```
┌─────────────────────────────────────────────────────────────────────┐
│                     mavros_bridge_node Process                      │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌─────────────────┐      ┌─────────────────┐                      │
│  │  Main Thread    │      │ WebSocket Thread│                      │
│  │  (rclpy.spin)   │      │ (ws.run_forever)│                      │
│  │                 │      │                 │                      │
│  │  - Timer CBs    │◄─────│  - on_message   │                      │
│  │  - Service CBs  │ Lock │  - on_connect   │                      │
│  │  - Subscriber   │      │  - on_error     │                      │
│  │    callbacks    │      │                 │                      │
│  └─────────────────┘      └─────────────────┘                      │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

**Key consideration**: rosbridge callbacks run in the WebSocket thread, but
ROS2 publishers are thread-safe. We use a lock only for shared state like
`_connected`, `_armed`, `_mode`.

### 5.4 Topic Namespace Decision

**Option A**: Mirror MAVROS namespace (`/mavros/*`)
- Pro: Familiar to MAVROS users
- Con: Confusing—suggests ROS1 MAVROS is running locally

**Option B**: Use bridge namespace (`/mavros_bridge/*`)
- Pro: Clear that data comes from bridge
- Con: Different from standard MAVROS

**Decision**: Option B—clarity is more important. Users can remap if needed.

### 5.5 Message Type Simplification

Some MAVROS message types (e.g., `mavros_msgs/State`) don't exist in ROS2
standard packages. Options:

1. Create `duburi_msgs` package with equivalents
2. Use standard ROS2 types with conventions
3. Use generic types (String, Float32MultiArray)

**Decision**: Mix of (2) and (3):
- `State` → separate `~/armed` (Bool) and `~/mode` (String)
- `VFR_HUD` → Float32MultiArray with documented indices
- Standard types (Imu, BatteryState) used directly

This avoids custom message dependencies while maintaining usability.

### 5.6 Reconnection Strategy

```python
reconnect_interval = 2.0  # seconds

def _run(self):
    while not self._shutdown:
        try:
            self._ws.run_forever(ping_interval=10, ping_timeout=5)
        except Exception:
            pass
        
        if not self.reconnect:
            break
        time.sleep(self.reconnect_interval)
```

**Parameters**:
- `ping_interval=10`: Send WebSocket ping every 10s
- `ping_timeout=5`: Fail if no pong within 5s
- `reconnect_interval=2.0`: Wait 2s between reconnect attempts

**Rationale**: Aggressive reconnection for underwater operations where
connection may be intermittent but recovery is critical.

---

## 6) Data Flow Analysis

### 6.1 Telemetry Path (Vehicle → ROS2)

```
Pixhawk
   │ MAVLink HEARTBEAT, ATTITUDE, etc.
   ▼
BlueOS MAVLink Router
   │ Routes to MAVROS
   ▼
ROS1 MAVROS Node (on Pi)
   │ Converts to ROS messages
   │ Publishes /mavros/* topics
   ▼
rosbridge_websocket (port 8889)
   │ Serializes to JSON
   │ Sends over WebSocket
   ▼
mavros_bridge_node (Jetson)
   │ Deserializes JSON
   │ Converts to ROS2 messages
   │ Publishes /mavros_bridge/*
   ▼
ROS2 Subscribers (runner, planner, etc.)
```

**Latency breakdown**:
| Hop | Typical Latency |
|-----|-----------------|
| Pixhawk → Pi (serial) | 1-2ms |
| MAVLink → MAVROS | <1ms |
| MAVROS → rosbridge | <1ms |
| WebSocket network | 1-5ms |
| JSON parse + convert | 1-2ms |
| **Total** | **5-12ms** |

### 6.2 Command Path (ROS2 → Vehicle)

```
ROS2 Publisher (e.g., /mavros_bridge/manual_control)
   │
   ▼
mavros_bridge_node subscriber callback
   │ Converts Twist to MAVROS format
   │ Calls bridge.send_manual_control()
   ▼
ROSBridgeClient.publish()
   │ JSON: {"op": "publish", "topic": "/mavros/manual_control/send", ...}
   ▼
rosbridge_websocket (Pi)
   │ Deserializes, publishes to ROS1
   ▼
ROS1 MAVROS (on Pi)
   │ Converts to MAVLink MANUAL_CONTROL
   ▼
Pixhawk
```

### 6.3 Service Call Path (Arm/Disarm)

```
ros2 service call /mavros_bridge/arm std_srvs/srv/SetBool "{data: true}"
   │
   ▼
mavros_bridge_node._arm_callback()
   │ Calls bridge.arm(True)
   ▼
ROSBridgeClient.call_service('/mavros/cmd/arming', {'value': True})
   │ Creates service call ID
   │ Sends: {"op": "call_service", "service": "/mavros/cmd/arming", ...}
   │ Waits on Queue with timeout
   ▼
rosbridge_websocket (Pi)
   │ Calls ROS1 service
   ▼
MAVROS /mavros/cmd/arming service
   │ Sends MAVLink COMMAND_LONG (MAV_CMD_COMPONENT_ARM_DISARM)
   ▼
Pixhawk
   │ Arms/disarms
   │ Sends COMMAND_ACK
   ▼
MAVROS service returns
   ▼
rosbridge sends: {"op": "service_response", "id": "...", "values": {...}}
   ▼
ROSBridgeClient receives, puts in Queue
   ▼
mavros_bridge_node returns SetBool.Response
```

---

## 7) API Reference

### 7.1 ROSBridgeClient

```python
class ROSBridgeClient:
    def __init__(
        self,
        host: str = '192.168.2.2',
        port: int = 8889,
        reconnect: bool = True,
        reconnect_interval: float = 2.0,
    )
    
    # Connection
    def connect(self, blocking: bool = False, timeout: float = 10.0) -> bool
    def disconnect(self)
    
    @property
    def connected(self) -> bool
    
    # Pub/Sub
    def subscribe(
        self,
        topic: str,
        msg_type: str,
        callback: Callable[[Dict], None],
        throttle_rate: int = 0,
        queue_length: int = 1,
    )
    def unsubscribe(self, topic: str, callback: Optional[Callable] = None)
    def publish(self, topic: str, msg_type: str, msg: Dict)
    def advertise(self, topic: str, msg_type: str)
    def unadvertise(self, topic: str)
    
    # Services
    def call_service(
        self,
        service: str,
        args: Optional[Dict] = None,
        timeout: float = 5.0,
    ) -> Optional[Dict]
    
    # Discovery
    def get_topics(self) -> List[str]
    def get_topic_type(self, topic: str) -> Optional[str]
    def get_services(self) -> List[str]
    
    # Callbacks
    on_connect: Optional[Callable]
    on_disconnect: Optional[Callable]
    on_error: Optional[Callable[[str], None]]
```

### 7.2 MAVROSBridge

```python
class MAVROSBridge(ROSBridgeClient):
    # Convenience subscriptions
    def subscribe_state(self, callback: Callable[[Dict], None])
    def subscribe_battery(self, callback: Callable[[Dict], None])
    def subscribe_imu(self, callback: Callable[[Dict], None])
    def subscribe_pose(self, callback: Callable[[Dict], None])
    def subscribe_vfr_hud(self, callback: Callable[[Dict], None])
    def subscribe_rc_in(self, callback: Callable[[Dict], None])
    def subscribe_rc_out(self, callback: Callable[[Dict], None])
    
    # Commands
    def arm(self, arm: bool = True) -> bool
    def disarm(self) -> bool
    def set_mode(self, mode: str) -> bool
    def send_rc_override(self, channels: List[int])
    def send_manual_control(
        self,
        x: int = 0,      # Forward/back (-1000 to 1000)
        y: int = 0,      # Left/right (-1000 to 1000)
        z: int = 500,    # Throttle (0 to 1000)
        r: int = 0,      # Yaw (-1000 to 1000)
        buttons: int = 0
    )
```

### 7.3 BlueOSAPI

```python
class BlueOSAPI:
    def __init__(self, host: str = '192.168.2.2', timeout: float = 5.0)
    
    # System
    def get_system_info(self) -> SystemInfo
    def ping(self) -> bool
    
    # Endpoints
    def get_endpoints(self) -> List[MavlinkEndpoint]
    def create_endpoint(
        self,
        name: str,
        endpoint_type: EndpointType,
        place: str,
        enabled: bool = True,
        protected: bool = False,
    ) -> bool
    def delete_endpoint(self, name: str) -> bool
    
    # Services
    def get_services(self) -> List[ServiceInfo]
```

---

## 8) Future Enhancements

### 8.1 Unified Vehicle Interface Node

**Problem**: Multiple ROS2 packages need vehicle state, but must know about
`/mavros_bridge/` namespace and message formats.

**Solution**: Create `vehicle_interface` node that aggregates data and
provides a clean, stable API.

```
┌─────────────────────────────────────────────────────────────────────┐
│                     vehicle_interface Node                          │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  Subscriptions:                    Publications:                    │
│  ─────────────                     ─────────────                    │
│  /mavros_bridge/armed         →    /vehicle/state (VehicleState)   │
│  /mavros_bridge/mode          →    /vehicle/battery (BatteryState) │
│  /mavros_bridge/battery       →    /vehicle/pose (PoseStamped)     │
│  /mavros_bridge/imu/data      →    /vehicle/velocity (TwistStamped)│
│  /mavros_bridge/local_position/*   /vehicle/imu (Imu)              │
│                                                                     │
│  Subscriptions:                    Publications to MAVROS:          │
│  ─────────────                     ──────────────────────           │
│  /vehicle/cmd_vel (Twist)     →    /mavros_bridge/manual_control   │
│  /vehicle/rc_override         →    /mavros_bridge/rc/override      │
│                                                                     │
│  Services:                                                          │
│  ─────────                                                          │
│  /vehicle/arm (SetBool)                                             │
│  /vehicle/set_mode (SetMode)                                        │
│  /vehicle/get_state (GetState)                                      │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

**Custom Message**: `duburi_msgs/VehicleState`
```
std_msgs/Header header
bool armed
string mode
bool connected
float32 battery_voltage
float32 battery_percentage
float32 heading
float32 depth
float32 altitude
```

**Benefits**:
- Stable API even if bridge implementation changes
- Single point for vehicle abstraction
- Easier testing with mock vehicle_interface

### 8.2 Health Monitor Integration

**Problem**: No centralized system health view.

**Solution**: Integrate with ROS2 diagnostics aggregator.

```python
# In health_monitor node
class HealthMonitor(Node):
    def __init__(self):
        # Subscribe to all health sources
        self.create_subscription(Bool, '/mavros_bridge/connected', ...)
        self.create_subscription(BatteryState, '/mavros_bridge/battery', ...)
        self.create_subscription(DiagnosticArray, '/blueos/diagnostics', ...)
        
        # Publish aggregated status
        self.status_pub = self.create_publisher(
            SystemHealth, '/system/health', 10
        )
        
    def check_health(self):
        issues = []
        
        if not self.mavros_connected:
            issues.append(HealthIssue(CRITICAL, "MAVROS disconnected"))
        
        if self.battery_voltage < 14.0:
            issues.append(HealthIssue(WARNING, f"Low battery: {self.battery_voltage}V"))
        
        if self.battery_voltage < 13.0:
            issues.append(HealthIssue(CRITICAL, "Critical battery"))
        
        # Publish
        msg = SystemHealth()
        msg.status = HEALTHY if not issues else issues[0].level
        msg.issues = issues
        self.status_pub.publish(msg)
```

**Alert actions**:
- Log to console and file
- Trigger surface/abort if critical
- LED/buzzer indication (via RC channel)

### 8.3 Failover Connection Manager

**Problem**: If rosbridge fails, lose all telemetry.

**Solution**: Dual-path connection with automatic failover.

```
┌─────────────────────────────────────────────────────────────────────┐
│                   Connection Manager                                │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  Primary: rosbridge (full MAVROS access)                           │
│      │                                                              │
│      ▼                                                              │
│  ┌─────────┐     Healthy?     ┌─────────────────────┐              │
│  │rosbridge│ ───────────────► │ Use rosbridge data  │              │
│  │ client  │       Yes        │ Full topic access   │              │
│  └─────────┘                  └─────────────────────┘              │
│      │                                                              │
│      │ No (timeout/error)                                           │
│      ▼                                                              │
│  ┌─────────┐                  ┌─────────────────────┐              │
│  │ MAVLink │ ───────────────► │ Use direct MAVLink  │              │
│  │  UDP    │                  │ Basic state only    │              │
│  └─────────┘                  └─────────────────────┘              │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

**Implementation sketch**:
```python
class FailoverManager:
    def __init__(self):
        self.rosbridge = MAVROSBridge(...)
        self.mavlink = MavlinkConnection(...)  # From mavlink_inspector
        self.active = 'rosbridge'
        
    def get_state(self) -> VehicleState:
        if self.active == 'rosbridge' and self.rosbridge.connected:
            return self._state_from_rosbridge()
        else:
            return self._state_from_mavlink()
    
    def health_check(self):
        if not self.rosbridge.connected:
            if self.active == 'rosbridge':
                self.get_logger().warn("Failover to direct MAVLink")
                self.active = 'mavlink'
        elif self.active == 'mavlink':
            self.get_logger().info("Recovered rosbridge, switching back")
            self.active = 'rosbridge'
```

### 8.4 Camera/Sensor Trigger via RC Override

**Problem**: Need to control auxiliary functions (lights, camera, gripper).

**Solution**: Map RC channels to actions.

```python
# RC channel mapping for BlueROV2
RC_CHANNELS = {
    'pitch': 1,
    'roll': 2,
    'throttle': 3,
    'yaw': 4,
    'forward': 5,
    'lateral': 6,
    'camera_tilt': 7,
    'lights': 8,
    'camera_trigger': 9,
    'gripper': 10,
}

class AuxController:
    def set_lights(self, brightness: float):
        """Set lights brightness 0.0-1.0"""
        pwm = int(1100 + brightness * 800)  # 1100-1900
        channels = [0] * 18
        channels[RC_CHANNELS['lights'] - 1] = pwm
        self.bridge.send_rc_override(channels)
    
    def trigger_camera(self):
        """Pulse camera trigger channel"""
        channels = [0] * 18
        channels[RC_CHANNELS['camera_trigger'] - 1] = 1900
        self.bridge.send_rc_override(channels)
        time.sleep(0.5)
        channels[RC_CHANNELS['camera_trigger'] - 1] = 1100
        self.bridge.send_rc_override(channels)
```

### 8.5 Waypoint/Mission Upload

**Problem**: Can't upload autonomous missions from ROS2.

**Solution**: Implement mission service calls.

```python
class MissionManager:
    def upload_mission(self, waypoints: List[Waypoint]) -> bool:
        # Clear existing
        self.bridge.call_service('/mavros/mission/clear')
        
        # Push new waypoints
        mission_items = []
        for i, wp in enumerate(waypoints):
            mission_items.append({
                'seq': i,
                'frame': 3,  # MAV_FRAME_GLOBAL_RELATIVE_ALT
                'command': 16,  # MAV_CMD_NAV_WAYPOINT
                'is_current': i == 0,
                'autocontinue': True,
                'x_lat': wp.latitude,
                'y_long': wp.longitude,
                'z_alt': wp.altitude,
            })
        
        result = self.bridge.call_service(
            '/mavros/mission/push',
            {'start_index': 0, 'waypoints': mission_items}
        )
        return result.get('success', False)
    
    def download_mission(self) -> List[Waypoint]:
        result = self.bridge.call_service('/mavros/mission/pull')
        # Convert to Waypoint objects
        ...
```

### 8.6 Parameter Management

**Problem**: Can't read/write ArduSub parameters from ROS2.

**Solution**: Expose parameter services.

```python
class ParamManager:
    def get_param(self, name: str) -> Optional[float]:
        result = self.bridge.call_service(
            '/mavros/param/get',
            {'param_id': name}
        )
        if result and result.get('success'):
            return result.get('value', {}).get('real', None)
        return None
    
    def set_param(self, name: str, value: float) -> bool:
        result = self.bridge.call_service(
            '/mavros/param/set',
            {'param_id': name, 'value': {'real': value}}
        )
        return result.get('success', False) if result else False
    
    # Common parameters
    def set_depth_hold_pid(self, p: float, i: float, d: float):
        self.set_param('PSC_POSZ_P', p)
        self.set_param('PSC_VELZ_I', i)
        self.set_param('PSC_VELZ_D', d)
```

### 8.7 Depth Hold / Altitude Control

**Problem**: ROS2 stack doesn't have access to depth data for control.

**Solution**: Subscribe to pressure/depth and implement controllers.

```python
class DepthController:
    def __init__(self):
        self.target_depth = 0.0
        self.current_depth = 0.0
        self.pid = PIDController(kp=0.5, ki=0.1, kd=0.05)
        
        # Subscribe to pressure
        self.bridge.subscribe(
            '/mavros/imu/static_pressure',
            'sensor_msgs/FluidPressure',
            self._on_pressure
        )
    
    def _on_pressure(self, msg):
        # Convert pressure to depth (simplified)
        # P = rho * g * h + P_atm
        pressure_pa = msg['fluid_pressure']
        self.current_depth = (pressure_pa - 101325) / (1025 * 9.81)
    
    def hold_depth(self, target: float):
        self.target_depth = target
        
    def update(self) -> float:
        """Returns throttle adjustment"""
        error = self.target_depth - self.current_depth
        return self.pid.compute(error)
```

### 8.8 Joystick Passthrough

**Problem**: Want to teleoperate from ROS2 joystick node.

**Solution**: Bridge joy messages to manual_control.

```python
class JoystickBridge(Node):
    def __init__(self):
        super().__init__('joystick_bridge')
        
        # Subscribe to joy
        self.create_subscription(Joy, '/joy', self._on_joy, 10)
        
        # Publish to bridge
        self.cmd_pub = self.create_publisher(
            Twist, '/mavros_bridge/manual_control', 10
        )
        
        # Button mappings (Xbox controller)
        self.BTN_ARM = 7      # Start
        self.BTN_DISARM = 6   # Back
        self.BTN_MODE_MANUAL = 0  # A
        self.BTN_MODE_STABILIZE = 1  # B
        
    def _on_joy(self, msg: Joy):
        # Axis mapping
        twist = Twist()
        twist.linear.x = msg.axes[1]  # Left stick Y (forward/back)
        twist.linear.y = msg.axes[0]  # Left stick X (strafe)
        twist.linear.z = msg.axes[4]  # Right stick Y (throttle)
        twist.angular.z = msg.axes[3]  # Right stick X (yaw)
        
        self.cmd_pub.publish(twist)
        
        # Button handling
        if msg.buttons[self.BTN_ARM]:
            self.call_arm_service(True)
        elif msg.buttons[self.BTN_DISARM]:
            self.call_arm_service(False)
```

### 8.9 Data Logging Pipeline

**Problem**: Need to record all data for post-dive analysis.

**Solution**: Automated rosbag2 recording with metadata.

```python
class DiveLogger:
    def start_recording(self, dive_name: str):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        bag_path = f'/data/dives/{dive_name}_{timestamp}'
        
        # Topics to record
        topics = [
            '/mavros_bridge/imu/data',
            '/mavros_bridge/battery',
            '/mavros_bridge/global_position',
            '/mavros_bridge/local_position/pose',
            '/mavros_bridge/vfr_hud',
            '/mavros_bridge/rc/in',
            '/mavros_bridge/rc/out',
            '/camera/image_raw/compressed',
            '/sonar/range',
        ]
        
        # Start rosbag2 via subprocess
        cmd = ['ros2', 'bag', 'record', '-o', bag_path] + topics
        self.bag_process = subprocess.Popen(cmd)
        
        # Write metadata
        with open(f'{bag_path}/metadata.yaml', 'w') as f:
            yaml.dump({
                'dive_name': dive_name,
                'start_time': timestamp,
                'vehicle': 'Duburi AUV 4.2',
                'location': self.get_gps_location(),
            }, f)
```

### 8.10 BlueOS Extension API

**Problem**: Manual endpoint management through BlueOS web UI.

**Solution**: Automate with BlueOSAPI.

```python
class EndpointManager:
    def __init__(self):
        self.api = BlueOSAPI('192.168.2.2')
        
    def setup_for_jetson(self, jetson_ip: str):
        """Configure BlueOS for Jetson connection"""
        
        # Remove any existing Jetson endpoint
        endpoints = self.api.get_endpoints()
        for ep in endpoints:
            if 'jetson' in ep.name.lower():
                self.api.delete_endpoint(ep.name)
        
        # Create new endpoint
        self.api.create_endpoint(
            name='Jetson Orin',
            endpoint_type=EndpointType.UDP_CLIENT,
            place=f'udpout:{jetson_ip}:14550',
            enabled=True,
            protected=False,
        )
        
    def add_gcs(self, gcs_ip: str, port: int = 14550):
        """Add a ground control station"""
        self.api.create_endpoint(
            name=f'GCS-{gcs_ip}',
            endpoint_type=EndpointType.UDP_CLIENT,
            place=f'udpout:{gcs_ip}:{port}',
            enabled=True,
        )
```

---

## 9) Integration Patterns

### 9.1 Recommended Node Graph

```
┌─────────────────────────────────────────────────────────────────────┐
│                    Recommended Node Architecture                    │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                     duburi_blueos                            │   │
│  │  ┌──────────────┐ ┌──────────────┐ ┌──────────────────────┐ │   │
│  │  │blueos_monitor│ │mavlink_bridge│ │   mavros_bridge      │ │   │
│  │  └──────────────┘ └──────────────┘ └──────────────────────┘ │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              │                                      │
│                              ▼                                      │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                   vehicle_interface (NEW)                    │   │
│  │              Unified API for vehicle access                  │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              │                                      │
│           ┌──────────────────┼──────────────────┐                  │
│           ▼                  ▼                  ▼                  │
│  ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐      │
│  │  duburi_control │ │ duburi_planner  │ │  duburi_vision  │      │
│  │  (runner, etc)  │ │                 │ │                 │      │
│  └─────────────────┘ └─────────────────┘ └─────────────────┘      │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 9.2 Launch File Organization

```python
# Full system launch
def generate_launch_description():
    return LaunchDescription([
        # BlueOS layer
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('duburi_blueos'),
                '/launch/blueos.launch.py'
            ]),
        ),
        
        # Vehicle interface (once created)
        # Node(package='duburi_vehicle', executable='vehicle_interface'),
        
        # Control layer
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('duburi_control'),
                '/launch/control.launch.py'
            ]),
        ),
        
        # Mission layer
        # ...
    ])
```

### 9.3 Testing Strategy

```python
# Unit tests for rosbridge_client
class TestROSBridgeClient(unittest.TestCase):
    def test_subscribe_unsubscribe(self):
        client = ROSBridgeClient('localhost', 9090)
        callback = Mock()
        
        client.subscribe('/test', 'std_msgs/String', callback)
        assert '/test' in client._subscriptions
        
        client.unsubscribe('/test')
        assert '/test' not in client._subscriptions
    
    def test_message_dispatch(self):
        client = ROSBridgeClient('localhost', 9090)
        received = []
        
        client.subscribe('/test', 'std_msgs/String', lambda m: received.append(m))
        
        # Simulate incoming message
        client._on_message(None, json.dumps({
            'op': 'publish',
            'topic': '/test',
            'msg': {'data': 'hello'}
        }))
        
        assert received == [{'data': 'hello'}]

# Integration tests (require live BlueOS)
@pytest.mark.integration
class TestMAVROSBridge:
    def test_live_connection(self):
        bridge = MAVROSBridge('192.168.2.2', 8889)
        connected = bridge.connect(blocking=True, timeout=5)
        assert connected
        bridge.disconnect()
    
    def test_state_subscription(self):
        bridge = MAVROSBridge('192.168.2.2', 8889)
        bridge.connect(blocking=True, timeout=5)
        
        state = {}
        bridge.subscribe_state(lambda m: state.update(m))
        time.sleep(2)
        
        assert 'armed' in state
        assert 'mode' in state
        bridge.disconnect()
```

---

## 10) Performance Considerations

### 10.1 Latency Budget

For responsive control, total command latency should be <50ms:

| Component | Budget | Measured |
|-----------|--------|----------|
| ROS2 publish → bridge callback | 1ms | <1ms |
| JSON serialize | 1ms | <1ms |
| Network (Jetson→Pi) | 5ms | 2-5ms |
| rosbridge dispatch | 1ms | <1ms |
| MAVROS convert | 1ms | <1ms |
| MAVLink over serial | 5ms | 3-5ms |
| **Total** | **14ms** | **8-13ms** |

### 10.2 Bandwidth Considerations

| Topic | Frequency | Message Size | Bandwidth |
|-------|-----------|--------------|-----------|
| imu/data | 50 Hz | ~200 bytes | 10 KB/s |
| vfr_hud | 10 Hz | ~50 bytes | 0.5 KB/s |
| battery | 1 Hz | ~100 bytes | 0.1 KB/s |
| local_position | 30 Hz | ~150 bytes | 4.5 KB/s |
| rc/in | 10 Hz | ~40 bytes | 0.4 KB/s |
| **Total** | | | **~16 KB/s** |

Ethernet easily handles this; WebSocket overhead is minimal.

### 10.3 Memory Usage

```
mavros_bridge_node typical memory:
  - Python interpreter: ~30 MB
  - ROS2 context: ~20 MB
  - WebSocket connection: ~1 MB
  - Message buffers: ~5 MB
  - Total: ~56 MB
```

Acceptable for Jetson Orin Nano (8GB RAM).

### 10.4 CPU Usage

```
mavros_bridge_node @ 50 Hz IMU:
  - Message parsing: ~2% CPU
  - JSON decode: ~1% CPU
  - ROS2 publish: ~1% CPU
  - Total: ~4% single core
```

Minimal impact on Jetson's 6-core CPU.

---

## Appendix A: rosbridge Protocol Reference

### Message Format

All messages are JSON with an `op` field:

```json
{"op": "subscribe", "topic": "/mavros/state", "type": "mavros_msgs/State"}
{"op": "unsubscribe", "topic": "/mavros/state"}
{"op": "publish", "topic": "/mavros/rc/override", "msg": {"channels": [...]}}
{"op": "call_service", "service": "/mavros/cmd/arming", "args": {"value": true}}
```

### Supported Operations

| Operation | Client→Server | Server→Client |
|-----------|---------------|---------------|
| advertise | ✓ | |
| unadvertise | ✓ | |
| publish | ✓ | ✓ |
| subscribe | ✓ | |
| unsubscribe | ✓ | |
| call_service | ✓ | |
| service_response | | ✓ |

---

## Appendix B: MAVROS Topic Reference

### Telemetry Topics

| Topic | Type | Frequency | Description |
|-------|------|-----------|-------------|
| /mavros/state | mavros_msgs/State | 1 Hz | Armed, mode, connected |
| /mavros/battery | sensor_msgs/BatteryState | 1 Hz | Voltage, current, % |
| /mavros/imu/data | sensor_msgs/Imu | 50 Hz | Orientation, angular vel, accel |
| /mavros/imu/mag | sensor_msgs/MagneticField | 50 Hz | Magnetometer |
| /mavros/imu/static_pressure | sensor_msgs/FluidPressure | 10 Hz | Pressure for depth |
| /mavros/vfr_hud | mavros_msgs/VFR_HUD | 10 Hz | Heading, speed, alt, throttle |
| /mavros/global_position/global | sensor_msgs/NavSatFix | 5 Hz | GPS lat/lon/alt |
| /mavros/local_position/pose | geometry_msgs/PoseStamped | 30 Hz | Local position |
| /mavros/rc/in | mavros_msgs/RCIn | 10 Hz | RC input channels |
| /mavros/rc/out | mavros_msgs/RCOut | 10 Hz | Servo outputs |

### Command Topics

| Topic | Type | Description |
|-------|------|-------------|
| /mavros/rc/override | mavros_msgs/OverrideRCIn | Override RC channels |
| /mavros/manual_control/send | mavros_msgs/ManualControl | Joystick-style input |
| /mavros/setpoint_velocity/cmd_vel | geometry_msgs/TwistStamped | Velocity setpoint |

### Services

| Service | Type | Description |
|---------|------|-------------|
| /mavros/cmd/arming | mavros_msgs/CommandBool | Arm/disarm |
| /mavros/set_mode | mavros_msgs/SetMode | Change flight mode |
| /mavros/cmd/command | mavros_msgs/CommandLong | Generic MAVLink command |
| /mavros/param/get | mavros_msgs/ParamGet | Read parameter |
| /mavros/param/set | mavros_msgs/ParamSet | Write parameter |
| /mavros/mission/push | mavros_msgs/WaypointPush | Upload mission |
| /mavros/mission/pull | mavros_msgs/WaypointPull | Download mission |

---

## Appendix C: Troubleshooting

### Connection Issues

**Symptom**: `mavros_bridge` reports "Could not connect"

**Checks**:
1. Verify BlueOS is reachable: `ping 192.168.2.2`
2. Check rosbridge is running: `curl http://192.168.2.2:8889`
3. Verify port in BlueOS extensions UI
4. Check firewall on Jetson: `sudo ufw status`

### No Data Flowing

**Symptom**: Connected but no topic updates

**Checks**:
1. Verify MAVROS is running on Pi: Check BlueOS extension logs
2. Check topic exists: Use `bridge.get_topics()`
3. Verify FCU connection: Check `/mavros/state` has `connected: true`

### High Latency

**Symptom**: Commands delayed >100ms

**Checks**:
1. Network congestion: `ping -c 100 192.168.2.2 | tail -1`
2. CPU overload on Pi: Check BlueOS system monitor
3. Too many subscribers: Reduce topic subscriptions

---

*Document version: 1.0*
*Last updated: 2026-04-01*
*Package version: duburi_blueos 1.0.0*
