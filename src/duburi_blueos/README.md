# duburi_blueos Package Documentation

## Overview

The `duburi_blueos` package provides ROS 2 integration with BlueOS running on a Raspberry Pi 4B companion computer. This enables monitoring, configuration, and management of BlueOS services from the Jetson Orin Nano.

## Network Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Network Switch                                │
└─────────────────────────────────────────────────────────────────┘
           │                                    │
           │                                    │
           ▼                                    ▼
┌─────────────────────────┐         ┌─────────────────────────────┐
│   Raspberry Pi 4B       │         │   Jetson Orin Nano          │
│   192.168.2.2           │         │   192.168.2.3               │
│                         │         │                             │
│   ┌─────────────────┐   │         │   ┌─────────────────────┐   │
│   │     BlueOS      │   │◄───────►│   │   duburi_blueos     │   │
│   │                 │   │  REST   │   │   ROS 2 package     │   │
│   │  - Web UI       │   │  API    │   │                     │   │
│   │  - REST API     │   │         │   │  - blueos_monitor   │   │
│   │  - MAVLink Hub  │   │         │   │  - mavlink_bridge   │   │
│   └────────┬────────┘   │         │   └─────────────────────┘   │
│            │            │         │             ▲               │
│            │            │         │             │               │
│   ┌────────▼────────┐   │         │   ┌─────────┴───────────┐   │
│   │  Pixhawk 2.4.8  │   │         │   │   mavlink_inspector │   │
│   │  ArduSub 4.5+   │   │◄────────┼───│   (pymavlink)       │   │
│   └─────────────────┘   │  MAVLink│   └─────────────────────┘   │
│                         │  UDP    │                             │
└─────────────────────────┘         └─────────────────────────────┘
```

## Package Components

### 1. BlueOS API Client (`blueos_api.py`)

Python client for the BlueOS REST API. Provides both synchronous and asynchronous interfaces.

**Key Features:**
- System information (CPU, memory, disk, temperature)
- MAVLink endpoint management
- Service discovery
- Network status
- ArduPilot/ArduSub management

**Usage:**
```python
from duburi_blueos import BlueOSAPI, MavlinkEndpoint, EndpointType

# Synchronous usage
api = BlueOSAPI("192.168.2.2")
info = api.get_system_info()
endpoints = api.get_mavlink_endpoints()

# Create UDP endpoint for Jetson
api.create_jetson_endpoint("192.168.2.3", port=14550)

# Asynchronous usage
async with BlueOSAPI("192.168.2.2") as api:
    info = await api.get_system_info_async()
```

**BlueOS Service Ports:**
| Port | Service |
|------|---------|
| 80 | Main web interface (NGINX) |
| 81 | Helper (service discovery) |
| 6030 | System Information |
| 6040 | MAVLink2Rest |
| 8000 | ArduPilot Manager |
| 9090 | Cable-guy (network) |

### 2. BlueOS Monitor Node (`blueos_monitor_node.py`)

ROS 2 node that polls BlueOS for system status and publishes diagnostics.

**Published Topics:**
- `/blueos/system_status` (`diagnostic_msgs/DiagnosticArray`) — System health
- `/blueos/connected` (`std_msgs/Bool`) — Connection status

**Services:**
- `/blueos/restart_autopilot` (`std_srvs/Trigger`) — Restart ArduPilot

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `blueos_host` | `192.168.2.2` | BlueOS IP address |
| `poll_rate_hz` | `1.0` | Status polling rate |
| `timeout_sec` | `2.0` | API request timeout |
| `warn_cpu_percent` | `80.0` | CPU warning threshold |
| `warn_memory_percent` | `80.0` | Memory warning threshold |
| `warn_disk_percent` | `90.0` | Disk warning threshold |
| `warn_temp_celsius` | `70.0` | Temperature warning threshold |

### 3. MAVLink Bridge Node (`mavlink_bridge_node.py`)

Manages MAVLink endpoints in BlueOS to route telemetry to the Jetson.

**On startup (with `auto_setup: true`):**
1. Connects to BlueOS REST API
2. Checks for existing Jetson endpoint
3. Creates UDP client endpoint if missing: `udpout:192.168.2.3:14550`
4. Verifies endpoint configuration

**Services:**
- `/blueos/setup_jetson_link` (`std_srvs/Trigger`) — Create/verify Jetson endpoint
- `/blueos/list_endpoints` (`std_srvs/Trigger`) — List all MAVLink endpoints
- `/blueos/remove_jetson_link` (`std_srvs/Trigger`) — Remove Jetson endpoint

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `blueos_host` | `192.168.2.2` | BlueOS IP address |
| `jetson_ip` | `192.168.2.3` | Jetson IP address |
| `mavlink_port` | `14550` | MAVLink UDP port |
| `auto_setup` | `true` | Auto-create endpoint on startup |
| `timeout_sec` | `5.0` | API request timeout |

## Launch

```bash
# Default configuration
ros2 launch duburi_blueos blueos.launch.py

# Custom IPs
ros2 launch duburi_blueos blueos.launch.py \
    blueos_host:=192.168.2.2 \
    jetson_ip:=192.168.2.3 \
    mavlink_port:=14550
```

## MAVLink Flow

After `mavlink_bridge` creates the endpoint, MAVLink flows as:

```
Pixhawk 2.4.8 (ArduSub)
        │
        │ Serial (TELEM1)
        ▼
BlueOS MAVLink Hub (mavlink-router)
        │
        │ UDP (192.168.2.3:14550)
        ▼
Jetson pymavlink (udpin:0.0.0.0:14550)
        │
        ▼
mavlink_inspector (ROS 2)
```

**Important Notes:**
1. The endpoint uses `udpout` (UDP client) from BlueOS's perspective
2. Jetson listens with `udpin:0.0.0.0:14550` to receive
3. This is a one-way link; for bidirectional MAVLink:
   - Add another endpoint for Jetson → BlueOS, OR
   - Use TCP connection instead

## Integration with mavlink_inspector

The `mavlink_inspector` package connects to MAVLink via pymavlink:

```python
# In mavlink_inspector, set connection_port parameter:
connection_port: "udpin:0.0.0.0:14550"
```

Or for direct connection through BlueOS TCP:
```python
connection_port: "tcp:192.168.2.2:5760"
```

## Diagnostics

Monitor BlueOS health via ROS 2 diagnostics:

```bash
# Check connection status
ros2 topic echo /blueos/connected

# View system diagnostics
ros2 topic echo /blueos/system_status

# List MAVLink endpoints
ros2 service call /blueos/list_endpoints std_srvs/srv/Trigger
```

## Troubleshooting

### Cannot reach BlueOS
1. Verify network connectivity: `ping 192.168.2.2`
2. Check BlueOS is running: `curl http://192.168.2.2:81/`
3. Verify firewall allows connections

### MAVLink not received on Jetson
1. Check endpoint exists: `ros2 service call /blueos/list_endpoints std_srvs/srv/Trigger`
2. Verify UDP port is not blocked: `netstat -ulnp | grep 14550`
3. Check for conflicting applications using the same port

### High latency diagnostics
1. Reduce `poll_rate_hz` to decrease load
2. Increase `timeout_sec` for slower networks
3. Check network switch for issues

## Dependencies

- `rclpy` — ROS 2 Python client
- `diagnostic_msgs` — Diagnostic messages
- `std_msgs`, `std_srvs` — Standard messages/services
- `requests` — Synchronous HTTP client
- `aiohttp` — Asynchronous HTTP client

## File Structure

```
duburi_blueos/
├── config/
│   └── blueos_config.yaml     # Default parameters
├── duburi_blueos/
│   ├── __init__.py            # Package exports
│   ├── blueos_api.py          # REST API client
│   ├── blueos_monitor_node.py # Monitor node
│   └── mavlink_bridge_node.py # Bridge node
├── launch/
│   └── blueos.launch.py       # Main launch file
├── resource/
│   └── duburi_blueos          # Ament marker
├── package.xml
├── setup.cfg
└── setup.py
```
