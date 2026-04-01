#!/usr/bin/env python3
"""
BlueOS REST API Client for BRACU Duburi AUV 4.2.

Provides a Python interface to BlueOS services running on Raspberry Pi 4B:
  - System information (CPU, memory, disk, temperature)
  - MAVLink endpoint management
  - Service discovery
  - Network status
  - ArduPilot/ArduSub management

BlueOS API ports (default):
  - 80: Main web interface (NGINX proxy)
  - 81: Helper service (service discovery)
  - 6030: System Information
  - 6040: MAVLink2Rest
  - 8000: ArduPilot Manager (MAVLink endpoints)
  - 9090: Cable-guy (network configuration)
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional

import aiohttp
import requests


class BlueOSError(Exception):
    """Exception raised for BlueOS API errors."""
    pass


class EndpointType(str, Enum):
    """Supported MAVLink endpoint types in BlueOS."""
    UDP_SERVER = "udpin"
    UDP_CLIENT = "udpout"
    TCP_SERVER = "tcpin"
    TCP_CLIENT = "tcpout"
    SERIAL = "serial"


@dataclass
class MavlinkEndpoint:
    """Represents a MAVLink endpoint configuration."""
    name: str
    owner: str
    connection_type: str
    place: str
    argument: int
    persistent: bool = False
    protected: bool = False
    enabled: bool = True

    def __str__(self) -> str:
        return f"{self.connection_type}:{self.place}:{self.argument}"

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'MavlinkEndpoint':
        return cls(
            name=data.get('name', ''),
            owner=data.get('owner', ''),
            connection_type=data.get('connection_type', ''),
            place=data.get('place', ''),
            argument=data.get('argument', 0),
            persistent=data.get('persistent', False),
            protected=data.get('protected', False),
            enabled=data.get('enabled', True),
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            'name': self.name,
            'owner': self.owner,
            'connection_type': self.connection_type,
            'place': self.place,
            'argument': self.argument,
            'persistent': self.persistent,
            'protected': self.protected,
            'enabled': self.enabled,
        }


@dataclass
class SystemInfo:
    """System information from BlueOS."""
    cpu_percent: float
    memory_percent: float
    disk_percent: float
    temperature_cpu: Optional[float]
    uptime_seconds: int
    blueos_version: str

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'SystemInfo':
        return cls(
            cpu_percent=data.get('cpu', {}).get('percent', 0.0),
            memory_percent=data.get('memory', {}).get('percent', 0.0),
            disk_percent=data.get('disk', {}).get('percent', 0.0),
            temperature_cpu=data.get('temperature', {}).get('cpu'),
            uptime_seconds=data.get('uptime', 0),
            blueos_version=data.get('blueos_version', 'unknown'),
        )


@dataclass
class ServiceInfo:
    """Information about a discovered BlueOS service."""
    title: str
    port: int
    valid: bool
    documentation_url: str
    versions: List[str]

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'ServiceInfo':
        return cls(
            title=data.get('title', 'Unknown'),
            port=data.get('port', 0),
            valid=data.get('valid', False),
            documentation_url=data.get('documentation_url', ''),
            versions=data.get('versions', []),
        )


class BlueOSAPI:
    """
    BlueOS REST API client.

    Provides synchronous and asynchronous methods for interacting with
    BlueOS services running on Raspberry Pi 4B.

    Usage:
        # Synchronous
        api = BlueOSAPI("192.168.2.2")
        info = api.get_system_info()

        # Asynchronous
        async with BlueOSAPI("192.168.2.2") as api:
            info = await api.get_system_info_async()
    """

    # BlueOS service ports
    PORT_HELPER = 81
    PORT_SYSTEM_INFO = 6030
    PORT_MAVLINK2REST = 6040
    PORT_ARDUPILOT_MANAGER = 8000
    PORT_CABLE_GUY = 9090

    def __init__(
        self,
        host: str = "192.168.2.2",
        timeout: float = 5.0,
    ):
        """
        Initialize BlueOS API client.

        Args:
            host: IP address or hostname of BlueOS (Raspberry Pi).
            timeout: Request timeout in seconds.
        """
        self.host = host
        self.timeout = timeout
        self._session: Optional[aiohttp.ClientSession] = None

    async def __aenter__(self) -> 'BlueOSAPI':
        self._session = aiohttp.ClientSession(
            timeout=aiohttp.ClientTimeout(total=self.timeout)
        )
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb) -> None:
        if self._session:
            await self._session.close()
            self._session = None

    def _url(self, port: int, path: str) -> str:
        """Build URL for a BlueOS service endpoint."""
        return f"http://{self.host}:{port}{path}"

    # ─────────────────────────────────────────────────────────────────
    # Synchronous API (using requests)
    # ─────────────────────────────────────────────────────────────────

    def _get(self, port: int, path: str) -> Any:
        """Make synchronous GET request."""
        try:
            response = requests.get(
                self._url(port, path),
                timeout=self.timeout,
            )
            response.raise_for_status()
            return response.json()
        except requests.RequestException as e:
            raise BlueOSError(f"GET {path} failed: {e}") from e

    def _post(self, port: int, path: str, data: Dict[str, Any]) -> Any:
        """Make synchronous POST request."""
        try:
            response = requests.post(
                self._url(port, path),
                json=data,
                timeout=self.timeout,
            )
            response.raise_for_status()
            return response.json() if response.content else None
        except requests.RequestException as e:
            raise BlueOSError(f"POST {path} failed: {e}") from e

    def _delete(self, port: int, path: str) -> Any:
        """Make synchronous DELETE request."""
        try:
            response = requests.delete(
                self._url(port, path),
                timeout=self.timeout,
            )
            response.raise_for_status()
            return response.json() if response.content else None
        except requests.RequestException as e:
            raise BlueOSError(f"DELETE {path} failed: {e}") from e

    # ── System Information ────────────────────────────────────────────

    def get_system_info(self) -> Dict[str, Any]:
        """
        Get system information from BlueOS.

        Returns CPU, memory, disk, temperature, and version info.
        """
        return self._get(self.PORT_SYSTEM_INFO, "/system")

    def get_cpu_info(self) -> Dict[str, Any]:
        """Get CPU usage and info."""
        return self._get(self.PORT_SYSTEM_INFO, "/system/cpu")

    def get_memory_info(self) -> Dict[str, Any]:
        """Get memory usage info."""
        return self._get(self.PORT_SYSTEM_INFO, "/system/memory")

    def get_disk_info(self) -> Dict[str, Any]:
        """Get disk usage info."""
        return self._get(self.PORT_SYSTEM_INFO, "/system/disk")

    def get_temperature_info(self) -> Dict[str, Any]:
        """Get system temperatures."""
        return self._get(self.PORT_SYSTEM_INFO, "/system/temperature")

    # ── Service Discovery ─────────────────────────────────────────────

    def get_services(self) -> List[ServiceInfo]:
        """
        Get list of running BlueOS services.

        Uses the Helper service to discover all available web services.
        """
        data = self._get(self.PORT_HELPER, "/v1.0/web_services")
        return [ServiceInfo.from_dict(s) for s in data]

    def check_internet_access(self) -> Dict[str, Any]:
        """Check internet connectivity from BlueOS."""
        return self._get(self.PORT_HELPER, "/v1.0/check_internet_access")

    # ── MAVLink Endpoints ─────────────────────────────────────────────

    def get_mavlink_endpoints(self) -> List[MavlinkEndpoint]:
        """
        Get configured MAVLink endpoints.

        Returns list of all MAVLink endpoints (UDP, TCP, Serial).
        """
        # BlueOS requires trailing slash on this endpoint
        data = self._get(self.PORT_ARDUPILOT_MANAGER, "/v1.0/endpoints/")
        return [MavlinkEndpoint.from_dict(e) for e in data]

    def add_mavlink_endpoint(self, endpoint: MavlinkEndpoint) -> None:
        """
        Add a new MAVLink endpoint.

        Args:
            endpoint: Endpoint configuration to add.
        """
        self._post(
            self.PORT_ARDUPILOT_MANAGER,
            "/v1.0/endpoints/",
            endpoint.to_dict(),
        )

    def remove_mavlink_endpoint(self, endpoint: MavlinkEndpoint) -> None:
        """
        Remove a MAVLink endpoint.

        Args:
            endpoint: Endpoint to remove.
        """
        self._delete(
            self.PORT_ARDUPILOT_MANAGER,
            f"/v1.0/endpoints?name={endpoint.name}",
        )

    def create_jetson_endpoint(
        self,
        jetson_ip: str,
        port: int = 14550,
        name: str = "Jetson-MAVLink",
    ) -> MavlinkEndpoint:
        """
        Create a UDP endpoint for Jetson companion computer.

        This creates a UDP client endpoint that sends MAVLink to the Jetson.

        Args:
            jetson_ip: IP address of Jetson Orin Nano.
            port: UDP port (default 14550).
            name: Endpoint name for identification.

        Returns:
            The created endpoint configuration.
        """
        endpoint = MavlinkEndpoint(
            name=name,
            owner="duburi_blueos",
            connection_type=EndpointType.UDP_CLIENT.value,
            place=jetson_ip,
            argument=port,
            persistent=True,
            protected=False,
            enabled=True,
        )
        self.add_mavlink_endpoint(endpoint)
        return endpoint

    # ── Vehicle / ArduPilot Status ────────────────────────────────────

    def get_vehicle_info(self) -> Dict[str, Any]:
        """Get connected vehicle/autopilot information."""
        return self._get(self.PORT_ARDUPILOT_MANAGER, "/v1.0/vehicle")

    def get_firmware_info(self) -> Dict[str, Any]:
        """Get autopilot firmware information."""
        return self._get(self.PORT_ARDUPILOT_MANAGER, "/v1.0/firmware_info")

    def get_board_info(self) -> Dict[str, Any]:
        """Get flight controller board information."""
        return self._get(self.PORT_ARDUPILOT_MANAGER, "/v1.0/board")

    def restart_autopilot(self) -> None:
        """Restart the ArduPilot process."""
        self._post(self.PORT_ARDUPILOT_MANAGER, "/v1.0/restart", {})

    # ── Network Configuration ─────────────────────────────────────────

    def get_network_interfaces(self) -> List[Dict[str, Any]]:
        """Get network interface configurations."""
        return self._get(self.PORT_CABLE_GUY, "/v1.0/ethernet")

    def get_network_status(self) -> Dict[str, Any]:
        """Get network status information."""
        return self._get(self.PORT_CABLE_GUY, "/v1.0/status")

    # ── MAVLink2Rest API ──────────────────────────────────────────────

    def get_mavlink_message(self, message_type: str) -> Dict[str, Any]:
        """
        Get a specific MAVLink message from MAVLink2Rest.

        Args:
            message_type: Message type name (e.g., "HEARTBEAT", "ATTITUDE").

        Returns:
            Latest message data.
        """
        return self._get(
            self.PORT_MAVLINK2REST,
            f"/mavlink/vehicles/1/components/1/messages/{message_type}",
        )

    def get_vehicle_state(self) -> Dict[str, Any]:
        """Get vehicle state from MAVLink2Rest."""
        return self._get(self.PORT_MAVLINK2REST, "/mavlink/vehicles/1/state")

    # ─────────────────────────────────────────────────────────────────
    # Asynchronous API (using aiohttp)
    # ─────────────────────────────────────────────────────────────────

    async def _get_async(self, port: int, path: str) -> Any:
        """Make asynchronous GET request."""
        if not self._session:
            raise BlueOSError("Session not initialized. Use 'async with' context.")
        try:
            async with self._session.get(self._url(port, path)) as response:
                response.raise_for_status()
                return await response.json()
        except aiohttp.ClientError as e:
            raise BlueOSError(f"GET {path} failed: {e}") from e

    async def _post_async(self, port: int, path: str, data: Dict[str, Any]) -> Any:
        """Make asynchronous POST request."""
        if not self._session:
            raise BlueOSError("Session not initialized. Use 'async with' context.")
        try:
            async with self._session.post(self._url(port, path), json=data) as response:
                response.raise_for_status()
                if response.content_length:
                    return await response.json()
                return None
        except aiohttp.ClientError as e:
            raise BlueOSError(f"POST {path} failed: {e}") from e

    async def get_system_info_async(self) -> Dict[str, Any]:
        """Get system information asynchronously."""
        return await self._get_async(self.PORT_SYSTEM_INFO, "/system")

    async def get_services_async(self) -> List[ServiceInfo]:
        """Get list of running services asynchronously."""
        data = await self._get_async(self.PORT_HELPER, "/v1.0/web_services")
        return [ServiceInfo.from_dict(s) for s in data]

    async def get_mavlink_endpoints_async(self) -> List[MavlinkEndpoint]:
        """Get MAVLink endpoints asynchronously."""
        data = await self._get_async(self.PORT_ARDUPILOT_MANAGER, "/v1.0/endpoints")
        return [MavlinkEndpoint.from_dict(e) for e in data]

    # ─────────────────────────────────────────────────────────────────
    # Utility / Health Check
    # ─────────────────────────────────────────────────────────────────

    def is_reachable(self) -> bool:
        """Check if BlueOS is reachable."""
        try:
            # Use web_services endpoint which returns JSON
            response = requests.get(
                self._url(self.PORT_HELPER, "/v1.0/web_services"),
                timeout=self.timeout,
            )
            return response.status_code == 200
        except requests.RequestException:
            return False

    async def is_reachable_async(self) -> bool:
        """Check if BlueOS is reachable asynchronously."""
        if not self._session:
            raise BlueOSError("Session not initialized. Use 'async with' context.")
        try:
            async with self._session.get(
                self._url(self.PORT_HELPER, "/v1.0/web_services")
            ) as response:
                return response.status == 200
        except aiohttp.ClientError:
            return False
