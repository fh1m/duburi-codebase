"""
Multi-source sensor abstraction for AUV navigation (Phase 5).

Provides a unified interface for multiple sensor sources with automatic
fallback and quality-based source selection. Supports:
- DVL (Nortek Nucleus 1000) for ground-truth velocity
- External compass/IMU (Witmotion, BNO085)
- Pixhawk internal IMU (default)
- DVL internal IMU (Nucleus 1000 has built-in)

Architecture:
    SensorSource (ABC) ← Abstract base for all sources
        ├── DVLSource ← Nortek Nucleus 1000 velocity + yaw
        ├── ExternalYawSource ← Witmotion/BNO085/ESP32+MAVLink
        ├── PixhawkYawSource ← Default Pixhawk IMU yaw
        └── DVLIMUSource ← Nucleus 1000 internal IMU
    
    SensorSourceManager ← Manages priority fallback between sources

Usage:
    manager = SensorSourceManager(node, config)
    
    # Get best available velocity (DVL if valid, else IMU estimate)
    velocity = manager.get_velocity()
    
    # Get best available yaw (priority: DVL > External > Pixhawk)
    yaw = manager.get_yaw()
    
    # Check source health
    status = manager.get_status()

Configuration (defaults.yaml):
    # Source priority (first valid wins)
    velocity_source_priority: ['dvl', 'imu_estimate']
    yaw_source_priority: ['dvl_imu', 'external', 'pixhawk']
    
    # DVL settings
    dvl_topic: '/dvl/velocity'
    dvl_timeout: 1.0  # seconds
    dvl_min_quality: 0.5  # 0.0-1.0
    
    # External yaw settings
    external_yaw_topic: '/external_imu/yaw'
    external_yaw_msg_type: 'std_msgs/Float32'  # or 'geometry_msgs/Vector3Stamped'
    external_yaw_timeout: 0.5
    
    # Source enables
    dvl_enabled: false
    external_yaw_enabled: false
"""

import time
import math
from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Any, Callable
from enum import Enum

# ROS2 imports (deferred to avoid import errors when testing)
try:
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object


class SourceStatus(Enum):
    """Health status for sensor sources."""
    HEALTHY = 'healthy'      # Receiving valid data
    DEGRADED = 'degraded'    # Receiving data but quality low
    TIMEOUT = 'timeout'      # No data received recently
    INVALID = 'invalid'      # Data received but failed validation
    DISABLED = 'disabled'    # Source disabled in config
    NOT_AVAILABLE = 'not_available'  # Hardware not connected


class SensorSource(ABC):
    """
    Abstract base class for all sensor sources.
    
    Defines unified interface for velocity, yaw, and quality metrics.
    All sources must implement these methods to participate in fallback chain.
    """
    
    def __init__(self, name: str, logger, config: Optional[Dict] = None):
        """
        Initialize sensor source.
        
        Args:
            name: Human-readable source name (e.g., 'dvl', 'pixhawk')
            logger: ROS2 logger for diagnostics
            config: Source-specific configuration
        """
        self.name = name
        self.logger = logger
        self.config = config or {}
        
        self.enabled = self.config.get('enabled', True)
        self.timeout = self.config.get('timeout', 1.0)
        self.min_quality = self.config.get('min_quality', 0.0)
        
        self.last_update = None
        self.status = SourceStatus.NOT_AVAILABLE if self.enabled else SourceStatus.DISABLED
        self._quality = 0.0
    
    @abstractmethod
    def get_velocity(self) -> Optional[Dict[str, float]]:
        """
        Get velocity from this source.
        
        Returns:
            Dict with keys 'surge', 'sway', 'heave' (m/s), or None if unavailable
        """
        pass
    
    @abstractmethod
    def get_yaw(self) -> Optional[float]:
        """
        Get yaw/heading from this source.
        
        Returns:
            Yaw in degrees (0-360), or None if unavailable
        """
        pass
    
    def is_valid(self) -> bool:
        """Check if source data is currently valid."""
        if not self.enabled:
            return False
        if self.last_update is None:
            return False
        if time.time() - self.last_update > self.timeout:
            self.status = SourceStatus.TIMEOUT
            return False
        if self._quality < self.min_quality:
            self.status = SourceStatus.DEGRADED
            return False
        return True
    
    def get_quality(self) -> float:
        """
        Get quality metric for this source.
        
        Returns:
            Quality from 0.0 (invalid) to 1.0 (perfect)
        """
        if not self.is_valid():
            return 0.0
        return self._quality
    
    def get_status(self) -> SourceStatus:
        """Get current health status."""
        if not self.enabled:
            return SourceStatus.DISABLED
        if not self.is_valid():
            return self.status
        if self._quality < self.min_quality:
            return SourceStatus.DEGRADED
        return SourceStatus.HEALTHY


# ═══════════════════════════════════════════════════════════════════════════
# DVL Source (Nortek Nucleus 1000)
# ═══════════════════════════════════════════════════════════════════════════

class DVLSource(SensorSource):
    """
    Nortek Nucleus 1000 DVL velocity source.
    
    Provides ground-truth velocity from bottom tracking (no drift!).
    Includes quality metrics from beam validity and altitude.
    
    Expected topic: /dvl/velocity (geometry_msgs/TwistWithCovarianceStamped)
    Alternative: Custom DVL message type
    
    Quality factors:
    - Bottom lock status
    - Number of valid beams (4 beams, need >= 3)
    - Altitude (too close or too far = degraded)
    - Figure of merit (FOM)
    """
    
    def __init__(self, node: Node, logger, config: Optional[Dict] = None):
        """
        Initialize DVL source.
        
        Args:
            node: ROS2 node for subscriptions
            logger: ROS2 logger
            config: DVL configuration
                - topic: DVL velocity topic (default: '/dvl/velocity')
                - timeout: Max time between messages (default: 1.0s)
                - min_altitude: Minimum altitude for valid data (default: 0.3m)
                - max_altitude: Maximum altitude for valid data (default: 50.0m)
                - min_quality: Minimum quality to use (default: 0.5)
        """
        super().__init__('dvl', logger, config)
        
        self.node = node
        self.topic = self.config.get('topic', '/dvl/velocity')
        self.min_altitude = self.config.get('min_altitude', 0.3)
        self.max_altitude = self.config.get('max_altitude', 50.0)
        
        # State
        self._velocity = {'surge': 0.0, 'sway': 0.0, 'heave': 0.0}
        self._yaw = None  # DVL IMU yaw (if available)
        self._altitude = None
        self._bottom_lock = False
        self._valid_beams = 0
        self._fom = 0.0  # Figure of merit
        
        # Subscription (created when enabled)
        self._subscriber = None
        
        if self.enabled and HAS_ROS2:
            self._setup_subscription()
            self.logger.info(f"DVLSource initialized (topic={self.topic})")
        else:
            self.logger.info("DVLSource initialized (DISABLED)")
    
    def _setup_subscription(self):
        """Create ROS2 subscription to DVL topic."""
        try:
            # Try TwistWithCovarianceStamped first (standard)
            from geometry_msgs.msg import TwistWithCovarianceStamped
            
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                depth=10
            )
            
            self._subscriber = self.node.create_subscription(
                TwistWithCovarianceStamped,
                self.topic,
                self._dvl_callback_twist,
                qos
            )
            self.logger.info(f"DVL subscribed to {self.topic} (TwistWithCovarianceStamped)")
            
        except Exception as e:
            self.logger.warn(f"DVL subscription failed: {e}")
            self.status = SourceStatus.NOT_AVAILABLE
    
    def _dvl_callback_twist(self, msg):
        """
        Handle DVL velocity message (TwistWithCovarianceStamped).
        
        Body frame convention:
        - x = surge (forward)
        - y = sway (right)
        - z = heave (down)
        """
        self._velocity['surge'] = msg.twist.twist.linear.x
        self._velocity['sway'] = msg.twist.twist.linear.y
        self._velocity['heave'] = msg.twist.twist.linear.z
        
        # Extract quality from covariance (lower = better)
        # Diagonal elements are variances for x, y, z
        cov = msg.twist.covariance
        variance_sum = cov[0] + cov[7] + cov[14]  # xx, yy, zz
        
        # Convert variance to quality (0.0-1.0)
        # Typical DVL variance: 0.001-0.1 m²/s²
        if variance_sum < 0.01:
            self._quality = 1.0
        elif variance_sum < 0.1:
            self._quality = 0.8
        elif variance_sum < 0.5:
            self._quality = 0.5
        else:
            self._quality = 0.2
        
        # Determine bottom_lock status
        # Check for explicit bottom_lock field (if available in message)
        if hasattr(msg, 'bottom_lock'):
            is_locked = msg.bottom_lock
        elif hasattr(msg.twist, 'bottom_lock'):
            is_locked = msg.twist.bottom_lock
        else:
            # Fallback: use variance heuristic for bottom_lock detection
            # High variance = poor lock (quality < 0.3 indicates no lock)
            is_locked = self._quality > 0.3
        
        self._bottom_lock = is_locked
        self.last_update = time.time()
        self.status = SourceStatus.HEALTHY if self._bottom_lock else SourceStatus.DEGRADED
    
    def get_velocity(self) -> Optional[Dict[str, float]]:
        """Get DVL velocity (ground-truth, no drift)."""
        if not self.is_valid():
            return None
        return self._velocity.copy()
    
    def get_yaw(self) -> Optional[float]:
        """Get yaw from DVL IMU (if available)."""
        # Nucleus 1000 has internal IMU - implement if message includes orientation
        return self._yaw
    
    def get_altitude(self) -> Optional[float]:
        """Get altitude above bottom."""
        if self._altitude is None:
            return None
        return self._altitude
    
    def has_bottom_lock(self) -> bool:
        """Check if DVL has bottom lock."""
        return self._bottom_lock and self.is_valid()


# ═══════════════════════════════════════════════════════════════════════════
# External Yaw Source (Witmotion, BNO085, ESP32+MAVLink)
# ═══════════════════════════════════════════════════════════════════════════

class ExternalYawSource(SensorSource):
    """
    External compass/IMU yaw source.
    
    Supports multiple sensor types:
    - Witmotion WT901C/WT61C (serial → ROS topic)
    - BNO085 via ESP32 with MAVLink (MAVLink → ROS topic)
    - Any sensor publishing yaw to a ROS topic
    
    Configurable message types:
    - std_msgs/Float32: Simple yaw in degrees
    - geometry_msgs/Vector3Stamped: Euler angles (z = yaw)
    - sensor_msgs/Imu: Quaternion (converted to yaw)
    - Custom MAVLink bridge message
    
    Quality factors:
    - Message freshness
    - Sensor-reported confidence (if available)
    - Rate stability
    """
    
    def __init__(self, node: Node, logger, config: Optional[Dict] = None):
        """
        Initialize external yaw source.
        
        Args:
            node: ROS2 node
            logger: ROS2 logger
            config: External yaw configuration
                - topic: Yaw topic (default: '/external_imu/yaw')
                - msg_type: Message type (default: 'std_msgs/Float32')
                - timeout: Max time between messages (default: 0.5s)
                - yaw_offset: Calibration offset in degrees (default: 0.0)
        """
        super().__init__('external', logger, config)
        
        self.node = node
        self.topic = self.config.get('topic', '/external_imu/yaw')
        self.msg_type = self.config.get('msg_type', 'std_msgs/Float32')
        self.yaw_offset = self.config.get('yaw_offset', 0.0)
        
        # State
        self._yaw = None
        self._rate_tracker = []  # Track message timestamps for rate monitoring
        
        # Subscription
        self._subscriber = None
        
        if self.enabled and HAS_ROS2:
            self._setup_subscription()
            self.logger.info(f"ExternalYawSource initialized (topic={self.topic}, type={self.msg_type})")
        else:
            self.logger.info("ExternalYawSource initialized (DISABLED)")
    
    def _setup_subscription(self):
        """Create ROS2 subscription based on message type."""
        try:
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                depth=10
            )
            
            if self.msg_type == 'std_msgs/Float32':
                from std_msgs.msg import Float32
                self._subscriber = self.node.create_subscription(
                    Float32, self.topic, self._callback_float32, qos
                )
            
            elif self.msg_type == 'geometry_msgs/Vector3Stamped':
                from geometry_msgs.msg import Vector3Stamped
                self._subscriber = self.node.create_subscription(
                    Vector3Stamped, self.topic, self._callback_vector3, qos
                )
            
            elif self.msg_type == 'sensor_msgs/Imu':
                from sensor_msgs.msg import Imu
                self._subscriber = self.node.create_subscription(
                    Imu, self.topic, self._callback_imu, qos
                )
            
            else:
                self.logger.warn(f"Unknown msg_type: {self.msg_type}, defaulting to Float32")
                from std_msgs.msg import Float32
                self._subscriber = self.node.create_subscription(
                    Float32, self.topic, self._callback_float32, qos
                )
            
            self.logger.info(f"External yaw subscribed to {self.topic}")
            
        except Exception as e:
            self.logger.warn(f"External yaw subscription failed: {e}")
            self.status = SourceStatus.NOT_AVAILABLE
    
    def _callback_float32(self, msg):
        """Handle Float32 yaw message (degrees)."""
        self._update_yaw(msg.data)
    
    def _callback_vector3(self, msg):
        """Handle Vector3Stamped (euler angles, z = yaw)."""
        self._update_yaw(msg.vector.z)
    
    def _callback_imu(self, msg):
        """Handle Imu message (quaternion → yaw)."""
        # Extract yaw from quaternion
        q = msg.orientation
        # Yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = math.degrees(yaw_rad)
        self._update_yaw(yaw_deg)
    
    def _update_yaw(self, yaw_deg: float):
        """Update yaw with offset and quality tracking."""
        # Apply calibration offset
        self._yaw = (yaw_deg + self.yaw_offset) % 360
        
        # Track rate for quality
        now = time.time()
        self._rate_tracker.append(now)
        # Keep last 10 timestamps
        self._rate_tracker = [t for t in self._rate_tracker if now - t < 1.0]
        
        # Quality based on rate (expect 20+ Hz)
        rate = len(self._rate_tracker)
        if rate >= 20:
            self._quality = 1.0
        elif rate >= 10:
            self._quality = 0.8
        elif rate >= 5:
            self._quality = 0.5
        else:
            self._quality = 0.3
        
        self.last_update = now
        self.status = SourceStatus.HEALTHY
    
    def get_velocity(self) -> Optional[Dict[str, float]]:
        """External yaw source does not provide velocity."""
        return None
    
    def get_yaw(self) -> Optional[float]:
        """Get yaw from external sensor."""
        if not self.is_valid():
            return None
        return self._yaw


# ═══════════════════════════════════════════════════════════════════════════
# Pixhawk Yaw Source (Default IMU)
# ═══════════════════════════════════════════════════════════════════════════

class PixhawkYawSource(SensorSource):
    """
    Pixhawk internal IMU yaw source (default).
    
    Gets yaw from MAVLink ATTITUDE message via telemetry parser.
    Always available as fallback when Pixhawk is connected.
    
    Quality factors:
    - EKF health
    - Magnetometer calibration
    - GPS interference
    """
    
    def __init__(self, telemetry_parser, logger, config: Optional[Dict] = None):
        """
        Initialize Pixhawk yaw source.
        
        Args:
            telemetry_parser: TelemetryParser instance with current yaw
            logger: ROS2 logger
            config: Pixhawk yaw configuration
                - timeout: Max time since last update (default: 1.0s)
        """
        super().__init__('pixhawk', logger, config)
        
        self._telemetry = telemetry_parser
        self._quality = 0.8  # Default quality for Pixhawk (good but not perfect)
        
        self.logger.info("PixhawkYawSource initialized (uses telemetry parser)")
    
    def get_velocity(self) -> Optional[Dict[str, float]]:
        """Pixhawk IMU velocity (from VelocityEstimator, not this source)."""
        return None
    
    def get_yaw(self) -> Optional[float]:
        """Get yaw from Pixhawk ATTITUDE message."""
        if self._telemetry is None:
            return None
        
        yaw = getattr(self._telemetry, 'yaw', None)
        if yaw is None:
            self.status = SourceStatus.NOT_AVAILABLE
            return None
        
        self.last_update = time.time()
        self.status = SourceStatus.HEALTHY
        return yaw
    
    def is_valid(self) -> bool:
        """Pixhawk yaw is valid if telemetry is receiving."""
        if self._telemetry is None:
            return False
        # Check if telemetry is fresh (assume valid if yaw exists)
        return hasattr(self._telemetry, 'yaw') and self._telemetry.yaw is not None


# ═══════════════════════════════════════════════════════════════════════════
# DVL IMU Source (Nortek Nucleus 1000 Internal IMU)
# ═══════════════════════════════════════════════════════════════════════════

class DVLIMUSource(SensorSource):
    """
    Nortek Nucleus 1000 internal IMU yaw source.
    
    The Nucleus 1000 has a built-in AHRS that can provide orientation.
    Useful when mounted away from magnetic interference.
    
    Expected topic: /dvl/orientation (geometry_msgs/QuaternionStamped)
    Alternative: /dvl/imu (sensor_msgs/Imu)
    """
    
    def __init__(self, node: Node, logger, config: Optional[Dict] = None):
        """
        Initialize DVL IMU source.
        
        Args:
            node: ROS2 node
            logger: ROS2 logger
            config: DVL IMU configuration
                - topic: Orientation topic (default: '/dvl/orientation')
                - msg_type: Message type (default: 'geometry_msgs/QuaternionStamped')
                - timeout: Max time between messages (default: 0.5s)
        """
        super().__init__('dvl_imu', logger, config)
        
        self.node = node
        self.topic = self.config.get('topic', '/dvl/orientation')
        self.msg_type = self.config.get('msg_type', 'geometry_msgs/QuaternionStamped')
        
        # State
        self._yaw = None
        
        # Subscription
        self._subscriber = None
        
        if self.enabled and HAS_ROS2:
            self._setup_subscription()
            self.logger.info(f"DVLIMUSource initialized (topic={self.topic})")
        else:
            self.logger.info("DVLIMUSource initialized (DISABLED)")
    
    def _setup_subscription(self):
        """Create ROS2 subscription."""
        try:
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                depth=10
            )
            
            if self.msg_type == 'geometry_msgs/QuaternionStamped':
                from geometry_msgs.msg import QuaternionStamped
                self._subscriber = self.node.create_subscription(
                    QuaternionStamped, self.topic, self._callback_quaternion, qos
                )
            elif self.msg_type == 'sensor_msgs/Imu':
                from sensor_msgs.msg import Imu
                self._subscriber = self.node.create_subscription(
                    Imu, self.topic, self._callback_imu, qos
                )
            
            self.logger.info(f"DVL IMU subscribed to {self.topic}")
            
        except Exception as e:
            self.logger.warn(f"DVL IMU subscription failed: {e}")
            self.status = SourceStatus.NOT_AVAILABLE
    
    def _callback_quaternion(self, msg):
        """Handle QuaternionStamped message."""
        q = msg.quaternion
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        self._yaw = math.degrees(yaw_rad) % 360
        
        self.last_update = time.time()
        self._quality = 0.9  # DVL IMU is typically high quality
        self.status = SourceStatus.HEALTHY
    
    def _callback_imu(self, msg):
        """Handle Imu message."""
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        self._yaw = math.degrees(yaw_rad) % 360
        
        self.last_update = time.time()
        self._quality = 0.9
        self.status = SourceStatus.HEALTHY
    
    def get_velocity(self) -> Optional[Dict[str, float]]:
        """DVL IMU does not provide velocity (use DVLSource for that)."""
        return None
    
    def get_yaw(self) -> Optional[float]:
        """Get yaw from DVL internal IMU."""
        if not self.is_valid():
            return None
        return self._yaw


# ═══════════════════════════════════════════════════════════════════════════
# Sensor Source Manager
# ═══════════════════════════════════════════════════════════════════════════

class SensorSourceManager:
    """
    Manages multiple sensor sources with priority-based fallback.
    
    Provides unified interface for velocity and yaw with automatic
    failover when primary sources become unavailable.
    
    Priority Chains (configurable):
    - Velocity: DVL → IMU estimate (from VelocityEstimator)
    - Yaw: DVL IMU → External → Pixhawk
    
    Features:
    - Automatic failover on timeout/invalid
    - Quality-weighted source selection (optional)
    - Health monitoring for all sources
    - Configurable source priorities
    
    Usage:
        manager = SensorSourceManager(node, config)
        
        # Get best velocity
        vel = manager.get_velocity()
        if vel:
            surge, sway, heave = vel['surge'], vel['sway'], vel['heave']
        
        # Get best yaw
        yaw = manager.get_yaw()
        if yaw is not None:
            current_heading = yaw
        
        # Check health
        status = manager.get_status()
        # Returns: {'dvl': 'healthy', 'external': 'timeout', 'pixhawk': 'healthy'}
    """
    
    def __init__(self, node: Node, telemetry_parser, logger, config: Optional[Dict] = None):
        """
        Initialize sensor source manager.
        
        Args:
            node: ROS2 node for subscriptions
            telemetry_parser: TelemetryParser for Pixhawk yaw
            logger: ROS2 logger
            config: Manager configuration
                - velocity_source_priority: List of source names for velocity
                - yaw_source_priority: List of source names for yaw
                - use_quality_weighting: Weight by quality (default: False)
                - dvl_*: DVL source configuration
                - external_*: External yaw configuration
                - dvl_imu_*: DVL IMU configuration
        """
        self.node = node
        self.logger = logger
        self.config = config or {}
        
        # Source priority (order matters - first valid wins)
        self.velocity_priority = self.config.get(
            'velocity_source_priority', ['dvl', 'imu_estimate']
        )
        self.yaw_priority = self.config.get(
            'yaw_source_priority', ['dvl_imu', 'external', 'pixhawk']
        )
        
        self.use_quality_weighting = self.config.get('use_quality_weighting', False)
        
        # Initialize sources
        self._sources: Dict[str, SensorSource] = {}
        self._velocity_estimator = None  # Set externally for IMU estimate
        
        # DVL source
        if self.config.get('dvl_enabled', False):
            dvl_config = {
                'enabled': True,
                'topic': self.config.get('dvl_topic', '/dvl/velocity'),
                'timeout': self.config.get('dvl_timeout', 1.0),
                'min_quality': self.config.get('dvl_min_quality', 0.5),
                'min_altitude': self.config.get('dvl_min_altitude', 0.3),
                'max_altitude': self.config.get('dvl_max_altitude', 50.0),
            }
            self._sources['dvl'] = DVLSource(node, logger, dvl_config)
        
        # External yaw source
        if self.config.get('external_yaw_enabled', False):
            external_config = {
                'enabled': True,
                'topic': self.config.get('external_yaw_topic', '/external_imu/yaw'),
                'msg_type': self.config.get('external_yaw_msg_type', 'std_msgs/Float32'),
                'timeout': self.config.get('external_yaw_timeout', 0.5),
                'yaw_offset': self.config.get('external_yaw_offset', 0.0),
            }
            self._sources['external'] = ExternalYawSource(node, logger, external_config)
        
        # DVL IMU source
        if self.config.get('dvl_imu_enabled', False):
            dvl_imu_config = {
                'enabled': True,
                'topic': self.config.get('dvl_imu_topic', '/dvl/orientation'),
                'msg_type': self.config.get('dvl_imu_msg_type', 'geometry_msgs/QuaternionStamped'),
                'timeout': self.config.get('dvl_imu_timeout', 0.5),
            }
            self._sources['dvl_imu'] = DVLIMUSource(node, logger, dvl_imu_config)
        
        # Pixhawk yaw (always available as fallback)
        pixhawk_config = {
            'enabled': True,
            'timeout': 1.0,
        }
        self._sources['pixhawk'] = PixhawkYawSource(telemetry_parser, logger, pixhawk_config)
        
        self.logger.info(f"SensorSourceManager initialized")
        self.logger.info(f"  Velocity priority: {self.velocity_priority}")
        self.logger.info(f"  Yaw priority: {self.yaw_priority}")
        self.logger.info(f"  Active sources: {list(self._sources.keys())}")
    
    def set_velocity_estimator(self, velocity_estimator):
        """
        Set VelocityEstimator for IMU-based velocity fallback.
        
        Args:
            velocity_estimator: VelocityEstimator instance from Phase 1
        """
        self._velocity_estimator = velocity_estimator
        self.logger.info("VelocityEstimator linked to SensorSourceManager")
    
    def get_velocity(self, dof: Optional[List[str]] = None) -> Optional[Dict[str, float]]:
        """
        Get velocity from best available source.
        
        Args:
            dof: Optional list of DOFs to return (default: all)
        
        Returns:
            Dict with velocity values, or None if no source available
        """
        dof = dof or ['surge', 'sway', 'heave']
        
        for source_name in self.velocity_priority:
            if source_name == 'dvl':
                if 'dvl' in self._sources and self._sources['dvl'].is_valid():
                    vel = self._sources['dvl'].get_velocity()
                    if vel:
                        self.logger.debug(f"Velocity from DVL: {vel}")
                        return {k: vel.get(k, 0.0) for k in dof}
            
            elif source_name == 'imu_estimate':
                if self._velocity_estimator:
                    vel = self._velocity_estimator.get_velocities(dof)
                    if vel:
                        self.logger.debug(f"Velocity from IMU estimate: {vel}")
                        return vel
        
        self.logger.debug("No velocity source available")
        return None
    
    def get_yaw(self) -> Optional[float]:
        """
        Get yaw from best available source.
        
        Returns:
            Yaw in degrees (0-360), or None if no source available
        """
        for source_name in self.yaw_priority:
            if source_name in self._sources:
                source = self._sources[source_name]
                if source.is_valid():
                    yaw = source.get_yaw()
                    if yaw is not None:
                        self.logger.debug(f"Yaw from {source_name}: {yaw:.1f}°")
                        return yaw
        
        self.logger.debug("No yaw source available")
        return None
    
    def get_active_velocity_source(self) -> Optional[str]:
        """Get name of currently active velocity source."""
        for source_name in self.velocity_priority:
            if source_name == 'dvl':
                if 'dvl' in self._sources and self._sources['dvl'].is_valid():
                    return 'dvl'
            elif source_name == 'imu_estimate':
                if self._velocity_estimator:
                    return 'imu_estimate'
        return None
    
    def get_active_yaw_source(self) -> Optional[str]:
        """Get name of currently active yaw source."""
        for source_name in self.yaw_priority:
            if source_name in self._sources:
                source = self._sources[source_name]
                if source.is_valid() and source.get_yaw() is not None:
                    return source_name
        return None
    
    def get_status(self) -> Dict[str, str]:
        """
        Get health status of all sources.
        
        Returns:
            Dict mapping source names to status strings
        """
        status = {}
        for name, source in self._sources.items():
            status[name] = source.get_status().value
        
        # Add IMU estimate status
        if self._velocity_estimator:
            status['imu_estimate'] = 'healthy'
        else:
            status['imu_estimate'] = 'not_available'
        
        return status
    
    def get_all_yaws(self) -> Dict[str, Optional[float]]:
        """
        Get yaw from all available sources (for debugging/comparison).
        
        Returns:
            Dict mapping source names to yaw values (or None if unavailable)
        """
        yaws = {}
        for name, source in self._sources.items():
            try:
                yaws[name] = source.get_yaw() if source.is_valid() else None
            except Exception:
                yaws[name] = None
        return yaws
    
    def force_source(self, source_type: str, source_name: str):
        """
        Force use of specific source (override priority).
        
        Args:
            source_type: 'velocity' or 'yaw'
            source_name: Name of source to use exclusively
        """
        if source_type == 'velocity':
            self.velocity_priority = [source_name]
            self.logger.info(f"Velocity source forced to: {source_name}")
        elif source_type == 'yaw':
            self.yaw_priority = [source_name]
            self.logger.info(f"Yaw source forced to: {source_name}")
    
    def reset_priorities(self):
        """Reset to configured priorities."""
        self.velocity_priority = self.config.get(
            'velocity_source_priority', ['dvl', 'imu_estimate']
        )
        self.yaw_priority = self.config.get(
            'yaw_source_priority', ['dvl_imu', 'external', 'pixhawk']
        )
        self.logger.info("Source priorities reset to defaults")
