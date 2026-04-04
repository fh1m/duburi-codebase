#!/usr/bin/env python3
"""
MAVLink Inspector — Thin orchestrator for BRACU Duburi 4.2.

Owns the ROS 2 node and wires together the focused modules:
  - ConnectionManager  — serial link, heartbeat, reconnect
  - TelemetryParser    — MAVLink messages → vehicle state
  - RcController       — RC override with velocity ramp
  - PidController      — depth & yaw PID loops
  - CommandHandler     — DriverCommand dispatch

All external APIs (topics, parameters, messages) are unchanged.
"""

from __future__ import annotations

import math
import os
import threading
import time

os.environ['MAVLINK20'] = '1'

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase

from duburi_interfaces.msg import (
    DriverCommand, DriverCommandFeedback, MavlinkEvent,
    TeleopCommand, VehicleDiagnostics, VehicleState,
)
from duburi_interfaces.srv import AuvCommand
from duburi_interfaces.action import Movement

from rclpy.action import ActionServer, CancelResponse, GoalResponse

from .connection_manager import ConnectionManager
from .telemetry_parser import TelemetryParser
from .rc_controller import (
    RcController, NEUTRAL_CHANNELS, NEUTRAL_PWM, PWM_RANGE,
    CH_FORWARD, CH_LATERAL, CH_THROTTLE, CH_YAW,
    set_pwm_config,
)
from .command_handler import CommandHandler
from .velocity_control import (
    VelocityEstimator, ConvergenceGate, 
    PositionEstimator, CascadeController,  # Phase 3
    GainScheduler, AccelerationLimiter     # Phase 4
)
from .sensor_sources import SensorSourceManager  # Phase 5


class MavlinkInspectorNode(Node):
    """Main MAVLink inspector node — owns Pixhawk connection."""

    def __init__(self):
        super().__init__('mavlink_inspector')
        self._boot_time = time.time()

        # ── ROS parameters ───────────────────────────────────────────
        # Issue #30: Simulation time support
        self._use_sim_time = self.declare_parameter('use_sim_time', False).value
        
        _conn_desc = ParameterDescriptor(
            description=(
                'Serial device (e.g. /dev/ttyACM0) or pymavlink URL. '
                'For ArduPilot SITL, match the transport of your link: '
                'if sim_vehicle/MAVProxy uses --out=udp:HOST:PORT, use '
                'udpin:HOST:PORT here. tcp:HOST:5760 is SITL’s TCP listener '
                'and often carries no stream for a second client once MAVProxy '
                'is connected.'
            ),
        )
        conn_port = self.declare_parameter(
            'connection_port', '/dev/ttyACM0', _conn_desc).value
        baud = self.declare_parameter('baud', 115200).value
        yaw_source = self.declare_parameter('yaw_source', 'attitude').value

        self._ramp_rate = self.declare_parameter('ramp_rate', 800).value

        # Yaw PID gains
        self._yaw_kp = self.declare_parameter('yaw_kp', 2.0).value
        self._yaw_ki = self.declare_parameter('yaw_ki', 0.05).value
        self._yaw_kd = self.declare_parameter('yaw_kd', 0.5).value
        self._yaw_max_integral = self.declare_parameter(
            'yaw_max_integral', 50.0).value

        # Depth PID gains (POOL-TUNED defaults)
        self._depth_kp = self.declare_parameter('depth_kp', 800.0).value
        self._depth_ki = self.declare_parameter('depth_ki', 50.0).value
        self._depth_kd = self.declare_parameter('depth_kd', 100.0).value
        self._depth_max_integral = self.declare_parameter(
            'depth_max_integral', 1.0).value
        self._depth_tolerance = self.declare_parameter(
            'depth_tolerance', 0.08).value

        self._pid_max_rate = self.declare_parameter(
            'pid_max_rate', 50).value
        self._nominal_voltage = self.declare_parameter(
            'nominal_voltage', 0.0).value
        self._surface_depth = self.declare_parameter(
            'surface_depth', 0.0).value
        self._ack_timeout = self.declare_parameter(
            'ack_timeout', 3.0).value
        self._surface_throttle_duration = self.declare_parameter(
            'surface_throttle_duration', 10.0).value

        # RC watchdog (C1 safety fix)
        self._rc_watchdog_timeout = self.declare_parameter(
            'rc_watchdog_timeout', 0.5).value
        self._last_rc_success = 0.0

        # Movement parameters (NEW)
        self._decel_time = self.declare_parameter('decel_time', 1.0).value
        self._brake_enabled = self.declare_parameter('brake_enabled', True).value
        self._brake_strength = self.declare_parameter('brake_strength', 0.3).value
        self._brake_duration = self.declare_parameter('brake_duration', 0.5).value
        self._min_speed_pct = self.declare_parameter('min_speed_pct', 10.0).value

        # Publishing rates (Hz)
        self._state_publish_rate = self.declare_parameter('state_publish_rate', 10.0).value
        self._diagnostics_publish_rate = self.declare_parameter('diagnostics_publish_rate', 2.0).value

        # Auto station-keeping after movement (NEW)
        self._auto_depth_hold = self.declare_parameter('auto_depth_hold', True).value
        self._auto_heading_hold = self.declare_parameter('auto_heading_hold', True).value

        # Yaw PID enhancements (NEW)
        self._yaw_ema_alpha = self.declare_parameter('yaw_ema_alpha', 0.3).value
        self._yaw_anti_windup = self.declare_parameter('yaw_anti_windup', True).value
        
        # Convergence gate parameters (Phase 1)
        self._convergence_velocity_threshold = self.declare_parameter(
            'convergence_velocity_threshold', 0.05).value  # m/s
        self._convergence_settling_time = self.declare_parameter(
            'convergence_settling_time', 200).value  # ms
        self._convergence_timeout = self.declare_parameter(
            'convergence_timeout', 5.0).value  # sec
        
        # IMU velocity estimation parameters (Phase 1)
        self._imu_stopped_accel_threshold = self.declare_parameter(
            'imu_stopped_accel_threshold', 0.02).value  # m/s²
        self._imu_stopped_time_required = self.declare_parameter(
            'imu_stopped_time_required', 0.3).value  # sec
        self._imu_bias_x = self.declare_parameter('imu_bias_x', 0.0).value
        self._imu_bias_y = self.declare_parameter('imu_bias_y', 0.0).value
        self._imu_bias_z = self.declare_parameter('imu_bias_z', 0.0).value
        
        # Enable convergence checks after movements (Phase 1)
        self._convergence_enabled = self.declare_parameter(
            'convergence_enabled', True).value
        
        # Precision yaw control (Phase 2)
        self._yaw_precision_deadband = self.declare_parameter(
            'yaw_precision_deadband', 5.0).value  # degrees
        self._yaw_final_deadband = self.declare_parameter(
            'yaw_final_deadband', 1.0).value  # degrees
        self._yaw_settling_time = self.declare_parameter(
            'yaw_settling_time', 0.5).value  # seconds
        self._yaw_precision_kp_reduction = self.declare_parameter(
            'yaw_precision_kp_reduction', 0.5).value  # multiply Kp by this
        self._yaw_precision_kd_reduction = self.declare_parameter(
            'yaw_precision_kd_reduction', 0.7).value  # multiply Kd by this
        self._rotate_in_place_enabled = self.declare_parameter(
            'rotate_in_place_enabled', True).value  # master enable for Phase 2
        
        # Yaw feedforward (Phase 2 - optional)
        self._yaw_feedforward_enabled = self.declare_parameter(
            'yaw_feedforward_enabled', False).value  # disabled by default (needs calibration)
        self._yaw_rate_to_pwm_ratio = self.declare_parameter(
            'yaw_rate_to_pwm_ratio', 20.0).value  # PWM per deg/s (empirical)
        
        # Cascade control (Phase 3)
        self._cascade_enabled = self.declare_parameter(
            'cascade_enabled', False).value  # disabled by default (experimental)
        self._position_kp = self.declare_parameter('position_kp', 0.5).value
        self._position_ki = self.declare_parameter('position_ki', 0.0).value
        self._position_kd = self.declare_parameter('position_kd', 0.1).value
        self._velocity_kp = self.declare_parameter('velocity_kp', 400.0).value
        self._velocity_ki = self.declare_parameter('velocity_ki', 50.0).value
        self._velocity_kd = self.declare_parameter('velocity_kd', 30.0).value
        self._max_velocity_setpoint = self.declare_parameter('max_velocity_setpoint', 0.5).value  # m/s
        self._position_tolerance = self.declare_parameter('position_tolerance', 0.1).value  # meters
        self._position_update_rate = self.declare_parameter('position_update_rate', 20.0).value  # Hz
        
        # ─── Phase 4: Gain Scheduling & Acceleration Limiting ───
        self._gain_scheduling_enabled = self.declare_parameter('gain_scheduling_enabled', False).value
        self._accel_limiting_enabled = self.declare_parameter('accel_limiting_enabled', False).value
        self._speed_range_low_max = self.declare_parameter('speed_range_low_max', 30).value
        self._speed_range_medium_max = self.declare_parameter('speed_range_medium_max', 60).value
        
        # Yaw gain sets
        self._yaw_gains_low_kp = self.declare_parameter('yaw_gains_low_kp', 2.5).value
        self._yaw_gains_low_ki = self.declare_parameter('yaw_gains_low_ki', 0.08).value
        self._yaw_gains_low_kd = self.declare_parameter('yaw_gains_low_kd', 0.6).value
        self._yaw_gains_medium_kp = self.declare_parameter('yaw_gains_medium_kp', 2.0).value
        self._yaw_gains_medium_ki = self.declare_parameter('yaw_gains_medium_ki', 0.05).value
        self._yaw_gains_medium_kd = self.declare_parameter('yaw_gains_medium_kd', 0.5).value
        self._yaw_gains_high_kp = self.declare_parameter('yaw_gains_high_kp', 1.2).value
        self._yaw_gains_high_ki = self.declare_parameter('yaw_gains_high_ki', 0.02).value
        self._yaw_gains_high_kd = self.declare_parameter('yaw_gains_high_kd', 0.3).value
        
        # Depth gain sets
        self._depth_gains_low_kp = self.declare_parameter('depth_gains_low_kp', 900).value
        self._depth_gains_low_ki = self.declare_parameter('depth_gains_low_ki', 60).value
        self._depth_gains_low_kd = self.declare_parameter('depth_gains_low_kd', 120).value
        self._depth_gains_medium_kp = self.declare_parameter('depth_gains_medium_kp', 800).value
        self._depth_gains_medium_ki = self.declare_parameter('depth_gains_medium_ki', 50).value
        self._depth_gains_medium_kd = self.declare_parameter('depth_gains_medium_kd', 100).value
        self._depth_gains_high_kp = self.declare_parameter('depth_gains_high_kp', 600).value
        self._depth_gains_high_ki = self.declare_parameter('depth_gains_high_ki', 30).value
        self._depth_gains_high_kd = self.declare_parameter('depth_gains_high_kd', 70).value
        
        # Velocity cascade gain sets
        self._velocity_gains_low_kp = self.declare_parameter('velocity_gains_low_kp', 450).value
        self._velocity_gains_low_ki = self.declare_parameter('velocity_gains_low_ki', 60).value
        self._velocity_gains_low_kd = self.declare_parameter('velocity_gains_low_kd', 35).value
        self._velocity_gains_medium_kp = self.declare_parameter('velocity_gains_medium_kp', 400).value
        self._velocity_gains_medium_ki = self.declare_parameter('velocity_gains_medium_ki', 50).value
        self._velocity_gains_medium_kd = self.declare_parameter('velocity_gains_medium_kd', 30).value
        self._velocity_gains_high_kp = self.declare_parameter('velocity_gains_high_kp', 300).value
        self._velocity_gains_high_ki = self.declare_parameter('velocity_gains_high_ki', 35).value
        self._velocity_gains_high_kd = self.declare_parameter('velocity_gains_high_kd', 20).value
        
        # Acceleration limiting
        self._max_accel_pct_per_sec = self.declare_parameter('max_accel_pct_per_sec', 50.0).value
        
        # ─── Phase 5: Multi-Source Sensor Architecture ───
        # DVL (Nortek Nucleus 1000)
        self._dvl_enabled = self.declare_parameter('dvl_enabled', False).value
        self._dvl_topic = self.declare_parameter('dvl_topic', '/dvl/velocity').value
        self._dvl_timeout = self.declare_parameter('dvl_timeout', 1.0).value
        self._dvl_min_quality = self.declare_parameter('dvl_min_quality', 0.5).value
        self._dvl_min_altitude = self.declare_parameter('dvl_min_altitude', 0.3).value
        self._dvl_max_altitude = self.declare_parameter('dvl_max_altitude', 50.0).value
        
        # DVL Internal IMU
        self._dvl_imu_enabled = self.declare_parameter('dvl_imu_enabled', False).value
        self._dvl_imu_topic = self.declare_parameter('dvl_imu_topic', '/dvl/orientation').value
        self._dvl_imu_msg_type = self.declare_parameter('dvl_imu_msg_type', 'geometry_msgs/QuaternionStamped').value
        self._dvl_imu_timeout = self.declare_parameter('dvl_imu_timeout', 0.5).value
        
        # External Yaw (Witmotion, BNO085, etc.)
        self._external_yaw_enabled = self.declare_parameter('external_yaw_enabled', False).value
        self._external_yaw_topic = self.declare_parameter('external_yaw_topic', '/external_imu/yaw').value
        self._external_yaw_msg_type = self.declare_parameter('external_yaw_msg_type', 'std_msgs/Float32').value
        self._external_yaw_timeout = self.declare_parameter('external_yaw_timeout', 0.5).value
        self._external_yaw_offset = self.declare_parameter('external_yaw_offset', 0.0).value
        
        # Source priorities
        self._velocity_source_priority = self.declare_parameter(
            'velocity_source_priority', ['dvl', 'imu_estimate']
        ).value
        self._yaw_source_priority = self.declare_parameter(
            'yaw_source_priority', ['dvl_imu', 'external', 'pixhawk']
        ).value

        # Connection health (Design Issue 3: parameterized timing)
        heartbeat_timeout = self.declare_parameter(
            'heartbeat_timeout', 3.0).value
        reconnect_backoff = self.declare_parameter(
            'reconnect_backoff', 2.0).value
        reconnect_max = self.declare_parameter(
            'reconnect_max', 15.0).value

        # Auto-search UDP endpoints when no serial found (for BlueOS)
        auto_search_udp = self.declare_parameter(
            'auto_search_udp', True,
            ParameterDescriptor(
                description=(
                    'When serial ports are not found, automatically try '
                    'UDP endpoints (udpin:0.0.0.0:14550) for BlueOS connection.'
                )
            ),
        ).value

        # PWM Configuration (Issue #14)
        pwm_neutral = self.declare_parameter('pwm_neutral', 1500).value
        pwm_range = self.declare_parameter('pwm_range', 400).value
        
        # Apply PWM configuration to rc_controller module
        set_pwm_config(pwm_neutral, pwm_range)

        # ── Modules ──────────────────────────────────────────────────
        self._conn = ConnectionManager(
            port=conn_port,
            baud=baud,
            heartbeat_timeout=heartbeat_timeout,
            reconnect_backoff=reconnect_backoff,
            reconnect_max=reconnect_max,
            auto_search_udp=auto_search_udp,
            logger=self.get_logger(),
            on_event=self._publish_event,
        )
        self._telemetry = TelemetryParser(yaw_source=yaw_source)
        self._rc = RcController(
            ramp_rate=self._ramp_rate,
            brake_enabled=self._brake_enabled,
            brake_strength=self._brake_strength,
            brake_duration=self._brake_duration,
            decel_time=self._decel_time,
            min_speed_pct=self._min_speed_pct,
        )
        self._cmd_handler = CommandHandler(self)
        
        # ── Velocity estimation & convergence (Phase 1) ──────────────
        vel_est_config = {
            'imu_stopped_accel_threshold': self._imu_stopped_accel_threshold,
            'imu_stopped_time_required': self._imu_stopped_time_required,
            'imu_bias_x': self._imu_bias_x,
            'imu_bias_y': self._imu_bias_y,
            'imu_bias_z': self._imu_bias_z,
        }
        self._velocity_estimator = VelocityEstimator(
            logger=self.get_logger(),
            config=vel_est_config
        )
        
        convergence_config = {
            'convergence_velocity_threshold': self._convergence_velocity_threshold,
            'convergence_settling_time': self._convergence_settling_time,
            'convergence_timeout': self._convergence_timeout,
        }
        self._convergence_gate = ConvergenceGate(
            logger=self.get_logger(),
            velocity_estimator=self._velocity_estimator,
            config=convergence_config
        )
        
        # ── Position estimation & cascade control (Phase 3) ──────────
        self._position_estimator = PositionEstimator(
            logger=self.get_logger(),
            velocity_estimator=self._velocity_estimator
        )
        
        cascade_config = {
            'position_kp': self._position_kp,
            'position_ki': self._position_ki,
            'position_kd': self._position_kd,
            'velocity_kp': self._velocity_kp,
            'velocity_ki': self._velocity_ki,
            'velocity_kd': self._velocity_kd,
            'max_velocity': self._max_velocity_setpoint,
            'position_tolerance': self._position_tolerance,
        }
        self._cascade_controller = CascadeController(
            logger=self.get_logger(),
            config=cascade_config
        )
        
        # ─── Phase 4: Gain Scheduler & Acceleration Limiter ───
        gain_scheduler_config = {
            'gain_scheduling_enabled': self._gain_scheduling_enabled,
            'speed_range_low_max': self._speed_range_low_max,
            'speed_range_medium_max': self._speed_range_medium_max,
            # Yaw gains
            'yaw_gains_low_kp': self._yaw_gains_low_kp,
            'yaw_gains_low_ki': self._yaw_gains_low_ki,
            'yaw_gains_low_kd': self._yaw_gains_low_kd,
            'yaw_gains_medium_kp': self._yaw_gains_medium_kp,
            'yaw_gains_medium_ki': self._yaw_gains_medium_ki,
            'yaw_gains_medium_kd': self._yaw_gains_medium_kd,
            'yaw_gains_high_kp': self._yaw_gains_high_kp,
            'yaw_gains_high_ki': self._yaw_gains_high_ki,
            'yaw_gains_high_kd': self._yaw_gains_high_kd,
            # Depth gains
            'depth_gains_low_kp': self._depth_gains_low_kp,
            'depth_gains_low_ki': self._depth_gains_low_ki,
            'depth_gains_low_kd': self._depth_gains_low_kd,
            'depth_gains_medium_kp': self._depth_gains_medium_kp,
            'depth_gains_medium_ki': self._depth_gains_medium_ki,
            'depth_gains_medium_kd': self._depth_gains_medium_kd,
            'depth_gains_high_kp': self._depth_gains_high_kp,
            'depth_gains_high_ki': self._depth_gains_high_ki,
            'depth_gains_high_kd': self._depth_gains_high_kd,
            # Velocity gains
            'velocity_gains_low_kp': self._velocity_gains_low_kp,
            'velocity_gains_low_ki': self._velocity_gains_low_ki,
            'velocity_gains_low_kd': self._velocity_gains_low_kd,
            'velocity_gains_medium_kp': self._velocity_gains_medium_kp,
            'velocity_gains_medium_ki': self._velocity_gains_medium_ki,
            'velocity_gains_medium_kd': self._velocity_gains_medium_kd,
            'velocity_gains_high_kp': self._velocity_gains_high_kp,
            'velocity_gains_high_ki': self._velocity_gains_high_ki,
            'velocity_gains_high_kd': self._velocity_gains_high_kd,
        }
        
        self._gain_scheduler = GainScheduler(
            logger=self.get_logger(),
            config=gain_scheduler_config
        )
        
        accel_limiter_config = {
            'accel_limiting_enabled': self._accel_limiting_enabled,
            'max_accel_pct_per_sec': self._max_accel_pct_per_sec
        }
        
        self._accel_limiter = AccelerationLimiter(
            logger=self.get_logger(),
            config=accel_limiter_config
        )
        
        # ─── Phase 5: Sensor Source Manager ───
        sensor_source_config = {
            # DVL
            'dvl_enabled': self._dvl_enabled,
            'dvl_topic': self._dvl_topic,
            'dvl_timeout': self._dvl_timeout,
            'dvl_min_quality': self._dvl_min_quality,
            'dvl_min_altitude': self._dvl_min_altitude,
            'dvl_max_altitude': self._dvl_max_altitude,
            # DVL IMU
            'dvl_imu_enabled': self._dvl_imu_enabled,
            'dvl_imu_topic': self._dvl_imu_topic,
            'dvl_imu_msg_type': self._dvl_imu_msg_type,
            'dvl_imu_timeout': self._dvl_imu_timeout,
            # External yaw
            'external_yaw_enabled': self._external_yaw_enabled,
            'external_yaw_topic': self._external_yaw_topic,
            'external_yaw_msg_type': self._external_yaw_msg_type,
            'external_yaw_timeout': self._external_yaw_timeout,
            'external_yaw_offset': self._external_yaw_offset,
            # Priorities
            'velocity_source_priority': self._velocity_source_priority,
            'yaw_source_priority': self._yaw_source_priority,
        }
        
        self._sensor_source_manager = SensorSourceManager(
            node=self,
            telemetry_parser=self._telemetry,
            logger=self.get_logger(),
            config=sensor_source_config
        )
        
        # Link velocity estimator to sensor source manager
        self._sensor_source_manager.set_velocity_estimator(self._velocity_estimator)

        # ── Movement state ───────────────────────────────────────────
        self._current_movement = None   # {channels, end_time, bypass_ramp, command}
        self._movement_lock = threading.Lock()

        # ── Depth PID state ──────────────────────────────────────────
        self._depth_pid = None          # PidController or None
        self._depth_pid_target = None   # target depth (m, negative)
        self._depth_pid_last_time = 0.0

        # ── Yaw heading state ────────────────────────────────────────
        self._yaw_pid = None            # PidController or None (PID mode)
        self._yaw_target = None         # target heading degrees or None
        self._yaw_tolerance = 3.0
        self._yaw_bang_offset = None    # PWM offset (bang-bang mode)
        self._yaw_command = ''          # command name for feedback
        self._yaw_pid_last_time = 0.0
        self._yaw_settle_start = None   # timestamp when yaw entered tolerance
        self._yaw_settle_duration = 0.3 # seconds to stay in tolerance before "reached"

        # ── ALT_HOLD depth target ────────────────────────────────────
        self._alt_hold_target = None

        # ── Command ACK tracking ─────────────────────────────────────
        self._pending_acks: dict[int, dict] = {}
        self._pending_acks_lock = threading.Lock()
        self._pending_mode_change = None

        # ── Publishers ───────────────────────────────────────────────
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, depth=10)
        reliable_1 = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, depth=1)

        self._event_pub = self.create_publisher(
            MavlinkEvent, '/mavlink/events', reliable_qos)
        self._state_pub = self.create_publisher(
            VehicleState, '/mavlink/vehicle_state', reliable_1)
        self._diag_pub = self.create_publisher(
            VehicleDiagnostics, '/mavlink/diagnostics', reliable_1)
        self._feedback_pub = self.create_publisher(
            DriverCommandFeedback, '/driver/feedback', reliable_qos)

        # ── Service server ────────────────────────────────────────────
        self._auv_command_srv = self.create_service(
            AuvCommand,
            '/auv/command',
            self._handle_auv_command
        )

        # ── Subscribers ──────────────────────────────────────────────
        self.create_subscription(
            DriverCommand, '/driver/command',
            self._cmd_handler.handle, reliable_qos)
        self.create_subscription(
            TeleopCommand, '/driver/teleop',
            self._cmd_handler.handle_teleop, reliable_qos)

        # ── Timers ───────────────────────────────────────────────────
        self.create_timer(0.02, self._read_mavlink)       # 50 Hz
        self.create_timer(1.0 / self._state_publish_rate, self._publish_state)
        self.create_timer(0.5, self._conn.send_heartbeat) # 2 Hz for ArduSub GCS failsafe
        self.create_timer(0.05, self._send_rc_override)   # 20 Hz
        self.create_timer(1.0 / self._diagnostics_publish_rate, self._publish_diagnostics)
        self.create_timer(0.5, self._resend_depth_target) # 2 Hz
        self.create_timer(0.5, self._check_ack_timeouts)  # 2 Hz
        self.create_timer(1.0, self._check_telemetry_watchdog)  # Issue #27: 1 Hz watchdog check
        
        # Phase 3: Position estimation update
        position_update_period = 1.0 / self._position_update_rate
        self.create_timer(position_update_period, self._update_position)  # Configurable Hz

        # Dynamic reconfigure callback
        from rcl_interfaces.msg import SetParametersResult
        self.add_on_set_parameters_callback(self._on_param_change)
        
        # ── Issue #28: Validate all parameters on startup ──────────────
        self._validate_parameters()

        # ── Start connection ─────────────────────────────────────────
        self._conn.start_background()
    
    # ── Issue #28: Parameter validation ──────────────────────────────
    
    def _validate_parameters(self):
        """
        Validate all parameter ranges on startup (Issue #28).
        
        Raises:
            ValueError: If any parameters are out of valid ranges
        """
        errors = []
        
        # PID gains must be non-negative
        if self._depth_kp < 0 or self._depth_ki < 0 or self._depth_kd < 0:
            errors.append(
                f"Depth PID gains must be non-negative: "
                f"kp={self._depth_kp}, ki={self._depth_ki}, kd={self._depth_kd}"
            )
        
        if self._yaw_kp < 0 or self._yaw_ki < 0 or self._yaw_kd < 0:
            errors.append(
                f"Yaw PID gains must be non-negative: "
                f"kp={self._yaw_kp}, ki={self._yaw_ki}, kd={self._yaw_kd}"
            )
        
        # PWM values must be in servo range [1000, 2000] PWM units
        # Neutral should typically be near 1500, range should allow +/- offset
        pwm_neutral = self.declare_parameter('pwm_neutral', 1500).value
        pwm_range = self.declare_parameter('pwm_range', 400).value
        pwm_min = pwm_neutral - pwm_range
        pwm_max = pwm_neutral + pwm_range
        
        if not (1000 <= pwm_min and pwm_max <= 2000):
            errors.append(
                f"PWM range out of servo limits [1000, 2000]: "
                f"neutral={pwm_neutral}, range={pwm_range} gives [{pwm_min}, {pwm_max}]"
            )
        
        # Update rates must be positive
        if self._state_publish_rate <= 0:
            errors.append(
                f"state_publish_rate must be > 0, got {self._state_publish_rate}"
            )
        
        if self._diagnostics_publish_rate <= 0:
            errors.append(
                f"diagnostics_publish_rate must be > 0, got {self._diagnostics_publish_rate}"
            )
        
        if self._position_update_rate <= 0:
            errors.append(
                f"position_update_rate must be > 0, got {self._position_update_rate}"
            )
        
        if self._pid_max_rate <= 0:
            errors.append(
                f"pid_max_rate must be > 0, got {self._pid_max_rate}"
            )
        
        # Ramp rate must be non-negative
        if self._ramp_rate < 0:
            errors.append(
                f"ramp_rate must be >= 0, got {self._ramp_rate}"
            )
        
        # Timeouts must be positive
        if self._ack_timeout <= 0:
            errors.append(
                f"ack_timeout must be > 0, got {self._ack_timeout}"
            )
        
        if self._rc_watchdog_timeout <= 0:
            errors.append(
                f"rc_watchdog_timeout must be > 0, got {self._rc_watchdog_timeout}"
            )
        
        # Depth tolerance must be non-negative
        if self._depth_tolerance < 0:
            errors.append(
                f"depth_tolerance must be >= 0, got {self._depth_tolerance}"
            )
        
        # Position parameters (cascade control)
        if self._max_velocity_setpoint < 0:
            errors.append(
                f"max_velocity_setpoint must be >= 0, got {self._max_velocity_setpoint}"
            )
        
        if self._position_tolerance < 0:
            errors.append(
                f"position_tolerance must be >= 0, got {self._position_tolerance}"
            )
        
        # Convergence parameters must be positive
        if self._convergence_velocity_threshold < 0:
            errors.append(
                f"convergence_velocity_threshold must be >= 0, got {self._convergence_velocity_threshold}"
            )
        
        if self._convergence_settling_time < 0:
            errors.append(
                f"convergence_settling_time must be >= 0, got {self._convergence_settling_time}"
            )
        
        if self._convergence_timeout <= 0:
            errors.append(
                f"convergence_timeout must be > 0, got {self._convergence_timeout}"
            )
        
        # Log any errors and raise exception if validation failed
        if errors:
            for err in errors:
                self.get_logger().error(f"Parameter validation failed: {err}")
            raise ValueError(
                f"Invalid parameters: {len(errors)} validation error(s). "
                f"See logs for details."
            )
        
        self.get_logger().info(
            "Parameter validation passed: all parameters within valid ranges"
        )
    
    # ── Issue #30: Simulation time support ────────────────────────────
    
    def get_current_time(self) -> float:
        """
        Get current time (simulation or wall-clock).
        
        Returns the current time in seconds. If use_sim_time is enabled,
        returns the ROS simulation time. Otherwise, returns wall-clock time.
        
        This method should be used instead of time.time() when simulator
        support is needed.
        
        Returns:
            float: Current time in seconds
        """
        if self._use_sim_time:
            # Return ROS simulation time in seconds
            return self.get_clock().now().nanoseconds / 1e9
        else:
            # Return wall-clock time
            return time.time()

    # ── Event / feedback helpers ─────────────────────────────────────

    def _publish_event(self, event_type: str, description: str,
                       raw_data: str = ''):
        """Publish a MAVLink event.  Safe during shutdown."""
        try:
            if not rclpy.ok():
                return
            msg = MavlinkEvent()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'mavlink'
            msg.event_type = event_type
            msg.description = description
            msg.raw_data = raw_data
            self._event_pub.publish(msg)
            self.get_logger().info(f'[{event_type}] {description}')
        except Exception as e:
            self.get_logger().error(f'Event publish failed: {e}')

    def _publish_feedback(self, command: str, status: str,
                          error: float = 0.0, detail: str = ''):
        """Publish command feedback (DESIGN 6).  Safe during shutdown."""
        try:
            if not rclpy.ok():
                return
            msg = DriverCommandFeedback()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'inspector'
            msg.command = command
            msg.status = status
            msg.error = float(error)
            msg.detail = detail
            self._feedback_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Feedback publish failed: {e}')

    # ── Dynamic parameter callback ───────────────────────────────────

    def _on_param_change(self, params) -> 'SetParametersResult':
        """Handle dynamic parameter updates."""
        from rcl_interfaces.msg import SetParametersResult
        for param in params:
            name = param.name
            value = param.value
            
            # Depth PID parameters
            if name == 'depth_kp' and hasattr(self, '_depth_pid') and self._depth_pid:
                self._depth_pid.kp = value
            elif name == 'depth_ki' and hasattr(self, '_depth_pid') and self._depth_pid:
                self._depth_pid.ki = value
            elif name == 'depth_kd' and hasattr(self, '_depth_pid') and self._depth_pid:
                self._depth_pid.kd = value
            elif name == 'depth_max_integral':
                self._depth_max_integral = value
                if hasattr(self, '_depth_pid') and self._depth_pid:
                    self._depth_pid.max_integral = value
            elif name == 'depth_tolerance':
                self._depth_tolerance = value
                if hasattr(self, '_depth_pid') and self._depth_pid:
                    self._depth_pid.tolerance = value
            
            # Yaw PID parameters
            elif name == 'yaw_kp':
                self._yaw_kp = value
            elif name == 'yaw_ki':
                self._yaw_ki = value
            elif name == 'yaw_kd':
                self._yaw_kd = value
            elif name == 'yaw_max_integral':
                self._yaw_max_integral = value
            elif name == 'yaw_ema_alpha':
                self._yaw_ema_alpha = value
            elif name == 'yaw_anti_windup':
                self._yaw_anti_windup = value
            
            # Movement/ramp parameters
            elif name == 'ramp_rate':
                self._ramp_rate = value
                self._rc.ramp_rate = value
            elif name == 'decel_time':
                self._decel_time = value
                self._rc.decel_time = value
            elif name == 'brake_enabled':
                self._brake_enabled = value
                self._rc.brake_enabled = value
            elif name == 'brake_strength':
                self._brake_strength = value
                self._rc.brake_strength = value
            elif name == 'brake_duration':
                self._brake_duration = value
                self._rc.brake_duration = value
            elif name == 'min_speed_pct':
                self._min_speed_pct = value
                self._rc.min_speed_pct = value
            
            # Other parameters
            elif name == 'pid_max_rate':
                self._pid_max_rate = value
            elif name == 'nominal_voltage':
                self._nominal_voltage = value
            elif name == 'surface_depth':
                self._surface_depth = value
            elif name == 'ack_timeout':
                self._ack_timeout = value
            elif name == 'rc_watchdog_timeout':
                self._rc_watchdog_timeout = value
            elif name == 'use_sim_time':
                # Issue #30: Handle simulation time parameter changes
                self._use_sim_time = value
            
            self.get_logger().info(f'Parameter updated: {name} = {value}')
        
        return SetParametersResult(successful=True)

    # ── State publishing ─────────────────────────────────────────────
    
    def _check_telemetry_watchdog(self):
        """
        Check for stale MAVLink messages (Issue #27).
        
        Called periodically (1 Hz) to detect if critical telemetry messages
        haven't been received within the watchdog timeout period.
        """
        stale = self._telemetry.check_watchdog()
        if stale:
            stale_str = ', '.join(
                f"{msg_type}: {elapsed:.1f}s"
                for msg_type, elapsed in stale.items()
            )
            self.get_logger().warning(
                f"Stale telemetry detected: {stale_str}"
            )

    def _publish_state(self):
        try:
            if not rclpy.ok():
                return
            telemetry = self._telemetry
            msg = VehicleState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.armed = telemetry.armed
            msg.flight_mode = telemetry.flight_mode
            msg.depth = float(telemetry.depth)
            msg.yaw = float(telemetry.yaw)
            msg.pitch = float(telemetry.pitch)
            msg.roll = float(telemetry.roll)
            msg.voltage = float(telemetry.voltage)
            msg.current = float(telemetry.current)
            self._state_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'State publish failed: {e}')

    def _publish_diagnostics(self):
        try:
            if not rclpy.ok():
                return
            telemetry = self._telemetry
            msg = VehicleDiagnostics()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.heading_rate = float(telemetry.heading_rate)
            msg.pressure = float(telemetry.pressure)
            msg.temperature = float(telemetry.temperature)
            msg.servo_output = [int(v) for v in telemetry.servo_output]
            msg.rc_channels = [int(v) for v in telemetry.rc_channels]
            msg.cpu_load = float(telemetry.cpu_load)
            self._diag_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Diagnostics publish failed: {e}')

    # ── MAVLink reading ──────────────────────────────────────────────

    def _read_mavlink(self):
        for msg in self._conn.read_messages():
            # Link alive if any inbound telemetry arrives (not only HEARTBEAT).
            # SITL + TCP often streams ATTITUDE/AHRS2 while HEARTBEAT to a
            # second client can be sparse or contended with MAVProxy on 5760.
            self._conn.last_heartbeat = time.time()
            self._process_message(msg)

    def _process_message(self, msg):
        msg_type = msg.get_type()

        # ── Connection health: first good packet after a loss ──────────
        if self._conn.heartbeat_lost_notified:
            self._conn.heartbeat_lost_notified = False
            self._publish_event(
                'heartbeat_restored', 'Vehicle telemetry restored')
            self.get_logger().info('Vehicle telemetry restored.')

        # ── COMMAND_ACK — own ACK tracking ───────────────────────────
        if msg_type == 'COMMAND_ACK':
            self._handle_command_ack(msg)
            return

        # ── All other messages → telemetry parser ────────────────────
        events = self._telemetry.process(msg, self._conn.master)
        for ev_type, ev_desc, ev_raw in events:
            self._publish_event(ev_type, ev_desc, ev_raw)
            # POOL FIX 4: clear control state on disarm
            if ev_type == 'disarmed':
                self._clear_all_control()
        
        # ── Update velocity estimator from IMU data ──────────────────
        if msg_type == 'SCALED_IMU2':
            # Create a simple object to pass IMU data to velocity estimator
            class ImuData:
                def __init__(self, x, y, z):
                    self.linear_acceleration = type('obj', (object,), {
                        'x': x, 'y': y, 'z': z
                    })()
            
            imu_msg = ImuData(
                self._telemetry.accel_x,
                self._telemetry.accel_y,
                self._telemetry.accel_z
            )
            # Get current orientation quaternion for gravity correction
            orientation_quat = self._telemetry.get_orientation()
            self._velocity_estimator.update(imu_msg, orientation_quat)
    
    def _update_position(self):
        """Update position estimate from velocity (Phase 3)."""
        try:
            if not rclpy.ok():
                return
            self._position_estimator.update()
        except Exception as e:
            self.get_logger().error(f'Position update failed: {e}')

    # ── Control state management ─────────────────────────────────────

    def _clear_all_control(self):
        """Clear all movement, PID, and heading control state."""
        with self._movement_lock:
            self._current_movement = None
            self._depth_pid = None
            self._depth_pid_target = None
            self._yaw_pid = None
            self._yaw_target = None
            self._yaw_bang_offset = None
        self._rc.clear_ramp()
        self._alt_hold_target = None

    def _stop_all(self):
        """Stop all thrusters — safety override (bypasses ramp)."""
        self._clear_all_control()
        self._rc.send_rc(NEUTRAL_CHANNELS, self._conn.master,
                         self.get_logger())
        self._publish_event('movement', 'Stop - all thrusters neutral')

    @staticmethod
    def _angle_error(current: float, target: float) -> float:
        """Shortest-path angle error in [−180, 180] degrees."""
        err = (target - current) % 360
        if err > 180:
            err -= 360
        return err

    # ── RC override (20 Hz) — 4-layer channel builder ────────────────

    def _send_rc_override(self):
        if not self._conn.connected or self._conn.master is None:
            return

        now = time.time()

        # Snapshot state under lock
        with self._movement_lock:
            mv = self._current_movement
            depth_pid = self._depth_pid
            depth_target = self._depth_pid_target
            yaw_pid = self._yaw_pid
            yaw_target = self._yaw_target
            yaw_tolerance = self._yaw_tolerance
            yaw_bang = self._yaw_bang_offset

        # ── Movement expiry ──────────────────────────────────────────
        if mv is not None and now >= mv['end_time']:
            expired_cmd = mv.get('command', 'unknown')
            with self._movement_lock:
                self._current_movement = None
            self._publish_event('movement', 'Movement duration expired')
            self._publish_feedback(expired_cmd, 'completed',
                                   detail='duration expired')
            mv = None

        # ── Layer 1+2: neutral + movement (with ramp) ────────────────
        bypass = mv.get('bypass_ramp', False) if mv else False
        channels = dict(NEUTRAL_CHANNELS)
        self._rc.apply_movement(channels, mv, bypass)

        # ── Layer 3: depth PID (overrides CH_THROTTLE) ───────────────
        if depth_pid is not None and depth_target is not None:
            dt = now - self._depth_pid_last_time if self._depth_pid_last_time > 0 else 0.05
            self._depth_pid_last_time = now
            if dt <= 0:
                dt = 0.05

            telemetry = self._telemetry
            error = depth_target - telemetry.depth
            raw_rate = (telemetry.depth - telemetry.prev_depth) / dt if dt > 0 else 0.0
            output = depth_pid.compute(error, dt,
                                       measurement_rate=raw_rate)

            if depth_pid.in_deadband:
                channels[CH_THROTTLE] = NEUTRAL_PWM
            else:
                channels[CH_THROTTLE] = NEUTRAL_PWM + output

        # ── Layer 4: yaw heading (overrides CH_YAW) ──────────────────
        if yaw_target is not None:
            err = self._angle_error(self._telemetry.yaw, yaw_target)

            if abs(err) <= yaw_tolerance:
                # Within tolerance - start or continue settling
                if self._yaw_settle_start is None:
                    self._yaw_settle_start = now
                    # Keep PID/bang-bang active during settling
                    if yaw_pid is not None:
                        dt_y = now - self._yaw_pid_last_time if self._yaw_pid_last_time > 0 else 0.05
                        self._yaw_pid_last_time = now
                        if dt_y <= 0:
                            dt_y = 0.05
                        output = yaw_pid.compute(
                            err, dt_y,
                            measurement_rate=self._telemetry.heading_rate)
                        channels[CH_YAW] = NEUTRAL_PWM + output
                    elif yaw_bang is not None:
                        channels[CH_YAW] = NEUTRAL_PWM + (
                            yaw_bang if err > 0 else -yaw_bang)

                elif now - self._yaw_settle_start >= self._yaw_settle_duration:
                    # Settled for required duration - declare "reached"
                    with self._movement_lock:
                        self._yaw_pid = None
                        self._yaw_target = None
                        self._yaw_bang_offset = None
                        self._yaw_settle_start = None
                    self._publish_event(
                        'movement', f'Heading reached: {yaw_target}° (settled)')
                    self._publish_feedback(
                        self._yaw_command or 'yaw_to_heading', 'reached',
                        error=err, detail=f'heading={yaw_target}° settled')
                else:
                    # Still settling, keep control active
                    if yaw_pid is not None:
                        dt_y = now - self._yaw_pid_last_time if self._yaw_pid_last_time > 0 else 0.05
                        self._yaw_pid_last_time = now
                        if dt_y <= 0:
                            dt_y = 0.05
                        output = yaw_pid.compute(
                            err, dt_y,
                            measurement_rate=self._telemetry.heading_rate)
                        channels[CH_YAW] = NEUTRAL_PWM + output
                    elif yaw_bang is not None:
                        channels[CH_YAW] = NEUTRAL_PWM + (
                            yaw_bang if err > 0 else -yaw_bang)

            else:
                # Outside tolerance - reset settling timer
                self._yaw_settle_start = None

                if yaw_pid is not None:
                    # PID mode
                    dt_y = now - self._yaw_pid_last_time if self._yaw_pid_last_time > 0 else 0.05
                    self._yaw_pid_last_time = now
                    if dt_y <= 0:
                        dt_y = 0.05
                    output = yaw_pid.compute(
                        err, dt_y,
                        measurement_rate=self._telemetry.heading_rate)
                    channels[CH_YAW] = NEUTRAL_PWM + output

                elif yaw_bang is not None:
                    # Bang-bang mode
                    channels[CH_YAW] = NEUTRAL_PWM + (
                        yaw_bang if err > 0 else -yaw_bang)

        # ── Send RC override and track success ────────────────────────
        success = self._rc.send_rc(channels, self._conn.master, self.get_logger())
        if success:
            self._last_rc_success = now

        # ── Watchdog timeout check ───────────────────────────────────
        if now - self._last_rc_success > self._rc_watchdog_timeout:
            self.get_logger().fatal(
                f'RC watchdog timeout! No successful RC send for '
                f'{self._rc_watchdog_timeout}s - forcing emergency neutral')
            self._emergency_neutral()

    def _emergency_neutral(self):
        """Force all RC channels to NEUTRAL_PWM (1500) for emergency safety."""
        if self._conn.master is None:
            return
        try:
            rc = [NEUTRAL_PWM] * 8 + [65535] * 10
            self._conn.master.mav.rc_channels_override_send(
                self._conn.master.target_system,
                self._conn.master.target_component, *rc)
            self._last_rc_success = time.time()
            self._rc.clear_ramp()
            with self._movement_lock:
                self._current_movement = None
            self._publish_event('safety', 'Emergency neutral applied')
        except Exception as e:
            self.get_logger().error(f'Emergency neutral failed: {e}')

    # ── MAVLink command infrastructure ───────────────────────────────

    @staticmethod
    def _mav_cmd_name(cmd_id: int) -> str:
        try:
            name = mavutil.mavlink.enums['MAV_CMD'][cmd_id].name
            return name[8:] if name.startswith('MAV_CMD_') else name
        except (KeyError, AttributeError):
            return f'CMD_{cmd_id}'

    @staticmethod
    def _mav_result_name(result: int) -> str:
        _MAP = {
            0: 'ACCEPTED', 1: 'TEMPORARILY_REJECTED', 2: 'DENIED',
            3: 'UNSUPPORTED', 4: 'FAILED', 5: 'IN_PROGRESS',
            6: 'CANCELLED',
        }
        return _MAP.get(result, f'RESULT_{result}')

    def _send_command_long(self, mav_cmd: int,
                           p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0,
                           description: str = '',
                           confirmation: int = 0) -> threading.Event | None:
        """Send COMMAND_LONG and register for ACK tracking."""
        if not self._conn.connected or self._conn.master is None:
            return None
        desc = description or self._mav_cmd_name(mav_cmd)
        self.get_logger().info(
            f'TX COMMAND_LONG  {desc}  '
            f'cmd={mav_cmd} p1={p1} p2={p2} p3={p3} '
            f'p4={p4} p5={p5} p6={p6} p7={p7}')

        ack_event = threading.Event()
        with self._pending_acks_lock:
            self._pending_acks[mav_cmd] = {
                'sent_at': time.time(), 'desc': desc,
                'event': ack_event, 'result': None,
            }
        try:
            self._conn.master.mav.command_long_send(
                self._conn.master.target_system,
                self._conn.master.target_component,
                mav_cmd, confirmation,
                p1, p2, p3, p4, p5, p6, p7)
        except Exception as e:
            self.get_logger().error(f'command_long_send failed: {e}')
            with self._pending_acks_lock:
                self._pending_acks.pop(mav_cmd, None)
            return None
        return ack_event

    def _handle_command_ack(self, msg):
        cmd_id = msg.command
        result = msg.result
        cmd_name = self._mav_cmd_name(cmd_id)
        result_name = self._mav_result_name(result)

        with self._pending_acks_lock:
            pending = self._pending_acks.pop(cmd_id, None)

        desc = pending['desc'] if pending else cmd_name

        if result == 0:
            ack_msg, ack_type = f'{desc}: ACCEPTED', 'command_accepted'
        elif result == 1:
            ack_msg = f'{desc}: TEMPORARILY REJECTED (retry later)'
            ack_type = 'command_rejected'
        elif result == 2:
            ack_msg = f'{desc}: DENIED (bad params or state)'
            ack_type = 'command_denied'
        elif result == 3:
            ack_msg = f'{desc}: UNSUPPORTED by firmware'
            ack_type = 'command_denied'
        elif result == 4:
            ack_msg, ack_type = f'{desc}: FAILED', 'command_failed'
        elif result == 5:
            progress = getattr(msg, 'progress', 255)
            pct = f' ({progress}%)' if progress != 255 else ''
            ack_msg = f'{desc}: IN PROGRESS{pct}'
            ack_type = 'command_ack'
            if pending:
                with self._pending_acks_lock:
                    self._pending_acks[cmd_id] = pending
                pending = None  # don't resolve yet
        elif result == 6:
            ack_msg, ack_type = f'{desc}: CANCELLED', 'command_cancelled'
        else:
            ack_msg, ack_type = f'{desc}: {result_name}', 'command_ack'

        self._publish_event(ack_type, ack_msg,
                            raw_data=str(msg.to_dict()))

        if pending is not None:
            pending['result'] = result
            pending['event'].set()

    def _check_ack_timeouts(self):
        now = time.time()
        timed_out = []
        with self._pending_acks_lock:
            for cmd_id, info in list(self._pending_acks.items()):
                if now - info['sent_at'] > self._ack_timeout:
                    timed_out.append((cmd_id, info))
            for cmd_id, _ in timed_out:
                self._pending_acks.pop(cmd_id, None)

        for cmd_id, info in timed_out:
            self._publish_event(
                'command_timeout',
                f'{info["desc"]}: NO RESPONSE '
                f'(timeout {self._ack_timeout:.0f}s)')
            info['result'] = -1
            info['event'].set()

        # Mode change verification
        pmc = self._pending_mode_change
        if pmc is not None:
            if self._telemetry.flight_mode == pmc['target']:
                self._publish_event('mode_verified',
                    f'Mode change confirmed: {pmc["target"]}')
                self._pending_mode_change = None
            elif now - pmc['sent_at'] > self._ack_timeout:
                self._publish_event(
                    'mode_timeout',
                    f'Mode change to {pmc["target"]} NOT confirmed '
                    f'(current: {self._telemetry.flight_mode})')
                self._pending_mode_change = None

    # ── Vehicle control helpers ──────────────────────────────────────

    def _set_target_attitude(self, roll: float, pitch: float, yaw: float):
        if not self._conn.connected or self._conn.master is None:
            return
        self._conn.master.mav.set_attitude_target_send(
            int(1e3 * (time.time() - self._boot_time)),
            self._conn.master.target_system,
            self._conn.master.target_component,
            mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
            QuaternionBase([math.radians(a) for a in (roll, pitch, yaw)]),
            0, 0, 0, 0)

    def _set_target_depth(self, depth: float):
        if not self._conn.connected or self._conn.master is None:
            return
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        self._conn.master.mav.set_position_target_global_int_send(
            int(1e3 * (time.time() - self._boot_time)),
            self._conn.master.target_system,
            self._conn.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask,
            0, 0, depth, 0, 0, 0, 0, 0, 0, 0, 0)

    def _resend_depth_target(self):
        if self._alt_hold_target is not None and self._conn.connected:
            self._set_target_depth(self._alt_hold_target)

    def _arm_disarm(self, do_arm: bool):
        if not self._conn.connected or self._conn.master is None:
            return
        action = 'Arming' if do_arm else 'Disarming'
        arm_value = 1 if do_arm else 0
        self.get_logger().info(
            f'TX COMMAND_LONG  COMPONENT_ARM_DISARM  arm={arm_value}')
        self._publish_event('arm' if do_arm else 'disarm',
                            f'{action}...')
        try:
            self._conn.master.mav.command_long_send(
                self._conn.master.target_system,
                self._conn.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                v, 0, 0, 0, 0, 0, 0)
        except Exception as e:
            ev = 'arm_failed' if do_arm else 'disarm_failed'
            self._publish_event(ev, str(e))

    def _set_mode(self, mode: str):
        if not self._conn.connected or self._conn.master is None:
            return
        mode = (mode or 'MANUAL').upper()
        mapping = self._conn.master.mode_mapping()
        if mode not in mapping:
            self.get_logger().error(
                f'Unknown mode: {mode}. '
                f'Available: {list(mapping.keys())}')
            return
        mode_id = mapping[mode]
        self._send_command_long(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            p1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            p2=mode_id,
            description=f'SET_MODE {mode}')
        self._pending_mode_change = {
            'target': mode, 'sent_at': time.time()}
        self._publish_event('mode_change', f'Setting mode to {mode}')

    def _set_servo_pwm(self, servo_n: int, microseconds: int):
        if not self._conn.connected or self._conn.master is None:
            return
        self.get_logger().info(
            f'TX COMMAND_LONG  DO_SET_SERVO  '
            f'ch={servo_n + 8} pwm={microseconds}')
        self._conn.master.mav.command_long_send(
            self._conn.master.target_system,
            self._conn.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
            servo_n + 8, microseconds, 0, 0, 0, 0, 0)
    
    def set_home_position(self, lat: float = None, lon: float = None, 
                          alt: float = None):
        """
        Set home position using MAV_CMD_DO_SET_HOME (Issue #29).
        
        Args:
            lat: Latitude in degrees. If None, use current position.
            lon: Longitude in degrees. If None, use current position.
            alt: Altitude in meters. If None, use current position.
        
        If all parameters are None, sets home to the vehicle's current position.
        """
        if not self._conn.connected or self._conn.master is None:
            self.get_logger().error(
                'Cannot set home position: vehicle not connected'
            )
            return
        
        if lat is None or lon is None or alt is None:
            # Use current position (param1=1)
            self.get_logger().info(
                'TX COMMAND_LONG MAV_CMD_DO_SET_HOME (use current position)'
            )
            self._conn.master.mav.command_long_send(
                self._conn.master.target_system,
                self._conn.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                0,  # confirmation
                1,  # param1: 1=use current position, 0=use specified
                0, 0, 0, 0, 0, 0  # params 2-7 unused
            )
        else:
            # Use specified position (param1=0)
            self.get_logger().info(
                f'TX COMMAND_LONG MAV_CMD_DO_SET_HOME '
                f'(lat={lat}, lon={lon}, alt={alt})'
            )
            self._conn.master.mav.command_long_send(
                self._conn.master.target_system,
                self._conn.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                0,  # confirmation
                0,  # param1: 0=use specified position
                0, 0, 0,
                lat, lon, alt  # params 5, 6, 7
            )

    # ── Service handler ──────────────────────────────────────────────

    def _handle_auv_command(self, request, response):
        """Handle AuvCommand service calls."""
        cmd = request.command.lower()

        try:
            if cmd == 'arm':
                self._cmd_handler._cmd_arm(None)
                response.success = True
                response.message = 'Armed'
            elif cmd == 'disarm':
                self._cmd_handler._cmd_disarm(None)
                response.success = True
                response.message = 'Disarmed'
            elif cmd == 'stop':
                self._stop_all()
                response.success = True
                response.message = 'Stopped'
            elif cmd == 'set_mode':
                mode = request.flight_mode.upper()
                if mode in ('MANUAL', 'STABILIZE', 'ALT_HOLD'):
                    self._set_mode(mode)
                    response.success = True
                    response.message = f'Mode set to {mode}'
                else:
                    response.success = False
                    response.message = f'Unknown mode: {mode}'
            elif cmd == 'calibrate_depth':
                self._cmd_handler._cmd_calibrate_depth(None)
                response.success = True
                response.message = 'Depth calibrated'
            elif cmd == 'pid_depth_off':
                self._cmd_handler._cmd_pid_depth_off(None)
                response.success = True
                response.message = 'PID depth disabled'
            else:
                response.success = False
                response.message = f'Unknown command: {cmd}'
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'Service command failed: {e}')

        return response


# ── Entry point ──────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = MavlinkInspectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send neutral RC directly (bypass ROS) for safety
        try:
            master = node._conn.master
            if master and node._conn.connected:
                rc = [NEUTRAL_PWM] * 8 + [65535] * 10
                master.mav.rc_channels_override_send(
                    master.target_system,
                    master.target_component, *rc)
        except Exception as e:
            print(f'Shutdown RC neutral failed: {e}')
        try:
            node.destroy_node()
        except Exception as e:
            print(f'Shutdown destroy_node failed: {e}')
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f'Shutdown rclpy failed: {e}')


if __name__ == '__main__':
    main()
