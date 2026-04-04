"""
Velocity estimation and convergence control for AUV missions.

Provides IMU-based velocity estimation with ZUPT (Zero-velocity Updates), 
convergence gating, cascade control, and dead reckoning position estimation.

Classes:
- VelocityEstimator: IMU-based velocity estimation
- ConvergenceGate: Command convergence control
- PositionEstimator: Dead reckoning position tracking (Phase 3)
- CascadeController: Position → Velocity → Thrust dual-loop control (Phase 3)
- GainScheduler: Speed-adaptive PID gain selection (Phase 4)
- AccelerationLimiter: Acceleration ramp limiting (Phase 4)
"""

import time
import math
from typing import Dict, List, Optional, Tuple
from rclpy.node import Node


class VelocityEstimator:
    """
    Estimates body-frame velocity from IMU acceleration integration.
    
    Uses trapezoidal integration with ZUPT (Zero-velocity Update) for drift correction.
    Velocity is estimated in body frame: surge (forward), sway (lateral), heave (vertical).
    
    Key features:
    - Trapezoidal integration (more accurate than Euler)
    - Zero-velocity updates when stopped (drift correction)
    - Configurable IMU bias correction
    - High update rate (~50 Hz from IMU)
    """
    
    def __init__(self, logger, config: Optional[Dict] = None):
        """
        Initialize velocity estimator.
        
        Args:
            logger: ROS2 logger for diagnostics
            config: Configuration dict with:
                - imu_stopped_accel_threshold: m/s² (default 0.5 - Issue #18)
                - imu_stopped_time_required: seconds (default 1.0 - Issue #18)
                - imu_bias_x, imu_bias_y, imu_bias_z: m/s² (default 0.0)
        """
        self.logger = logger
        
        # Configuration
        config = config or {}
        # Issue #18: Increased threshold from 0.1 to 0.5 m/s² for less aggressive ZUPT
        # Issue #18: Increased time from 0.5 to 1.0 s to require longer stationary period
        self.stopped_threshold = config.get('imu_stopped_accel_threshold', 0.5)  # m/s²
        self.stopped_time_required = config.get('imu_stopped_time_required', 1.0)  # sec
        self.bias_x = config.get('imu_bias_x', 0.0)
        self.bias_y = config.get('imu_bias_y', 0.0)
        self.bias_z = config.get('imu_bias_z', 0.0)
        
        # State
        self.velocity = {'surge': 0.0, 'sway': 0.0, 'heave': 0.0}  # m/s
        self.last_accel = None
        self.last_time = None
        
        # ZUPT state
        self.stopped_duration = 0.0
        self.last_zupt_time = None
        
        # Statistics
        self.update_count = 0
        self.zupt_count = 0
        
        self.logger.info(f"VelocityEstimator initialized: threshold={self.stopped_threshold} m/s², "
                        f"ZUPT_time={self.stopped_time_required}s")
    
    def _rotate_gravity_to_body(self, quat) -> Tuple[float, float, float]:
        """
        Rotate gravity vector (0, 0, 9.81) from world to body frame.
        
        This corrects the IMU acceleration measurement by removing the effect of 
        gravity when the vehicle is pitched/rolled. Without this correction, 
        gravity appears as acceleration in body-frame axes proportional to pitch/roll:
        - pitch 30° → ~4.9 m/s² contribution to surge axis
        - After 10s → 49 m/s velocity drift
        
        Args:
            quat: Object with (w, x, y, z) quaternion components representing 
                  vehicle orientation from world frame to body frame
        
        Returns:
            (gx, gy, gz) - gravity components in body frame (m/s²)
        
        Reference: 
            Quaternion rotation formula: v' = q* ⊗ v ⊗ q
            For unit quaternion, q* = conjugate
            For gravity vector (0, 0, 9.81) in world frame (NED/ENU convention)
        """
        # Quaternion components
        w, x, y, z = quat.w, quat.x, quat.y, quat.z
        
        # Gravity in world frame. For AUVs in typical NED convention:
        # Down (gravity direction) is +Z, so gravity vector is (0, 0, 9.81)
        # This is the standard gravity acceleration magnitude
        grav_world = 9.81
        
        # Rotate (0, 0, g) by inverse quaternion (conjugate for unit quaternion)
        # Using simplified formula for rotating (0, 0, gz) by conjugate(q):
        # This comes from the quaternion rotation: v' = q* ⊗ v ⊗ q
        gx = 2 * (x * z - w * y) * grav_world
        gy = 2 * (y * z + w * x) * grav_world
        gz = (w * w - x * x - y * y + z * z) * grav_world
        
        return (gx, gy, gz)
    
    def update(self, imu_msg, orientation_quat=None):
        """
        Update velocity estimate from IMU measurement.
        
        Should be called at high rate (~50 Hz) from IMU callback.
        
        Args:
            imu_msg: ROS2 Imu message with linear_acceleration.{x,y,z}
            orientation_quat: geometry_msgs/Quaternion with (w, x, y, z) representing
                              current vehicle attitude. If None, gravity correction
                              is skipped (backward compatible).
        """
        now = time.time()
        
        # First call - initialize
        if self.last_time is None:
            self.last_accel = imu_msg.linear_acceleration
            self.last_time = now
            return
        
        dt = now - self.last_time
        
        # Sanity check on dt (prevent huge jumps if callback was delayed)
        if dt > 0.5 or dt <= 0:
            self.logger.warning(f"IMU dt anomaly: {dt:.3f}s - resetting")
            self.last_accel = imu_msg.linear_acceleration
            self.last_time = now
            return
        
        # Apply bias correction
        accel = imu_msg.linear_acceleration
        accel_x = accel.x - self.bias_x
        accel_y = accel.y - self.bias_y
        accel_z = accel.z - self.bias_z
        
        # CRITICAL FIX: Rotate gravity to body frame and subtract
        # This corrects the IMU reading for the effect of gravity
        # Without this, pitched/rolled vehicles accumulate large velocity errors
        if orientation_quat is not None:
            gx, gy, gz = self._rotate_gravity_to_body(orientation_quat)
            accel_x -= gx
            accel_y -= gy
            accel_z -= gz
        else:
            # Fallback: no gravity compensation (will drift!)
            self.logger.warning_throttle(5.0,
                "VelocityEstimator: No orientation quaternion - velocity will drift!")
        
        # Get previous acceleration values (already bias-corrected)
        prev_accel_x = self.last_accel.x - self.bias_x
        prev_accel_y = self.last_accel.y - self.bias_y
        prev_accel_z = self.last_accel.z - self.bias_z
        
        # Apply gravity correction to previous acceleration as well
        # (for consistency in trapezoidal integration)
        if orientation_quat is not None:
            # Note: This is an approximation since we don't have the previous
            # quaternion. For smooth motion, orientation changes slowly.
            gx, gy, gz = self._rotate_gravity_to_body(orientation_quat)
            prev_accel_x -= gx
            prev_accel_y -= gy
            prev_accel_z -= gz
        
        # Trapezoidal integration: v += (a_new + a_old) / 2 * dt
        self.velocity['surge'] += (accel_x + prev_accel_x) / 2.0 * dt
        self.velocity['sway'] += (accel_y + prev_accel_y) / 2.0 * dt
        self.velocity['heave'] += (accel_z + prev_accel_z) / 2.0 * dt
        
        # ZUPT: Detect stopped state and correct drift
        accel_magnitude = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
        
        if accel_magnitude < self.stopped_threshold:
            self.stopped_duration += dt
            
            if self.stopped_duration >= self.stopped_time_required:
                # Vehicle is stopped - zero out velocity (drift correction)
                prev_mag = math.sqrt(sum(v**2 for v in self.velocity.values()))
                if prev_mag > 0.01:  # Only log if there was meaningful drift
                    self.logger.debug(f"ZUPT correction: velocity was {prev_mag:.3f} m/s, now zeroed")
                
                self.velocity = {'surge': 0.0, 'sway': 0.0, 'heave': 0.0}
                self.zupt_count += 1
                self.last_zupt_time = now
        else:
            self.stopped_duration = 0.0
        
        self.last_accel = accel
        self.last_time = now
        self.update_count += 1
        
        # Periodic diagnostics (every 100 updates ~2 sec)
        if self.update_count % 100 == 0:
            vel_mag = math.sqrt(sum(v**2 for v in self.velocity.values()))
            self.logger.debug(f"VelEst: |v|={vel_mag:.3f} m/s, surge={self.velocity['surge']:.3f}, "
                             f"ZUPT_count={self.zupt_count}")
    
    def get_velocities(self, dof: List[str] = None) -> Dict[str, float]:
        """
        Get current velocity estimates.
        
        Args:
            dof: List of degrees of freedom ['surge', 'sway', 'heave']
                 If None, returns all
        
        Returns:
            Dict mapping DOF name to velocity in m/s
        """
        if dof is None:
            dof = ['surge', 'sway', 'heave']
        
        return {k: self.velocity[k] for k in dof if k in self.velocity}
    
    def get_velocity_magnitude(self, dof: List[str] = None) -> float:
        """
        Get magnitude of velocity vector.
        
        Args:
            dof: Which DOFs to include in magnitude calculation
        
        Returns:
            Speed in m/s (always positive)
        """
        velocities = self.get_velocities(dof)
        return math.sqrt(sum(v**2 for v in velocities.values()))
    
    def reset(self):
        """Reset velocity to zero (for mission start, etc.)"""
        self.velocity = {'surge': 0.0, 'sway': 0.0, 'heave': 0.0}
        self.stopped_duration = 0.0
        self.logger.info("VelocityEstimator reset")


class ConvergenceGate:
    """
    Blocks command execution until vehicle reaches stable state.
    
    Prevents next movement command from starting while vehicle still has
    residual velocity from previous command. Critical for multi-step missions
    where cumulative error would otherwise occur.
    
    Uses velocity estimator to check if vehicle is actually stopped, not just
    that the command timer expired.
    """
    
    def __init__(self, logger, velocity_estimator: VelocityEstimator, config: Optional[Dict] = None):
        """
        Initialize convergence gate.
        
        Args:
            logger: ROS2 logger
            velocity_estimator: VelocityEstimator instance to query
            config: Configuration dict with:
                - convergence_velocity_threshold: m/s (default 0.05)
                - convergence_settling_time: milliseconds (default 200)
                - convergence_timeout: seconds (default 5.0)
        """
        self.logger = logger
        self.velocity_estimator = velocity_estimator
        
        # Configuration
        config = config or {}
        self.velocity_threshold = config.get('convergence_velocity_threshold', 0.05)  # m/s
        self.settling_time = config.get('convergence_settling_time', 200) / 1000.0  # convert ms to sec
        self.timeout = config.get('convergence_timeout', 5.0)  # sec
        
        # Statistics
        self.total_waits = 0
        self.timeout_count = 0
        self.avg_wait_time = 0.0
        
        self.logger.info(f"ConvergenceGate initialized: threshold={self.velocity_threshold} m/s, "
                        f"settling={self.settling_time*1000:.0f}ms, timeout={self.timeout}s")
    
    def wait_for_convergence(self, dof: List[str] = None, blocking: bool = True) -> bool:
        """
        Block until vehicle velocity drops below threshold and stays stable.
        
        Args:
            dof: Which degrees of freedom to check ['surge', 'sway', 'heave']
                 Default: ['surge', 'sway'] (horizontal motion)
            blocking: If True, actually wait. If False, just check and return immediately
        
        Returns:
            True if converged successfully
            False if timeout occurred (still proceeds but logs warning)
        """
        if dof is None:
            dof = ['surge', 'sway']  # Default: check horizontal motion
        
        # Non-blocking mode: just check current state
        if not blocking:
            velocities = self.velocity_estimator.get_velocities(dof)
            max_vel = max(abs(v) for v in velocities.values())
            return max_vel < self.velocity_threshold
        
        # Blocking mode: wait for convergence
        start_time = time.time()
        stable_since = None
        
        self.logger.debug(f"Waiting for convergence on {dof}...")
        
        while True:
            velocities = self.velocity_estimator.get_velocities(dof)
            max_vel = max(abs(v) for v in velocities.values())
            
            if max_vel < self.velocity_threshold:
                # Below threshold
                if stable_since is None:
                    stable_since = time.time()
                    self.logger.debug(f"Below threshold ({max_vel:.4f} m/s) - settling timer started")
                
                # Check if stable long enough
                time_stable = time.time() - stable_since
                if time_stable >= self.settling_time:
                    wait_duration = time.time() - start_time
                    self.logger.info(f"✓ Converged in {wait_duration:.2f}s (final velocity: {max_vel:.4f} m/s)")
                    
                    # Update statistics
                    self.total_waits += 1
                    self.avg_wait_time = (self.avg_wait_time * (self.total_waits - 1) + wait_duration) / self.total_waits
                    
                    return True
            else:
                # Above threshold - reset settling timer
                if stable_since is not None:
                    self.logger.debug(f"Velocity increased to {max_vel:.4f} m/s - resetting settling timer")
                stable_since = None
            
            # Check timeout
            elapsed = time.time() - start_time
            if elapsed > self.timeout:
                self.logger.warning(f"⚠ Convergence TIMEOUT after {elapsed:.2f}s! "
                                   f"Velocity still {max_vel:.4f} m/s (threshold {self.velocity_threshold})")
                self.timeout_count += 1
                return False
            
            time.sleep(0.05)  # 20 Hz check rate
    
    def wait_for_stop(self, dof: List[str] = None, timeout: Optional[float] = None) -> bool:
        """
        Convenience method: wait for vehicle to stop moving.
        
        Same as wait_for_convergence but with custom timeout.
        """
        original_timeout = self.timeout
        if timeout is not None:
            self.timeout = timeout
        
        result = self.wait_for_convergence(dof=dof, blocking=True)
        
        self.timeout = original_timeout
        return result
    
    def get_statistics(self) -> Dict:
        """Get convergence statistics for diagnostics"""
        return {
            'total_waits': self.total_waits,
            'timeout_count': self.timeout_count,
            'timeout_rate': self.timeout_count / max(1, self.total_waits),
            'avg_wait_time': self.avg_wait_time
        }


# ══════════════════════════════════════════════════════════════════════
# Phase 3: Position Estimation & Cascade Control
# ══════════════════════════════════════════════════════════════════════


class PositionEstimator:
    """
    Estimates position via dead reckoning (velocity integration).
    
    Uses VelocityEstimator output to track position in body frame.
    ZUPT corrections from velocity estimator help reduce drift.
    
    Note: This is dead reckoning and will drift over time. DVL integration
    in Phase 5 will provide ground-truth position.
    """
    
    def __init__(self, logger, velocity_estimator: VelocityEstimator):
        """
        Initialize position estimator.
        
        Args:
            logger: ROS2 logger
            velocity_estimator: VelocityEstimator instance to read from
        """
        self.logger = logger
        self.velocity_estimator = velocity_estimator
        
        # Position state (meters from origin)
        self.position = {'surge': 0.0, 'sway': 0.0, 'heave': 0.0}
        self.last_update = None
        
        # Origin tracking
        self.origin_set = False
        
        self.logger.info("PositionEstimator initialized (dead reckoning)")
    
    def reset_origin(self):
        """Reset position to origin (0, 0, 0)"""
        self.position = {'surge': 0.0, 'sway': 0.0, 'heave': 0.0}
        self.last_update = time.time()
        self.origin_set = True
        self.logger.info("Position reset to origin")
    
    def update(self):
        """
        Update position estimate by integrating velocity.
        
        Should be called regularly (~20 Hz) to maintain accurate position.
        """
        now = time.time()
        
        # First call - set origin
        if self.last_update is None:
            self.last_update = now
            self.origin_set = True
            return
        
        dt = now - self.last_update
        
        # Sanity check
        if dt > 1.0 or dt <= 0:
            self.logger.warning(f"Position update dt anomaly: {dt:.3f}s")
            self.last_update = now
            return
        
        # Integrate velocity to get position
        velocities = self.velocity_estimator.get_velocities()
        self.position['surge'] += velocities['surge'] * dt
        self.position['sway'] += velocities['sway'] * dt
        self.position['heave'] += velocities['heave'] * dt
        
        self.last_update = now
    
    def get_position(self, dof: List[str] = None) -> Dict[str, float]:
        """
        Get current position estimate.
        
        Args:
            dof: Degrees of freedom to return (default: all)
        
        Returns:
            Dict mapping DOF to position in meters
        """
        if dof is None:
            dof = ['surge', 'sway', 'heave']
        
        return {k: self.position[k] for k in dof if k in self.position}
    
    def get_distance_from_origin(self, dof: List[str] = None) -> float:
        """
        Get straight-line distance from origin.
        
        Args:
            dof: Which DOFs to include (default: ['surge', 'sway'])
        
        Returns:
            Distance in meters
        """
        if dof is None:
            dof = ['surge', 'sway']  # Horizontal distance
        
        pos = self.get_position(dof)
        return math.sqrt(sum(v**2 for v in pos.values()))


class CascadeController:
    """
    Dual-loop cascade controller: Position → Velocity → Thrust.
    
    Outer loop (position): Computes desired velocity from position error
    Inner loop (velocity): Computes thrust (PWM) from velocity error
    
    This provides much better position control than open-loop timing.
    
    Example:
        # Move forward 2 meters
        controller.set_target(surge=2.0)
        while not controller.reached_target():
            pwm_offset = controller.update(current_pos, current_vel)
            apply_pwm(NEUTRAL + pwm_offset)
    """
    
    def __init__(self, logger, config: Optional[Dict] = None):
        """
        Initialize cascade controller.
        
        Args:
            logger: ROS2 logger
            config: Configuration dict with:
                - position_kp, position_ki, position_kd: Position loop gains
                - velocity_kp, velocity_ki, velocity_kd: Velocity loop gains
                - position_tolerance: Meters (default 0.1m)
                - velocity_tolerance: m/s (default 0.05m/s)
                - max_velocity: m/s (default 0.5m/s)
                - max_thrust: PWM offset (default 400)
        """
        self.logger = logger
        
        # Configuration
        config = config or {}
        
        # Position PID (outer loop) - outputs velocity setpoint
        self.pos_kp = config.get('position_kp', 0.5)
        self.pos_ki = config.get('position_ki', 0.0)  # Usually 0 (velocity loop handles steady-state)
        self.pos_kd = config.get('position_kd', 0.1)
        self.pos_integral = {'surge': 0.0, 'sway': 0.0, 'heave': 0.0}
        self.pos_prev_error = {'surge': 0.0, 'sway': 0.0, 'heave': 0.0}
        
        # Velocity PID (inner loop) - outputs thrust
        self.vel_kp = config.get('velocity_kp', 400.0)
        self.vel_ki = config.get('velocity_ki', 50.0)
        self.vel_kd = config.get('velocity_kd', 30.0)
        self.vel_integral = {'surge': 0.0, 'sway': 0.0, 'heave': 0.0}
        self.vel_prev_error = {'surge': 0.0, 'sway': 0.0, 'heave': 0.0}
        
        # Limits
        self.max_velocity = config.get('max_velocity', 0.5)  # m/s
        self.max_thrust = config.get('max_thrust', 400)  # PWM offset
        self.position_tolerance = config.get('position_tolerance', 0.1)  # meters
        self.velocity_tolerance = config.get('velocity_tolerance', 0.05)  # m/s
        
        # Target
        self.target_position = None
        self.dof = None
        
        # State
        self.last_update = None
        
        self.logger.info(
            f"CascadeController initialized: "
            f"pos(Kp={self.pos_kp}, Kd={self.pos_kd}), "
            f"vel(Kp={self.vel_kp}, Ki={self.vel_ki}, Kd={self.vel_kd})")
    
    def set_target(self, **targets):
        """
        Set target position.
        
        Args:
            **targets: e.g., surge=2.0, sway=0.5 (meters)
        
        Example:
            controller.set_target(surge=2.0)  # Move 2m forward
        """
        self.target_position = targets
        self.dof = list(targets.keys())
        
        # Reset integrators (per-DOF)
        self.pos_integral = {'surge': 0.0, 'sway': 0.0, 'heave': 0.0}
        self.pos_prev_error = {'surge': 0.0, 'sway': 0.0, 'heave': 0.0}
        self.vel_integral = {'surge': 0.0, 'sway': 0.0, 'heave': 0.0}
        self.vel_prev_error = {'surge': 0.0, 'sway': 0.0, 'heave': 0.0}
        self.last_update = time.time()
        
        self.logger.info(f"Cascade target set: {targets}")
    
    def update(self, current_position: Dict[str, float], 
               current_velocity: Dict[str, float]) -> Dict[str, int]:
        """
        Update cascade controller and compute thrust outputs.
        
        Args:
            current_position: Position dict (e.g., {'surge': 1.2, 'sway': 0.3})
            current_velocity: Velocity dict (e.g., {'surge': 0.15, 'sway': 0.02})
        
        Returns:
            Dict of PWM offsets per DOF (e.g., {'surge': 150, 'sway': -20})
        """
        if self.target_position is None:
            return {dof: 0 for dof in current_position.keys()}
        
        now = time.time()
        if self.last_update is None:
            dt = 0.05
        else:
            dt = now - self.last_update
            if dt <= 0 or dt > 1.0:
                dt = 0.05
        self.last_update = now
        
        outputs = {}
        
        for dof in self.dof:
            # ── Outer loop: Position → Velocity setpoint ────────────
            pos_error = self.target_position[dof] - current_position.get(dof, 0.0)
            
            # PID terms (per-DOF state)
            self.pos_integral[dof] += pos_error * dt
            pos_derivative = (pos_error - self.pos_prev_error[dof]) / dt if dt > 0 else 0.0
            self.pos_prev_error[dof] = pos_error
            
            # Compute desired velocity
            desired_velocity = (
                self.pos_kp * pos_error +
                self.pos_ki * self.pos_integral[dof] +
                self.pos_kd * pos_derivative
            )
            
            # Clamp desired velocity
            desired_velocity = max(-self.max_velocity, min(self.max_velocity, desired_velocity))
            
            # ── Inner loop: Velocity → Thrust ────────────────────────
            vel_error = desired_velocity - current_velocity.get(dof, 0.0)
            
            # PID terms (per-DOF state)
            self.vel_integral[dof] += vel_error * dt
            vel_derivative = (vel_error - self.vel_prev_error[dof]) / dt if dt > 0 else 0.0
            self.vel_prev_error[dof] = vel_error
            
            # Compute thrust (PWM offset)
            thrust = (
                self.vel_kp * vel_error +
                self.vel_ki * self.vel_integral[dof] +
                self.vel_kd * vel_derivative
            )
            
            # Clamp thrust
            thrust = max(-self.max_thrust, min(self.max_thrust, thrust))
            outputs[dof] = int(thrust)
        
        return outputs
    
    def reached_target(self, current_position: Dict[str, float],
                      current_velocity: Dict[str, float]) -> bool:
        """
        Check if target position has been reached.
        
        Criteria:
        - Position error < position_tolerance for all DOFs
        - Velocity < velocity_tolerance for all DOFs
        
        Returns:
            True if target reached and stable
        """
        if self.target_position is None:
            return True
        
        for dof in self.dof:
            pos_error = abs(self.target_position[dof] - current_position.get(dof, 0.0))
            velocity = abs(current_velocity.get(dof, 0.0))
            
            if pos_error > self.position_tolerance:
                return False
            if velocity > self.velocity_tolerance:
                return False
        
        return True
    
    def reset(self):
        """Reset controller state (clear integrators)"""
        self.target_position = None
        self.dof = None
        self.pos_integral = 0.0
        self.vel_integral = 0.0
        self.pos_prev_error = 0.0
        self.vel_prev_error = 0.0
        self.logger.info("CascadeController reset")


# ═══════════════════════════════════════════════════════════════════════════
# Phase 4: Gain Scheduling & Acceleration Limiting
# ═══════════════════════════════════════════════════════════════════════════

class GainScheduler:
    """
    Speed-adaptive PID gain selection (Phase 4).
    
    Selects appropriate PID gains based on current commanded speed to improve
    performance across the full speed range:
    - Low speed (0-30%): More aggressive gains (responsive)
    - Medium speed (30-60%): Balanced gains (default)
    - High speed (60-100%): Conservative gains (prevent overshoot)
    
    Prevents issues where:
    - High speed with low-speed gains → overshoots, instability
    - Low speed with high-speed gains → sluggish, unresponsive
    
    Usage:
        scheduler = GainScheduler(logger, config)
        gains = scheduler.select_gains(speed_pct=70, controller_type='yaw')
        yaw_pid.kp = gains['kp']
        yaw_pid.ki = gains['ki']
        yaw_pid.kd = gains['kd']
    """
    
    def __init__(self, logger, config: Optional[Dict] = None):
        """
        Initialize gain scheduler.
        
        Args:
            logger: ROS2 logger
            config: Configuration dict with gain sets for each controller type:
                - yaw_gains_low_kp, yaw_gains_low_ki, yaw_gains_low_kd
                - yaw_gains_medium_kp, yaw_gains_medium_ki, yaw_gains_medium_kd
                - yaw_gains_high_kp, yaw_gains_high_ki, yaw_gains_high_kd
                - depth_gains_* (same pattern)
                - velocity_gains_* (same pattern)
                - speed_range_low_max: Speed threshold for low→medium (default 30%)
                - speed_range_medium_max: Speed threshold for medium→high (default 60%)
                - gain_scheduling_enabled: Master enable (default False)
        """
        self.logger = logger
        config = config or {}
        
        self.enabled = config.get('gain_scheduling_enabled', False)
        
        # Speed range breakpoints
        self.low_max = config.get('speed_range_low_max', 30)      # 0-30%
        self.medium_max = config.get('speed_range_medium_max', 60) # 30-60%
        # High range: 60-100%
        
        # Yaw gain sets (default values from pool tuning + extrapolations)
        self.yaw_gains = {
            'low': {
                'kp': config.get('yaw_gains_low_kp', 2.5),
                'ki': config.get('yaw_gains_low_ki', 0.08),
                'kd': config.get('yaw_gains_low_kd', 0.6)
            },
            'medium': {
                'kp': config.get('yaw_gains_medium_kp', 2.0),
                'ki': config.get('yaw_gains_medium_ki', 0.05),
                'kd': config.get('yaw_gains_medium_kd', 0.5)
            },
            'high': {
                'kp': config.get('yaw_gains_high_kp', 1.2),
                'ki': config.get('yaw_gains_high_ki', 0.02),
                'kd': config.get('yaw_gains_high_kd', 0.3)
            }
        }
        
        # Depth gain sets
        self.depth_gains = {
            'low': {
                'kp': config.get('depth_gains_low_kp', 900),
                'ki': config.get('depth_gains_low_ki', 60),
                'kd': config.get('depth_gains_low_kd', 120)
            },
            'medium': {
                'kp': config.get('depth_gains_medium_kp', 800),
                'ki': config.get('depth_gains_medium_ki', 50),
                'kd': config.get('depth_gains_medium_kd', 100)
            },
            'high': {
                'kp': config.get('depth_gains_high_kp', 600),
                'ki': config.get('depth_gains_high_ki', 30),
                'kd': config.get('depth_gains_high_kd', 70)
            }
        }
        
        # Velocity cascade gain sets (Phase 3 inner loop)
        self.velocity_gains = {
            'low': {
                'kp': config.get('velocity_gains_low_kp', 450),
                'ki': config.get('velocity_gains_low_ki', 60),
                'kd': config.get('velocity_gains_low_kd', 35)
            },
            'medium': {
                'kp': config.get('velocity_gains_medium_kp', 400),
                'ki': config.get('velocity_gains_medium_ki', 50),
                'kd': config.get('velocity_gains_medium_kd', 30)
            },
            'high': {
                'kp': config.get('velocity_gains_high_kp', 300),
                'ki': config.get('velocity_gains_high_ki', 35),
                'kd': config.get('velocity_gains_high_kd', 20)
            }
        }
        
        # Track current range to avoid excessive logging
        self.current_range = 'medium'
        
        if self.enabled:
            self.logger.info("GainScheduler initialized (ENABLED)")
            self.logger.info(f"  Speed ranges: low=0-{self.low_max}%, medium={self.low_max}-{self.medium_max}%, high={self.medium_max}-100%")
        else:
            self.logger.info("GainScheduler initialized (DISABLED - using fixed gains)")
    
    def get_speed_range(self, speed_pct: float) -> str:
        """
        Determine which speed range the current speed falls into.
        
        Args:
            speed_pct: Commanded speed (0-100%)
        
        Returns:
            'low', 'medium', or 'high'
        """
        if speed_pct < self.low_max:
            return 'low'
        elif speed_pct < self.medium_max:
            return 'medium'
        else:
            return 'high'
    
    def select_gains(self, speed_pct: float, controller_type: str = 'yaw') -> Dict[str, float]:
        """
        Select appropriate PID gains for current speed.
        
        Args:
            speed_pct: Commanded speed (0-100%)
            controller_type: 'yaw', 'depth', or 'velocity'
        
        Returns:
            Dict with keys 'kp', 'ki', 'kd'
        """
        if not self.enabled:
            # Disabled: return medium (default) gains
            gain_table = {
                'yaw': self.yaw_gains,
                'depth': self.depth_gains,
                'velocity': self.velocity_gains
            }.get(controller_type, self.yaw_gains)
            return gain_table['medium'].copy()
        
        # Determine speed range
        range_name = self.get_speed_range(speed_pct)
        
        # Log range transitions
        if range_name != self.current_range:
            self.logger.info(
                f"Gain schedule: {self.current_range} → {range_name} "
                f"(speed={speed_pct:.0f}%, controller={controller_type})"
            )
            self.current_range = range_name
        
        # Select gain table for controller type
        gain_table = {
            'yaw': self.yaw_gains,
            'depth': self.depth_gains,
            'velocity': self.velocity_gains
        }.get(controller_type, self.yaw_gains)
        
        gains = gain_table[range_name].copy()
        
        return gains
    
    def apply_to_cascade(self, cascade_controller, speed_pct: float):
        """
        Apply scheduled gains to CascadeController velocity loop.
        
        Args:
            cascade_controller: CascadeController instance
            speed_pct: Current speed percentage
        """
        if not self.enabled:
            return  # No-op if disabled
        
        gains = self.select_gains(speed_pct, 'velocity')
        
        # Update velocity PID gains (inner loop)
        cascade_controller.vel_kp = gains['kp']
        cascade_controller.vel_ki = gains['ki']
        cascade_controller.vel_kd = gains['kd']
        
        # Reset integral to prevent windup from old gains
        cascade_controller.vel_integral = 0.0


class AccelerationLimiter:
    """
    Prevents instantaneous speed changes (Phase 4).
    
    Limits acceleration to prevent:
    - 0% → 90% instant jumps (mechanical stress, control instability)
    - Sudden deceleration (inertia issues)
    
    Implements smooth ramping:
    - Target 70% from 0%: 0 → 10 → 20 → ... → 70 over time
    - Max rate: 50%/sec (configurable)
    
    Usage:
        limiter = AccelerationLimiter(logger, config)
        
        # In control loop (20 Hz):
        target_speed = 70.0  # User requested 70%
        safe_speed = limiter.limit(target_speed)
        # First call: safe_speed = 2.5 (50%/sec × 0.05s)
        # After 1.4s: safe_speed = 70.0 (reached target)
    """
    
    def __init__(self, logger, config: Optional[Dict] = None):
        """
        Initialize acceleration limiter.
        
        Args:
            logger: ROS2 logger
            config: Configuration dict with:
                - max_accel_pct_per_sec: Max speed change rate (default 50.0 %/sec)
                - accel_limiting_enabled: Master enable (default False)
        """
        self.logger = logger
        config = config or {}
        
        self.enabled = config.get('accel_limiting_enabled', False)
        self.max_accel = config.get('max_accel_pct_per_sec', 50.0)  # %/sec
        
        # State
        self.current_speed = 0.0  # Current limited speed output
        self.last_update = None
        
        if self.enabled:
            self.logger.info(f"AccelerationLimiter initialized (ENABLED, max_accel={self.max_accel}%/s)")
        else:
            self.logger.info("AccelerationLimiter initialized (DISABLED - no ramping)")
    
    def limit(self, target_speed_pct: float) -> float:
        """
        Compute safe speed to command right now.
        
        Args:
            target_speed_pct: Desired speed (-100 to 100%)
        
        Returns:
            Safe speed to command (may be less than target if accelerating)
        """
        if not self.enabled:
            # Disabled: return target directly (no limiting)
            self.current_speed = target_speed_pct
            return target_speed_pct
        
        now = time.time()
        
        # First call: initialize
        if self.last_update is None:
            self.last_update = now
            self.current_speed = target_speed_pct
            return target_speed_pct
        
        # Compute time step
        dt = now - self.last_update
        self.last_update = now
        
        # Max allowed speed change this step
        max_step = self.max_accel * dt
        
        # Compute error
        speed_error = target_speed_pct - self.current_speed
        
        # Limit change magnitude
        if abs(speed_error) > max_step:
            # Need to ramp
            step = max_step if speed_error > 0 else -max_step
            safe_speed = self.current_speed + step
        else:
            # Can reach target in this step
            safe_speed = target_speed_pct
        
        self.current_speed = safe_speed
        
        return safe_speed
    
    def reset(self, initial_speed: float = 0.0):
        """
        Reset limiter state (call at start of new movement command).
        
        Args:
            initial_speed: Starting speed (default 0.0%)
        """
        self.current_speed = initial_speed
        self.last_update = None
        self.logger.debug(f"AccelerationLimiter reset (initial_speed={initial_speed}%)")
    
    def get_current_speed(self) -> float:
        """Get current limited speed output."""
        return self.current_speed

