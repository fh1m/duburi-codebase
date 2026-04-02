"""
High-level Python API for controlling the Duburi AUV.

Provides a clean, Pythonic interface for mission scripts and autonomous control.
Designed for integration with perception systems (vision, sonar, etc.).

Usage:
    duburi = DuburiClient(node)
    duburi.arm()
    duburi.pid_depth(0.5)
    duburi.move_forward(speed=50, duration=5.0)
    duburi.yaw_to(90, method='pid')
    duburi.stop()

For perception integration:
    # Vision tells us to adjust
    duburi.move_at(bearing=detected_bearing, speed=40, duration=1.0)
    duburi.yaw_to(detected_heading, method='pid')
"""

from __future__ import annotations
from typing import Optional
import math

from rclpy.node import Node
from duburi_interfaces.msg import DriverCommand


class DuburiClient:
    """High-level Python API for controlling the Duburi AUV.
    
    This class provides method-based access to AUV commands, replacing
    the old make_command() pattern with explicit methods like:
        duburi.move_forward(speed=50, duration=3.0)
    
    Designed for easy integration with perception systems:
        - All methods accept explicit parameters
        - Heading and bearing use degrees (compass convention)
        - Depth uses meters (positive = below surface)
        - Speed uses percentage (0-100)
    """
    
    def __init__(self, node: Node, topic: str = '/driver/command'):
        """Initialize the DuburiClient.
        
        Args:
            node: ROS2 node to create publisher on
            topic: Command topic name (default: /driver/command)
        """
        self._node = node
        self._pub = node.create_publisher(DriverCommand, topic, 10)
    
    def _publish(self, **kwargs) -> DriverCommand:
        """Create and publish a DriverCommand."""
        msg = DriverCommand()
        msg.command = kwargs.get('command', '')
        msg.speed_pct = float(kwargs.get('speed_pct', 0.0))
        msg.duration = float(kwargs.get('duration', 0.0))
        msg.target_heading = float(kwargs.get('target_heading', 0.0))
        msg.target_depth = float(kwargs.get('target_depth', 0.0))
        msg.bearing = float(kwargs.get('bearing', 0.0))
        msg.direction = kwargs.get('direction', '')
        msg.flight_mode = kwargs.get('flight_mode', '')
        msg.bypass_ramp = kwargs.get('bypass_ramp', False)
        msg.use_pid = kwargs.get('use_pid', False)
        
        # Backward compat fields (set if new fields not used)
        if kwargs.get('_compat_mode'):
            msg.mode = kwargs.get('mode', '')
            msg.depth = float(kwargs.get('depth', 0.0))
            msg.angle = float(kwargs.get('angle', 0.0))
            msg.speed = int(kwargs.get('speed', 0))
        
        self._pub.publish(msg)
        return msg
    
    # ══════════════════════════════════════════════════════════════════
    # SYSTEM COMMANDS
    # ══════════════════════════════════════════════════════════════════
    
    def arm(self) -> DriverCommand:
        """Arm the vehicle for operation."""
        return self._publish(command='arm')
    
    def disarm(self) -> DriverCommand:
        """Disarm the vehicle."""
        return self._publish(command='disarm')
    
    def set_mode(self, mode: str) -> DriverCommand:
        """Set flight mode.
        
        Args:
            mode: Flight mode - 'MANUAL', 'STABILIZE', or 'ALT_HOLD'
        """
        return self._publish(command='set_mode', flight_mode=mode.upper())
    
    def stop(self) -> DriverCommand:
        """Emergency stop - all channels neutral, clear PIDs."""
        return self._publish(command='stop')
    
    def calibrate_depth(self) -> DriverCommand:
        """Calibrate surface depth offset."""
        return self._publish(command='calibrate_depth')
    
    # ══════════════════════════════════════════════════════════════════
    # TRANSLATION COMMANDS
    # ══════════════════════════════════════════════════════════════════
    
    def move_forward(self, speed: float = 50, duration: float = 0, 
                     instant: bool = False) -> DriverCommand:
        """Move forward.
        
        Args:
            speed: Speed percentage (0-100)
            duration: Duration in seconds (0 = indefinite)
            instant: If True, bypass PWM ramp (immediate thrust)
        """
        return self._publish(
            command='move_forward',
            speed_pct=speed,
            duration=duration,
            bypass_ramp=instant,
        )
    
    def move_back(self, speed: float = 50, duration: float = 0,
                  instant: bool = False) -> DriverCommand:
        """Move backward."""
        return self._publish(
            command='move_back',
            speed_pct=speed,
            duration=duration,
            bypass_ramp=instant,
        )
    
    def move_left(self, speed: float = 50, duration: float = 0,
                  instant: bool = False) -> DriverCommand:
        """Strafe left."""
        return self._publish(
            command='move_left',
            speed_pct=speed,
            duration=duration,
            bypass_ramp=instant,
        )
    
    def move_right(self, speed: float = 50, duration: float = 0,
                   instant: bool = False) -> DriverCommand:
        """Strafe right."""
        return self._publish(
            command='move_right',
            speed_pct=speed,
            duration=duration,
            bypass_ramp=instant,
        )
    
    def move_up(self, speed: float = 50, duration: float = 0,
                instant: bool = False) -> DriverCommand:
        """Move up (ascend)."""
        return self._publish(
            command='move_up',
            speed_pct=speed,
            duration=duration,
            bypass_ramp=instant,
        )
    
    def move_down(self, speed: float = 50, duration: float = 0,
                  instant: bool = False) -> DriverCommand:
        """Move down (descend)."""
        return self._publish(
            command='move_down',
            speed_pct=speed,
            duration=duration,
            bypass_ramp=instant,
        )
    
    def move_at(self, bearing: float, speed: float = 50, duration: float = 0,
                instant: bool = False) -> DriverCommand:
        """Move at arbitrary body-frame angle.
        
        Args:
            bearing: Thrust direction in degrees (0=forward, 90=right)
            speed: Speed percentage (0-100)
            duration: Duration in seconds (0 = indefinite)
            instant: If True, bypass PWM ramp
        
        This is useful for perception-based control where the vision
        system calculates a bearing to a target.
        """
        return self._publish(
            command='move_at',
            bearing=bearing,
            speed_pct=speed,
            duration=duration,
            bypass_ramp=instant,
        )
    
    def move_diagonal(self, direction: str, speed: float = 50, 
                      duration: float = 0, instant: bool = False) -> DriverCommand:
        """Move in a diagonal direction.
        
        Args:
            direction: Compound direction like 'forward-right', 'back-left'
            speed: Speed percentage (0-100)
            duration: Duration in seconds
            instant: If True, bypass PWM ramp
        """
        cmd_name = 'move_' + direction.replace('-', '_')
        return self._publish(
            command=cmd_name,
            speed_pct=speed,
            duration=duration,
            bypass_ramp=instant,
        )
    
    # ══════════════════════════════════════════════════════════════════
    # HEADING COMMANDS
    # ══════════════════════════════════════════════════════════════════
    
    def yaw_to(self, heading: float, speed: float = 50, 
               method: str = 'pid') -> DriverCommand:
        """Yaw to absolute compass heading.
        
        Args:
            heading: Target heading in degrees (0-360)
            speed: Speed percentage for yaw (0-100)
            method: 'pid' for smooth PID control, 'bang' for bang-bang
        """
        command = 'pid_yaw_to_heading' if method == 'pid' else 'yaw_to_heading'
        return self._publish(
            command=command,
            target_heading=heading % 360,
            speed_pct=speed,
            use_pid=(method == 'pid'),
        )
    
    def turn(self, degrees: float, direction: str = 'left',
             method: str = 'pid', current_heading: float = 0) -> DriverCommand:
        """Turn by relative degrees.
        
        Args:
            degrees: Degrees to turn (positive number)
            direction: 'left' or 'right'
            method: 'pid' for smooth, 'bang' for fast
            current_heading: Current heading (needed for relative turn)
        
        Note: For autonomous operation, pass the current heading from telemetry.
        """
        delta = degrees if direction == 'right' else -degrees
        target = (current_heading + delta) % 360
        return self.yaw_to(target, method=method)
    
    def yaw_left(self, speed: float = 50, duration: float = 1.0) -> DriverCommand:
        """Open-loop rotate left for duration."""
        return self._publish(
            command='yaw_left',
            speed_pct=speed,
            duration=duration,
        )
    
    def yaw_right(self, speed: float = 50, duration: float = 1.0) -> DriverCommand:
        """Open-loop rotate right for duration."""
        return self._publish(
            command='yaw_right',
            speed_pct=speed,
            duration=duration,
        )
    
    # ══════════════════════════════════════════════════════════════════
    # DEPTH COMMANDS
    # ══════════════════════════════════════════════════════════════════
    
    def set_depth(self, meters: float) -> DriverCommand:
        """Set firmware depth hold (requires ALT_HOLD mode).
        
        Args:
            meters: Target depth in meters (positive = below surface)
        """
        return self._publish(
            command='set_depth',
            target_depth=meters,
        )
    
    def pid_depth(self, meters: float = 0.0) -> DriverCommand:
        """Enable software PID depth hold.
        
        Args:
            meters: Target depth in meters (positive = below surface)
                   Use 0.0 to hold current depth.
        
        This works in MANUAL/STABILIZE mode (no firmware depth hold).
        Preferred for perception integration as it doesn't require mode switch.
        """
        return self._publish(
            command='pid_depth',
            target_depth=meters,
            use_pid=True,
        )
    
    def pid_depth_off(self) -> DriverCommand:
        """Disable software PID depth hold."""
        return self._publish(command='pid_depth_off')
    
    def surface(self) -> DriverCommand:
        """Ascend to surface."""
        return self._publish(command='surface')
    
    # ══════════════════════════════════════════════════════════════════
    # COMPOUND COMMANDS (multi-axis coordinated control)
    # ══════════════════════════════════════════════════════════════════
    
    def go(self, direction: str, heading: float, speed: float = 50,
           duration: float = 0) -> DriverCommand:
        """Move in direction while maintaining heading with PID yaw.
        
        Args:
            direction: 'forward', 'back', 'left', 'right', or compound
            heading: Target heading to maintain (compass degrees)
            speed: Speed percentage (0-100)
            duration: Duration in seconds (0 = indefinite)
        
        This is the preferred command for autonomous navigation:
        "go forward at heading 90 for 5 seconds"
        """
        return self._publish(
            command='go',
            direction=direction,
            target_heading=heading,
            speed_pct=speed,
            duration=duration,
        )
    
    def cruise(self, bearing: float, heading: float, depth: float,
               speed: float = 50, duration: float = 0) -> DriverCommand:
        """Full 3-axis coordinated movement with depth and heading hold.
        
        Args:
            bearing: Body-frame thrust direction (0=fwd, 90=right)
            heading: Target compass heading to maintain
            depth: Target depth in meters
            speed: Speed percentage (0-100)
            duration: Duration in seconds (0 = indefinite)
        
        The most sophisticated movement command - maintains depth via PID,
        heading via PID, while thrusting at the specified bearing.
        Ideal for autonomous waypoint navigation.
        """
        return self._publish(
            command='cruise',
            bearing=bearing,
            target_heading=heading,
            target_depth=depth,
            speed_pct=speed,
            duration=duration,
        )
    
    # ══════════════════════════════════════════════════════════════════
    # ACTUATOR COMMANDS
    # ══════════════════════════════════════════════════════════════════
    
    def open_grabber(self) -> DriverCommand:
        """Open the grabber mechanism."""
        return self._publish(command='open_grabber')
    
    def close_grabber(self) -> DriverCommand:
        """Close the grabber mechanism."""
        return self._publish(command='close_grabber')
    
    # ══════════════════════════════════════════════════════════════════
    # PERCEPTION INTEGRATION HELPERS
    # ══════════════════════════════════════════════════════════════════
    
    def adjust_from_vision(self, lateral_error: float, depth_error: float,
                           heading_error: float, base_speed: float = 30) -> None:
        """Adjust position based on vision feedback.
        
        Args:
            lateral_error: Left/right error (negative = need to go left)
            depth_error: Depth error in meters (negative = need to go up)
            heading_error: Heading error in degrees (negative = need to turn left)
            base_speed: Base speed for corrections
        
        This is a convenience method for visual servoing. For fine control,
        use individual methods or TeleopCommand.
        """
        # Lateral correction
        if abs(lateral_error) > 0.05:  # 5cm deadband
            if lateral_error < 0:
                self.move_left(speed=base_speed, duration=0.5)
            else:
                self.move_right(speed=base_speed, duration=0.5)
        
        # Depth correction handled by PID if enabled
        if abs(depth_error) > 0.03:  # 3cm deadband
            if depth_error < 0:
                self.move_up(speed=base_speed, duration=0.5)
            else:
                self.move_down(speed=base_speed, duration=0.5)
        
        # Heading correction - use PID
        if abs(heading_error) > 2:  # 2 degree deadband
            # Note: This needs current heading, which we don't have here
            # For real use, call yaw_to() directly with computed target
            pass
