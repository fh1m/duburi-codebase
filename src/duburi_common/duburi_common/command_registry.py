"""
Central command registry for Duburi AUV 4.2 control stack.

All commands are registered here using the @register decorator.
This enables:
- Single source of truth for command definitions
- Auto-generation of help text, CLI parsers, mission parsers
- Clean separation of command metadata from implementation
- Easy extension for perception-based control commands

Usage:
    from duburi_common.command_registry import register, CommandCategory, CommandTransport, get_command

    @register('move_forward', CommandCategory.TRANSLATION, CommandTransport.TOPIC,
              description='Thrust forward', channels=['CH_FORWARD'])
    def cmd_move_forward(handler, cmd):
        ...
"""

from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Optional, Any

class CommandCategory(Enum):
    """Command categories for organization and dispatch routing."""
    SYSTEM = 'system'           # arm, disarm, set_mode, stop
    TRANSLATION = 'translation' # move_forward, move_back, etc.
    HEADING = 'heading'         # yaw_to_heading, pid_yaw_to_heading
    DEPTH = 'depth'             # set_depth, pid_depth, surface
    COMPOUND = 'compound'       # go, cruise (multi-axis coordinated)
    ACTUATOR = 'actuator'       # open_grabber, close_grabber
    VISION = 'vision'           # Vision-based alignment commands (future)

class CommandTransport(Enum):
    """ROS2 transport mechanism for the command."""
    SERVICE = 'service'    # Instant, confirmed (arm, disarm, stop)
    ACTION = 'action'      # Long-running with feedback (movements)
    TOPIC = 'topic'        # Fire-and-forget (backward compat)

@dataclass
class CommandSpec:
    """Specification for a single AUV command."""
    name: str                                    # Canonical command name
    category: CommandCategory
    transport: CommandTransport
    handler: Optional[Callable] = None           # Set by @register decorator
    description: str = ''                        # Human-readable description
    channels: list[str] = field(default_factory=list)  # RC channels affected
    supports_duration: bool = True               # Accepts duration parameter
    supports_speed: bool = True                  # Accepts speed parameter
    supports_angle: bool = False                 # Accepts angle/heading parameter
    supports_depth: bool = False                 # Accepts depth parameter
    supports_bearing: bool = False               # Accepts bearing (move_at, cruise)
    supports_direction: bool = False             # Accepts direction string (go, compound)
    requires_armed: bool = True                  # Must vehicle be armed?
    is_pid: bool = False                         # Is this a PID-controlled variant?
    is_instant: bool = False                     # Bypass ramp by default (just_*)
    aliases: list[str] = field(default_factory=list)  # Alternative names

# Global registry - populated by @register decorators
_REGISTRY: dict[str, CommandSpec] = {}

def register(
    name: str,
    category: CommandCategory,
    transport: CommandTransport,
    **kwargs
) -> Callable:
    """Decorator to register a command handler.
    
    Args:
        name: Canonical command name (e.g., 'move_forward')
        category: Command category for dispatch routing
        transport: ROS2 transport mechanism
        **kwargs: Additional CommandSpec fields (description, channels, etc.)
    
    Returns:
        Decorator that registers the function and returns it unchanged.
    
    Example:
        @register('move_forward', CommandCategory.TRANSLATION, CommandTransport.TOPIC,
                  description='Thrust forward on CH_FORWARD',
                  channels=['CH_FORWARD'])
        def cmd_move_forward(handler, cmd):
            handler.set_movement({CH_FORWARD: NEUTRAL_PWM + handler.offset}, 'Forward')
    """
    def decorator(func: Callable) -> Callable:
        spec = CommandSpec(
            name=name,
            category=category,
            transport=transport,
            handler=func,
            **kwargs,
        )
        _REGISTRY[name] = spec
        return func
    return decorator

def get_command(name: str) -> Optional[CommandSpec]:
    """Get command spec by canonical name."""
    return _REGISTRY.get(name)

def get_all_commands() -> dict[str, CommandSpec]:
    """Get all registered commands."""
    return dict(_REGISTRY)

def get_commands_by_category(category: CommandCategory) -> list[CommandSpec]:
    """Get all commands in a category."""
    return [s for s in _REGISTRY.values() if s.category == category]

def get_commands_by_transport(transport: CommandTransport) -> list[CommandSpec]:
    """Get all commands using a specific transport."""
    return [s for s in _REGISTRY.values() if s.transport == transport]

def is_registered(name: str) -> bool:
    """Check if a command is registered."""
    return name in _REGISTRY

def clear_registry():
    """Clear all registered commands (for testing)."""
    _REGISTRY.clear()

# Pre-register system commands that don't need handlers from movement_commands
# These are handled directly in command_handler.py
def _register_system_commands():
    """Register system commands that are handled directly by CommandHandler."""
    system_cmds = [
        ('arm', 'Arm the vehicle for operation'),
        ('disarm', 'Disarm the vehicle'),
        ('set_mode', 'Set flight mode (MANUAL, STABILIZE, ALT_HOLD)'),
        ('stop', 'Emergency stop - all channels neutral, clear PIDs'),
        ('calibrate_depth', 'Calibrate surface depth offset'),
    ]
    for cmd_name, desc in system_cmds:
        if cmd_name not in _REGISTRY:
            _REGISTRY[cmd_name] = CommandSpec(
                name=cmd_name,
                category=CommandCategory.SYSTEM,
                transport=CommandTransport.SERVICE,
                description=desc,
                requires_armed=cmd_name not in ('arm', 'set_mode', 'calibrate_depth'),
            )
    
    # Depth commands
    depth_cmds = [
        ('set_depth', 'Set firmware depth hold (ALT_HOLD mode)', False),
        ('pid_depth', 'Enable software PID depth hold', True),
        ('pid_depth_off', 'Disable software PID depth hold', False),
        ('surface', 'Ascend to surface (composite command)', False),
    ]
    for cmd_name, desc, is_pid in depth_cmds:
        if cmd_name not in _REGISTRY:
            _REGISTRY[cmd_name] = CommandSpec(
                name=cmd_name,
                category=CommandCategory.DEPTH,
                transport=CommandTransport.SERVICE if cmd_name == 'pid_depth_off' else CommandTransport.ACTION,
                description=desc,
                supports_depth=cmd_name in ('set_depth', 'pid_depth'),
                is_pid=is_pid,
                requires_armed=cmd_name != 'surface',
            )
    
    # Actuator commands
    actuator_cmds = [
        ('open_grabber', 'Open the grabber mechanism'),
        ('close_grabber', 'Close the grabber mechanism'),
    ]
    for cmd_name, desc in actuator_cmds:
        if cmd_name not in _REGISTRY:
            _REGISTRY[cmd_name] = CommandSpec(
                name=cmd_name,
                category=CommandCategory.ACTUATOR,
                transport=CommandTransport.SERVICE,
                description=desc,
                supports_duration=False,
                supports_speed=False,
            )

# Auto-register system commands on module import
_register_system_commands()
