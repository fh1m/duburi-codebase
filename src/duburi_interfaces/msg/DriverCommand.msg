# DriverCommand.msg — v2 (backward compatible)
# Command message for AUV control - used by runner, missions, planner, and vision
#
# Design principle: explicit fields for each parameter type
# Old fields kept for backward compatibility (deprecated)

string command           # Canonical command name (from registry)

# === NEW fields (preferred) ===
# Use these for new code - explicit and type-safe

float32 speed_pct        # Speed as 0-100% (0 = use default 50%)
float32 target_heading   # Compass heading degrees (0-360) for yaw commands
float32 target_depth     # Depth in meters (positive = below surface)
float32 bearing          # Body-frame thrust direction for move_at/cruise (0=fwd, 90=right)
string direction         # Direction for go/compound: 'forward', 'forward-right', etc.
string flight_mode       # For set_mode: MANUAL, ALT_HOLD, STABILIZE
bool bypass_ramp         # If true, skip PWM ramping (instant thrust) - just_* commands
bool use_pid             # If true, use PID variant (for heading/depth commands)

# === SHARED fields (used by both old and new) ===
float32 duration         # Duration in seconds (0 = indefinite)
string status            # Optional response/status field

# === DEPRECATED fields (kept for backward compat) ===
# These are read if new fields are zero/empty (migration support)
# New code should NOT use these

string mode              # OLD: flight mode or heading hack (use flight_mode or target_heading)
float32 depth            # OLD: target depth (use target_depth)
float32 angle            # OLD: target heading (use target_heading)
int32 speed              # OLD: PWM offset or percentage (use speed_pct)
