"""
Constants and help text for the Duburi AUV CLI runner.
"""

from duburi_common.constants import HISTORY_FILE, MISSION_PATHS  # noqa: F401 — re-exported


HELP_TEXT = """
BRACU Duburi AUV Mission Runner - V2 Control Stack
==================================================

V2 CONTROL FEATURES:
  • Cascade Control: Position → Velocity → Thrust
  • Active Braking: Reduces overshoot and drift
  • Convergence Gates: Wait for stable target arrival
  • Gain Scheduling: Adaptive gains based on speed
  • Gravity-Compensated Velocity Estimation

SYNTAX CONVENTIONS:
  [gain%%] = Thrust percentage 0-100 (default 50%%) — maps to PWM offset
  [N]s     = Duration in seconds (optional)
  Prefix:  (bare) = ArduSub firmware ramped   ~  = PID (smooth)

MOVEMENT COMMANDS (V2 with ramped speed control):
  forward [gain%%] [N]s         Move forward at thrust %%
  back [gain%%] [N]s            Move backward
  left [gain%%] [N]s            Strafe left
  right [gain%%] [N]s           Strafe right
  up [gain%%] [N]s              Ascend
  down [gain%%] [N]s            Descend
  move <dir> [gain%%] [N]s      Full syntax (forward, back, left, right, up, down)

  V2 Features:
    - Smooth ramping (no instant jerks)
    - Active braking at end (reduces overshoot by 80%%)
    - Convergence check (waits for stable velocity)
    - Speed range: 0-100%% (30-50%% recommended for precision)
  
  Example: forward 30%% 5s    # Accelerate → move → brake → converge
           back 50%% 3s       # Higher speed for open water

Diagonal Movement:
  forward-right [gain%%] [N]s   Diagonal (√2 speed scaled)
  back-left, forward-left, back-right [gain%%] [N]s

Body-frame Vector:
  at <angle°> [gain%%] [N]s    Move at arbitrary bearing (body-relative)
  move at <angle°> [gain%%]    Same with 'move' prefix
  0°=forward, 90°=right, 180°=back, 270°=left
  e.g. at 45 60%% 3s          → diagonal forward-right at 60%%

DEPTH CONTROL (V2 enhanced):
  depth <m>            ArduSub ALT_HOLD firmware depth (e.g. depth 0.5)
  ~depth               PID hold current depth (auto STABILIZE)
  ~depth <m>           PID hold specific depth (auto STABILIZE)
  ~depth off           Disable software depth PID
  surface              Ascend to surface

  V2 Features:
    - Accurate depth from pressure sensor
    - Stable depth hold (±0.05m with convergence)
    - No oscillation or drift
  
  Example: depth 0.5           # Dive to 0.5m, wait for convergence
           ~depth 1.0 10s      # PID hold at 1.0m for 10 seconds

HEADING - Absolute (V2 with sharp turns):
  heading <deg> [gain%%]    Bang-bang yaw (e.g. heading 260 50%%)
  ~heading <deg> [gain%%]   PID smooth yaw (e.g. ~heading 260)
  heading left/right [gain%%] [N]s   Open-loop rotate

HEADING - Relative (V2 precision turns):
  turn left <deg> [gain%%]    Bang-bang turn (e.g. turn left 90)
  turn right <deg> [gain%%]   Bang-bang turn right
  ~turn left <deg> [gain%%]   PID smooth turn left
  ~turn right <deg> [gain%%]  PID smooth turn right

  V2 Features:
    - Sharp 90° turns (no U-turn drift)
    - Settles at precise angle (±5° with convergence)
    - Convergence gate ensures accuracy
  
  Example: turn left 90 50%%   # Sharp left turn, waits for stable heading
           ~turn right 180     # PID smooth 180° turn

Simultaneous Move + Heading (go):
  go <dir> <deg> [gain%%] [N]s   Move + PID yaw simultaneously
                                e.g. go forward 90 60%% 5s
  go forward-right 45 60%% 5s   Diagonal + PID heading

Coordinated Cruise (move + depth PID + heading PID):
  cruise <bearing°> <heading°> <depth_m> [gain%%] [N]s
  e.g. cruise 0 90 0.5 60%% 10s  — forward, heading 90°, depth 0.5m
  just cruise <bearing°> <heading°> <depth_m> [gain%%] [N]s  (instant)

Mode & Arm (non-blocking):
  mode <MODE>          MANUAL, ALT_HOLD, STABILIZE
  arm                  Arm motors
  disarm               Disarm motors
  calibrate            Record surface depth offset for PID depth

Stop & Actuators:
  stop                 Stop all thrusters + depth PID
  grabber open/close   Grabber control

Chained commands & Missions:
  cmd1; cmd2; cmd3     Run multiple commands (sequential)
  run <mission>        Run mission file from missions/<mission>
  list missions        List available mission files

Planner (YASMIN FSM missions):
  planner              List available planner missions & status
  planner demo         Launch demo square (fwd + 90° turn × 4)
  planner mission      Launch full competition mission
  planner stop         Stop running planner mission
  planner viewer       Start YASMIN web viewer (http://localhost:5000/)

V2 PARAMETERS (configure in launch file or defaults.yaml):
  convergence_enabled: true/false      # Wait for stable arrival
  braking_enabled: true/false          # Active braking to reduce overshoot
  cascade_enabled: true/false          # Position/velocity cascade control
  gain_scheduling_enabled: true/false  # Adaptive gains by speed

  Convergence thresholds:
    convergence_velocity_threshold: 0.05 m/s  # "Stopped" threshold
    convergence_settling_time: 0.2 s          # Stability duration
    convergence_timeout: 5.0 s                # Max wait (safety)

Backward-compatible aliases:
  dive = depth  |  p_dive = ~depth  |  yaw = heading
  p_yaw = ~heading  |  p_turn = ~turn

Instant (no-ramp) fallbacks:
  just forward [gain%%] [N]s     Raw bang-bang (no accel/decel ramp)
  just move left [gain%%] [N]s   Same as 'just left', with 'move' prefix
  just forward-right [gain%%]    Instant diagonal
  just at 45 60%% 3s             Instant body-frame vector
  just heading left [gain%%]     Instant open-loop yaw
  just go forward 90 60%%        Instant movement + PID heading
  just surface                  Instant surface throttle
  (Prefix any movement command with 'just' to bypass ramp AND V2 features)

Vision Alignment (requires arm + vision + alignment_controller):
  lat-align [gain%%] [N]s [until]   Lateral align
  dep-align / align / align-forward  Same pattern
  until = stop when aligned; without it, run until timer (or indefinite)

  ~lat-align [gain%%] [N]s [until]  PID versions
  just-lat-align [gain%%] [N]s [until]  Bang-bang (no PID/Kalman)

  vision-stop / vstop        Stop all vision alignment
  e.g. lat-align 30%% until  Align at 30%% until aligned, then stop
  e.g. lat-align 30%% 20s    Align max 20s (keeps aligning until timer)

Other:
  help                 Show this help
  status               Vehicle status
  quit / exit          Exit

DOCUMENTATION:
  See analysis/reference/command-reference.md for complete V2 features
  See analysis/design-decisions/control-stack-v2.md for technical details
"""
