"""
Constants and help text for the Duburi AUV CLI runner.
"""

from duburi_common.constants import HISTORY_FILE, MISSION_PATHS  # noqa: F401 — re-exported


HELP_TEXT = """
Duburi 4.2 AUV Control - Quick Reference
========================================

  Prefix:  bare = ArduSub (firmware)   ~  = PID (software, smooth)

Movement (gain as %%, duration as Ns):
  move left [gain%%] [N]s      e.g. move left 50%% 10s
  move right/forward/back/up/down [gain%%] [N]s
  forward [gain%%] [N]s         Shorthand (no 'move' prefix)

Diagonal Movement:
  move forward-right [gain%%] [N]s     Diagonal (√2 speed scaled)
  forward-right [gain%%] [N]s          Shorthand
  Combos: forward-right, forward-left, back-right, back-left

Body-frame Vector:
  at <angle°> [gain%%] [N]s    Move at arbitrary bearing (body-relative)
  move at <angle°> [gain%%]    Same with 'move' prefix
  0°=forward, 90°=right, 180°=back, 270°=left
  e.g. at 45 60%% 3s          → diagonal forward-right at 60%%

Depth Control:
  depth <m>            ArduSub ALT_HOLD firmware depth (e.g. depth 0.5)
  ~depth               PID hold current depth (auto STABILIZE)
  ~depth <m>           PID hold specific depth (auto STABILIZE)
  ~depth off           Disable software depth PID
  surface              Ascend to surface

Heading (absolute):
  heading <deg> [gain%%]    Bang-bang yaw (e.g. heading 260 50%%)
  ~heading <deg> [gain%%]   PID smooth yaw (e.g. ~heading 260)
  heading left/right [gain%%] [N]s   Open-loop rotate

Heading (relative — from current heading):
  turn left <deg> [gain%%]    Bang-bang turn (e.g. turn left 90)
  turn right <deg> [gain%%]   Bang-bang turn right
  ~turn left <deg> [gain%%]   PID smooth turn left
  ~turn right <deg> [gain%%]  PID smooth turn right

Simultaneous Move + Heading (go):
  go <dir> <deg> [gain%%] [N]s   Move + PID yaw simultaneously
                                e.g. go forward 90 60%% 5s
  go forward-right 45 60%% 5s   Diagonal + PID heading
                                Dirs: forward, back, left, right
                                (and diagonal combos)

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
  (Prefix any movement command with 'just' to bypass ramp)

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
"""
