"""
alignment_controller.py -- Visual servo alignment node for BRACU Duburi 4.2.

Command-driven: waits idle until a vision alignment command arrives on
/driver/command, then activates the requested alignment axes.

Commands accepted (via /driver/command):
    lat_align        -- lateral only (left/right to center object X)
    dep_align        -- depth only (up/down to center object Y)
    align            -- lateral + depth (center object in frame)
    align_forward    -- lateral + depth + forward (center + approach)
    vision_stop      -- stop all vision-driven alignment

PID versions (prefix with pid_):
    pid_lat_align, pid_dep_align, pid_align, pid_align_forward

Non-PID commands use simple proportional control.
PID commands use the full PID controller with derivative-on-measurement
and anti-windup (Wescott pattern).

Both always run through Kalman filter when use_kalman=true.

Published topics:
    /driver/command          (DriverCommand)      -- movement commands
    /vision/alignment_status (AlignmentStatus)    -- telemetry for runner

Subscribed topics:
    /driver/command          (DriverCommand)      -- vision commands from runner
    /vision/detections       (DetectionArray)     -- from detector_node
    /mavlink/vehicle_state   (VehicleState)       -- depth/heading context
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from duburi_interfaces.msg import (
    AlignmentStatus,
    DetectionArray,
    DriverCommand,
    VehicleState,
)

from . import config as C
from .kalman_tracker import KalmanObjectTracker
from .pid_controller import PIDController


_BANNER = """
╔══════════════════════════════════════════════════════════════╗
║           DUBURI 4.2 -- Vision Alignment Controller         ║
╚══════════════════════════════════════════════════════════════╝"""

_SEP = "────────────────────────────────────────────────────"

_VISION_COMMANDS = {
    "lat_align", "dep_align", "align", "align_forward",
    "pid_lat_align", "pid_dep_align", "pid_align", "pid_align_forward",
    "just_lat_align", "just_dep_align", "just_align", "just_align_forward",
    "vision_stop",
}


class AlignmentControllerNode(Node):
    """Command-driven visual servo alignment node."""

    def __init__(self) -> None:
        super().__init__("alignment_controller")

        # ── Parameters ────────────────────────────────────────────────
        self._declare_params()
        self._load_params()

        # ── Control objects (created once, reset per command) ─────────
        self._build_controllers()

        # ── Alignment state ───────────────────────────────────────────
        self._active = False
        self._active_command = ""
        self._do_lateral = False
        self._do_vertical = False
        self._do_forward = False
        self._force_pid = False
        self._force_just = False

        self._last_detection_time = 0.0
        self._vehicle_depth = 0.0
        self._vehicle_yaw = 0.0
        self._vehicle_armed = False
        self._frame_counter = 0
        self._cmd_counter = 0
        self._last_verbose_time = 0.0
        self._state = "IDLE"
        self._active_gain = 100.0
        self._active_end_time = 0.0
        self._align_until_only = False
        self._warned_not_armed = False

        # ── Publishers ────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(DriverCommand, "/driver/command", 10)
        self._status_pub = self.create_publisher(
            AlignmentStatus, "/vision/alignment_status", 10
        )

        # ── Subscribers ───────────────────────────────────────────────
        self.create_subscription(
            DetectionArray, "/vision/detections", self._detection_cb, 10
        )
        self.create_subscription(
            VehicleState, "/mavlink/vehicle_state", self._vehicle_state_cb, 10
        )
        reliable_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        self.create_subscription(
            DriverCommand, "/driver/command", self._command_cb, reliable_qos
        )

        # ── Control timer ─────────────────────────────────────────────
        period = 1.0 / self._control_rate
        self._timer = self.create_timer(period, self._control_loop)

        # ── Latest detection storage ──────────────────────────────────
        self._latest_target: _TargetInfo | None = None

        # ── Startup banner ────────────────────────────────────────────
        print(_BANNER)
        print(f"  Target class:   {self._target_class}")
        print(f"  Kalman filter:  {'ON' if self._use_kalman else 'OFF'}")
        print(f"  Control rate:   {self._control_rate} Hz")
        print(f"  Dead zone:      X={self._dead_zone_x}  Y={self._dead_zone_y}  Area={self._dead_zone_area}")
        print(f"  Target area:    {self._target_area_ratio:.0%} of frame")
        print(f"  Max speed:      {self._max_speed} PWM")
        print(f"  Lost timeout:   {self._lost_timeout}s")
        print(_SEP)
        print("  IDLE -- Waiting for alignment command from runner.")
        print("  Proportional: lat-align, dep-align, align, align-forward")
        print("  PID:          ~lat-align, ~dep-align, ~align, ~align-forward")
        print("  Bang-bang:    just-lat-align, just-dep-align, just-align, just-align-forward")
        print("  Kalman filter:", "ON" if self._use_kalman else "OFF")
        print()

        self.get_logger().info(
            f"Alignment controller ready  target={self._target_class!r}  "
            f"kalman={'on' if self._use_kalman else 'off'}"
        )

    # ══════════════════════════════════════════════════════════════════
    #  Parameters
    # ══════════════════════════════════════════════════════════════════

    def _declare_params(self) -> None:
        p = self.declare_parameter
        p("target_class", C.DEFAULT_TARGET_CLASS)
        p("use_kalman", True)

        p("dead_zone_x", C.DEAD_ZONE_X)
        p("dead_zone_y", C.DEAD_ZONE_Y)
        p("dead_zone_area", C.DEAD_ZONE_AREA)
        p("target_area_ratio", C.TARGET_AREA_RATIO)

        p("max_speed", C.MAX_SPEED)
        p("output_deadband", C.OUTPUT_DEADBAND)
        p("control_rate", C.CONTROL_RATE_HZ)
        p("lost_timeout", C.LOST_TIMEOUT_SEC)

        p("proportional_gain", C.PROPORTIONAL_GAIN)

        p("pid_lat_kp", C.PID_LAT_KP)
        p("pid_lat_ki", C.PID_LAT_KI)
        p("pid_lat_kd", C.PID_LAT_KD)
        p("pid_vert_kp", C.PID_VERT_KP)
        p("pid_vert_ki", C.PID_VERT_KI)
        p("pid_vert_kd", C.PID_VERT_KD)
        p("pid_fwd_kp", C.PID_FWD_KP)
        p("pid_fwd_ki", C.PID_FWD_KI)
        p("pid_fwd_kd", C.PID_FWD_KD)
        p("pid_d_filter", C.PID_D_FILTER_COEFF)

        p("kf_process_noise", C.KF_PROCESS_NOISE)
        p("kf_measurement_noise", C.KF_MEASUREMENT_NOISE)
        p("kf_max_dropout", C.KF_MAX_DROPOUT_FRAMES)

        p("verbose_rate", 0.5)

    def _load_params(self) -> None:
        g = self.get_parameter
        self._target_class = g("target_class").value
        self._use_kalman = g("use_kalman").value

        self._dead_zone_x = g("dead_zone_x").value
        self._dead_zone_y = g("dead_zone_y").value
        self._dead_zone_area = g("dead_zone_area").value
        self._target_area_ratio = g("target_area_ratio").value

        self._max_speed = g("max_speed").value
        self._output_deadband = g("output_deadband").value
        self._control_rate = g("control_rate").value
        self._lost_timeout = g("lost_timeout").value

        self._proportional_gain = g("proportional_gain").value
        self._pid_d_filter = g("pid_d_filter").value

        self._pid_lat_gains = (g("pid_lat_kp").value, g("pid_lat_ki").value, g("pid_lat_kd").value)
        self._pid_vert_gains = (g("pid_vert_kp").value, g("pid_vert_ki").value, g("pid_vert_kd").value)
        self._pid_fwd_gains = (g("pid_fwd_kp").value, g("pid_fwd_ki").value, g("pid_fwd_kd").value)

        self._kf_process_noise = g("kf_process_noise").value
        self._kf_measurement_noise = g("kf_measurement_noise").value
        self._kf_max_dropout = g("kf_max_dropout").value
        self._verbose_rate = g("verbose_rate").value

    def _build_controllers(self) -> None:
        lo, hi = -float(self._max_speed), float(self._max_speed)
        df = self._pid_d_filter
        self._pid_lat = PIDController(*self._pid_lat_gains, output_min=lo, output_max=hi, d_filter_coeff=df)
        self._pid_vert = PIDController(*self._pid_vert_gains, output_min=lo, output_max=hi, d_filter_coeff=df)
        self._pid_fwd = PIDController(*self._pid_fwd_gains, output_min=lo, output_max=hi, d_filter_coeff=df)

        dt = 1.0 / self._control_rate
        self._tracker = KalmanObjectTracker(
            enabled=self._use_kalman,
            process_noise=self._kf_process_noise,
            measurement_noise=self._kf_measurement_noise,
            max_dropout_frames=self._kf_max_dropout,
            dt=dt,
        )

    # ══════════════════════════════════════════════════════════════════
    #  Command callback -- activates/deactivates alignment
    # ══════════════════════════════════════════════════════════════════

    def _command_cb(self, msg: DriverCommand) -> None:
        cmd = msg.command.lower().strip()
        if cmd not in _VISION_COMMANDS:
            return

        if cmd == "vision_stop":
            if self._active:
                self._deactivate("vision_stop received")
            return

        is_pid = cmd.startswith("pid_")
        is_just = cmd.startswith("just_")
        base = cmd[4:] if is_pid else (cmd[5:] if is_just else cmd)

        self._do_lateral = base in ("lat_align", "align", "align_forward")
        self._do_vertical = base in ("dep_align", "align", "align_forward")
        self._do_forward = base == "align_forward"
        self._force_pid = is_pid and not is_just
        self._force_just = is_just  # raw bang-bang, no PID, no Kalman

        # Gain (0-100% of max_speed) and duration (0 = indefinite)
        gain = max(1, min(100, int(msg.speed))) if msg.speed else 100
        duration = max(0.0, float(msg.duration)) if msg.duration else 0.0
        self._active_gain = float(gain)
        self._active_end_time = time.monotonic() + duration if duration > 0 else 0.0
        self._align_until_only = (msg.status or "").strip().lower() == "until_aligned"

        self._active = True
        self._active_command = cmd
        self._state = "SEARCHING"
        self._last_detection_time = 0.0
        self._latest_target = None

        self._pid_lat.reset()
        self._pid_vert.reset()
        self._pid_fwd.reset()
        if not self._force_just:
            self._tracker.reset(self._target_class)
        self._warned_not_armed = False

        mode = "Just (bang-bang)" if is_just else ("PID" if is_pid else "Proportional")
        axes = []
        if self._do_lateral:
            axes.append("lateral")
        if self._do_vertical:
            axes.append("depth")
        if self._do_forward:
            axes.append("forward")

        extra = []
        if gain != 100:
            extra.append(f"gain={gain}%")
        if duration > 0:
            extra.append(f"time={duration}s")
        if self._align_until_only:
            extra.append("until")
        extra_str = "  " + " ".join(extra) if extra else ""
        print(f"\n  >>> VISION: {cmd} activated  [{mode}]  axes=[{', '.join(axes)}]{extra_str}")
        self.get_logger().info(f"Alignment activated: {cmd}  mode={mode}  axes={axes}")

    def _deactivate(self, reason: str) -> None:
        self._active = False
        self._state = "IDLE"
        self._send_stop()
        print(f"\n  --- VISION: stopped ({reason})")
        self.get_logger().info(f"Alignment deactivated: {reason}")

    # ══════════════════════════════════════════════════════════════════
    #  Detection + vehicle state callbacks
    # ══════════════════════════════════════════════════════════════════

    def _detection_cb(self, msg: DetectionArray) -> None:
        if not self._active:
            return

        # Pick the biggest bounding box (by area) for the target class
        best = None
        best_area = -1.0
        for det in msg.detections:
            if det.class_name != self._target_class:
                continue
            area = det.bbox_w * det.bbox_h
            if area > best_area:
                best = det
                best_area = area

        if best is not None:
            iw = max(msg.image_width, 1)
            ih = max(msg.image_height, 1)
            self._latest_target = _TargetInfo(
                cx=best.center_x, cy=best.center_y,
                w=best.bbox_w / iw, h=best.bbox_h / ih,
                area=(best.bbox_w * best.bbox_h) / (iw * ih),
                confidence=best.confidence, detected=True,
                stamp=time.monotonic(),
            )
            self._last_detection_time = time.monotonic()
        else:
            if self._latest_target is not None:
                self._latest_target = _TargetInfo(
                    cx=self._latest_target.cx, cy=self._latest_target.cy,
                    w=self._latest_target.w, h=self._latest_target.h,
                    area=self._latest_target.area, confidence=0.0,
                    detected=False, stamp=self._latest_target.stamp,
                )

    def _vehicle_state_cb(self, msg: VehicleState) -> None:
        self._vehicle_depth = msg.depth
        self._vehicle_yaw = msg.yaw
        self._vehicle_armed = msg.armed

    # ══════════════════════════════════════════════════════════════════
    #  Control loop (only runs when active)
    # ══════════════════════════════════════════════════════════════════

    def _control_loop(self) -> None:
        if not self._active:
            return

        now = time.monotonic()

        # Duration limit: auto-stop when time expires
        if self._active_end_time > 0 and now >= self._active_end_time:
            self._deactivate("duration expired")
            return
        dt = 1.0 / self._control_rate
        self._frame_counter += 1
        should_print = (now - self._last_verbose_time) >= self._verbose_rate

        target = self._latest_target
        raw_detected = target is not None and target.detected

        # ── Kalman filter (skipped when just_* / bang-bang mode) ──────
        if self._force_just:
            if raw_detected and target is not None:
                cx, cy, w, h, is_lost = target.cx, target.cy, target.w, target.h, False
                kalman_predicted = False
            else:
                cx, cy, w, h, is_lost = 0.0, 0.0, 0.0, 0.0, True
                kalman_predicted = False
        elif target is not None:
            cx, cy, w, h, is_lost = self._tracker.update(
                self._target_class, detected=raw_detected,
                cx=target.cx if raw_detected else 0.0,
                cy=target.cy if raw_detected else 0.0,
                w=target.w if raw_detected else 0.0,
                h=target.h if raw_detected else 0.0,
            )
            kalman_predicted = not raw_detected and not is_lost
        else:
            cx, cy, w, h, is_lost = 0.0, 0.0, 0.0, 0.0, True
            kalman_predicted = False

        # ── Lost handling ─────────────────────────────────────────────
        time_since = now - self._last_detection_time if self._last_detection_time > 0 else float("inf")

        if is_lost or time_since > self._lost_timeout:
            if self._state != "LOST":
                self._state = "LOST"
                self._send_stop()
                print(f"\n  !!! TARGET LOST ({time_since:.1f}s) -> thrusters stopped, still searching...")
                self.get_logger().warn(f"Target '{self._target_class}' lost")
            self._publish_status(
                detected=False, kalman_predicted=False,
                error_x=0.0, error_y=0.0, error_area=0.0,
                pid_lat=0.0, pid_vert=0.0, pid_fwd=0.0,
            )
            return

        # ── Compute errors ────────────────────────────────────────────
        error_x = cx - C.FRAME_CENTER       # +right / -left
        error_y = cy - C.FRAME_CENTER       # +down  / -up
        area_ratio = w * h
        error_area = area_ratio - self._target_area_ratio  # +too_close / -too_far

        aligned_x = abs(error_x) < self._dead_zone_x
        aligned_y = abs(error_y) < self._dead_zone_y
        aligned_area = abs(error_area) < self._dead_zone_area

        # Only check axes that are active
        axis_aligned = True
        if self._do_lateral and not aligned_x:
            axis_aligned = False
        if self._do_vertical and not aligned_y:
            axis_aligned = False
        if self._do_forward and not aligned_area:
            axis_aligned = False

        # ── Compute control outputs ───────────────────────────────────
        # Apply gain scaling (0-100% of max_speed)
        effective_max = float(self._max_speed) * (self._active_gain / 100.0)
        use_pid = self._force_pid and not self._force_just
        if use_pid:
            speed_lat = self._pid_lat.compute(error_x, cx, dt)
            speed_vert = self._pid_vert.compute(error_y, cy, dt)
            speed_fwd = self._pid_fwd.compute(error_area, area_ratio, dt)
            # Clamp PID output to effective max
            speed_lat = max(-effective_max, min(speed_lat, effective_max))
            speed_vert = max(-effective_max, min(speed_vert, effective_max))
            speed_fwd = max(-effective_max, min(speed_fwd, effective_max))
        elif self._force_just:
            # Bang-bang: full or nothing
            speed_lat = effective_max if error_x > self._dead_zone_x else (-effective_max if error_x < -self._dead_zone_x else 0.0)
            speed_vert = effective_max if error_y > self._dead_zone_y else (-effective_max if error_y < -self._dead_zone_y else 0.0)
            speed_fwd = effective_max if error_area < -self._dead_zone_area else (-effective_max if error_area > self._dead_zone_area else 0.0)
        else:
            g = self._proportional_gain
            speed_lat = max(-effective_max, min(g * error_x, effective_max))
            speed_vert = max(-effective_max, min(g * error_y, effective_max))
            speed_fwd = max(-effective_max, min(g * error_area, effective_max))

        # ── Send commands (only for active axes) ──────────────────────
        # Skip movement commands when not armed to avoid rejection spam.
        commands_sent: list[str] = []

        if not self._vehicle_armed:
            if not self._warned_not_armed:
                self._warned_not_armed = True
                print("\n  [VISION] Vehicle not armed — alignment holding (arm to enable thrusters)")
            self._send_stop()
            commands_sent.append("HOLD (not armed)")
            new_state = self._state if self._state == "LOST" else "TRACKING"
        elif axis_aligned:
            self._send_stop()
            commands_sent.append("HOLD (aligned)")
            new_state = "ALIGNED"
        else:
            # Output deadband to reduce jitter near alignment
            deadband = float(self._output_deadband)
            if self._do_lateral and not aligned_x and abs(speed_lat) >= deadband:
                cmd = "move_right" if error_x > 0 else "move_left"
                spd = int(abs(speed_lat))
                self._send_movement(cmd, spd)
                commands_sent.append(f"{cmd} spd={spd}")

            if self._do_vertical and not aligned_y and abs(speed_vert) >= deadband:
                cmd = "move_down" if error_y > 0 else "move_up"
                spd = int(abs(speed_vert))
                self._send_movement(cmd, spd)
                commands_sent.append(f"{cmd} spd={spd}")

            if self._do_forward and not aligned_area and abs(speed_fwd) >= deadband:
                cmd = "move_forward" if error_area < 0 else "move_back"
                spd = int(abs(speed_fwd))
                self._send_movement(cmd, spd)
                commands_sent.append(f"{cmd} spd={spd}")

            if not commands_sent:
                commands_sent.append("HOLD (deadband)")
            new_state = "TRACKING"

        # ── State transitions ─────────────────────────────────────────
        if new_state != self._state:
            if new_state == "ALIGNED":
                print(f"\n  *** ALIGNED with '{self._target_class}'  "
                      f"[cx={cx:.3f} cy={cy:.3f} area={area_ratio:.3f}]")
            elif new_state == "TRACKING" and self._state in ("LOST", "SEARCHING", "ALIGNED"):
                print(f"\n  >>> TRACKING '{self._target_class}'  "
                      f"confidence={target.confidence if target else 0:.2f}")
            self._state = new_state
            if new_state == "ALIGNED" and self._align_until_only:
                self._deactivate("aligned")
                return

        # ── Verbose output (rate-limited) ─────────────────────────────
        if should_print:
            self._last_verbose_time = now
            src = "KF" if kalman_predicted else "DET"
            mode = "PID" if use_pid else ("JUST" if self._force_just else "PROP")

            ax = "OK" if (aligned_x or not self._do_lateral) else ("R" if error_x > 0 else "L")
            ay = "OK" if (aligned_y or not self._do_vertical) else ("D" if error_y > 0 else "U")
            aa = "OK" if (aligned_area or not self._do_forward) else ("->" if error_area < 0 else "<-")

            cmds_str = ", ".join(commands_sent) if commands_sent else "none"

            print(
                f"  [{src}|{mode}] "
                f"ctr=({cx:.3f},{cy:.3f}) "
                f"err=({error_x:+.3f},{error_y:+.3f},{error_area:+.4f}) "
                f"align=[{ax},{ay},{aa}] "
                f"cmd=[{cmds_str}]"
            )

        # ── ROS log ───────────────────────────────────────────────────
        self.get_logger().info(
            f"[{self._active_command}] {self._state} "
            f"cx={cx:.3f} cy={cy:.3f} -> [{', '.join(commands_sent)}]",
            throttle_duration_sec=1.0,
        )

        # ── Publish status ────────────────────────────────────────────
        # Use bool() to ensure Python bool (ROS msg rejects numpy.bool_)
        self._publish_status(
            detected=True, kalman_predicted=kalman_predicted,
            error_x=error_x, error_y=error_y, error_area=error_area,
            pid_lat=speed_lat, pid_vert=speed_vert, pid_fwd=speed_fwd,
            tcx=cx, tcy=cy, tarea=area_ratio, tw=w, th=h,
            aligned_x=bool(aligned_x if self._do_lateral else True),
            aligned_y=bool(aligned_y if self._do_vertical else True),
            aligned_area=bool(aligned_area if self._do_forward else True),
        )

    # ══════════════════════════════════════════════════════════════════
    #  Helpers
    # ══════════════════════════════════════════════════════════════════

    def _send_movement(self, command: str, speed: float) -> None:
        """Send movement command. Speed is 0-100 percent (capped by active_gain)."""
        msg = DriverCommand()
        msg.command = command
        # Inspector expects 0-100 as percent; never exceed user's gain
        effective_max = float(self._max_speed) * (self._active_gain / 100.0)
        pct = (abs(speed) / effective_max) * self._active_gain if effective_max > 0 else 0
        msg.speed = int(max(1, min(100, round(pct))))
        msg.duration = 0.0
        self._cmd_pub.publish(msg)
        self._cmd_counter += 1

    def _send_stop(self) -> None:
        msg = DriverCommand()
        msg.command = "stop"
        msg.speed = 0
        msg.duration = 0.0
        self._cmd_pub.publish(msg)

    def _publish_status(
        self, *, detected: bool, kalman_predicted: bool,
        error_x: float, error_y: float, error_area: float,
        pid_lat: float, pid_vert: float, pid_fwd: float,
        tcx: float = 0.0, tcy: float = 0.0, tarea: float = 0.0,
        tw: float = 0.0, th: float = 0.0,
        aligned_x: bool = False, aligned_y: bool = False, aligned_area: bool = False,
    ) -> None:
        # Ensure Python bool for ROS msg (rejects numpy.bool_)
        ax = bool(aligned_x)
        ay = bool(aligned_y)
        aa = bool(aligned_area)
        msg = AlignmentStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.target_class = self._target_class
        msg.target_detected = bool(detected)
        msg.kalman_predicted = bool(kalman_predicted)
        msg.aligned_x = ax
        msg.aligned_y = ay
        msg.aligned_area = aa
        msg.fully_aligned = ax and ay and aa
        msg.error_x = float(error_x)
        msg.error_y = float(error_y)
        msg.error_area = float(error_area)
        msg.pid_output_lateral = float(pid_lat)
        msg.pid_output_vertical = float(pid_vert)
        msg.pid_output_forward = float(pid_fwd)
        msg.target_center_x = float(tcx)
        msg.target_center_y = float(tcy)
        msg.target_area_ratio = float(tarea)
        msg.target_width = float(tw)
        msg.target_height = float(th)
        msg.use_pid = bool(self._force_pid and not self._force_just)
        msg.use_kalman = bool(self._use_kalman and not self._force_just)
        self._status_pub.publish(msg)

    def destroy_node(self):
        print(f"\n{_SEP}")
        print(f"  Alignment controller shutting down.")
        print(f"  Frames: {self._frame_counter}  Commands: {self._cmd_counter}")
        print(f"{_SEP}\n")
        super().destroy_node()


class _TargetInfo:
    __slots__ = ("cx", "cy", "w", "h", "area", "confidence", "detected", "stamp")

    def __init__(self, cx: float, cy: float, w: float, h: float,
                 area: float, confidence: float, detected: bool, stamp: float) -> None:
        self.cx, self.cy, self.w, self.h = cx, cy, w, h
        self.area, self.confidence, self.detected, self.stamp = area, confidence, detected, stamp


def main(args=None):
    rclpy.init(args=args)
    node = AlignmentControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
