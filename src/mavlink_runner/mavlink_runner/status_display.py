"""
Status display for the Duburi AUV CLI runner.
"""

import re
import time


def print_status(last_state, last_diag, last_state_time):
    """Print a human-friendly status dashboard.

    Args:
        last_state: Latest VehicleState message (or None).
        last_diag: Latest VehicleDiagnostics message (or None).
        last_state_time: Monotonic timestamp of last VehicleState received.
    """
    s = last_state
    d = last_diag
    if s is None:
        print('  No telemetry received yet. Is mavlink_inspector running?')
        return

    G = '\033[92m'  # green
    DIM = '\033[90m'  # dim grey
    R = '\033[0m'    # reset

    arm_label = f'{G}ARMED{R}' if s.armed else f'{DIM}DISARMED{R}'
    depth_str = f'{abs(s.depth):.2f}m' if s.depth != 0.0 else '0.00m (sfc)'
    compass = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']
    heading_arrow = compass[int((s.yaw + 22.5) % 360 / 45)]

    # Battery bar: 16.8V=full, 14.0V=empty for 4S LiPo
    batt_pct = max(0, min(100, (s.voltage - 14.0) / (16.8 - 14.0) * 100)) if s.voltage > 0 else 0
    n = int(batt_pct / 10)
    batt_bar = f'{G}{"█" * n}{DIM}{"░" * (10 - n)}{R}'

    def row(text: str):
        """Print a row padded to fixed width (50 visible chars)."""
        # Strip ANSI to measure visible length
        vis = re.sub(r'\033\[[0-9;]*m', '', text)
        pad = 50 - len(vis)
        if pad < 0:
            pad = 0
        print(f'  │{text}{" " * pad}│')

    sep = '─' * 50
    print(f'  ┌{sep}┐')
    row(f'{"Duburi AUV Status":^50}')
    print(f'  ├{sep}┤')
    row(f'  Motors: {arm_label}    Mode: {s.flight_mode}')
    row(f'  Depth:  {depth_str:<12s}Heading: {s.yaw:5.1f}° {heading_arrow}')
    row(f'  Pitch:  {s.pitch:+6.1f}°       Roll:    {s.roll:+6.1f}°')
    if d:
        row(f'  Yaw rate: {d.heading_rate:+.1f}°/s')
    print(f'  ├{sep}┤')
    row(f'  Battery: {s.voltage:.1f}V  {s.current:.1f}A  {batt_bar} {batt_pct:.0f}%')
    if d:
        row(f'  Pressure: {d.pressure:.0f} hPa   Temp: {d.temperature:.1f}°C')
        row(f'  CPU load: {d.cpu_load:.0f}%')
    print(f'  ├{sep}┤')
    if d:
        servos = ' '.join(f'{v:4d}' for v in d.servo_output)
        rc_vals = ' '.join(f'{v:4d}' for v in d.rc_channels)
        row(f'  Servos: {servos}')
        row(f'  RC in:  {rc_vals}')
    else:
        row('  (diagnostics not yet received)')
    # Inspector health
    if last_state_time > 0:
        age = time.monotonic() - last_state_time
        if age > 5.0:
            Y = '\033[93m'
            row(f'  {Y}⚠ Telemetry stale ({age:.0f}s ago){R}')
    print(f'  └{sep}┘')
