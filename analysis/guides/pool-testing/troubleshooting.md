# Pool Troubleshooting

## Quick Fixes

| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| No heartbeat | Serial issue | Check cable, try different port |
| Arm timeout | Pre-arm checks | Check QGC, battery voltage |
| Depth drift | PID not active | Use `~depth` not `depth` |
| Yaw oscillation | PID too aggressive | Reduce yaw_kp |
| No camera | Wrong device | Run `camera_enum` |
| No detections | Model not loaded | Check YOLO path |

## Detailed Troubleshooting

### "No telemetry" Error

```bash
# Check serial port
ls /dev/ttyACM*

# If not found, check USB
dmesg | tail -20

# Try different port
ros2 run mavlink_inspector inspector --ros-args \
    -p connection_port:=/dev/ttyACM1
```

### Arm Fails

```bash
# Check pre-arm status
# Look at /mavlink/events for failure reason

# Common issues:
# - Battery too low (< 10.5V)
# - Accelerometer not calibrated
# - Compass not calibrated

# Bypass for testing (not recommended):
# Connect QGroundControl, disable pre-arm checks
```

### Vehicle Doesn't Move

```bash
# Check armed status
status

# Check RC override is being sent
ros2 topic hz /mavlink/rc_override

# Check command received
ros2 topic echo /driver/feedback
```
