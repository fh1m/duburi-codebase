# Diagnostics and health info for the AUV.
# Published at lower rate (2 Hz) by mavlink_inspector for monitoring.
# Use VehicleState.msg for control-loop data (10 Hz).

std_msgs/Header header

# Angular velocity
float32 heading_rate       # yaw rate (deg/s) from ATTITUDE.yawspeed

# Environment
float32 pressure           # absolute pressure (hPa) from SCALED_PRESSURE
float32 temperature        # temperature (°C) from SCALED_PRESSURE.temperature

# Servo/thruster outputs (PWM values Pixhawk sends to motors)
int32[8] servo_output      # SERVO_OUTPUT_RAW ch1-ch8 (µs, 0 = not connected)

# RC channel readback (what Pixhawk is actually using internally)
int32[8] rc_channels       # RC_CHANNELS ch1-ch8 (µs, 0 = not available)

# System health
float32 cpu_load           # Pixhawk CPU load (%) from SYS_STATUS.load
