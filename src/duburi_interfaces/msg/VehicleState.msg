# Current vehicle state from mavlink_inspector (telemetry)

std_msgs/Header header

bool armed           # motors armed
string flight_mode   # current ArduSub mode
float32 depth        # current depth (m, negative below surface)
float32 yaw          # heading in degrees (0-360)
float32 pitch        # pitch in degrees
float32 roll         # roll in degrees
float32 voltage      # battery voltage (V)
float32 current      # battery current (A)
