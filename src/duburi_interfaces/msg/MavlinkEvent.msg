# Event published by mavlink_inspector for any Pixhawk/MAVLink activity

std_msgs/Header header

string event_type    # arm, disarm, mode_change, movement, heartbeat_lost, etc.
string description   # human-readable description
string raw_data      # optional JSON or raw message dump for debugging
