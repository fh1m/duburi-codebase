# Driver command for mavlink_driver
# Used by mavlink_runner and mission nodes to control the AUV

string command       # move_forward, move_back, move_left, move_right,
                     # move_up, move_down, yaw_angle, yaw_to_heading, yaw_left, yaw_right,
                     # set_depth, stop, arm, disarm, set_mode,
                     # open_grabber, close_grabber,
                     # lat_align, dep_align, align, align_forward,
                     # pid_lat_align, pid_dep_align, pid_align, pid_align_forward,
                     # just_lat_align, just_dep_align, just_align, just_align_forward,
                     # vision_stop

string mode          # MANUAL, ALT_HOLD, STABILIZE (for set_mode)
float32 depth        # target depth in meters (positive = below surface)
float32 angle        # target heading in degrees (0-360) for yaw_angle
float32 duration     # duration in seconds (0 = indefinite)
int32 speed          # PWM offset from 1500 or percentage
string status        # optional status/response field
