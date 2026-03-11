# Visual servo alignment status from vision alignment controller.
# Published on /vision/alignment_status for telemetry and mission planners.

std_msgs/Header header

string target_class          # class being tracked (e.g. "bottle", "gate")
bool target_detected         # is the target currently visible (or KF-predicted)
bool kalman_predicted        # true if position is KF prediction (no raw detection this frame)

bool aligned_x               # lateral alignment within dead zone
bool aligned_y               # vertical alignment within dead zone
bool aligned_area            # distance (area) within target range
bool fully_aligned           # all three axes aligned

float32 error_x              # normalized lateral error (-0.5 to +0.5)
float32 error_y              # normalized vertical error (-0.5 to +0.5)
float32 error_area           # area error (current_ratio - target_ratio)

float32 pid_output_lateral   # current lateral control output (PWM offset)
float32 pid_output_vertical  # current vertical control output (PWM offset)
float32 pid_output_forward   # current forward control output (PWM offset)

float32 target_center_x      # tracked target normalized center x (0.0-1.0)
float32 target_center_y      # tracked target normalized center y (0.0-1.0)
float32 target_area_ratio    # tracked target bbox area / frame area
float32 target_width         # tracked target normalized bbox width (0.0-1.0)
float32 target_height        # tracked target normalized bbox height (0.0-1.0)

bool use_pid                 # whether PID is currently active
bool use_kalman              # whether Kalman filter is currently active
