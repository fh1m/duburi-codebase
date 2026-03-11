# Status of an individual camera managed by vision_inspector.
# Published as part of CameraStatusArray on /vision_inspector/status.

string camera_name         # logical name (e.g. "forward", "downward")
bool   connected           # true if currently streaming frames
string device_path         # opened device (e.g. "/dev/video0") or empty
uint32 width               # current frame width  (0 if disconnected)
uint32 height              # current frame height (0 if disconnected)
float32 actual_fps         # measured FPS (rolling window)
uint64  frame_count        # total frames captured since connection
float32 uptime             # seconds since connection was established
bool    calibrated         # true if calibration data was loaded
string  last_error         # last error message or empty
