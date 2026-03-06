# Array of detections from a single frame
std_msgs/Header header   # timestamp + frame_id
Detection[] detections   # list of detections
int32 image_width        # source image width
int32 image_height       # source image height
