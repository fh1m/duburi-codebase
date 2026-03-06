# Single object detection result
string class_name        # detected class label (e.g. "person", "gate")
int32 class_id           # numeric class id from YOLO model
float32 confidence       # detection confidence 0.0-1.0

# Bounding box (pixel coordinates)
int32 bbox_x             # top-left x
int32 bbox_y             # top-left y
int32 bbox_w             # width
int32 bbox_h             # height

# Center point (normalized 0.0-1.0 for resolution independence)
float32 center_x         # center x normalized (bbox center / image width)
float32 center_y         # center y normalized (bbox center / image height)
