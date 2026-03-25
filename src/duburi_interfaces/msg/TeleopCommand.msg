# Teleop command — gamepad/joystick input for direct thruster control.
# Replaces the overloaded DriverCommand.teleop pattern with clean semantics.
#
# Published by teleop_driver on /driver/teleop, consumed by mavlink_inspector.

float32 linear_x    # forward (+) / back (-) axis [-1.0, 1.0]
float32 linear_y    # right (+) / left (-) axis [-1.0, 1.0]
float32 linear_z    # up (+) / down (-) axis [-1.0, 1.0]
float32 angular_z   # yaw: CCW/left (+) / CW/right (-) axis [-1.0, 1.0]
int32 speed         # max PWM offset from neutral (default 200)
bool idle           # true = joystick centred, clear movement without full stop
