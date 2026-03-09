# Feedback from inspector → caller for command acknowledgement.
# Published on /driver/feedback after each command is processed.
#
# status values:
#   "accepted"  — command was accepted and is being executed
#   "reached"   — target achieved (heading, depth, etc.)
#   "rejected"  — command was rejected (e.g. not armed, invalid params)
#   "timeout"   — command timed out before reaching target
#   "completed" — duration-based command finished normally

std_msgs/Header header
string command           # echo of original DriverCommand.command
string status            # accepted, reached, rejected, timeout, completed
float32 error            # current error (depth m, heading deg) for progress
string detail            # human-readable detail (rejection reason, etc.)
