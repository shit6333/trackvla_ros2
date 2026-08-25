# A short-horizon trajectory predicted by a VLA backend.
#
# header.stamp is the timestamp of the observation the prediction was made
# from, not the time it was published. Consumers use it to reject stale
# predictions. header.frame_id is the robot frame the waypoints are relative
# to, normally base_link.
std_msgs/Header header

# Identifies the backend that produced this prediction, so that a recorded bag
# is interpretable without the launch configuration.
string backend_name

# Waypoint 0 is the trajectory origin, i.e. the robot's pose at prediction
# time. Executable waypoints therefore begin at index 1.
Waypoint2D[] waypoints

# Seconds between consecutive waypoints. Supplied by the backend or its
# checkpoint metadata and never inferred by a consumer.
float32 dt

# False when the backend could not produce a usable prediction. Consumers must
# not execute an invalid trajectory.
bool valid

# Human-readable detail, especially when valid is false.
string status
