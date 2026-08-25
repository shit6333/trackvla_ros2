# One predicted pose on the ground plane, expressed in the parent trajectory's
# frame. Deliberately project-owned rather than geometry_msgs/Pose2D, which ROS
# has marked deprecated since Foxy, and rather than the full 3D
# geometry_msgs/Pose, which would force a yaw/quaternion conversion on every
# publish and every execution step for a model that only predicts planar poses.

float64 x      # forward, metres
float64 y      # lateral, left positive, metres
float64 theta  # yaw, radians
