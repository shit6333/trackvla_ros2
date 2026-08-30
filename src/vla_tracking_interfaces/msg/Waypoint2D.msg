# One predicted pose on the ground plane, expressed in the parent trajectory's
# frame. Deliberately project-owned rather than geometry_msgs/Pose2D, which ROS
# has marked deprecated since Foxy, and rather than the full 3D
# geometry_msgs/Pose, which would force a yaw/quaternion conversion on every
# publish and every execution step for a model that only predicts planar poses.
#
# The values are normalized, not metric. Divided by the parent trajectory's dt
# they give the fraction of the robot's full speed the model asked for, bounded
# to [-1, 1]. A consumer applies the robot's own full-speed values to obtain
# metres and radians per second; nothing here carries a length. See D019.

float64 x      # forward, normalized
float64 y      # lateral, left positive, normalized
float64 theta  # yaw, normalized
