# Defines a point that is part of the track or path of a 2D detection.

# The center point of this trajectory

geometry_msgs/Pose2D center

# The tracking state of this point, we currently support:
#   PROVISIONARY_TARGET = 1
#   ACTIVE_TARGET = 2
#   LOST_TARGET = 3

int32 tracking_state