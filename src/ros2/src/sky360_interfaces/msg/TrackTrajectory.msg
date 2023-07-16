# Defines a trajectory or path of a 2D detection.

# ID used for consistency across multiple detection messages. Detections
# of the same object in different detection messages should have the same id.
# This field may be empty.
string id

# The points representing the trajectory of this detection
TrackPoint[] trajectory