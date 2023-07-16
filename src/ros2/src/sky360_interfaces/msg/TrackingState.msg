# Defines the tracking state of the application and is updated on every frame processed

std_msgs/Header header

# The number of blobs being tracked that have been promoted to a state of: ACTIVE_TARGET
int32 trackable

# The number of blobs currently being tracked of all states
int32 alive

# The total number of blobs tracked, past and present
int32 started

# The total number of blobs no longer bing tracked
int32 ended