# The number of blobs being tracked that have been promoted to a state of: ACTIVE_TARGET
int32 trackable

# The number of blobs currently being tracked of all states
int32 alive

# The total number of blobs tracked, past and present
int32 started

# The total number of blobs no longer bing tracked
int32 ended

# Any hints please set them here
string tracking_hint

# Has the max blob threshold been hit
bool max_blobs_reached

# Are we currently recording
bool recording

# The percentage cloud cover currently being determined
float32 percentage_cloud_cover

# Is it currently day or night
int32 day_night_enum
int32 avg_brightness