builtin_interfaces/Time stamp

# Whether a message from each required input has ever been received.
bool gpsfix_received
bool quality_received
bool rtcm_received

# Whether the latest message arrived within its configured timeout.
bool gpsfix_fresh
bool quality_fresh
bool rtcm_fresh

# Payload and cross-topic consistency checks.
bool fix_valid
bool quality_valid
bool timestamps_aligned
bool integrity_ok

# Age is measured from receipt by the monitor, so it still detects a stopped
# publisher when GPS/ROS time is not advancing. -1.0 means no message received.
float32 gpsfix_age_s
float32 quality_age_s
float32 rtcm_age_s
float32 timestamp_delta_s
float32 covariance_scale
float32 horizontal_stddev_m
float32 vertical_stddev_m

int8 navsat_status
uint8 gps_quality
string reason
