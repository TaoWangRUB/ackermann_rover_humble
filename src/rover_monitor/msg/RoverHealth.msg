int32 seq
int64 timestamp
rover_monitor/CamStatus[] cameras
rover_monitor/Px4Status px4
rover_monitor/JetsonStatus jetson
rover_monitor/Nav2Status nav2
float32 slam_latency_ms  # Legacy field name: stores map->odom TF age/freshness in ms.
string overall_health
string[] active_alerts
