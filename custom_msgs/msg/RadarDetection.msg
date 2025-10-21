# Need to use with the RadarDetectionArray, since timestamp has to be present

# unitque ID that will be assigned to different 
uint16 uid

# long dist : x coord
# lat dist : y coord
# Z is assumed to be 0
geometry_msgs/Point position

# distance in m
float32 range

# velocity_mps in m/s
geometry_msgs/Vector3 velocity_mps

# velocity_kmph in km/h
geometry_msgs/Vector3 velocity_kmph

# speed in km/h
float32 speed_kmph

# speed in mps
float32 speed_mps
