# This includes both IMU and GPS information
Header header

# IMU information
nav_msgs/Odometry odometry_ecef
nav_msgs/Odometry odometry_enu
nav_msgs/Odometry odometry_enu_smooth
nav_msgs/Odometry odometry_smooth # ECEF frame
geometry_msgs/Vector3Stamped ypr # This is just an estimate with bias

# GPS information
sensor_msgs/NavSatFix odometry_llh

