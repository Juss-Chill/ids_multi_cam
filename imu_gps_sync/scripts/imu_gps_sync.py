#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import message_filters

from custom_msgs.msg import imu_data

class FixpositionSyncNode:
    def __init__(self):
        rospy.init_node("fixposition_sync_node", anonymous=True)

        rospy.loginfo("Syncing IMU and GPS Data")

        # --- Subscribers ---
        # gnss1_sub = message_filters.Subscriber("/fixposition/gnss1", NavSatFix)
        # gnss2_sub = message_filters.Subscriber("/fixposition/gnss2", NavSatFix)
        # imu_ypr_sub = message_filters.Subscriber("/fixposition/imu_ypr", Vector3Stamped)
        # tf_sub = message_filters.Subscriber("/tf", TFMessage)
        # poiimu_sub = message_filters.Subscriber("/fixposition/poiimu", Imu)

        self.odom_ecef_sub = message_filters.Subscriber("/fixposition/odometry_ecef", Odometry)
        self.odom_enu_sub = message_filters.Subscriber("/fixposition/odometry_enu", Odometry)
        self.odom_enu_smooth_sub = message_filters.Subscriber("/fixposition/odometry_enu_smooth", Odometry)
        self.odom_llh_sub = message_filters.Subscriber("/fixposition/odometry_llh", NavSatFix)
        self.odom_smooth_sub = message_filters.Subscriber("/fixposition/odometry_smooth", Odometry)
        self.ypr_sub = message_filters.Subscriber("/fixposition/ypr", Vector3Stamped)
        
        # --- Synchronizer ---
        # ApproximateTimeSynchronizer tolerates small differences between timestamps
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [
                self.odom_ecef_sub, self.odom_enu_sub, self.odom_enu_smooth_sub,
                self.odom_llh_sub, self.odom_smooth_sub, self.ypr_sub
            ],
            queue_size=100,      # Number of messages to store for matching
            slop=0.05,          # Allowed time difference between messages (in seconds)
            allow_headerless=True
        )

        self.ts.registerCallback(self.synced_callback)

        # --- Publisher ---
        self.pub_fused = rospy.Publisher("/imu_gps_synced_data", imu_data, queue_size=100)

        rospy.loginfo("Subscribers initialized and synchronizer set up.")
        rospy.spin()

    def synced_callback(
        self,odom_ecef, odom_enu, odom_enu_smooth, odom_llh, odom_smooth, ypr
    ):
        print("IMU_GPS_Sync Success")
        """
        Called whenever all subscribed messages are approximately synchronized in time.
        """
        rospy.loginfo_once("Synchronized callback triggered.")
        imu_gps_synced_msg = imu_data()
        imu_gps_synced_msg.header.frame_id = "IMU_GPS"
        imu_gps_synced_msg.header.stamp = rospy.Time.now()
        imu_gps_synced_msg.odometry_ecef = odom_ecef
        imu_gps_synced_msg.odometry_enu = odom_enu
        imu_gps_synced_msg.odometry_enu_smooth = odom_enu_smooth
        imu_gps_synced_msg.odometry_smooth = odom_smooth
        imu_gps_synced_msg.ypr = ypr
        imu_gps_synced_msg.odometry_llh = odom_llh

        # publish the synced msg
        self.pub_fused.publish(imu_gps_synced_msg)

if __name__ == "__main__":
    try:
        FixpositionSyncNode()
    except rospy.ROSInterruptException:
        pass
