#!/usr/bin/env python3

import rospy
from custom_msgs.msg import RadarDetection, RadarDetectionArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose

"""
Subscribes to Radar detection data and visualizes it as cuboid markers with text infromation

Subscribers: 
    RadarDetectionArray

Publisher:
    MarkerArray

Default FrameID : os_sensor_right
    Right lidar


"""

class RadarMsgViz:
    def __init__(self):
        rospy.init_node('Radar_Message_Vizualization_Node', anonymous=True)
        self.marker_pub = rospy.Publisher('/radar_markers', MarkerArray, queue_size=1000) # high queue to compensate B.W
        self.radar_detec_sub = rospy.Subscriber('/radar_detections', RadarDetectionArray, self.detections_cb)

    def detections_cb(self, detections):
        marker_array = MarkerArray()
        marker_array.markers.clear()

        print("Number of detections : ", len(detections.detections))
        for detection in detections.detections:
            # visualization of cubiod markers
            marker = Marker()
            marker.header = detections.header
            marker.ns = "radar_objects"
            marker.id = detection.uid
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position.x = detection.position.x
            marker.pose.position.y = detection.position.y
            marker.pose.position.z = 0.
            marker.pose.orientation.w = 1
        
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 3

            marker.color.r = 0
            marker.color.g = 1
            marker.color.b = 0
            marker.color.a = 0.8

            marker.lifetime = rospy.Duration(0.01)  # keeps refreshing
            marker_array.markers.append(marker)

            # visualization of Text markers
            text_marker = Marker()
            text_marker.header.frame_id = "os_sensor_right"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "radar_objects"
            text_marker.id = detection.uid + 1000  # Ensuring unique ID for the text marker
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            # Positioning the text above the object (slightly raised)
            text_marker.pose.position.x = detection.position.x 
            text_marker.pose.position.y = detection.position.y
            text_marker.pose.position.z = 3.5  # Adjust height if needed
            text_marker.pose.orientation.w = 1.0

            # Format text with speed and distance
            # text_marker.text = f"Dist: {Range:.3f} m\nSpeed: ({long_speed:.3f} , {lat_speed:.3f}) km/h"
            # text_marker.text = f"Dist: {Range:.3f} m\nSpeed: {Speed:.3f} km/h"
            text_marker.text = f"(X, Y): ({detection.position.x :.3f}, {detection.position.y :.3f}) m\nSpeed: {detection.speed_kmph:.3f} km/h"

            text_marker.scale.z = 0.3  # Adjust text size
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            text_marker.lifetime = rospy.Duration(0.01)  # keeps refreshing
            marker_array.markers.append(text_marker)

            # print("Detected position : ", detection.position.x , " , ", detection.position.y)

        self.marker_pub.publish(marker_array)

if __name__ == "__main__":
    try:
        viz_radar_data = RadarMsgViz()
        rospy.spin()
        print("Node execution done")
        
    except:
        print("Not sure what's the issue...:-D")