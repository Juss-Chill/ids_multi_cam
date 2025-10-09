#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose
from custom_msgs.msg import RadarDetection, RadarDetectionArray

from ctypes import *
import radar_utils
import time
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
import os

translation = [0.15, 0.50, 0.0] # in meters(make sure its not in inches)
roll    = np.deg2rad(0)         # in degrees
pitch   = np.deg2rad(0)
yaw     = np.deg2rad(0)

rot = R.from_euler('xyz',[roll, pitch, yaw])

T_mat           = np.eye(4)
T_mat[:3, :3]   = rot.as_matrix()
T_mat[:3, 3]    = np.asarray(translation).T

class RadarParse:
    def __init__(self):
        rospy.init_node('radar_cuboid_visualizer', anonymous=True)
        self.marker_pub = rospy.Publisher('/radar_markers', MarkerArray, queue_size=1000)
        self.radar_detections_pub = rospy.Publisher('/radar_detections', RadarDetectionArray, queue_size=1000)
        self.frame_id = rospy.get_param('~frame_id', 'os_sensor_right')
        self.rate = rospy.Rate(10)  # Hz
        print("Note Init success")
        radar_utils.utils_check()
       
        self.device_handle = radar_utils.open_device()
        radar_utils.set_baud_rate(self.device_handle)
        radar_utils.configure_canfd_mode(self.device_handle)

        self.dev_ch1 = radar_utils.init_channel(self.device_handle, 0)
        self.dev_ch2 = radar_utils.init_channel(self.device_handle, 1)
        radar_utils.start_channel(self.dev_ch2)

        marker_array = MarkerArray()
        radar_detections_msg = RadarDetectionArray()
        
        try:
            while True:
                raw_msgs = radar_utils.receive_can_data(self.dev_ch2)

                radar_detections_msg.header.stamp = rospy.Time.now()
                radar_detections_msg.header.frame_id = "os_sensor_right"
                
                print(type(raw_msgs), ",,,,,,",len(raw_msgs))
                marker_array.markers.clear()
                radar_detections_msg.detections.clear()

                for i in range(len(raw_msgs)):

                    radar_detection = RadarDetection()
              
                    if(raw_msgs[i]['can_id'] == "0x600"):
                        num_objs = int(raw_msgs[i]['data'][0], 16)
                        # body_speed = (( int(raw_msgs[i]['data'][3], 16) * 256 ) + int(raw_msgs[i]['data'][4], 16) )*0.1 - 20
                        print(f"Number of detected objects: {num_objs}")
                        # print(f"Speed: {body_speed}")
                    elif(raw_msgs[i]['can_id'] == "0x701"):

                        # differentiate between frames and subframes
                        obj_id = int(raw_msgs[i]['data'][0], 16) & 0x7F
                        frame_code = int(raw_msgs[i]['data'][0], 16) & 0x80

                        if(frame_code == 0x00):
                            # distance in m
                            # speed in m/s
                            # lat dist : y coord
                            # long dist : x coord

                            long_dist = (int(raw_msgs[i]['data'][1], 16) * 32 + (int(raw_msgs[i]['data'][2], 16) >> 3)) * 0.05 - 100
                            lat_dist = (((int(raw_msgs[i]['data'][2], 16) & 0x07) * 256) + int(raw_msgs[i]['data'][3], 16)) * 0.05 - 50 
                            
                            # The below is the velocity information componnet wise, can be negative
                            long_speed = (((int(raw_msgs[i]['data'][4], 16) * 4) + (int(raw_msgs[i]['data'][5], 16) >> 6)) * 0.25 - 128 )
                            lat_speed  = (((int(raw_msgs[i]['data'][5], 16) & 0x3F) * 8) + (int(raw_msgs[i]['data'][6], 16) >> 5)) * 0.25 - 64
                            radar_detection.velocity_mps.x, radar_detection.velocity_mps.y, radar_detection.velocity_mps.z = long_speed, lat_speed, 0.
                            radar_detection.speed_mps = math.sqrt(long_speed*long_speed + lat_speed*lat_speed)

                            # mps to KMPH
                            long_speed *= 3.6
                            lat_speed  *= 3.6
                            radar_detection.velocity_kmph.x, radar_detection.velocity_kmph.y, radar_detection.velocity_kmph.z = long_speed, lat_speed, 0.

                            Range = math.sqrt(long_dist*long_dist + lat_dist*lat_dist)
                            radar_detection.range = Range

                            Speed = math.sqrt(long_speed*long_speed + lat_speed*lat_speed)
                            radar_detection.speed_kmph = Speed 

                            radar_in_lidar = T_mat @ np.asarray([long_dist, lat_dist, 0., 1.]).T #(4,)
                            
                            obj_x   = radar_in_lidar[0]
                            obj_y   = radar_in_lidar[1]

                            # Radar's x-axis is longitudinal, current code does not retrieve the height of the object
                            radar_detection.position.x, radar_detection.position.y, radar_detection.position.z = long_dist, lat_dist, 0.
                            # velocity remains constant as the lidar and radar are mounted static, hence not transformed
                            
                            # write a seperate node for the radar visualzation
                            # visualization
                            marker = Marker()
                            marker.header.frame_id = "os_sensor_right"
                            marker.header.stamp = rospy.Time.now()
                            marker.ns = "radar_objects"
                            marker.id = obj_id
                            marker.type = Marker.CUBE
                            marker.action = Marker.ADD
                            
                            marker.pose.position.x = obj_x
                            marker.pose.position.y = obj_y
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

                            text_marker = Marker()
                            text_marker.header.frame_id = "os_sensor_right"
                            text_marker.header.stamp = rospy.Time.now()
                            text_marker.ns = "radar_objects"
                            text_marker.id = obj_id + 100  # Ensuring unique ID for the text marker
                            text_marker.type = Marker.TEXT_VIEW_FACING
                            text_marker.action = Marker.ADD

                            # Positioning the text above the object (slightly raised)
                            text_marker.pose.position.x = obj_x 
                            text_marker.pose.position.y = obj_y
                            text_marker.pose.position.z = 3.5  # Adjust height if needed
                            text_marker.pose.orientation.w = 1.0

                            # Format text with speed and distance
                            # text_marker.text = f"Dist: {Range:.3f} m\nSpeed: ({long_speed:.3f} , {lat_speed:.3f}) km/h"
                            # text_marker.text = f"Dist: {Range:.3f} m\nSpeed: {Speed:.3f} km/h"
                            text_marker.text = f"(X, Y): ({obj_x:.3f}, {obj_y:.3f}) m\nSpeed: {Speed:.3f} km/h"

                            text_marker.scale.z = 0.3  # Adjust text size
                            text_marker.color.r = 1.0
                            text_marker.color.g = 1.0
                            text_marker.color.b = 1.0
                            text_marker.color.a = 1.0

                            text_marker.lifetime = rospy.Duration(0.01)  # keeps refreshing

                            marker_array.markers.append(text_marker)

                            self.marker_pub.publish(marker_array)
                        
                        elif(frame_code == 0x80):
                            # print("Sub-frame: ", obj_id)
                            # parse sub-frame data
                            pass
                    
                    # store it in the array to publish
                    radar_detections_msg.detections.append(radar_detection)

                self.radar_detections_pub.publish(radar_detections_msg)

        except KeyboardInterrupt:
            radar_utils.close_device(self.dev_ch1, self.dev_ch2, self.device_handle)

if __name__ == '__main__':
    try:
        visualizer = RadarParse()
        
    except rospy.ROSInterruptException:
        pass
