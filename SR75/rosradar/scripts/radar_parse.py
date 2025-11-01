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
        print("Node Init success")
        radar_utils.utils_check()
       
        self.device_handle = radar_utils.open_device()
        radar_utils.set_baud_rate(self.device_handle)
        radar_utils.configure_canfd_mode(self.device_handle)

        self.dev_ch1 = radar_utils.init_channel(self.device_handle, 0)
        self.dev_ch2 = radar_utils.init_channel(self.device_handle, 1)
        radar_utils.start_channel(self.dev_ch2)
    
    def get_data(self):
        """
            Function that decodes the data and publshed into ROS message format
            To visualzie these markers run the radar_msg_visualization node
        """
        radar_detections_msg = RadarDetectionArray()
        radar_detections_msg.header.frame_id = "os_sensor_right"
        try:
            while True:
                raw_msgs = radar_utils.receive_can_data(self.dev_ch2)
                radar_detections_msg.header.stamp = rospy.Time.now()
                radar_detections_msg.detections.clear()

                for i in range(len(raw_msgs)):

                    if(raw_msgs[i]['can_id'] == "0x600"):
                        # Check the header for the number of detections
                        num_objs = int(raw_msgs[i]['data'][0], 16)
                        # body_speed = (( int(raw_msgs[i]['data'][3], 16) * 256 ) + int(raw_msgs[i]['data'][4], 16) )*0.1 - 20
                        print(f"Number of detected objects: {num_objs}")
                        # print(f"Speed: {body_speed}")
                    elif(raw_msgs[i]['can_id'] == "0x701"):
                        # Decode the payload

                        # Update the ROS Message with the detection data
                        radar_detection = RadarDetection()

                        # differentiate between frames and subframes
                        obj_id = int(raw_msgs[i]['data'][0], 16) & 0x7F
                        frame_code = int(raw_msgs[i]['data'][0], 16) & 0x80

                        radar_detection.uid = obj_id

                        if(frame_code == 0x00):
                            # subframe-A

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
                            radar_detection.position.x, radar_detection.position.y, radar_detection.position.z = obj_x, obj_y, 0.
                            # velocity remains constant as the lidar and radar are mounted static, hence not transformed
                            
                            # store it in the array to publish
                            radar_detections_msg.detections.append(radar_detection)
                        
                        elif(frame_code == 0x80):
                            # subframe-B

                            # Refer data sheet for more informaton on the data present in the subframe
                            # print("Sub-frame: ", obj_id)
                            # parse sub-frame data
                            pass
                
                self.radar_detections_pub.publish(radar_detections_msg)

        except KeyboardInterrupt:
            radar_utils.close_device(self.dev_ch1, self.dev_ch2, self.device_handle)

if __name__ == '__main__':
    try:
        visualizer = RadarParse()
        visualizer.get_data()
        
    except rospy.ROSInterruptException:
        pass
