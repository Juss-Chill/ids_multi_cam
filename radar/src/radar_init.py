#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
import logging

from kld7 import KLD7
from visualization_msgs.msg import Marker
from custom_msgs.msg import object_info

# Configure logging
logging.basicConfig(level=logging.INFO)
                    

def radar_init():
    """
        initilize the radar and publish the marker
    """
    rospy.init_node('radar_data')

    # varaible initialization
    x = []
    y = []

    #marker publisher
    marker_pub = rospy.Publisher('/radar_data_viz', Marker, queue_size=10)
    text_pub = rospy.Publisher('/radar_data_text_viz', Marker, queue_size=10)

    #Dominant object information pub
    object_info_pub = rospy.Publisher('/dominant_obj_info', object_info, queue_size=10)

    marker = Marker()
    text_marker = Marker()
    object_data = object_info()

    marker.header.frame_id = "velodyne"
    text_marker.header.frame_id = "velodyne"
    object_data.header.frame_id = "velodyne"

    marker.type = Marker().CUBE # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    text_marker.type = Marker.TEXT_VIEW_FACING

    # Set the scale of the marker
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 1.0

    text_marker.scale.x = 0.25 # font size
    text_marker.scale.y = 0.25
    text_marker.scale.z = 0.25 

    # Set the color
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 0.5

    # white color
    text_marker.color.r = 1.0
    text_marker.color.g = 1.0
    text_marker.color.b = 1.0
    text_marker.color.a = 1.0


    while not rospy.is_shutdown():
        with KLD7(port="/dev/ttyUSB0", baudrate=115200) as radar:
            try:
                print(radar._param_dict)
                for target_info in radar.stream_TDAT():
                    '''
                    Note : values of the object 'target_info' shouldnot be modified, Modifying the value will result in an error
                    '''
                    if target_info is not None:

                        real_speed = 0.
                        Vx = 0.
                        Vy = 0.

                        # print(target_info)
                        print(f'Speed : {target_info.speed}, Distance : {target_info.distance}, Angle : {target_info.angle}, magnitude : {target_info.magnitude}')

                        # speed of the object is same as obtained data if the object moves radial to the sensor
                        # otherwise we need to compenste it (source : datasheet , page = 6/23).
                        # Vreal = Vmeasured / cos(measure_angle)

                        if target_info.angle != 0:
                            real_speed= target_info.speed/np.cos(np.radians(target_info.angle))
                            print("speed modified")
                        
                        
                        x_cord = np.sin(np.radians(target_info.angle))*target_info.distance
                        y_cord = np.cos(np.radians(target_info.angle))*target_info.distance

                        Vx = real_speed * np.sin(np.radians(target_info.angle))
                        Vy = real_speed * np.cos(np.radians(target_info.angle))
                        
                        #plot the co-ordinates
                        x.append(x_cord)
                        y.append(y_cord)

                        marker.id = 1 # Unique id assigned to this marker. It is your responsibility to keep these unique within your namespace. 
                        text_marker.id = 10

                        # erase the existing marker
                        marker.action = Marker.DELETEALL
                        text_marker.action = Marker.DELETEALL
                        marker_pub.publish(marker)
                        text_pub.publish(text_marker)

                        # Set the pose of the markers
                        marker.pose.position.x = x_cord
                        marker.pose.position.y = y_cord
                        # print(f'(x,y) = {x,y}')
                        marker.pose.position.z = 0
                        marker.pose.orientation.x = 0.0
                        marker.pose.orientation.y = 0.0
                        marker.pose.orientation.z = 0.0
                        marker.pose.orientation.w = 1.0

                        text_marker.pose.position.x = x_cord
                        text_marker.pose.position.y = y_cord
                        text_marker.pose.position.z = 1.5         # just to be visible above the sphere

                        text_marker.pose.orientation.x = 0.0
                        text_marker.pose.orientation.y = 0.0
                        text_marker.pose.orientation.z = 0.0
                        text_marker.pose.orientation.w = 1.0


                        marker.action = Marker.ADD
                        text_marker.action = Marker.ADD

                        text_marker.text = "Vx : " + str(f"{Vx:.3f}") + "  Vy : " + str(f"{Vy:.3f}") + "\nX : " + str(f"{x_cord:.3f}") + "  Y : " + str(f"{y_cord:.3f}") 

                        # publish the Dominant Object information message
                        object_data.distance = target_info.distance
                        object_data.speed = real_speed
                        object_data.angle = target_info.angle
                        object_data.magnitude = target_info.magnitude
                        object_data.obj_flag = True # will set to True if there is an existing object

                        object_data.header.stamp = marker.header.stamp = text_marker.header.stamp =rospy.Time.now()

                        # publisht the marker and the dominant object info
                        marker_pub.publish(marker)
                        text_pub.publish(text_marker)
                        object_info_pub.publish(object_data)
                    else:
                        # erase the existing marker since there exists no detected object
                        marker.action = Marker.DELETEALL
                        text_marker.action = Marker.DELETEALL

                        # publish the Dominant Object information message
                        object_data.distance = 0.
                        object_data.speed = 0.
                        object_data.angle = 0.
                        object_data.magnitude = 0.
                        object_data.obj_flag = False # will set to True if there is an existing object

                        object_data.header.stamp = marker.header.stamp = text_marker.header.stamp =rospy.Time.now()

                        marker_pub.publish(marker)
                        text_pub.publish(text_marker)
                        object_info_pub.publish(object_data)

            except Exception as e:
                logging.error("An error occurred: %s", e)

            plt.scatter(x,y)       
            plt.show()
    


if __name__ == '__main__':
    try:
        radar_init()
    except rospy.ROSInterruptException:
        #print this error meaasge
        rospy.logerr("ROS Interrupt Exception! Shutting down the radar initlization node.")
    except Exception as e:
        logging.error("Node terminated with error: %s", e)