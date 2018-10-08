#!/usr/bin/env python

import rospy
from robot import *
from cs373_visualization.msg import MeasurementInfo
from visualization_msgs.msg import Marker

#-------------------------init robot, rosnode, publisher--------------------------------------#
init_x = 0.0
init_y = 0.0
test_target = robot(init_x, init_y, 0.5, 2*pi/34.0, 1.5)
test_target.set_noise(0.0, 0.0, 0.0)
rospy.init_node('runaway_robot')
meas_pub = rospy.Publisher('rr_measurement', MeasurementInfo, queue_size=10)
robot_pub = rospy.Publisher('rr_visualization', Marker, queue_size=10)

#-------------------------------------marker setting------------------------------------------#

marker = Marker()
marker.header.frame_id = "/world"
marker.id = 0
marker.ns = "tiles"
marker.header.stamp = rospy.get_rostime()
marker.type = marker.SPHERE
marker.action = marker.ADD
marker.scale.x = 0.5
marker.scale.y = 0.5
marker.scale.z = 0.5
marker.pose.orientation.w = 1.0
marker.pose.position.x = init_x
marker.pose.position.y = init_y
marker.color.r = 0 / 255.0
marker.color.g = 204 / 255.0
marker.color.b = 0 / 255.0
marker.color.a = 1.0 #1.0 -> complete not transparent 0.0-> complete transparent


def update_position(marker, measurement):
    '''
    update robot position x,y into marker position
    '''
    marker.pose.position.x = measurement[0]
    marker.pose.position.y = measurement[1]
    


rate = rospy.Rate(1)

while not rospy.is_shutdown():
    
    measurement = test_target.sense()
    measurement_msg = MeasurementInfo()
    measurement_msg.x = measurement[0]
    measurement_msg.y = measurement[1]
    meas_pub.publish(measurement_msg)
    rospy.loginfo("True position: %f, %f"%(measurement[0], measurement[1]))
    update_position(marker, measurement)    
    test_target.move_in_circle()
    rate.sleep()
    robot_pub.publish(marker)