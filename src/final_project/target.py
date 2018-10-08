#!/usr/bin/env python

import rospy
from robot import *
from cs373_visualization.msg import MeasurementInfo
from visualization_msgs.msg import Marker

#-------------------------define callback function--------------------------------------#
def est_callback(msg, args):
    '''
    compare estimation value with target robot real position
    '''
    target_position = (args[0], args[1])
    hunter_position = (msg.x, msg.y)
    separation = distance_between(hunter_position, target_position)
    separation_tolerance = 0.1 * 1.5
    if separation < separation_tolerance:
        print "You got it right! You caught the robot!"
        rospy.signal_shutdown("Finished")

#-------------------------init robot, rosnode, publisher--------------------------------------#
init_x = 0.0
init_y = 10.0
test_target = robot(init_x, init_y, 0.0, 2*pi/30.0, 1.5)
measurement_noise = .05 * test_target.distance
test_target.set_noise(0.0, 0.0, 0.0)
rospy.init_node('target_robot')
meas_pub = rospy.Publisher('target_measurement', MeasurementInfo, queue_size=10)
robot_pub = rospy.Publisher('target_visualization', Marker, queue_size=10)
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

def distance_between(point1, point2):
    '''
    Computes distance between point1 and point2. Points are (x, y) pairs.
    '''
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def update_position(marker, measurement):
    '''
    update robot position x,y into marker position
    '''
    marker.pose.position.x = measurement[0]
    marker.pose.position.y = measurement[1]
    
rate = rospy.Rate(2)
n = 0

while not rospy.is_shutdown():   
    # publish measurement data in specific frequency
    measurement = test_target.sense()
    measurement_msg = MeasurementInfo()
    measurement_msg.x = measurement[0]
    measurement_msg.y = measurement[1]
    meas_pub.publish(measurement_msg)
    rospy.loginfo("Count: %d. True position: %f, %f"%(n, measurement[0], measurement[1]))
    robot_pub.publish(marker)
    update_position(marker, measurement)        
    # dynamically changed the argument for subscriber callback function
    test_target.move_in_circle()
    sub = rospy.Subscriber('est_measurement', MeasurementInfo, est_callback, (test_target.x, test_target.y))
    
    rate.sleep()
    n += 1