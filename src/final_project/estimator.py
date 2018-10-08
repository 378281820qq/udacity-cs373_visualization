#!/usr/bin/env python

import rospy
from robot import *
from cs373_visualization.msg import MeasurementInfo
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# A helper function you may find useful.
def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

class Estimator:    
    def __init__(self):
        self.OTHER = None
        self.robot_pub = rospy.Publisher('est_visualization', MarkerArray, queue_size=10)
        self.marker_array = MarkerArray()

        marker = Marker()
        marker.header.frame_id = "/world"
        marker.id = 1
        marker.ns = "tiles"
        marker.header.stamp = rospy.get_rostime()
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.color.r = 200 / 255.0
        marker.color.g = 0 / 255.0
        marker.color.b = 0 / 255.0
        marker.color.a = 1.0 #1.0 -> complete not transparent 0.0-> complete transparent
        self.marker_array.markers.append(marker)

    def estimate_next_pos(self, measurement, OTHER=None):
        """Estimate the next (x, y) position of the wandering Traxbot
        based on noisy (x, y) measurements."""

        # You must return xy_estimate (x, y), and OTHER (even if it is None) 
        # in this order for grading purposes.
        # OTHER = [previous_xy, heading, turning, distance]

        if not self.OTHER:
            self.OTHER = [measurement, 0, 0, 0]
            xy_estimate = (measurement[0],measurement[1])   
        else:
            distance = distance_between(measurement, OTHER[0])
            heading = atan2(measurement[1] - OTHER[0][1], measurement[0] - OTHER[0][0])
            turning = heading - self.OTHER[2]
            self.OTHER = [measurement, heading, turning, distance]
            #print self.OTHER

            guess_robot = robot(measurement[0], measurement[1], heading, turning, distance)
            guess_robot.move(guess_robot.turning, guess_robot.distance)
            xy_estimate = (guess_robot.x, guess_robot.y)
        return xy_estimate, self.OTHER

    def meas_callback(self, msg):
        '''
        change msg into tuple type and run estimate function
        '''
        measurement = (msg.x, msg.y)
        estimate_pos, self.OTHER = self.estimate_next_pos(measurement, self.OTHER)
        self.update_position(estimate_pos)
        self.robot_pub.publish(self.marker_array)
        rospy.loginfo("Estimate pos (%f, %f)"%(estimate_pos[0], estimate_pos[1]))
    
    def receive(self):
        rospy.Subscriber('rr_measurement', MeasurementInfo, self.meas_callback)
        rospy.spin()
    
    def update_position(self, measurement):
        '''
        update robot position x,y into marker position
        '''
        self.marker_array.markers[0].pose.position.x = measurement[0]
        self.marker_array.markers[0].pose.position.y = measurement[1]
    


rospy.init_node('estimator_robot', anonymous=True)
estimator = Estimator()
estimator.receive()
#est_pub = rospy.Publisher('est_measurement', MeasurementInfo, queue_size=10)


