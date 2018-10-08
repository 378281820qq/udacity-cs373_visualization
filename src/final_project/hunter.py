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
        self.est_pub = rospy.Publisher('est_measurement', MeasurementInfo, queue_size=10)
        self.marker_array = MarkerArray()
        self.hunter = robot(-10.0, -10.0, 0.0)

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

    def estimate_next_pos(self, hunter_position, hunter_heading, target_measurement, max_distance, OTHER=None):
        """Estimate the next (x, y) position of the wandering Traxbot
        based on noisy (x, y) measurements."""

        # You must return xy_estimate (x, y), and OTHER (even if it is None) 
        # in this order for grading purposes.
        # OTHER = [previous_xy, heading, turning, distance]

        if not OTHER:
            OTHER = [target_measurement, 0, 0, 0, 1]    
        
        n = OTHER[4]
        local_distance = distance_between(target_measurement, OTHER[0])
        avg_distance = (local_distance + OTHER[3] * n) / (n + 1)

        heading = atan2(target_measurement[1] - OTHER[0][1], target_measurement[0] - OTHER[0][0])
        local_turning = heading - OTHER[2]
        local_turning = angle_trunc(local_turning)
        avg_turning = (local_turning + OTHER[2] * n) / (n + 1)

        OTHER = [target_measurement, heading, avg_turning, avg_distance, n+1]

        guess_robot = robot(target_measurement[0], target_measurement[1], heading, avg_turning, avg_distance)
        
        xy_estimate_list = []
        min_chase_distance = 1000000
        min_index = 0
        for i in range(6):
            guess_robot.move(guess_robot.turning, guess_robot.distance)
            xy_estimate = (guess_robot.x, guess_robot.y)
            xy_estimate_list.append(xy_estimate)
            cal_chase_distance = distance_between(hunter_position, xy_estimate)
            if cal_chase_distance < min_chase_distance:
                min_chase_distance = cal_chase_distance
                min_index = i
        
        min_xy_estimate = xy_estimate_list[min_index]
        chase_distance = distance_between(hunter_position, min_xy_estimate)
        if chase_distance > max_distance:
            chase_distance = max_distance
        
        heading = get_heading(hunter_position, xy_estimate_list[min_index])
        turning = heading - hunter_heading
        distance = chase_distance
        return turning, distance, OTHER

    def meas_callback(self, msg):
        '''
        change msg into tuple type and run estimate function
        '''
        hunter_pos = (self.hunter.x, self.hunter.y)
        target_measurement = (msg.x, msg.y)
        max_distance = 1.94 * 1.5
        turning, distance, self.OTHER = self.estimate_next_pos(hunter_pos, self.hunter.heading, target_measurement, max_distance, self.OTHER)
        self.hunter.move(turning, distance)
        estimate_pos = (self.hunter.x, self.hunter.y)
        #publish visualization topic
        self.update_position(estimate_pos)
        self.robot_pub.publish(self.marker_array)

        #publish numerical data topic
        estimate_msg = MeasurementInfo()
        estimate_msg.x = estimate_pos[0]
        estimate_msg.y = estimate_pos[1]
        self.est_pub.publish(estimate_msg)
        rospy.loginfo("Estimate pos (%f, %f)"%(estimate_pos[0], estimate_pos[1]))
    
    def receive(self):
        rospy.Subscriber('target_measurement', MeasurementInfo, self.meas_callback)
        rospy.spin()
    
    def update_position(self, measurement):
        '''
        update robot position x,y into marker position
        '''
        self.marker_array.markers[0].pose.position.x = measurement[0]
        self.marker_array.markers[0].pose.position.y = measurement[1]
    


rospy.init_node('hunter_robot', anonymous=True)
estimator = Estimator()
estimator.receive()



