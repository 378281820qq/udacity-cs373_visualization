#!/usr/bin/env python

import rospy
import sys
import math
from copy import deepcopy
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

#-----------------------------------Original Function----------------------------------------#
def printpaths(path,newpath):
    for old,new in zip(path,newpath):
        print '['+ ', '.join('%.3f'%x for x in old) + \
               '] -> ['+ ', '.join('%.3f'%x for x in new) +']'

def smooth(path, weight_data = 0.1, weight_smooth = 0.1, tolerance = 0.00001):
    
    newpath = deepcopy(path)
    converge = 100
    while converge >= tolerance:
        converge  = 0.0
        for i in range(0, len(path)):
            for j in range(len(path[0])):
                before = newpath[i][j]
                newpath[i][j] += weight_data * (path[i][j] - newpath[i][j]) + \
                weight_smooth * (newpath[(i-1)%len(path)][j] + newpath[(i+1)%len(path)][j] - 2.0 * newpath[i][j])
                converge += abs(before - newpath[i][j])
    
    print "Return the smoothed path"
    return newpath

class cyclic_smooth():
    def __init__(self, path):
        #--------------------------------Parameter Setup-------------------------------#
        self.publisher = rospy.Publisher('visualize_marker_array', MarkerArray, queue_size=10)
        self.marker_array = MarkerArray()
        self.path = deepcopy(path)
        self.newpath = deepcopy(path)
        self.converge = 10.0
        self.is_converge = False
        self.make_array(0, 1.0, 0, 200, 0)
        self.make_array(len(self.path)+1, 0.3, 0, 200, 0) # record original path
        self.publisher.publish(self.marker_array)
        rospy.loginfo("Finished INITIALIZE process")
        rospy.sleep(1)        

    def iterate(self, weight_data = 0.1, weight_smooth = 0.1, tolerance = 0.00012):
        '''
        execute path smoothing algorithm 
        '''
        while self.converge >= tolerance:
            self.converge = 0.0
            for x in range(0, len(self.path)):
                for y in range(len(self.path[0])):
                    before = self.newpath[x][y]
                    self.newpath[x][y] += weight_data * (self.path[x][y] - self.newpath[x][y]) + \
                    weight_smooth * (self.newpath[(x-1)%len(self.path)][y] + self.newpath[(x+1)%len(self.path)][y] - 2.0 * self.newpath[x][y])
                    self.converge += abs(before - self.newpath[x][y])
                    self.marker_array.markers[x].pose.position.x = self.newpath[x][0]
                    self.marker_array.markers[x].pose.position.y = self.newpath[x][1]
                    self.publisher.publish(self.marker_array)   
            #rospy.loginfo("itering, converge value: %f"%self.converge)
            rospy.sleep(0.05)
        self.is_converge = True
        print "Find Smoothed path"

    def make_array(self, begin, a, r, g, b):
        '''
        helper function to draw every points in path
        '''
        for i in range(len(self.path)):
            marker = Marker()
            marker.header.frame_id = "/world"
            marker.id = begin + i 
            marker.ns = "tiles"
            marker.header.stamp = rospy.get_rostime()
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = self.path[i][0]
            marker.pose.position.y = self.path[i][1]
            marker.color.a = a
            marker.color.r = r / 255.0
            marker.color.g = g / 255.0
            marker.color.b = b / 255.0            
            self.marker_array.markers.append(marker)

    def draw_final(self):
        rospy.loginfo("Showing FINAL status")
        # make smoothed path another color
        for i in range(len(self.path)):
            self.marker_array.markers[i].color.r = 255 / 255.0
            self.marker_array.markers[i].color.g = 26 / 255.0
            self.marker_array.markers[i].color.b = 25 / 255.0
        self.publisher.publish(self.marker_array)

def main(args):
    path=[[0, 0], 
      [1, 0],
      [2, 0],
      [3, 0],
      [4, 0],
      [5, 0],
      [6, 0],
      [6, 1],
      [6, 2],
      [6, 3],
      [5, 3],
      [4, 3],
      [3, 3],
      [2, 3],
      [1, 3],
      [0, 3],
      [0, 2],
      [0, 1]]
    rospy.init_node('cyclic_smoothing', anonymous=True)
    new_path_cls = cyclic_smooth(path)

    rate = rospy.Rate(2)
    while(not rospy.is_shutdown()) and (not new_path_cls.is_converge):
        new_path_cls.iterate()
        if new_path_cls.is_converge:
            #show result after smoothing is done
            new_path_func = smooth(path)
            printpaths(path,new_path_func)
            new_path_cls.draw_final()
        rate.sleep()

if __name__ == '__main__':
    try:
        main(sys.argv)
    #terminate program when using ctrl+c or close node
    except rospy.ROSInterruptException:
        pass