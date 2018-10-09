#!/usr/bin/env python

import rospy
import sys
import math
from copy import deepcopy
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

def printpaths(path,newpath):
    for old,new in zip(path,newpath):
        print '['+ ', '.join('%.3f'%x for x in old) + \
               '] -> ['+ ', '.join('%.3f'%x for x in new) +']'

def get_visualize_array(marker_array, path, r, g, b):
    for i in range(len(path)):
        marker = Marker()
        marker.header.frame_id = "/world"
        marker.id = i 
        marker.ns = "tiles"
        marker.header.stamp = rospy.get_rostime()
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = path[i][0]
        marker.pose.position.y = path[i][1]
        marker.color.a = 1.0
        marker.color.r = r / 255.0
        marker.color.g = g / 255.0
        marker.color.b = b / 255.0
        
        marker_array.markers.append(marker)
    return marker_array 

def smooth(path, weight_data = 0.0, weight_smooth = 0.1, tolerance = 0.000001):

    # Make a deep copy of path into newpath
    newpath = deepcopy(path)

    converge = 100
    while converge >= tolerance:
        converge  = 0.0
        for x in range(1, len(path) - 1):
            for y in range(len(path[0])):
                before = newpath[x][y]
                newpath[x][y] += weight_data * (path[x][y] - newpath[x][y]) + \
                weight_smooth * (newpath[x-1][y] + newpath[x+1][y] - 2.0 * newpath[x][y])
                converge += abs(before - newpath[x][y])
    print "Return the smoothed path"
    return newpath 

def main(args):
    path = [[0, 0],
        [0, 1],
        [0, 2],
        [1, 2],
        [2, 2],
        [3, 2],
        [4, 2],
        [4, 3],
        [4, 4]]
    rospy.init_node('path_smoothing', anonymous=True)
    publisher = rospy.Publisher('visualize_marker_array', MarkerArray, queue_size=10)
    marker_array_init = MarkerArray()
    marker_array_init = get_visualize_array(marker_array_init, path, 0, 200, 0)

    smooth_path = smooth(path)
    marker_array_smooth = get_visualize_array(marker_array_init, smooth_path, 200, 0, 0)

    printpaths(path,smooth_path)
    while not rospy.is_shutdown():
        publisher.publish(marker_array_init)
        publisher.publish(marker_array_smooth)

if __name__ == '__main__':
    try:
        main(sys.argv)
    #terminate program when using ctrl+c or close node
    except rospy.ROSInterruptException:
        pass