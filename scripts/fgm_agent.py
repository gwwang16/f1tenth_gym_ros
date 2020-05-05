#!/usr/bin/env python
import numpy as np
import math
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

def polar_to_cart(theta, r):
    """
    Assumes theta in radians & returns x,y
    """
    x = r*np.cos(theta)
    y = r*np.sin(theta)
    return x,y

def lidar_polar_to_cart(ranges, angle_min, angle_increment):
    """
    Convert a lidar_dict to cartesian & return x_ranges & y_ranges
    """
    x_ranges = []
    y_ranges = []
    for i, r in enumerate(ranges):
        if r == np.nan:
            x_ranges.append(1000000)
            y_ranges.append(1000000)
        else:
            theta = angle_min + i * angle_increment
            x, y = polar_to_cart(theta + math.pi/2, r*100.0)
            x_ranges.append(x)
            y_ranges.append(y)
    return x_ranges, y_ranges

class FGM(object):
    """
    Follow the Gap Method that approximates the best steering angle given a local lidar scan
    """
    def __init__(self):
        #Set these when you get a callback
        self.angle_min = -1
        self.angle_increment = -1
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)

    def refine_laserscan(self, ranges):
        if(len(ranges) < 180):
            diff = 180 - len(ranges)
            last_elt = ranges[-1]
            for i in range(diff):
                ranges.append(last_elt)
        ranges = np.array(ranges)
        ranges[np.isinf(ranges)] = np.nan
        means = np.nanmean(ranges.reshape(-1, 4), axis = 1)
        means[means > 3] = np.nan 
        refined_ranges = np.repeat(means, 4)
        return refined_ranges

    def find_max_gap(self, free_space_ranges):
        num_trims = 0
        non_zeros = np.nonzero(free_space_ranges)[0] #Find nonzero indicies
        splits = np.split(non_zeros, np.where(np.diff(non_zeros) != 1)[0]+1)
        max_length = max(splits, key = lambda arr: len(arr))
        start_i = max_length[0] + num_trims
        end_i = max_length[-1] - num_trims
        return (start_i , end_i, 0)

    def find_safest_point(self, start_i, end_i, refined_ranges):
	#If start_i and end_i and both greater than ranges_length/2, then they're on the leftside, so the closest point would be end_i (safest would be start_i) and vice-versa
        halflen = len(refined_ranges) //2
        if (start_i >= halflen and end_i >= halflen):
            edge_i =  end_i
        elif (start_i <= halflen and end_i <= halflen):
            edge_i =  start_i
        elif (math.fabs(start_i - halflen) >= math.fabs(end_i - halflen)):
            edge_i =  end_i
        else:
            edge_i =  start_i

        #center
        static_avoid = True
        if(static_avoid):
            centerlen = start_i + abs(start_i - end_i) // 2
            centerdist = refined_ranges[centerlen]
            refined_ranges = np.array(refined_ranges)
            edge_i = start_i + np.argmax(refined_ranges[start_i:end_i])
            consideration_dist = 2.0
            x = max(centerdist - 0.3, 0)
            if (np.isnan(x) or x > consideration_dist):
                x=consideration_dist
            grad = ((centerlen - edge_i) / consideration_dist)
            const =  edge_i
            y = grad * x + const
            return int(y)
        else:
            return int(edge_i)

    def scan_callback(self, data):
        self.angle_min = data.angle_min
        self.angle_increment = data.angle_increment
        ranges = data.ranges
        angle, vel =  self.do_FGM(ranges)

        drive = AckermannDriveStamped()
        drive.drive.speed = vel
        drive.drive.steering_angle = angle
        self.drive_pub.publish(drive)

    def do_FGM(self, ranges):
        # ranges = ranges[::-1]
        angle_min = self.angle_min
        angle_incr = self.angle_increment
        #clean up & publish LIDAR SCANS so they're a bit more consistent
        refined_ranges = self.refine_laserscan(ranges)
        # refined_ranges = ranges

        #closest point to LIDAR in refined_ranges
        min_range = np.nanmin(refined_ranges)
        min_idx = np.nanargmin(refined_ranges)

        #Create Free Space 'Fake LaserScan'
        free_space_ranges = np.empty_like(refined_ranges)
        free_space_ranges[:] = min_range
        free_space_ranges[refined_ranges < min_range + 0.9] = 0

        #Find the max length gap
        start_i , end_i, max_i = self.find_max_gap(free_space_ranges)

        free_space_ranges[0:start_i] = np.nan
        free_space_ranges[end_i + 1 :] = np.nan

        #Find safest point and convert polar coordinates to x, y. 
        safest_i = self.find_safest_point(start_i, end_i, refined_ranges)
        safest_theta = angle_min + safest_i * angle_incr
        centerdist = min(3.0, refined_ranges[safest_i])
        angle = safest_theta

        if (angle > 0.34):
            angle = 0.34
        if (angle < -0.34):
            angle = -0.34

        return 1.0 * angle, 1.0

if __name__ == '__main__':
    rospy.init_node('fgm_agent')
    fgm_agent = FGM()
    rospy.spin()