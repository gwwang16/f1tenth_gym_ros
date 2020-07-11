#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

# ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class reactive_follow_gap:
    def __init__(self):

        drive_topic = rospy.get_param('ego_drive_topic')
        # odom_topic = rospy.get_param('ego_odom_topic')
        scan_topic = rospy.get_param('ego_scan_topic')

        # create ROS subscribers and publishers
        self.drive_pub = rospy.Publisher(
            drive_topic, AckermannDriveStamped, queue_size=1)
        # self.odom_sub = rospy.Subscriber(
        #     odom_topic, Odometry, self.pose_callback, queue_size=10)
        self.scan_sub = rospy.Subscriber(
            scan_topic, LaserScan, self.scan_callback, queue_size=10)

        # Params
        self.max_scan_distance = rospy.get_param('max_scan_distance')
        self.scan_distance_to_base_link = rospy.get_param(
            'scan_distance_to_base_link')
        self.bubble_radius = rospy.get_param('bubble_radius')
        self.filter_size = rospy.get_param('smoothing_filter_size')
        self.steering_angle_reactivity = rospy.get_param(
            'steering_angle_reactivity')

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        # Lidar fov is 270 degree, 1080 beams, 0.25 degree interval
        # truncate lidar range
        ranges = np.array(ranges)
        proc_ranges = ranges[np.arange(270, 810)]
        # proc_ranges = ranges[np.arange(300, 780)]

        # smooth the lidar points
        proc_ranges = self.running_mean(proc_ranges, self.filter_size)

        # Remove high values
        idxs = np.where(proc_ranges >= self.max_scan_distance)
        proc_ranges[idxs] = self.max_scan_distance

        # idxs = np.where(proc_ranges<=self.scan_distance_to_base_link)
        # proc_ranges[idxs] = self.scan_distance_to_base_link

        return proc_ranges

    def running_mean(self, x, N):
        cumsum = np.cumsum(np.insert(x, 0, 0))
        return (cumsum[N:] - cumsum[:-N]) / float(N)

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        size, max_size, max_start, max_end = 0, 0, 0, 0
        for i in range(len(free_space_ranges)):
            if free_space_ranges[i] != 0:
                size += 1
            else:
                size = 0

            if size > max_size:
                max_size = size
                max_end = i

        max_start = max_end - max_size

        return max_start, max_end + 1

    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
        Naive: Choose the furthest point within ranges and go there
        """
        return (start_i + end_i)//2

    def zero_out_bubble(self, ranges, min_distance, min_idx, angle_increment):
        radius = 0.2
        if min_distance == 0:
            alpha = np.arcsin(0)
        elif min_distance > radius:
            alpha = np.arcsin(radius/min_distance)
        else:
            alpha = np.arcsin(1)

        min_angle = angle_increment*min_idx - alpha
        max_angle = angle_increment*min_idx + alpha

        for i in range(len(ranges)):
            point_angle = angle_increment * i
            if min_angle < point_angle < max_angle:
                ranges[i] = 0.0

        return ranges

    def set_speed(self, distance):
        """ Set speed according to the distance in front of car"""
        return (1-np.exp(-abs(distance)*0.2))*7 + 1.0

    def scan_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        proc_ranges = self.preprocess_lidar(data.ranges)

        # Find closest point to LiDAR
        min_distance = np.min(proc_ranges)
        min_idx = np.argmin(proc_ranges)

        # Eliminate all points inside 'bubble' (set them to zero)
        proc_ranges = self.zero_out_bubble(
            proc_ranges, min_distance, min_idx, data.angle_increment)

        # Find max length gap
        start_i, end_i = self.find_max_gap(proc_ranges)

        # Find the best point in the gap
        best_idx = self.find_best_point(start_i, end_i, proc_ranges)

        # Calculate steering angle
        steering_angle = data.angle_increment * (best_idx - len(proc_ranges)//2)
        steering_angle = np.clip(steering_angle, -0.4, 0.4)

        # Calculate speed
        goal_distance = proc_ranges[best_idx]
        # front_distance = proc_ranges[len(proc_ranges)//2]
        speed = self.set_speed(goal_distance)
        # print("distance: {}, speed: {}".format(min_distance, speed))

        # Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)


def main(args):
    rospy.init_node("gap_follow_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.02)
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
