#!/usr/bin/env python
import rospy
import numpy as np
import atexit
import tf
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

class WaypointLogger():
    def __init__(self):
        home = expanduser('~')
        # world_name = rospy.get_param("/waypoints_dir").
        self.file = open(strftime(home+'/wp-%Y-%m-%d-%H-%M-%S',gmtime())+'.csv', 'w')
        self.previous_pose = Odometry()
        self.interval_ = 0.2

    def save_waypoint(self,data):
        quaternion = np.array([data.pose.pose.orientation.x, 
                            data.pose.pose.orientation.y, 
                            data.pose.pose.orientation.z, 
                            data.pose.pose.orientation.w])

        euler = np.array(tf.transformations.euler_from_quaternion(quaternion))
        speed = LA.norm(np.array([data.twist.twist.linear.x, 
                                data.twist.twist.linear.y, 
                                data.twist.twist.linear.z]),2)
 
        pose_x, pose_y, distance = self.outpout_process(data, self.previous_pose)
        
        # if car moves [interval] meter
        if distance > self.interval_:
            vel_x = data.twist.twist.linear.x
            print("x: {}, y: {}, vel: {}".format(pose_x, pose_y, vel_x))
            self.previous_pose = data
            self.file.write('%f, %f, %f, %f\n' % (data.pose.pose.position.x,
                                            data.pose.pose.position.y,
                                            euler[2],
                                            speed))


            # print(euler[2], euler[0], euler[1], np.array(euler))

    def outpout_process(self, current_pose, previous_pose):
        pose_x = current_pose.pose.pose.position.x
        pose_y = current_pose.pose.pose.position.y

        previous_pose_x = previous_pose.pose.pose.position.x
        previous_pose_y = previous_pose.pose.pose.position.y

        distance = np.sqrt(np.power((pose_x - previous_pose_x),2)+ np.power((pose_y - previous_pose_y),2))

        return pose_x, pose_y, distance


    def shutdown(self):
        self.file.close()
        print('waypoints saved')
    
    def listener(self):
        rospy.init_node('waypoints_logger', anonymous=True)
        rospy.Subscriber('/odom', Odometry, self.save_waypoint)
        rospy.spin()

if __name__ == '__main__':
    wp = WaypointLogger()
    atexit.register(wp.shutdown)
    print('Saving waypoints...')
    try:
        wp.listener()
    except rospy.ROSInterruptException:
        pass
