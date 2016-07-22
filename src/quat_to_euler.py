#!/usr/bin/python
# -*- coding: utf-8 -*-

# ROS libraries 
PKG = 'ksas'
import roslib; roslib.load_manifest(PKG)
import rospy
import tf

# ROS messages.
from geometry_msgs.msg import PoseStamped
from ksas.msg import Eulers, TotalPose

class QuatToEuler():
    def __init__(self):
        self.got_new_msg = False
        self.euler_msg = Eulers()
        self.tpose_msg = TotalPose()

        # Create subscribers and publishers.
        sub_sptam  = rospy.Subscriber('sptam/robot/pose', PoseStamped, self.pose_callback)
        pub_euler = rospy.Publisher('euler', Eulers, queue_size=10)
        pub_tpose = rospy.Publisher('totalpose', TotalPose, queue_size=10)

        # Main while loop.
        while not rospy.is_shutdown():
            # Publish new data if we got a new message.
            if self.got_new_msg:
                pub_euler.publish(self.euler_msg)
                pub_tpose.publish(self.tpose_msg)
                self.got_new_msg = False

    # Pose callback function.
    def pose_callback(self, msg):
        # Convert quaternions to Euler angles.
        (r, p, y) = tf.transformations.euler_from_quaternion(
[msg.pose.orientation.x, msg.pose.orientation.y, 
msg.pose.orientation.z, msg.pose.orientation.w])
        self.fill_euler_msg(msg, r, p, y)

    
    # Fill in Euler angle message.
    def fill_euler_msg(self, msg, r, p, y):
        self.got_new_msg = True
        self.euler_msg.header.stamp = msg.header.stamp
        self.euler_msg.roll  = r
        self.euler_msg.pitch = p
        self.euler_msg.yaw   = y
        self.tpose_msg.euler = self.euler_msg
        self.tpose_msg.pose = msg.pose

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('ksas_quat_to_euler', anonymous=True)
    #Initialize the class
    try:
        quat_to_euler = QuatToEuler()
    except rospy.ROSInterruptException: pass
