#!/usr/bin/python
# -*- coding: utf-8 -*-

# Start up ROS pieces.
PKG = 'ksas'
import roslib; roslib.load_manifest(PKG)
import rospy
import tf
import csv
import os
# Processing libraries
from math import tan, sqrt

# ROS messages.
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from ksas.msg import Eulers, TotalPose
from std_msgs.msg import Int32, Float32, Int64, Float64

path_list = list()

class ErrorGenerator():

    global path_list


    def __init__(self):
        self.got_new_msg = False
        self.tpose_msg = TotalPose()
        self.error_msg = Float32()
        self.path_file = '/home/sidd/paths/path1.csv'
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.send_error_ = False
        #rospy.loginfo('You are here')
        with open(self.path_file, 'rb') as fr:
            reader = csv.reader(fr, delimiter=',')
            for row in reader:
                path_list.append(row[0].strip().split(','))
            rospy.loginfo('Path is ready')

        # Create subscribers and publishers.
        sub_euler  = rospy.Subscriber('ksas/totalpose', TotalPose, self.tpose_callback)

        pub_error = rospy.Publisher('ksas/error', Float32, queue_size=10)

        # Main while loop.
        while not rospy.is_shutdown():
            # Publish new data if we got a new message.
            if self.send_error_:
                pub_error.publish(self.error_msg)
                self.send_error_ = False

    def find_error(self, xc, yc, thetac):
        global path_list
        xds = path_list[1]
        yds = path_list[0]
        dmin = 10000.0 #Arbitrarily high value

        for i in xrange(len(xds)):
            y_d = -(1/tan(thetac))*float(xds[i]) + yc + (1/tan(thetac))*xc
            if dmin > (y_d - float(yds[i])):
                dmin = y_d - float(yds[i])

        return dmin

      
    # Odometry callback function.
    def tpose_callback(self, msg):
        self.roll = msg.euler.roll
        self.pitch = msg.euler.pitch
        self.yaw = msg.euler.yaw


        thetac = msg.euler.pitch
        xc = msg.pose.position.z
        yc = msg.pose.position.x

        error = self.find_error(xc, yc, thetac)
        self.gen_err_msg(error)
        #equation is y' = -cot(thetac)x' + yc + cot(thetac)xc 

   
    # Fill in Euler angle message.
    def gen_err_msg(self, error ):
        self.send_error_ = True
        self.error_msg = float(error)
        
# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('ksas_error')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        errorer = ErrorGenerator()
    except rospy.ROSInterruptException: pass
