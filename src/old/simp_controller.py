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
from math import tan, sqrt, fabs, hypot, pi
import sys
from numpy import linspace
# ROS messages.
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
from ksas.msg import Eulers, TotalPose
from std_msgs.msg import Float32, Int32, String

path_list = list()

class Controller():

    global path_list


    def __init__(self):
        #Messages
        self.tpose_msg = TotalPose()
        self.err_msg = Float32()
        #Total pose counter
        self.tpose_count = 0
        self.unset_tpose_count = 5
        self.tpose_set = False        
        #Current pose
        self.xc = 0.0
        self.zc = 0.0
        self.thetac = 0.0
        #For the msgs
        self.send_err_ = False
            
        # Create subscribers and publishers.
        sub_euler  = rospy.Subscriber('totalpose', TotalPose, self.tpose_callback)
        pub_err = rospy.Publisher('lateral/error', Float32, queue_size=10)
        
        # Main while loop.
        while not rospy.is_shutdown():
            # Publish if all data is ready
            if self.send_err_:
                self.send_err_ = False
                pub_err.publish(self.err_msg)
 
    def find_error(self):
        errmin = self.xc
        return float(errmin)

    # Total Pose callback function.
    def tpose_callback(self, msg):
        
        self.tpose_set = True
        self.tpose_count+=1
        if self.tpose_count > self.unset_tpose_count:
            self.tpose_set = False
            self.tpose_count = 0

        if not self.tpose_set: #change only if unset
            self.thetac = msg.euler.pitch
            self.zc = msg.pose.position.z
            self.xc = msg.pose.position.x

        if self.xc<0.2 and self.xc>-0.2:
            if self.zc<2.2 and self.zc>1.8:
                rospy.loginfo('Reached destination!') 

        #Find the lateral error and it's line
        laterr = self.find_error()
        self.gen_err_msg(laterr)        
 
    #Error messsages
    def gen_err_msg(self, error):
        self.send_err_ = True
        self.err_msg = float(error)    
        

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('ksas_control', anonymous=True)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        control = Controller()
    except rospy.ROSInterruptException: pass
