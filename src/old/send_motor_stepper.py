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

# ROS messages.
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
from ksas.msg import Eulers, TotalPose
from std_msgs.msg import Float32, Int32, String

class Arduino():

    def __init__(self):
        #Messages
        self.prperr_msg = Float32()
        self.laterr_msg = Float32()
        self.dir_msg = Int32()
        self.motor_msg = Int32()
        self.stppr_msg = Int32()
        #Values of the messages
        self.prp_err = 0.0
        self.lat_err = 0.0
        self.dir_ = 0
        #
        self.got_pperr_ = False
        self.got_lterr_ = False
        self.got_direc_ = False

        # Create subscribers and publishers.

        sub_pperr = rospy.Subscriber('perpendicular/error', Float32, self.pp_callback)
        sub_lterr = rospy.Subscriber('lateral/error', Float32, self.lt_callback)
        sub_direc = rospy.Subscriber('direction', Int32, self.dr_callback)

        pub_motor = rospy.Publisher('arduino/motor', Int32, queue_size=10)
        pub_stppr = rospy.Publisher('arduino/stepper', Int32, queue_size=10)

        # Main while loop.
        while not rospy.is_shutdown():
            # Publish if all data is ready
            if self.got_pperr_ and self.got_lterr_ and self.got_direc_:
                pub_motor.publish(self.motor_msg)
                pub_stppr.publish(self.stppr_msg)
                
    def pp_callback(self, msg):
        #rospy.loginfo('PP callback')
        self.prp_err = msg.data
        self.got_pperr_ = True
        self.gen_ard()
             
    def lt_callback(self, msg):
        #rospy.loginfo('LT Callback')
        self.got_lterr_ = True
        self.lat_err = msg.data
        self.gen_ard()

    def dr_callback(self, msg):
        #rospy.loginfo('DR callback')
        self.dir_ = msg.data
        self.got_direc_ = True
        self.gen_ard()


    def gen_ard(self):

        if self.got_pperr_ and self.got_lterr_ and self.got_direc_:
            k = self.prp_err
            d = self.lat_err
            dir_ = self.dir_
            #rospy.loginfo(str(k))
            stppr = int((k-d)*80)
            motor = int(0)

            if stppr>100:  stppr = 100
            if stppr<-100:  stppr = -100

            self.motor_msg.data = motor
            self.stppr_msg.data = stppr

        
# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('ksas_arduino', anonymous=True)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        ard = Arduino()
    except rospy.ROSInterruptException: pass
