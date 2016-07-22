#!/usr/bin/python
# -*- coding: utf-8 -*-

# ROS libraries
PKG = 'ksas'
import roslib; roslib.load_manifest(PKG)
import rospy
import tf
# Processing libraries
import csv
import os
from math import tan, sqrt, fabs, hypot, pi
from serial import Serial
from time import sleep
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
        #For messages
        self.got_pperr_ = False
        self.got_lterr_ = False
        self.got_direc_ = False
        #For scaling
        self.scale = 60
        #For Arduino
        self.baud = 9600
        self.timeout = 0.1
        self.arduino = Serial('/dev/ttyACM1', self.baud, timeout=self.timeout)
        sleep(3)
        # Create subscribers and publishers.

        sub_pperr = rospy.Subscriber('perpendicular/error', Float32, self.pp_callback)
        sub_lterr = rospy.Subscriber('lateral/error', Float32, self.lt_callback)
        #sub_direc = rospy.Subscriber('direction', Int32, self.dr_callback)

        #pub_motor = rospy.Publisher('arduino/motor', Int32, queue_size=10)
        #pub_stppr = rospy.Publisher('arduino/stepper', Int32, queue_size=10)

        # Main while loop.
        while not rospy.is_shutdown():
            # Publish if all data is ready
            if self.got_pperr_ and self.got_lterr_: # and self.got_direc_:        
                dat = self.stppr_msg.data + 100; #dat \in [0,200]
                rospy.loginfo('Got ' + str(dat))
                rospy.loginfo(self.arduino.readline())
                self.arduino.write(chr(dat))

    #Perpendicular error message calback function
    def pp_callback(self, msg):
        self.prp_err = msg.data
        self.got_pperr_ = True
        self.gen_ard()
    
    #Lateral error message callback function         
    def lt_callback(self, msg):
        self.got_lterr_ = True
        self.lat_err = msg.data
        self.gen_ard()
    
    #Direction message callback function
    def dr_callback(self, msg):
        self.dir_ = msg.data
        self.got_direc_ = True
        self.gen_ard()

    #Generate arduino signal function
    def gen_ard(self):
        if self.got_pperr_ and self.got_lterr_ and self.got_direc_:
            k = self.prp_err
            d = self.lat_err
            dir_ = self.dir_
            stppr = int((k-d)*self.scale)
            motor = int(0)

            if stppr>100:  stppr = 100
            if stppr<-100:  stppr = -100

            self.motor_msg.data = motor
            self.stppr_msg.data = stppr

        
# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('ksas_arduino', anonymous=True)
    # Initialize the object
    try:
        ard = Arduino()
    except rospy.ROSInterruptException: pass
