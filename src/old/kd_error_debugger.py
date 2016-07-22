#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'ksas'
import roslib; roslib.load_manifest(PKG)
import rospy
import tf
import csv
import os

#Messages
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
from ksas.msg import Eulers, TotalPose, KDDebug
from std_msgs.msg import Float32

class Error_Debugger():


    def __init__(self):
        #Flags
        self.debug_ready = False
        self.got_laterr = False
        self.got_latpt = False
        self.got_prperr = False
        self.got_errpt = False
        self.got_des = False
        #Messages
        self.laterr = Float32()
        self.prperr = Float32()
        self.latpt = Point()
        self.prppt = Point()

        #self.despt = Point()
        self.kddebug = KDDebug()
        #sub_tpose = rospy.Subscriber('totalpose', TotalPose, self.tpose_callback)
        sub_laterr = rospy.Subscriber('lateral/error', Float32, self.laterr_callback)
        sub_latpt = rospy.Subscriber('lateral/error/minpt', Point, self.latpt_callback)
          
        sub_prperr = rospy.Subscriber('perpendicular/error', Float32, self.prperr_callback)
        sub_prppt = rospy.Subscriber('perpendicular/error/minpt', Point, self.prppt.callback)

        #sub_des = rospy.Subscriber('des', Point, self.des_callback)
        pub_debug = rospy.Publisher('kddebug', KDDebug, queue_size=10)

        while not rospy.is_shutdown():
            if self.debug_ready:
                self.debug_ready = False
                pub_debug.publish(self.kddebug)
                

    def laterr_callback(self, msg):
        self.laterr = msg.data
        self.got_laterr = True
        self.fill_debug_msg()

    def prperr_callback(self, msg):
        self.prperr = msg.data
        self.got_prperr = True
        self.fill_debug_msg()

    def latpt_callback(self, msg):
        self.latpt = msg.data
        self.got_latpt = True
        self.fill_debug_msg()

    def prppt_callback(self, msg):
        self.prppt = msg.data
        self.got_prppt = True
        self.fill_debug_msg()

    '''
    def des_callback(self, msg):
        self.xdes = msg.x
        self.zdes = msg.z
        self.got_des = True
        self.fill_debug_msg()
    '''
    def fill_debug_msg(self):
        if self.got_laterr and self.got_prperr and self.got_latpt and self.got_prppt:# and self.got_des:
            self.debug_ready = True

            self.kddebug.laterr = self.laterr
            self.kddebug.prperr = self.prperr
            self.kddebug.latpt = self.latpt
            self.kddebug.prppt = self.prppt
            #self.ksasdebug.despt.x = self.xdes
            #self.ksasdebug.despt.z = self.zdes
            self.got_laterr = False
            self.got_prperr = False
            self.got_latpt = False
            self.got_prppt = False
            #self.got_des = False

if __name__ == '__main__':
    #Initialize the node
    rospy.init_node('ksas_error_debug')

    try:
        debug = Error_Debugger()
    except rospy.ROSInterruptException:  pass
