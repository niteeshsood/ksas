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
from ksas.msg import Eulers, TotalPose, KsasDebug
from std_msgs.msg import Float32

class Error_Debugger():


    def __init__(self):

        self.debug_ready = False
        self.got_tpose = False
        self.got_error = False
        self.got_des = False
        self.pose = Pose()
        self.error = Float32()
        self.despt = Point()
        self.ksasdebug = KsasDebug()
        sub_tpose = rospy.Subscriber('totalpose', TotalPose, self.tpose_callback)
        sub_error = rospy.Subscriber('error', Float32, self.error_callback)

        sub_des = rospy.Subscriber('des', Point, self.des_callback)
        pub_debug = rospy.Publisher('debug', KsasDebug, queue_size=10)

        while not rospy.is_shutdown():
            if self.debug_ready:
                self.debug_ready = False
                pub_debug.publish(self.ksasdebug)
                

    def tpose_callback(self, msg):

        self.pose = msg.pose
        #self.y = msg.pose.position.y
        self.got_tpose = True
        self.fill_debug_msg()


    def error_callback(self, msg):
        self.error = msg.data
        self.got_error = True
        self.fill_debug_msg()



    def des_callback(self, msg):
        self.xdes = msg.x
        self.zdes = msg.z
        self.got_des = True
        self.fill_debug_msg()

    def fill_debug_msg(self):
        if self.got_tpose and self.got_error and self.got_des:
            self.debug_ready = True
            self.ksasdebug.pose = self.pose
            self.ksasdebug.error = self.error
            self.ksasdebug.despt.x = self.xdes
            self.ksasdebug.despt.z = self.zdes
            self.got_tpose = False
            self.got_error = False
            self.got_des = False




if __name__ == '__main__':
    #Initialize the node
    rospy.init_node('ksas_error_debug')

    try:
        debug = Error_Debugger()
    except rospy.ROSInterruptException:  pass
