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
from std_msgs.msg import Float32

path_list = list()

class ErrorGenerator():

    global path_list


    def __init__(self):
        self.got_new_msg = False
        self.tpose_msg = TotalPose()
        self.error_msg = Float32()
        self.path_file = '/home/niteesh/paths/path1.csv'
        self.tpose_count = 0 #Counter for tpose.
        self.xc = 0.0
        self.zc = 0.0
        self.thetac = 0.0
        self.unset_tpose_count = 15
        self.tpose_set = False

        
        self.pt = Point()
        self.send_des_ = False
        self.send_error_ = False
        
        with open(self.path_file, 'rb') as fr:
            reader = csv.reader(fr, delimiter=',')
            for row in reader:
                path_list.append(row)
            rospy.loginfo('Path is ready')

        # Create subscribers and publishers.
        sub_euler  = rospy.Subscriber('totalpose', TotalPose, self.tpose_callback)

        pub_error = rospy.Publisher('error', Float32, queue_size=10)
        pub_des = rospy.Publisher('des', Point, queue_size=10)
        # Main while loop.
        while not rospy.is_shutdown():
            # Publish new data if we got a new message.
            if self.send_error_ and self.send_des_:
                pub_error.publish(self.error_msg)
                pub_des.publish(self.pt)
                self.send_error_ = False
                self.send_des_ = False

    def find_error(self):
        #rospy.loginfo('In find error')
        global path_list
        xds = path_list[0]
        zds = path_list[1]
        kmin = 1000.0  #Arbitrarily high value

        a = 1
        b = -tan(pi/2 - self.thetac)
        c = -self.xc - self.zc*tan(pi/2 - self.thetac)
 
        for i in xrange(len(xds)):

            k = fabs(a*float(xds[i]) + b*float(zds[i]) + c ) / hypot(a,b)
                       #rospy.loginfo( str(zds[i]) + ' is z value to go to')
            #rospy.loginfo( str(xds[i]) + ' is x value to go to')
            if kmin > k:
                kmin = k
                x_d = float(xds[i])
                z_d = float(zds[i])
                index = i
                #dmin = sqrt( (float(xds[i]) -xc)**2 + (z_d - zc)**2 )
                rospy.loginfo(str(kmin) + ' is minimum distance')

                rospy.loginfo( str(zds[i]) + ' is z value to go to')
                rospy.loginfo( str(xds[i]) + ' is x value to go to')
                #rospy.loginfo( str(zc) + ' is zc')
                #rospy.loginfo( str(xc) + ' is xc')

           # rospy.loginfo( str(dmin) + ' is d minimum')
        #Find the sign

        #equation of line 
        if float(xds[index+1]) - x_d == 0:
            s = self.xc - x_d
        else:
            m = tan( (float(zds[index+1]) - z_d)/ (float(xds[index+1]) - x_d))
            s = self.zc - z_d - m*(self.xc - x_d)
        
        if s < 0:
            sign = -1
        else:
            sign = +1

        dmin = sign*sqrt( (self.xc - x_d)**2 + (self.zc - z_d)**2)
        self.pt.x = x_d
        self.pt.z = z_d
        self.pt.y = 0.0

        return float(dmin)

      
    # Odometry callback function.
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

        error = self.find_error()
        self.gen_err_msg(error)
        self.gen_des_msg()
        #equation is y' = -cot(thetac)x' + yc + cot(thetac)xc 

   
    def gen_des_msg(self):
        self.send_des_ = True
        
    # Fill in Euler angle message.
    def gen_err_msg(self, error):
        self.send_error_ = True
        self.error_msg = float(error)
        
# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('ksas_error', anonymous=True)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        errorer = ErrorGenerator()
    except rospy.ROSInterruptException: pass
