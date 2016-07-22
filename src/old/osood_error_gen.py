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
        #Messages
        self.tpose_msg = TotalPose()
        self.prperr_msg = Float32()
        self.laterr_msg = Float32()
        #Path file
        self.path_file = '/home/niteesh/paths/path1.csv'
        #Total pose counter
        self.tpose_count = 0
        #Current pise
        self.xc = 0.0
        self.zc = 0.0
        self.thetac = 0.0
        self.unset_tpose_count = 15
        self.tpose_set = False        
        self.perppt = Point()
        self.lateralpt = Point()
        #For the msgs
        self.send_lterr_ = False
        self.send_ldes_ False
        self.send_pdes_ = False
        self.send_pperr_ = False
        #For the lines
        self.alpha = 0.0
        self.beta = 0.0
        self.gamma = 0.0
        self.delta = 0.0
        self.eps = 0.0
        self.zeta = 0.0
        
        with open(self.path_file, 'rb') as fr:
            reader = csv.reader(fr, delimiter=',')
            for row in reader:
                path_list.append(row)
            rospy.loginfo('Path is ready')

        # Create subscribers and publishers.
        sub_euler  = rospy.Subscriber('totalpose', TotalPose, self.tpose_callback)

        pub_perpt = rospy.Publisher('perpendicular/error/minpt', Point, queue_size=10)
        pub_pperr = rospy.Publisher('perpendicular/error', Float32, queue_size=10)

        pub_lerpt = rospy.Publisher('lateral/error/minpt', Point, queue_size=10)
        pub_lterr = rospy.Publisehr('lateral/error', Float32, queue_size=10)

        # Main while loop.
        while not rospy.is_shutdown():
            # Publish if all data is ready
            if self.send_pperr_ and self.send_pde_ and self.send_lterr_ and self.send_lde_:
                pub_perpt.publish(self.perppt)
                pub_pperr.publish(self.prperr)
                pub_lerpt.publish(self.lateralpt)
                pub_lterr.publish(self.laterr)
                self.send_pperr_ = False
                self.send_pde_ = False
		self.send_lterr_ = False
                self.send_lde_ = False

 
     def find_lateral_error(self):
        global path_list
        xall = path_list[0]
        zall = path_list[1]
        errmin = sqrt( (self.xc - float(xall[0]))**2 + (self.zc - float(zall[0]))**2 )

        #Cycle through all values and find minimum lateral error 
        #Lateral error is defined as shown in the diagram 1
        for i in xrange(len(xall)):

            err = sqrt( (self.xc - float(xall[i]))**2 + (self.zc - float(zall[i]))**2 )
             
            if errmin > err:
                errmin = err
                xde = float(xall[i])
                zde = float(zall[i])
                minind = i
                rospy.loginfo(str(errmin) + ' is minimum lateral error')
                #rospy.loginfo( str(zall[i]) + ' is z value to go to')
                #rospy.loginfo( str(xall[i]) + ' is x value to go to')

        

        #Create a line defined as the line connecting (xde,zde) and the
        #immediate next point in the path
        #This line is 
        # \alpha x + \beta z + \gamma = 0
        #Calculate it here
         if float(xall[minind+1]) - xde == 0:
            #line equation ->  x - xde = 0
            self.alpha = 1 
            self.beta = 0
            self.gamma = -xde
        else:
            m = tan((float(zall[index+1]) - zde)/(float(xall[index+1]) - xde))

            #line equation ->  z - zde - m*(x - xde) = 0
            #-m*x + z + (-zde +m*xde) = 0
            self.alpha = -m
            self.beta = 1
            self.gamma = -zde + m*xde
       
       #Lateral error message
        self.lateralpt.x = xde
        self.lateralpt.z = zde
        self.lateralpt.y = 0.0

        return float(errmin)

      def find_perp_error(self):
        global path_list
        xall = path_list[0]
        zall = path_list[1]
        kmin = 1000.0 #arbitrarily high value

        self.delta = 1
        self.eps = -tan(pi/2 - self.thetac)
        self.zeta = -self.xc - self.zc*tan(pi/2 - self.thetac)

        a1 = self.alpha 
        b1 = self.beta
        c1 = self.gamma
        a2 = self.delta
        b2 = self.eps
        c2 = self.zeta

        xdm = (b2*c2 - b1*c2)/ fabs(a1*b2 - a2*b1)
        zdm = (-a2*c1 + a1*c2)/ fabs(a1*b2 - a2*b1)
               
       #Perpendicular error message
        self.perppt.x = xdm
        self.perppt.z = zdm
        self.perppt.y = 0.0

       #Perpendicular error 
        xde = self.lateralpt.x
        zde = self.lateralpt.z
        
        kerr = sqrt((xde-xdm)**2 + (zde-zdm)**2)

        return float(kerr)
    
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

        #Find the lateral error and it's line
        laterr = self.find_lateral_error()
        self.gen_lerr_msg(laterr)
        self.gen_lde_msg()

        #Find the perpendicular error and it's line
        prperr = self.find_perp_error()
        self.gen_perr_msg(prperr)
        self.gen_pde_msg()

        
    #Lateral error messsages
    def gen_lerr_msg(self, error):
        self.send_laterr_ = True
        self.laterr_msg = float(error)
    def gen_lde_msg(self):
        self.ldes_ = True
    #Perpendicular error messages
    def gen_perr_msg(self, error):
        self.send_prperr_ = True
        self.prperr_msg = float(error)
    def gen_pde_msg(self):
        self.send_pde_ = True
        
        
# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('ksas_error', anonymous=True)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        errorer = ErrorGenerator()
    except rospy.ROSInterruptException: pass
