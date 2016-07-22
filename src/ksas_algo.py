#!/usr/bin/python
# -*- coding: utf-8 -*-

# ROS libraries
PKG = 'ksas'
import roslib; roslib.load_manifest(PKG)
import rospy
import tf
from dynamic_reconfigure import client

#Python libraries
import csv
import os
import sys
# Processing libraries
from math import tan, sqrt, fabs, hypot, pi, atan2, atan
import numpy as np
# My defined libraries
from util import *

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
        self.prperr_msg = Float32()
        self.laterr_msg = Float32()
        self.dir_msg = Int32()
        self.control_msg = String()
        #For the path
        #self.path_file = '/home/niteesh/paths/zigzag_path.csv'
        self.path_file = '/home/niteesh/paths/nn.csv'
        self.maxind = 0
        self.path_list = list()
        #Total pose counter
        self.tpose_count = 0
        self.unset_tpose_count = 0
        self.tpose_set = False        
        #Current pose
        self.xc = 0.0
        self.zc = 0.0
        self.thetac = 0.0
        self.perppt = Point()
        self.lateralpt = Point()
        #For the msgs
        self.send_laterr_ = False
        self.send_prperr_ = False
        self.send_control_ = False
        #For the dictionary implementation of lines
        self.dzliner = {'a':0,'b':0,'c':0}
        self.dxliner = {'a':0,'b':0,'c':0}
        self.dliner = {'a':0,'b':0,'c':0}
        self.pliner = {'a':0,'b':0,'c':0}
        self.tliner = {'a':0,'b':0,'c':0}
        self.oliner = {'a':0,'b':0,'c':0}
        #For the signed k and d values
        self.signedk = 0.0
        self.signedd = 0.0
        #Comparison tolerances
        self.eps = 0.1
        
        with open(self.path_file, 'rb') as fr:
            reader = csv.reader(fr, delimiter=',')
            for row in reader:
                path_list.append(row)
                self.path_list.append(row)
        self.maxind = len(path_list[0]) - 1
            
        # Create subscribers and publishers.
        sub_euler  = rospy.Subscriber('totalpose', 
TotalPose, self.tpose_callback)

        pub_perpt = rospy.Publisher('perpendicular/error/minpt',
 Point, queue_size=10)
        pub_pperr = rospy.Publisher('perpendicular/error',
 Float32, queue_size=10)

        pub_lerpt = rospy.Publisher('lateral/error/minpt', 
Point, queue_size=10)
        pub_lterr = rospy.Publisher('lateral/error', Float32, queue_size=10)
        pub_dir = rospy.Publisher('direction', Int32, queue_size=10)
        pub_ard = rospy.Publisher('subsc_actuator', String, queue_size=10)

        # Main while loop.
        while not rospy.is_shutdown():
            # Publish if all data is ready
            if self.send_prperr_ and self.send_laterr_:
                pub_perpt.publish(self.perppt)
                pub_pperr.publish(self.signedk)
                pub_lerpt.publish(self.lateralpt)
                pub_lterr.publish(self.signedd)
                self.send_prperr_ = False
		self.send_laterr_ = False
                self.send_control_ = False

#    def rotation_mat(theta):
#        return lambda theta:  np.matrix( [ [cos(theta), -sin(theta)], [sin(theta), cos(theta)]]

    def find_lateral_basis_and_error(self):
        global path_list
        xall = path_list[0]
        zall = path_list[1]
        errmin = 100.0 #Arbitrary high value
        minind = 0
        xde = float(xall[minind])
        zde = float(zall[minind])

        #Cycle through all values and find minimum lateral error 
        #Lateral error is defined as shown in the diagram 1
        for i in xrange(len(xall)):
            err = sqrt( (self.xc - float(xall[i]))**2 + (self.zc - float(zall[i]))**2 )
            #Find minimum point             
            if errmin > err:
                errmin = err
                xde = float(xall[i])
                zde = float(zall[i])
                minind = i

        #Create a line defined as the line connecting (xde,zde) and the
        #immediate next point in the path
        #This line is 
        # \alpha x + \beta z + \gamma = 0
        #Calculate it here

        #Make sure the robot has not reached 
        if minind == self.maxind - 1:
            rospy.logwarn('Robot believes it is at the end of path')
            #sys.exit()
            rospy.exit()
        rospy.loginfo(str(minind) + '-' +  str(self.maxind))

        #If it's a straight line parallel to z axis    
        try:
            if float(xall[minind+1]) - xde == 0: 
                #line equation ->  x - xde = 0
                self.tliner['a'] = 1
                self.tliner['b'] = 0
                self.tliner['c'] = -xde
            else:
                m = tan((float(zall[minind+1]) - zde)/(float(xall[minind+1]) - xde))
                #line equation ->  z - zde - m*(x - xde) = 0
                #-m*x + z + (-zde +m*xde) = 0
                self.tliner['a'] = -m
                self.tliner['b'] = 1
                self.tliner['c'] = (-zde + m*xde)
        except Exception:
            rospy.logwarn('Robot believes it has reached the end of path')

        #Now find the d line which is the line which connects (xde,zde) with (xc,zc)

        if xde - self.xc ==0:# self.eps: #xc is a continuously varying value and 
				#will never be exactly equal to xde. 
            #line equation ->  x - xde = 0. Because xde is more stable than xc
            self.dliner['a'] = 1
            self.dliner['b'] = 0
            self.dliner['c'] = -xde
            self.dzliner = self.dliner #As per definition
            self.dxliner['a'] = 0
            self.dxliner['b'] = 1
            self.dxliner['c'] = -zde
        else:
            m = tan((zde - self.zc)/(xde - self.xc))
            #line equation ->  z - zde - m*(x - xde) = 0
            #-m*x + z + (-zde +m*xde) = 0
            self.dliner['a'] = -m
            self.dliner['b'] = 1
            self.dliner['c'] = (-zde + m*xde) #xde is more stable
            self.dzliner = self.dliner
            #The perpendicular to this line is
            #x + m*z + (xde - m*zde) = 0
            self.dxliner['a'] = 1
            self.dxliner['b'] = m
            self.dxliner['c'] = (xde - m*zde)

       #Lateral error message
        self.lateralpt.x = xde
        self.lateralpt.z = zde
        self.lateralpt.y = 0.0
        #Return errmin anyway 
        return float(errmin)

    def find_perp_error(self):
        kmin = 1000.0 #arbitrarily high value
        
        #Find the perpendicular line
        self.pliner['a'] = 1
        if self.thetac == pi/2:# < self.eps:
            self.pliner['b'] = 0
            self.pliner['c'] = -self.xc
        else:
            self.pliner['b'] = (-tan(pi/2 - self.thetac))
            self.pliner['c'] = -self.xc - self.zc*tan(pi/2 - self.thetac)

        a1 = self.tliner['a']; b1 = self.tliner['b']; c1 = self.tliner['c'];
        a2 = self.pliner['a']; b2 = self.pliner['b']; c2 = self.pliner['c'];

        if (a2*b1 - b2*a1) == 0:
           xdm = 10000 #Ridiculously high values
           zdm = 10000
        else:
          xdm = -(b2*c1 - b1*c2)/(a2*b1 - b2*a1)
          zdm = -(a2*c1 - a1*c2)/(a2*b1 - b2*a1)
                          
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

        #Create the pose vector and thus pose line
        #It is a unit vector oriented in \theta_c direction 
        #from (xc,zc)

        dx = 1*cos(pi/2 - self.thetac)
        dz = 1*sin(pi/2 - self.thetac)
 
        #The line joining (xc,zc) to (xc+dx, zc+dz) is
        #a) if dx = 0
        if(dx == 0):
            self.oliner['a'] = 1
            self.oliner['b'] = 0
            self.oliner['c'] = -self.xc
        else:
            m = dz/dx
            self.oliner['a'] = m
            self.oliner['b'] = 1
            self.oliner['c'] = (-self.zc + m*self.xc)
       
        #Find the lateral error and it's line
        laterr = self.find_lateral_basis_and_error()
        self.gen_lerr_msg(laterr)
     

        #Find the perpendicular error and it's line
        prperr = self.find_perp_error()
        self.gen_perr_msg(prperr)
        
        #Control System
        self.control_it()

    def control_it(self):
        xde = self.lateralpt.x
        zde = self.lateralpt.z
        xdm = self.perppt.x
        zdm = self.perppt.z
        xc = self.xc
        zc = self.zc
        thetac = self.thetac

	#Let's shift the coordinate axes

        nxde = xde - xde
        nzde = zde - zde
        nxdm = xdm - xde
        nzdm = zdm - zde
        nxc = xc - xde
        nzc = zc - zde
	
        #Now let's find k and d

        d = hypot(nxc, nzc)
        k = hypot(nxdm, nzdm)

        #Now if we put a very high x value in the line equation of the tangent
        #to see on which side it lies. (10000,0) represents 10km on the x 
        #axis. If Xc lies on the same side d is positive. Else d is negative

        convex_or_concave = 10000*self.tliner['a'] + 0*self.tliner['b'] + self.tliner['c']

        coc_c = xc*self.tliner['a'] + zc*self.tliner['b'] + self.tliner['c']

        if convex_or_concave>0:
            if coc_c > 0:
                signedd = d
            else:
                signedd = -d
        else:
            if coc_c < 0:
                signedd = d
            else:
                signedd = -d

        psi = atan2(-self.tliner['a'],self.tliner['b']) #Angle of tangent line wrt world frame

        if signedd > 0:

            if ConvertToRadians(90) - self.thetac - psi > 0:
                signedk = -k
            if ConvertToRadians(90) - self.thetac - psi < 0:
                signedk = k

        else:

            if ConvertToRadians(90) - self.thetac - psi > 0:
                signedk = k
            if ConvertToRadians(90) - self.thetac - psi < 0:
                signedk = -k


        self.signedk = signedk
        self.signedd = signedd


        rospy.loginfo(str(signedk) + ' is signed k value')
        rospy.loginfo(str(signedd) + ' is signed d value')
        
        if (signedk - signedd)>0:  rospy.loginfo('Turn clockwise')

    
    #Lateral error messsages and flag
    def gen_lerr_msg(self, error):
        self.send_laterr_ = True
        self.laterr_msg = float(error)
    
    #Perpendicular error messages
    def gen_perr_msg(self, error):
        self.send_prperr_ = True
        self.prperr_msg = float(error)
        
        
# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('ksas_control', anonymous=True)
    # Instantiate and run
    try:
        control = Controller()
    except rospy.ROSInterruptException: pass
