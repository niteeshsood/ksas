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
# Processing libraries
from math import tan, sqrt, fabs, hypot, pi
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
        self.path_file = '/home/niteesh/paths/path1.csv'
        self.maxind = 0
        self.path_list = list()
        #Total pose counter
        self.tpose_count = 0
        self.unset_tpose_count = 5
        self.tpose_set = False        
        #Current pose
        self.xc = 0.0
        self.zc = 0.0
        self.thetac = 0.0
        self.perppt = Point()
        self.lateralpt = Point()
        #For the msgs
        self.send_laterr_ = False
        self.send_lde_ = False
        self.send_pde_ = False
        self.send_prperr_ = False
        self.send_control_ = False
        self.send_dir_ = False
        #For the lines
        self.alpha = 0.0
        self.beta = 0.0
        self.gamma = 0.0
        self.delta = 0.0
        self.eps = 0.0
        self.zeta = 0.0
        #For the signed k and d values
        self.signedk = 0.0
        self.signedd = 0.0
        self.dir_ = 1 #Default is no turn
        #Numpy vectors
        #For the different vectors
        self.dvec = np.array([0,0])
        self.tvec = np.array([0,0])
        self.kvec = np.array([0,0])
        self.ovec = np.array([0,0]) #Pose vector
        #For the basis
        self.dxcap = np.array([0,0])
        self.dycap = np.array([0,0])
        
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
            if (self.send_prperr_ and self.send_pde_ and
 self.send_laterr_ and self.send_lde_ and self.send_dir_ 
and self.send_control_):
                pub_perpt.publish(self.perppt)
                pub_pperr.publish(self.signedk)
                pub_lerpt.publish(self.lateralpt)
                pub_lterr.publish(self.signedd)
                pub_dir.publish(self.dir_msg)
                pub_ard.publish(self.control_msg)
                self.send_prperr_ = False
                self.send_pde_ = False
		self.send_laterr_ = False
                self.send_lde_ = False
                self.send_control_ = False
#                rospy.loginfo('Publishing')
 

    def rotation_mat(theta):
        return lambda theta:  np.matrix( [ [cos(theta), -sin(theta)], [sin(theta), cos(theta)]]

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
        if not minind == self.maxind -1:
            rospy.logwarn('Robot believes it is at the end of path')
            minind = self.maxind - 1
        rospy.loginfo(str(minind) + '-' +  str(self.maxind))

        #If it's a straight line parallel to z axis    
        try:
            if float(xall[minind+1]) - xde == 0: 
                #line equation ->  x - xde = 0
                self.alpha = 1 
                self.beta = 0
                self.gamma = -xde
            else:
                m = tan((float(zall[minind+1]) - zde)/(float(xall[minind+1]) - xde))

                #line equation ->  z - zde - m*(x - xde) = 0
                #-m*x + z + (-zde +m*xde) = 0
                self.alpha = -m
                self.beta = 1
                self.gamma = -zde + m*xde
        except Exception:
            rospy.logwarn('Robot believes it has reached the end of path')

        #Find the vector for (xde,zde) <- (xc,zc)
        #Create the d vector
        self.dvec[0] = (xde-xc); self.dvec[1] = (zde-zc)
        self.dycap = self.dvec/np.linalg.norm(self.dvec)
        self.dxcap = np.dot(rotate_mat(-pi/2), self.dvec)
        rospy.loginfo('dvec[0] is ' + str(self.dvec[0])); rospy.loginfo('dvec[1] is ' + str(self.dvec[1]))

        #Create the t vector
        self.tvec[0] = (xall[minind+1] - xde); self.tvec[1] = (zall[mindind+1] - zde)
        rospy.loginfo('tvec[0] is ' + str(self.dvec[0])); rospy.loginfo('tvec[1] is ' + str(self.dvec[1]))

       #Lateral error message
        self.lateralpt.x = xde
        self.lateralpt.z = zde
        self.lateralpt.y = 0.0
        #Return errmin anyway 
        return float(errmin)

    def find_perp_error(self):
        kmin = 1000.0 #arbitrarily high value
        
        '''
        #perpendicular line is 
        #\delta x + \eps z + \zeta = 0
        self.delta = 1
        self.eps = -tan(pi/2 - self.thetac)
        self.zeta = -self.xc - self.zc*tan(pi/2 - self.thetac)
        '''

        #Find the p vector
        xcomp = 1*cos(pi/2 - self.thetac)
        zcomp = 1*sin(pi/2 - self.thetac)

        self.pvec[0] = xcomp + xc
        self.pvec[1] = zcomp + zc
        '''
        a1 = self.alpha 
        b1 = self.beta
        c1 = self.gamma
        a2 = self.delta
        b2 = self.eps
        c2 = self.zeta

        xdm = -(b2*c1 - b1*c2)/(a2*b1 - b2*a1)
        zdm = -(a2*c1 - a1*c2)/(a2*b1 - b2*a1)
        '''       
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

        #Create the pose vector
        #It is a unit vector oriented in \theta_c direction 
        #from (xc,zc)

        xcomp = 1*cos(pi/2 - self.thetac)
        zcomp = 1*sin(pi/2 - self.thetac)
        self.ovec[0] = xcomp + xc; self.ovec[1] = zcomp + zc;

        rospy.loginfo('The lenght of the o vector is ' + str(np.linalg.norm(self.ovec)))       
       
        #Find the lateral error and it's line
        laterr = self.find_lateral_basis_and_error()
        self.gen_lerr_msg(laterr)
        self.set_lde_flag()

        #Find the perpendicular error and it's line
        #prperr = self.find_perp_error()
        #self.gen_perr_msg(prperr)
        #self.set_pde_flag()

        #Control System
        
        #self.control_it()


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

        if thetac>=0 and thetac<pi/2:
            signedk = k; signedd = d;

        if thetac>=-pi/2 and thetac<0:
            signedk = -k; signedd = +d;
 
        if thetac >=-pi and thetac<-pi/2:
            signedk = -k; signedd = -d;
 
        if thetac>=pi/2 and thetac<pi:
            signedk = -k; signedd = +d;

        self.signedk = signedk
        self.signedd = signedd

        #Now check if it's anticlockwise or clockwise
        if d < 0.02:
            self.dir_ = 1; rospy.loginfo('No need to turn')
        else:
            if signedk>0 and signedd<0:
                self.dir_ = 2; rospy.loginfo('Turn clockwise')
            if signedk>0 and signedd>0:
                self.dir_ = 0; rospy.loginfo('Turn anti-clockwise')

            if fabs(signedk)<fabs(signedd) and signedk<0 and signedd>0:
                self.dir_ = 0; rospy.loginfo('Turn anti-clockwise')
            if fabs(signedk)>fabs(signedd) and signedk<0 and signedd>0:
                self.dir_ = 2; rospy.loginfo('Turn clockwise')

            if fabs(signedk)>fabs(signedd) and signedk<0 and signedd<0:
                self.dir_ = 0; rospy.loginfo('Turn anti-clockwise')
            if fabs(signedk)<fabs(signedd) and signedk<0 and signedd<0:
                self.dir_ = 2; rospy.loginfo('Turn clockwise')
       
        rospy.loginfo(str(signedk) + ' is signed k value')
        rospy.loginfo(str(signedd) + ' is signed d value')
        self.gen_dir_msg()

        # 0 is anti 1 is no turn 2 is clock 
        self.fill_control_msg()
        self.gen_control_msg()

    def fill_control_msg(self):
        str_laterr_msg = str(self.laterr_msg)
        str_laterr_msg = str_laterr_msg[:str_laterr_msg.find('.')+4]
        #rospy.loginfo('lateral error message: ' + str_laterr_msg)
        str_prperr_msg = str(self.prperr_msg)
        str_prperr_msg = str_prperr_msg[:str_prperr_msg.find('.')+4]
        #rospy.loginfo('k error message: ' + str_prperr_msg)
        str_dir_msg = str(self.dir_msg)
        #rospy.loginfo('Direction: ' + str_dir_msg)
        self.control_msg = str_laterr_msg + '|' + str_prperr_msg + '|' + str_dir_msg
        #rospy.loginfo(self.control_msg)
    def gen_control_msg(self):
        self.send_control_ = True



 
    def gen_dir_msg(self):
        self.send_dir_ = True
        self.dir_msg = int(self.dir_)
    #Lateral error messsages
    def gen_lerr_msg(self, error):
        self.send_laterr_ = True
        self.laterr_msg = float(error)
    def set_lde_flag(self):
        self.send_lde_ = True
    #Perpendicular error messages
    def gen_perr_msg(self, error):
        self.send_prperr_ = True
        self.prperr_msg = float(error)
    def set_pde_flag(self):
        self.send_pde_ = True
        
        
# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('ksas_control', anonymous=True)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        control = Controller()
    except rospy.ROSInterruptException: pass
