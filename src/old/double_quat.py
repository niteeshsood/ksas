#!/usr/bin/python
# -*- coding: utf-8 -*-

# Start up ROS pieces.
PKG = 'ksas'
import roslib; roslib.load_manifest(PKG)
import rospy
import tf

# ROS messages.
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from sensor_msgs.msg import Imu
from ksas.msg import Eulers, TotalPose

# Processing libraries
from math import fabs

class QuatToEuler():
    def __init__(self):
        self.got_new_msg = False
        self.first = False
        self.euler_msg = Eulers()
        self.tpose_msg = TotalPose()
        self.oldposestamped = PoseStamped()      
        self.newposestamped = PoseStamped()
        self.deltapos = 0.2
        self.deltaqua = 0.1
        

        # Create subscribers and publishers.
        #sub_imu   = rospy.Subscriber("imu", Imu, self.imu_callback)
        sub_sptam  = rospy.Subscriber('sptam/robot/pose', PoseStamped, self.odom_callback)
        pub_euler = rospy.Publisher('euler', Eulers, queue_size=10)
        pub_tpose = rospy.Publisher('totalpose', TotalPose, queue_size=10)

        # Main while loop.
        while not rospy.is_shutdown():
            # Publish new data if we got a new message.
            if self.got_new_msg:
                #rospy.loginfo('First received: ' + str(self.first))
                pub_euler.publish(self.euler_msg)
                pub_tpose.publish(self.tpose_msg)
                self.got_new_msg = False

    # Odometry callback function.
    def odom_callback(self, msg):
        #add the new message to the history
        if not self.first:
            self.oldposestamped = msg
            self.first = True        
            (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
            self.fill_euler_msgf(msg, r, p, y)
            #rospy.loginfo('First is filled')
        else:
            self.newposestamped = msg
            #rospy.loginfo('New msg')
            skip = self.doublefilter(msg)
            if skip: 
                self.got_new_msg = True #Send old msg again
            else:
                (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
                self.fill_euler_msgf(msg, r, p, y)
                self.oldposestamped = msg

        
    # Fill in Euler angle message.
    def fill_euler_msgf(self, msg, r, p, y):
        self.got_new_msg = True
        self.euler_msg.header.stamp = msg.header.stamp
        self.euler_msg.roll  = r
        self.euler_msg.pitch = p
        self.euler_msg.yaw   = y
        self.tpose_msg.euler = self.euler_msg
        self.tpose_msg.pose = msg.pose       

    def doublefilter(self, msg):
        px = msg.pose.position.x
        py = msg.pose.position.y
        pz = msg.pose.position.z
        
        ox = msg.pose.orientation.x  
        oy = msg.pose.orientation.y
        oz = msg.pose.orientation.z
        ow = msg.pose.orientation.w

        _px = self.oldposestamped.pose.position.x
        _py = self.oldposestamped.pose.position.y
        _pz = self.oldposestamped.pose.position.z
        
        _ox = self.oldposestamped.pose.orientation.x  
        _oy = self.oldposestamped.pose.orientation.y
        _oz = self.oldposestamped.pose.orientation.z
        _ow = self.oldposestamped.pose.orientation.w

        if fabs(px - _px) > self.deltapos:  return True
        if fabs(py - _py) > self.deltapos:  return True
        if fabs(pz - _pz) > self.deltapos:  return True
       
        if fabs(ox - _ox) > self.deltaqua:  return True
        if fabs(oy - _oy) > self.deltaqua:  return True
        if fabs(oz - _oz) > self.deltaqua:  return True
        if fabs(ow - _ow) > self.deltaqua:  return True
        #rospy.loginfo('point x ' + str(fabs(px - _px)))
        #rospy.loginfo('point y ' + str(fabs(py - _py)))       
        #rospy.loginfo('point z ' + str(fabs(pz - _pz)))
        #rospy.loginfo('point y ' + str(fabs(pz - _pz) > self.deltapos)) 
        return False
      # Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('filtered_ksas_quat_to_euler', anonymous=True)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        quat_to_euler = QuatToEuler()
    except rospy.ROSInterruptException: pass
