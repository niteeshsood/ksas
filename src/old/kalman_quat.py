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

class QuatToEuler():
    def __init__(self):
        self.got_new_msg = False
        self.euler_msg = Eulers()
        self.tpose_msg = TotalPose()
        self.apose = Pose()
        self.ideltapos = 0.05
        self.ideltaqua = 0.05
        #self.ideltarpy = 0.05
        self.adeltapos = 0.05
        self.adeltaqua = 0.05
        #self.adeltarpy = 0.05
        #self.pt = Point()
        #self.send_des_ = False
        #self.send_error_ = False
        self.iposx = 0.0
        self.iposy = 0.0
        self.iposz = 0.0
        self.iquax = 0.0
        self.iquay = 0.0
        self.iquaz = 0.0
        self.iquaw = 0.0
        #self.iroll = 0.0
        #self.ipitc = 0.0
        #self.iyaw = 0.0
        #self.start_time = rospy.get_rostime()
        self.got_history = False
        self.window_size = 3
        self.history = list()#list of all poses coming from /sptam/robot/pose
        self.current_tpose = TotalPose()

        # Create subscribers and publishers.
        #sub_imu   = rospy.Subscriber("imu", Imu, self.imu_callback)
        sub_sptam  = rospy.Subscriber('sptam/robot/pose', PoseStamped, self.odom_callback)
        pub_euler = rospy.Publisher('euler', Eulers, queue_size=10)
        pub_tpose = rospy.Publisher('totalpose', TotalPose, queue_size=10)

        # Main while loop.
        while not rospy.is_shutdown():
            # Publish new data if we got a new message.
            if self.got_new_msg:
                pub_euler.publish(self.euler_msg)
                pub_tpose.publish(self.tpose_msg)
                self.got_new_msg = False

    # Odometry callback function.
    def odom_callback(self, msg):
        #add the new message to the history
        self.history.append(msg)


        #if the window is full do a filter
        if len(self.history) == self.window_size + 1:
            self.a_filter()
            #The last added msg is the new msg
            (r, p, y) = tf.transformations.euler_from_quaternion([self.history[-1].pose.orientation.x, self.history[-1].pose.orientation.y, self.history[-1].pose.orientation.z, self.history[-1].pose.orientation.w])
            rospy.loginfo(str(self.history[-1].pose.orientation.x) + ' is x')
            self.fill_euler_msga(r, p, y)


        #if the window is not full do i filter
        if len(self.history) < self.window_size:
	    #rospy.loginfo("Window is at " + str(self.window_size-len(self.history))    
            self.i_filter()
            #The last added msg is the new msg
            (r, p, y) = tf.transformations.euler_from_quaternion([self.history[-1].pose.orientation.x, self.history[-1].pose.orientation.y, self.history[-1].pose.orientation.z, self.history[-1].pose.orientation.w])
            self.fill_euler_msgi(r, p, y)

        # Convert quaternions to Euler angles.
        
    # Fill in Euler angle message.
    def fill_euler_msgi(self, r, p, y):
        self.got_new_msg = True
        self.euler_msg.header.stamp = self.history[-1].header.stamp
        self.euler_msg.roll  = r
        self.euler_msg.pitch = p
        self.euler_msg.yaw   = y
        self.tpose_msg.euler = self.euler_msg
        self.apose.position.x = self.iposx
        self.apose.position.y = self.iposy
        self.apose.position.z = self.iposz
        self.apose.orientation.x = self.iquax
        self.apose.orientation.y = self.iquay
        self.apose.orientation.z = self.iquaz
        self.apose.orientation.w = self.iquaw
        self.tpose_msg.pose = self.apose
        '''
        self.tpose_msg.pose.position.x = self.iposx
        self.tpose_msg.pose.position.y = self.iposy
        self.tpose_msg.pose.pose.position.z = self.iposz
        self.tpose_msg.pose.orientation.x = self.iquax
        self.tpose_msg.pose.orientation.y = self.iquay
        self.tpose_msg.pose.orientation.z = self.iquaz
        self.tpose_msg.pose.orientation.w = self.iquaw
        '''

    def fill_euler_msga(self, r, p, y):
        self.got_new_msg = True
        self.euler_msg.header.stamp = self.history[-1].header.stamp
        self.euler_msg.roll  = r
        self.euler_msg.pitch = p
        self.euler_msg.yaw   = y
        self.tpose_msg.euler = self.euler_msg
        self.tpose_msg.pose = self.history[-1].pose
        

    #initial filtering
    def i_filter(self):

        for i in xrange(-1,-2,-1): #quick hack to have i = -1
            #because I didn't want to reformat all the code
            
            cposx = self.history[i].pose.position.x; 
            if cposx > self.iposx + self.ideltapos:
                cposx = self.iposx 
            self.iposx = cposx


            cposy = self.history[i].pose.position.y
            if cposy > self.iposy + self.ideltapos:
                cposy = self.iposy 
            self.iposy = cposy

            cposz = self.history[i].pose.position.z
            if cposz > self.iposz + self.ideltapos:
                cposz = self.iposz 
            self.iposz = cposz

            cquax = self.history[i].pose.orientation.x
            if cquax > self.iquax + self.ideltapos:
                cquax = self.iquax 
            self.iquax = cquax
              
            cquay = self.history[i].pose.orientation.y
            if cquay > self.iquay + self.ideltapos:
                cquay = self.iquay 
            self.iquay = cquay

            cquaz = self.history[i].pose.orientation.z
            if cquaz > self.iquaz + self.ideltapos:
                cposx = self.iquaz 
            self.iposx = cquaz

            cquaw = self.history[i].pose.orientation.w
            if cquaw > self.iquaw + self.ideltapos:
                cquaw = self.iquaw 
            self.iquaw = cquaw


    #Filtering after the window is filled
    def a_filter(self):
		
	tposx = 0.0; tposy = 0.0; tposz = 0.0; tquax = 0.0; tquay = 0.0; tquaz = 0.0; tquaw = 0.0; troll = 0.0; tpitc = 0.0; tyaw = 0.0;
		
	#Average the values over the window
	for i in xrange(self.window_size):
	    cposx = self.history[i].pose.position.x; tposx += cposx
	    cposy = self.history[i].pose.position.y; tposy += cposy
	    cposz = self.history[i].pose.position.z; tposz += cposz

	    cquax = self.history[i].pose.orientation.x; tquax += cquax
	    cquay = self.history[i].pose.orientation.y; tquay += cquay
	    cquaz = self.history[i].pose.orientation.z; tquaz += cquaz
	    cquaw = self.history[i].pose.orientation.w; tquax += cquaw

	posxavg = tposx/self.window_size
	posyavg = tposy/self.window_size
	poszavg = tposz/self.window_size
		
	quaxavg = tquax/self.window_size
	quayavg = tquay/self.window_size
	quazavg = tquaz/self.window_size
	quawavg = tquaw/self.window_size

        #New value should vary about the average only
        nposx = self.history[-1].pose.position.x
        nposy = self.history[-1].pose.position.y
        nposz = self.history[-1].pose.position.z

        nquax = self.history[-1].pose.orientation.x
        nquay = self.history[-1].pose.orientation.y
        nquaz = self.history[-1].pose.orientation.z
        nquaw = self.history[-1].pose.orientation.w


	#if not good enough, just pop it  

        #First check the positions
        if nposx > tposx + self.adeltapos:  
            self.history.pop()
    
        if nposy > tposy + self.adeltapos:  
            self.history.pop()

        if nposz > tposz + self.adeltapos:  
            self.history.pop()
            
        #Now check the quaternion
        if nquax > tquax + self.adeltapos:  
            self.history.pop()
            

        if nquay > tquay + self.adeltapos:  
            self.history.pop()
            

        if nquaz > tquaz + self.adeltapos:  
            self.history.pop()
            

        if nquaw > tquaw + self.adeltapos:  
            self.history.pop()
            

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('ksas_quat_to_euler', anonymous=True)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        quat_to_euler = QuatToEuler()
    except rospy.ROSInterruptException: pass
