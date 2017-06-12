#!/usr/bin/env python

__author__ = 'thaus'

import rospy
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped, Pose, Vector3
import dynamic_reconfigure.client
from mav_msgs.cfg import MavAttitudeCtlParamsConfig
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
import math
import numpy

class ReferencePublisher:

    def __init__(self):
        self.time_old = rospy.Time.now()
        self.pose_old = Pose()
        self.start_flag = False
        self.vel_ref_msg = Vector3()
        self.position = Vector3()
        self.vel_ref = [[0.5, 0], [0, 0], [-0.5, 0], [0,0], [0, 0.5], [0, 0], [0, -0.5], [0, 0],[0.5, 0.5], [0, 0], [-0.5, -0.5], [0,0]]

        self.vel_ref_pub = rospy.Publisher('/arducopter/vel_ref', Vector3, queue_size=5)
        self.position_pub = rospy.Publisher('/arducopter/pos_ref', Vector3, queue_size=5)
        #self.client = dynamic_reconfigure.client.Client('/mass_ctl_attitude', timeout=30, config_callback=self.reconf_cb)

        rospy.sleep(0.1)

    def run(self):
        rate = rospy.Rate(10.0) # 7 sec
        i = 8
        self.position.x = 0
        self.position.y = 0
        self.position.z = 2
        self.position_pub.publish(self.position)
        self.position_pub.publish(self.position)
        self.position_pub.publish(self.position)
        rospy.sleep(1) #wait to climb to height 
       	
       	time_start = rospy.Time.now();
    
       	while not rospy.is_shutdown():
       		
	   		now = rospy.Time.now()	
			if(now - time_start)<= 4.0:
				self.vel_ref_msg.x = 0.125*(now-time_start);    
			if (now-time_start)<=8.0 and (now-time_start)>rospy.Duration.from_sec(4.0) :
				self.vel_ref_msg.x = 0.5
		   	if (now-time_start)>8.0:
		   		self.vel_ref_msg.x = 0.5-0.125*(now-time_start-8.0)	
			
			self.vel_ref_msg.y = 0
			self.vel_ref_msg.z = 0
			self.vel_ref_pub.publish(self.vel_ref_msg)
			rate.sleep()
			if (now-time_start)>12: 		      
				break

		
if __name__ == '__main__':

    rospy.init_node('mav_vel_ref_publisher')
    ref_pub = ReferencePublisher()
    print "Starting publishing"
    ref_pub.run()
