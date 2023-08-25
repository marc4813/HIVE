#!/usr/bin/python3

"""
boot.py

Defines a state that waits for 45 seconds before returning 'success'. Returns 'fail' otherwise. 

"""

import rospy 
import smach
from sensor_msgs.msg import PointCloud2 as pcl2
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry as odom
import time


class Boot(smach.State):
	def __init__(self, agent_id):
		smach.State.__init__(self, outcomes=['success', 'fail'])
		self.agent_id = agent_id
		self.namespace = f"/agent{self.agent_id}/"
		self.pcl_topic = f"{self.namespace}laserscan"
		self.imu_topic = f"{self.namespace}raw_imu"
		self.odom_topic = f"{self.namespace}odom"
		self.timeout = 45
		self.pcl_sub = rospy.Subscriber(pcl_topic, pcl2, self.pcl_cb)
		self.imu_sub = rospy.Subscriber(imu_topic, Imu,  self.imu_cb)
		self.odom_sub = rospy.Subscriber(odom_topic, odom, self.odom_cb)
	
	def pcl_cb(msg):
		self.pcl_online = True
	
	def imu_cb(msg):
		self.imu_online = True
		
	def odom_cb(msg):
		self.odom_online = True
	
	def execute(self, userdata):
		start = time.time()
		while start - time.time() < self.timeout:

			if self.pcl_online and self.imu_online and self.odom_online:
				rospy.loginfo("All data streams online, transitioning to STANDBY..")
				return 'success'
		
			rospy.loginfo(f"Agent {self.agent_id} Data stream status:"
			              f"Laserscan: {self.pcl_online}"
				      f"Imu: {self.imu_online}"
				      f"Odom: {self.odom_online}")
			
			self.pcl_online = False
			self.imu_online = False
			self.odom_online = False
			
		rospy.loginfo(f"Agent {self.agent_id} Boot failed.")
		return 'fail'

