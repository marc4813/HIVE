#! /usr/bin/python3

"""
	Starts the explore_lite package for individual agents.
	
	1: Start the explore_lite package 
	2: Check if frontiers have been published.
	3: If new frontiers haven't been created for {timeout} seconds, mapping is finished.
	TODO: 4: Save map and move to standby using map_server + map_saver
"""

import rospy
import smach
from rospkg import RosPack
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import roslaunch
from Map.srv import Map

class Map(smach.State):
	def __init__(self, agent_id = '1'):
		smach.State.__init__(self, outcomes=['complete', 'exit'])
		self.agent_id = agent_id
		self.namespace = f"agent{self.agent_id}"
		self.cmd_topic = f"/{self.namespace}/command"
		self.map_command = f"/{self.namespace}/start_mapping"
		self.frontier_topic = f"/{self.namespace}/explore/frontiers"

		self.timeout = 20 # How long to wait for new frontiers (seconds)
		self.command_sub = rospy.Subscriber(self.cmd_topic, Int32, self.cmd_callback)
		self.frontier_sub = rospy.Subscriber(self.frontier_topic, MarkerArray, self.marker_callback)

		self.kill_explore = False
		self.last_recieved = rospy.get_time()
		self.hive_explore = rospy.ServiceProxy("hive_explore_service", Map)


	def cmd_callback(self, msg):
		# For now, 0 stops mapping. 
		if msg.data == 0:
			self.kill_explore = True

	def marker_callback(self, msg):
		# Whenever we recieive a message, set hasMarkers to True.
		self.hasMarkers = True
		self.last_recieved = rospy.get_time()
	
	def execute(self, userdata):
		rospy.wait_for_service("hive_explore_service")
		rospy.loginfo("Starting mapping")
		self.result = self.hive_explore(not self.kill_explore)

		while not rospy.is_shutdown():
			# Mapping has been killed
			if self.kill_explore:
				self.result = self.hive_explore(not self.kill_explore)
				return 'exit'
			
			# We're finished mapping
			if rospy.get_time() - self.last_recieved >= self.timeout:
				self.result = self.hive_explore(not self.kill_explore)
				return 'complete'

		return 'complete' if not self.result else 'exit'
