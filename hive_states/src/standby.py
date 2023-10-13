#! /usr/bin/python3

"""
	Standby.py
	Listens for commands and transitions to the corresponding state. Valid transitions are: 
		* Map - Agent will begin building a map of the environment 
		* Teleop - Agent will respond to teleop commands 
		* Autonav - Agent will begin navigating to a specified point
		
		Currently transitions to teleop and map. 
	
"""

import rospy 
import smach
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from rospkg import RosPack
import roslaunch

class Standby(smach.State):
	def __init__(self, agent_id = '1'):
		self.agent_id = agent_id
		self.namespace = f"agent{self.agent_id}"
		self.cmd_topic = f"/{self.namespace}/command"
		self.command_sub = rospy.Subscriber(self.cmd_topic, Int32, self.command_cb)
		smach.State.__init__(self, outcomes=['teleop', 'map']) # TODO: Add auto-nav and map state transitions.
		self.command = 0
		self.waypoint = [0,0]

	def command_cb(self, msg):
		self.command = msg.data

	def execute(self, userdata):
		while not rospy.is_shutdown():
			if self.command == 1:
				return 'teleop'
			if self.command == 2:
				return 'map'