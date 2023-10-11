#! /usr/bin/python3

"""
	teleop.py
	Listens for the joystick topic and publishes cmd_vel for direct actuation.
	Currently transitions to standby and map. 
	
"""

import rospy 
import smach
from geometry_msgs.msg import Twist

class Teleop(smach.State):
	def __init__(self, agent_id = '1'):
		self.agent_id = agent_id
		self.namespace = f"agent{self.agent_id}"
		self.cmd_topic = f"/{self.namespace}/command"
		self.joy_topic = f"/{self.namespace}/joystick"
		self.vel_topic = f"/{self.namespace}/cmd_vel"
		self.joystick_sub = rospy.Subscriber(self.joy_topic, Twist, self.joystick_cb)
		self.command_sub = rospy.Subscriber(self.cmd_topic, Twist, self.command_cb)
		self.twist_pub = rospy.Publisher(self.vel_topic, Twist, queue_size=10)
		smach.State.__init__(self, outcomes=['standby', 'map']) # TODO: Add auto-nav and map state transitions.
		self.command = 0
		self.waypoint = [0,0]


	def command_cb(self, msg):
		self.command = msg.data

	def joystick_cb(self, msg):
		self.twist_pub.publish(msg)

	def execute(self, userdata):
		while not rospy.is_shutdown():
			if self.command == 0:
				return 'standby'
			if self.command == 2:
				return 'map'

