#! /usr/bin/python3

"""
	teleop.py
	Listens for the joystick topic and publishes cmd_vel for direct actuation.
	Currently transitions to standby, map, and nav.
	
"""

import rospy 
import smach
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class Teleop(smach.State):
	def __init__(self, agent_id = '1'):
		self.agent_id = agent_id
		self.namespace = f"agent{self.agent_id}"
		self.cmd_topic = f"/{self.namespace}/command"
		self.joy_topic = "/joystick"
		self.vel_topic = f"/{self.namespace}/cmd_vel"
		self.command = None
		self.twist_pub = rospy.Publisher(self.vel_topic, Twist, queue_size=10)
		
		# Topic, publisher, and message for actuating grippers
		self.actPubTopic = f'{self.namespace}/actRouter'
		self.actPublisher = rospy.Publisher(self.actPubTopic, Int32, queue_size=10)
		self.actmsg = Int32()
		self.actmsg.data = 0

		smach.State.__init__(self, outcomes=['standby', 'map', 'nav'])


	def command_cb(self, msg):
		self.command = msg.data

	def joystick_cb(self, msg):
		self.twist_pub.publish(msg)
		# Since we're in 2D mode, use Z to actuate grippers.
		self.actmsg = msg.linear.z
		self.actPublisher.publish(self.actmsg)

	def execute(self, userdata):
		
		# If we're here, we recieved the teleop command. 
		self.command = 1
		# Begin listening on topics
		command_sub = rospy.Subscriber(self.cmd_topic, Twist, self.command_cb)
		joystick_sub = rospy.Subscriber(self.joy_topic, Twist, self.joystick_cb)

		while not rospy.is_shutdown():
			if self.command == 0:
				command_sub.unregister()
				joystick_sub.unregister()
				return 'standby'
			if self.command == 2:
				command_sub.unregister()
				joystick_sub.unregister()
				return 'map'
			if self.command == 3:
				command_sub.unregister()
				joystick_sub.unregister()
				return 'nav'

