#!/usr/bin/python3

"""
	Standby.py
	Listens for commands and transitions to the corresponding state. Valid transitions are: 
		* Map - Agent will begin building a map of the environment 
		* Teleop - Agent will respond to teleop commands 
		* Autonav - Agent will begin navigating to a specified point 
	
"""

import rospy 
import smach 
from hive_msg.msg import HIVE_AUTO
from hive_msg.msg import HIVE_RC



