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
import roslaunch

class Map(smach.State):
	def __init__(self, agent_id = '1'):
		smach.State.__init__(self, outcomes=['complete', 'exit'])
		self.agent_id = agent_id
		self.namespace = f"agent{self.agent_id}"
		self.cmd_topic = f"/{self.namespace}/command"
		self.frontier_topic = f"/{self.namespace}/explore/frontiers"
		self.timeout = 20 # How long to wait for new frontiers (seconds)
		rospy.Subscriber(self.cmd_topic, Int32, self.cmd_callback)
		rospy.Subscriber(self.frontier_topic, MarkerArray, self.marker_callback)
		self.hasMarkers = True
		self.kill_explore = False
		self.last_recieved = rospy.get_time()

	def cmd_callback(self, msg):
		# For now, 0 stops mapping. 
		if msg.data == 0:
			self.kill_explore = True

	def marker_callback(self, msg):
		# Whenever we recieive a message, set hasMarkers to True.
		self.hasMarkers = True
		self.last_recieved = rospy.get_time()
	
	def execute(self, userdata):
		# start explore_lite
		rp = RosPack()
		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		cli_args = [rp.get_path('hive_states') + '/launch/explore.launch','ns:=' + self.namespace,
					'agent_costmap_topic:=map', 'agent_costmap_updates_topic:=move_base/global_costmap/map_updates']
		roslaunch_args = cli_args[1:]
		roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

		parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
		parent.start()

		while not rospy.is_shutdown():
			# Mapping has been killed
			if self.kill_explore:
				parent.shutdown()
				return 'exit'
			
			# We're finished mapping
			if rospy.get_time() - self.last_recieved >= self.timeout:
				parent.shutdown()
				return 'complete'