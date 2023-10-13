#! /usr/bin/python3

"""
	state_machine.py
	
	Implements all of the states written in the hive_states package. 
	This state machine currently supports teleoperation only. 
	
	To be added: 
	Autonomous mapping 
	Autonomous navigation
	Payload pick-up/drop-off
	Instatiation by parameter
"""

import rospy
import smach

# States here:
from boot import Boot 
from standby import Standby
from teleop import Teleop
from map import Map

def main():
	rospy.init_node('agent_state_machine')
	
	sm = smach.StateMachine(outcomes=['shutdown'])

	with sm: 
		smach.StateMachine.add('BOOT', Boot(), 
					transitions={'success': 'STANDBY',
						     'fail': 'shutdown'})

		# TODO: Add mapping state and auto-nav state(s)
		smach.StateMachine.add('STANDBY', Standby(),
					transitions={'teleop': 'TELEOP',
				  			'map': 'MAP'})

		smach.StateMachine.add('TELEOP', Teleop(),
					transitions={'standby': 'STANDBY',
				  			'map': 'MAP'})
		
		smach.StateMachine.add('MAP', Map(),
					transitions={'complete': 'STANDBY',
				  			'exit': 'STANDBY'})

	outcome = sm.execute()

if __name__ == '__main__':
	main()
