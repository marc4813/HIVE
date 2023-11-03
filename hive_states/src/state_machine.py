#! /usr/bin/python3

"""
	state_machine.py
	
	Implements all of the states written in the hive_states package. 
	This state machine currently supports teleoperation only. 
	
	To be tested:
	Nav state

"""

import rospy
import smach

# States here:
from boot import Boot 
from standby import Standby
from teleop import Teleop
from map import Map
from nav import Nav

def main():
	rospy.init_node('agent_state_machine')
	id = rospy.get_param('~agent_id')
	sm = smach.StateMachine(outcomes=['shutdown'])

	with sm: 
		smach.StateMachine.add('BOOT', Boot(agent_id=id), 
					transitions={'success': 'STANDBY',
						     	 'fail': 'shutdown'})

		# TODO: Add mapping state and auto-nav state(s)
		smach.StateMachine.add('STANDBY', Standby(agent_id=id),
					transitions={'teleop': 'TELEOP',
				  				 'map': 'MAP'})

		smach.StateMachine.add('TELEOP', Teleop(agent_id=id),
					transitions={'standby': 'STANDBY',
				  				 'map': 'MAP',
								 'nav': 'NAV'})
		
		smach.StateMachine.add('MAP', Map(agent_id=id),
					transitions={'complete': 'STANDBY',
				  				 'exit': 'STANDBY'})

		smach.StateMachine.add('NAV', Nav(agent_id=id),
					transitions={'complete': 'STANDBY',
								 'exit': 'STANDBY'})

	outcome = sm.execute()

if __name__ == '__main__':
	main()
