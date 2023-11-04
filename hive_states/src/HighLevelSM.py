#! /usr/bin/python3

"""
	HighLevelSM.py
	
    Creates a ROS node that recieves topics and directs agents accordingly.
	
	To be added:
    Queue for payload pickup and drop-off requests.

    TODO: Actually test this thing

"""

import rospy
from std_msgs.msg import Int32
from hive_states.msg import Decider
from hive_states.srv import HIVE_AUTO
from geometry_msgs.msg import Pose2D


class AgentManager:
    def __init__(self):
        rospy.init_node('command_interpreter')
        self.sub = rospy.Subscriber('/decider', Decider, self.deciderCallback)
        
        # Total number of agents
        if rospy.has_param('~totalAgents'):
            self.num_agents = rospy.get_param('~totalAgents')
        else:
            self.num_agents = 1
    
        self.publishers = [rospy.Publisher(f'/agent{i}'+'/command', Int32, queue_size=10) for i in range(1,self.num_agents+1)]
        rospy.spin()

    def deciderCallback(self, msg):
        cmd = Int32()

        if msg.command == 1:
            cmd.data = msg.command
            # We can simply send the command. The agent will listen on /joystick.
            self.publishers[msg.id - 1].publish(cmd)
        
        if msg.command == 2:
            cmd.data = msg.command
            # Once again, we can just send the command. Agents will begin mapping.
            for i in range(0, self.num_agents):
                self.publishers[i].publish(cmd)

        if msg.command == 3:
            cmd = Int32()
            cmd.data = 3
            self.publishers[msg.id - 1].publish(cmd)
            payload = (msg.autoType == 1)
            drop = (msg.autoType == 2)
            no_task = (msg.autoType == 0)
            wpt = Pose2D()
            wpt.x = msg.x
            wpt.y = msg.y
            wpt.theta = msg.orientation
            # Wait for auto server for specific agent
            auto_server = f"agent{msg.id}" + '/auto'
            rospy.wait_for_service(auto_server)
            try: 
                auto_msg = rospy.ServiceProxy(auto_server, HIVE_AUTO)
                res = auto_msg(msg.id, payload, drop, no_task, wpt)
            except rospy.ServiceException as e:
                rospy.logerr(f"Auto nav msguest for agent {msg.id} failed.")
                return



if __name__ == "__main__":
    AgentManager()
