#! /usr/bin/python3

"""
	HighLevelSM.py
	
    Creates a ROS service that recieves requests and directs agents accordingly.
	
	To be added:
    Queue for payload pickup and drop-off requests.

    TODO: Actually test this thing

"""

import rospy
from std_msgs.msg import Int32
from hive_states.srv import Decider, DeciderResponse
from hive_states.srv import HIVE_AUTO
from geometry_msgs.msg import Pose2D

class AgentManager:
    def __init__(self):
        rospy.init_node('command_interpreter')
        self.server = rospy.Service('decider', Decider, self.deciderCallback)
        # Total number of agents
        if rospy.has_param('~totalAgents'):
            self.num_agents = rospy.get_param('~totalAgents')
        else:
            self.num_agents = 1
    
        rospy.spin()

    def deciderCallback(req, self):
        namespace = f'/agent{req.id}'
        cmdTopic = namespace + '/command'
        cmdPublisher = rospy.Publisher(cmdTopic, Int32, queue_size = 10)

        if req.command == 3:
            cmd = Int32()
            cmd.data = 3
            cmdPublisher.publish(cmd)
            payload = (req.autoType == 1)
            drop = (req.autoType == 2)
            no_task = (req.autoType == 0)
            wpt = Pose2D()
            wpt.x = req.x
            wpt.y = req.y
            wpt.theta = req.orientation
            # Wait for auto server for specific agent
            auto_server = namespace + '/auto'
            rospy.wait_for_service(auto_server)
            try: 
                auto_req = rospy.ServiceProxy(auto_server, HIVE_AUTO)
                res = auto_req(req.id, payload, drop, no_task, wpt)
            except rospy.ServiceException as e:
                rospy.logerr(f"Auto nav request for agent {req.id} failed.")
                return DeciderResponse(True)

        if req.command == 2:
            cmd = Int32()
            cmd.data = req.command    
            # We need to tell every agent to map.
            for i in range (1, self.num_agents + 1):
                # Instantiate publisher
                cmdtopic = f'agent{i}/command'
                pub = rospy.Publisher(cmdtopic, Int32, queue_size=10)
                # Publish command
                pub.publish(cmd)
                # Destroy publisher
                pub.unregister()
        
        if req.command == 1:
            cmd = Int32()
            cmd.data = req.command
            # We can simply send the command. The agent will listen on /joystick.
            cmdPublisher.publish(cmd)

        cmdPublisher.unregister()
        return DeciderResponse(False)


if __name__ == "__main__":
    AgentManager()