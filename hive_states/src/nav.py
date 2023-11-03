"""
    nav.py

    Listens for an auto-nav service request, then sends the goal to the
    move_base server for an agent. Once there it will:

    - Actuate grippers if payload or drop field in request is set
        OR
    - Transition to standby 

    Implemented asynchronously to prevent clients from being blocked.

    TODO: Actually test this thing

"""

import rospy
import actionlib
import smach
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from std_msgs.msg import Int32
from hive_states.srv import HIVE_AUTO, HIVE_AUTOResponse
from tf.transformations import quaternion_from_euler
import threading
import queue

class Nav(smach.State):
    def __init__(self, agent_id = '1'):
        smach.State.__init__(self, outcomes=['complete', 'exit'])
        self.id = agent_id
        self.namespace = f'agent{agent_id}'
        self.received = False
        self.busy = False
        self.result = False
        self.incomplete = True
        self.queue = queue.Queue()

        # Topic and publisher for actuating grippers
        self.actPubTopic = f'{self.namespace}/actRouter'
        self.actPublisher = rospy.Publisher(self.actPubTopic, Int32)

        # Service for auto-nav requests
        self.goalServer = rospy.Service('auto', HIVE_AUTO, self.wpWrapper)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # Service calls and navigation commands are blocking, so we spin up a thread
    # to do the actual work, then return if the command will be executed
    def wpWrapper(self, req):
        # We're already moving to a goal.
        if self.busy:
            return HIVE_AUTOResponse(False)
        
        # Spin up a thread
        self.busy = True
        thread = threading.Thread(target=self.wpHandler, args=(req, self.queue))
        thread.start()

        return HIVE_AUTOResponse(True)


    def wpHandler(self, req):
        self.client.wait_for_server()
        self.data = MoveBaseGoal()
        self.data.target_pose.header.frame_id = "map"
        self.data.target_pose.header.stamp = rospy.Time.now()
        
        self.data.target_pose.pose.x = req.wpt.x
        self.data.target_pose.pose.y = req.wpt.y
        # We're not using 3D mapping
        self.data.target_pose.pose.z = 0

        q = quaternion_from_euler(0,0,req.wpt.theta)
        self.data.target_pose.pose.orientation = q

        self.client.send_goal(goal = self.data)

        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            self.busy = False
            self.incomplete = False
            return
        
        self.result = self.client.get_result()

        if req.payload or req.drop:
            # Actuate grippers
            actuationcmd = Int32()
            actuationcmd.data = self.id
            self.actPublisher.publish(actuationcmd)
        
        self.incomplete = False

    def execute(self):

        while not rospy.is_shutdown() and self.incomplete:
            rospy.spin()

        return 'complete' if self.result else 'exit'