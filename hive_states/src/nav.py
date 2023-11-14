"""
    nav.py

    Listens for an auto-nav service request, then sends the goal to the
    move_base server for an agent. Once there it will:

    - Actuate grippers if payload or drop field in request is set
        OR
    - Transition to standby 

    Implemented asynchronously to prevent clients from being blocked.

    TODO: Verify this transitions to standby

"""

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatusArray
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
        self.result = None
        self.incomplete = True
        self.queue = queue.Queue()

        # Use parameter server to signal if we're already navigating to a goal 
        rospy.set_param(f'{self.namespace}/goalSet', False)

        # Topic and publisher for actuating grippers
        self.actPubTopic = f'/{self.namespace}/actRouter'
        self.actPublisher = rospy.Publisher(self.actPubTopic, Int32)

        # Service for auto-nav requests
        self.goalServer = rospy.Service(f'{self.namespace}/auto', HIVE_AUTO, self.wpWrapper)
        self.client = actionlib.SimpleActionClient(f'{self.namespace}/move_base', MoveBaseAction)

    # Service calls and navigation commands are blocking, so we spin up a thread
    # to do the actual work, then return if the command will be executed
    def wpWrapper(self, req):
        # We're already moving to a goal.
        if self.busy:
            return HIVE_AUTOResponse(False)
        
        # Spin up a thread
        self.busy = True
        rospy.set_param(f'/{self.namespace}/goalSet', True)
        thread = threading.Thread(target=self.wpHandler, args=(req, self.queue))
        thread.start()

        return HIVE_AUTOResponse(True)


    def wpHandler(self, req, queue):
        self.client.wait_for_server()
        goalPose = MoveBaseGoal()
        goalPose.target_pose.header.frame_id = "map"
        goalPose.target_pose.header.stamp = rospy.Time.now()
        
        goalPose.target_pose.pose.position.x = req.wpt.x
        goalPose.target_pose.pose.position.y = req.wpt.y
        # We're not using 3D mapping
        goalPose.target_pose.pose.position.z = 0

        q = quaternion_from_euler(0,0,req.wpt.theta)
        goalPose.target_pose.pose.orientation.x = q[0]
        goalPose.target_pose.pose.orientation.y = q[1]
        goalPose.target_pose.pose.orientation.z = q[2]
        goalPose.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal= goalPose)

        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            self.busy = False
            self.incomplete = False
            return
        
        status_msg = rospy.wait_for_message(f'{self.namespace}/move_base/status', GoalStatusArray)

        self.result = (status_msg.status_list[0].status == 3)

        if req.payload or req.drop:
            # Actuate grippers
            actuationcmd = Int32()
            actuationcmd.data = self.id
            self.actPublisher.publish(actuationcmd)
        
        self.incomplete = False
        self.busy = False

    def execute(self, userdata):
        self.result = None
        while not rospy.is_shutdown():
            if self.result is not None:
                print(f'Goal reached:{self.result}')
                rospy.set_param(f'{self.namespace}/goalSet', False)
                return 'complete' if self.result else 'exit'