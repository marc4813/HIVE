import ros
import actionlib
import smach
from geometry_msgs import Pose
from move_base_msgs import MoveBaseGoal, MoveBaseAction
from std_msgs import Int32, Header

class Nav(smach.State):
    def __init__(self, agent_id = '1'):
        smach.State.__init__(self, outcomes=['complete', 'exit'])
        self.id = agent_id
        self.namespace = f'agent{agent_id}'
        self.received = False
        self.incomplete = False
        self.wpTopic = f'{self.namespace}/waypoint'
        self.actSubTopic = f'{self.namespace}/act'
        self.actPubTopic = f'{self.namespace}/actRouter'
        self.wpSubscriber = rospy.Subscriber(self.wpTopic, Pose, wpSub)
        self.actSubscriber = rospy.Subscriber(self.actSubTopic, Int32, actSub)
        self.actPublisher = rospy.Publisher(self.actPubTopic, Int32)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def wpSub(self, msg):
        self.client.wait_for_server()
        
        self.data = MoveBaseGoal()
        self.data.target_pose.header.frame_id = "map"
        self.data.target_pose.header.stamp = rospy.Time.now()
        self.data.target_pose.pose = msg
        
        self.client.send_goal(goal)

        self.wait = self.cient.wait_for_result()
        self.result = self.client.get_result()

        if(is self.wait and is self.result.data):
            self.received = True
        else:
            self.incomplete = True

    def actSub(self, msg):
        if(is self.received):
            self.received = False

            self.actPublisher.publish(msg)

    def execute(self):
        self.wpSubscriber.spin()
        self.actSubscriber.spin()

        return 'complete' if not self.incomplete else 'exit'