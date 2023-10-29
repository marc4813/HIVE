import rospy
from std_msgs.msg import Int32
from hive_states.srv import Decider

def deciderCallback(id, type, action):
    namespace = f'/agent{msg.id}'
    cmdTopic = namespace + '/command'
    actTopic = namespace + '/act'
    cmdPublisher = rospy.Publisher(topic, Int32, queue_size = 10)
    actPublisher = rospy.Publisher(actTopic, Int32, queue_size = 10)

    cmdPublisher.publish(Int32(type))
    actPublisher.publish(Int32(action))

server = rospy.Service('decider', Decider, deciderCallback)
server.spin()
