#! /usr/bin/python3


"""

    teleopTest.py

    Tests the teleop functionality by making a Decider service call, then sending
    Twist commands on /joystick

"""

import rospy
from hive_states.msg import Decider
from geometry_msgs.msg import Twist

rospy.init_node('teleop_test')
decider = rospy.Publisher('decider', Decider, queue_size=1)
joystick_pub = rospy.Publisher('/joystick', Twist, queue_size=1)
msg = Decider()
turn = Twist()

print("Starting test...")
msg.id = 1
msg.command = 1
decider.publish(msg)
turn.angular.z = 1.0
turn.linear.x = 0
turn.linear.y = 0
rospy.sleep(3.)
joystick_pub.publish(turn)
turn.angular.z = 0
joystick_pub.publish(turn)