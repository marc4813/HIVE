#! /usr/bin/python3

import rospy
import numpy as np
from detector import findPayloads
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray

class PayloadDetector:

    def __init__(self, mapTopic="agent1/map"):
        
        rospy.init_node('recognizer', anonymous = True)
        
        self.sub = rospy.Subscriber('agent1/map', OccupancyGrid, self.callback)
        self.publisher = rospy.Publisher('payloadCoords', Float32MultiArray, queue_size = 10)

    def callback(self, grid):
        info = grid.info
        data = grid.data
        width = info.width
        height = info.height
        payloadMap = np.empty((height, width))
        next = 0

        for y in range(height):
            for x in range(width):
                payloadMap[y][x] = data[next]
                next+=1
        
        payloads = findPayloads(payloadMap, height, width)

        print(payloads)
        
        self.publisher.publish(Float32MultiArray(data=payloads))


if __name__ == '__main__':
    recognizer = PayloadDetector()

    while not rospy.is_shutdown():
        rospy.spin()