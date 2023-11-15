#! /usr/bin/python3

import rospy
import numpy as np
from detector import findPayloads
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header, Empty
from geometry_msgs.msg import PoseArray, Pose

class PayloadDetector:

    def __init__(self, mapTopic="agent1/map"):
        
        rospy.init_node('recognizer', anonymous = True)
        self.grid = OccupancyGrid()
        self.grid_recieved = False
        self.sub = rospy.Subscriber('agent1/map', OccupancyGrid, self.callback)
        self.cmd_sub = rospy.Subscriber('/findPayloads', Empty, self.cmd_callback)
        self.publisher = rospy.Publisher('payloadCoords', PoseArray, queue_size = 10)
        
    def cmd_callback(self, msg):
        
        if not self.grid_recieved:
            return
        
        info = self.grid.info
        data = self.grid.data
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
        data = PoseArray()
        data.header = Header()
        poses = []

        for coord in payloads:
            pose = Pose()
            pose.position.x = coord[1]
            pose.position.y = coord[0]
            pose.position.z = 0
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 1
            poses.append(pose)

        data.poses = poses
        self.publisher.publish(data)

    def callback(self, grid):
        self.grid_recieved = True
        self.grid = grid


if __name__ == '__main__':
    recognizer = PayloadDetector()

    while not rospy.is_shutdown():
        rospy.spin()