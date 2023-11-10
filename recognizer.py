import rospy
import numpy as np
from detector import findPayloads
from nav_msgs import OccupancyGrid
from std_msgs import Int16MultiArray

publisher = rospy.Publisher('payloadCoords', Int16MultiArray, queue_size = 10)

def callback(grid):
    info = grid.info
    data = grid.data
    width = info.width
    height = info.height
    payloadMap = np.empty((height, width))
    next = 0

    for y in range(height):
        for x in range(width):
            payloadMap[y][x] = data[next]
            data+=1
    
    payloads = findPayloads(payloadMap)
    
    publisher.publish(Int16MultiArray(data = payloads))

def recognizer():
    rospy.init_node('recognizer', anonymous = True)
    
    rospy.Subscriber('agent1/map', OccupancyGrid, callback)

    rospy.spin()

if __name__ == '__main__':
    recognizer()

