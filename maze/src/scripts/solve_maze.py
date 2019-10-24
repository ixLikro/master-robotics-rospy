#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

wallRight=False
wallBottom=False
wallLeft=False
wallTop=False

LOOKUP_RANGE=20
RANGE_UNTIL_DETECT=0.20

def callback(data):
    #split the data
    subLeft = data.ranges[90-LOOKUP_RANGE:90+LOOKUP_RANGE]
    subButtom = data.ranges[180-LOOKUP_RANGE:180+LOOKUP_RANGE]
    subRight = data.ranges[270-LOOKUP_RANGE:270+LOOKUP_RANGE]
    subTop = data.ranges[0:LOOKUP_RANGE] + data.ranges[360-LOOKUP_RANGE:359]

    if(min(subTop) <= RANGE_UNTIL_DETECT):
        wallTop=True
    else:
        wallTop=False
    if(min(subLeft) <= RANGE_UNTIL_DETECT):
        wallLeft=True
    else:
        wallLeft=False
    if(min(subButtom) <= RANGE_UNTIL_DETECT):
        wallBottom=True
    else:
        wallBottom=False
    if(min(subRight) <= RANGE_UNTIL_DETECT):
        wallRight=True
    else:
        wallRight=False
    print("Top \tmin: "+str(min(subTop))+", max: "+str(max(subTop)))
    print("Left \tmin: "+str(min(subLeft))+", max: "+str(max(subLeft)))
    print("Bottom \tmin: "+str(min(subButtom))+", max: "+str(max(subButtom)))
    print("Right \tmin: "+str(min(subRight))+", max: "+str(max(subRight)))
    print("")
    print("")
    print("")

def init():
    print("Init Maze Solver")
    rospy.init_node('maze_solver', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback)
    #spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    init()