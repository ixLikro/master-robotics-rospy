#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

velPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

wallRight=False
wallBottom=False
wallLeft=False
wallTop=False
minRight=0
minBottom=0
minLeft=0
minTop=0

LOOKUP_RANGE=5
RANGE_UNTIL_DETECT_TOP=0.25
RANGE_UNTIL_DETECT_LEFT=0.8
RANGE_UNTIL_DETECT_BOTTOM=0.3
RANGE_UNTIL_DETECT_RIGHT=0.8

#reset the hole gazebo world
def reset():
    req = rospy.ServiceProxy('gazebo/reset_world', Empty)
    req.wait_for_service()
    req()

def callback(data):
    #split the data
    subLeft = data.ranges[90-LOOKUP_RANGE:90+LOOKUP_RANGE]
    subBottom = data.ranges[180-LOOKUP_RANGE:180+LOOKUP_RANGE]
    subRight = data.ranges[270-LOOKUP_RANGE:270+LOOKUP_RANGE]
    subTop = data.ranges[0:LOOKUP_RANGE] + data.ranges[360-LOOKUP_RANGE:359]

    global wallBottom, wallLeft, wallRight, wallTop, minBottom, minLeft, minRight, minTop

    # for x in data.ranges:
    #     print(x)

    #find min
    minBottom=min(subBottom)
    minLeft=min(subLeft)
    minRight=min(subRight)
    minTop=min(subTop)

    if(minTop <= RANGE_UNTIL_DETECT_TOP):
        wallTop=True
        print("\rWall Top Detected")
    else:
        wallTop=False
    if(minLeft <= RANGE_UNTIL_DETECT_LEFT):
        wallLeft=True
        print("\rWall Left Detected")
    else:
        wallLeft=False
    if(minBottom <= RANGE_UNTIL_DETECT_BOTTOM):
        wallBottom=True
        print("\rWall Bottom Detected")
    else:
        wallBottom=False
    if(minRight <= RANGE_UNTIL_DETECT_RIGHT):
        wallRight=True
        print("\rWall Right Detected")
    else:
        wallRight=False
    drive()
    print("\r")
    print("\r")

def drive():
    # setup message, reset all
    velMessage = Twist()
    velMessage.linear.x = 0
    velMessage.linear.y = 0
    velMessage.linear.z = 0
    velMessage.angular.x = 0
    velMessage.angular.y = 0
    velMessage.angular.z = 0
    
    #win detection
    if((not wallTop) and (not wallBottom) and (not wallLeft) and (not wallRight)):
        print("\rwin!")
        velPublisher.publish(velMessage)
        return

    #turn right 
    if(not wallRight):
        print("\rturn Right and little forward")
        velMessage.angular.z = -0.2
        velMessage.linear.x = 0.07
        velPublisher.publish(velMessage)

        return

    #turn left
    if(wallTop and (not wallLeft)):
        print("\rturn left")
        velMessage.angular.z = 0.1
        velPublisher.publish(velMessage)
        return

    #drive just forward
    if(not wallTop and (not wallLeft or not wallRight)):
        print("\rdrive forward")
        velMessage.linear.x = 0.15
        velPublisher.publish(velMessage)
        return

    #try to align and maybe drive forward
    if(wallLeft and wallRight):
        if((abs(minLeft-minRight) >= 0.15)):
            print("\rtry to align")
            print(abs(minLeft-minRight))
            if(minLeft > minRight):
                print("\rtoLeft")
                velMessage.angular.z=0.1
            else:
                print("\rtoRight")
                velMessage.angular.z=-0.1
        if(not wallTop and (abs(minLeft-minRight) <= 0.5)):
            velMessage.linear.x = 0.15
            print("\rand drive foreward")
        velPublisher.publish(velMessage)
        return

    print("\rNOTHING!!")
    velPublisher.publish(velMessage)


def init():
    print("\rInit Maze Solver. Press s to stop!")
    print("Note: This File doses not work! Please run solve_maze.py")
    rospy.init_node('maze_solver', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback)
    try:
        while(1):
            key = getKey()
            if(key == 's'):
                print("\rStop!")
                break
    finally:
        #stop robot at end
        velMessage = Twist()
        velMessage.linear.x = 0
        velMessage.linear.y = 0
        velMessage.linear.z = 0
        velMessage.angular.x = 0
        velMessage.angular.y = 0
        velMessage.angular.z = 0
        velPublisher.publish(velMessage)

def getKey():
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key



if __name__ == '__main__':
    init()