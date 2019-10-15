#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from turtlesim.srv import TeleportAbsolute
from std_srvs.srv import Empty
from turtlesim.srv import SetPen
from geometry_msgs.msg import Twist

velPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=100)

def init():
    rospy.init_node('tutle_house_drawer', anonymous=True)
    reset()
    turnPenOff()

#reset the hole canvas
def reset():
    # req = rospy.ServiceProxy('reset', Empty)
    # req.wait_for_service()
    # req()
    print("reset")

#port the given turtle to the given position and the given angle (0 - 360)
def teleportTo(x, y, angle = 0, turtle = 1):
    # req = rospy.ServiceProxy('turtle'+str(turtle)+'/teleport_absolute', TeleportAbsolute)
    # req.wait_for_service()
    # req(float(x),float(y), float(angle)/(180.0/3.14))
    print("teleport")

#turn the pen of the given turtle off or on (False)
def turnPenOff(off = True, turtle = 1):
    # req = rospy.ServiceProxy('turtle'+str(turtle)+'/set_pen', SetPen)
    # req.wait_for_service()
    # if (off == True):
    #     req(255,255,255,1,1)
    # else:
    #     req(255,255,255,1,0)
    print("turnPenOff")

#move turtle 1 straight foreward with the given speed(coordinate per second) to the given distace
def move(distance, speed = 3.5):
    # setup message
    velMessage = Twist()
    velMessage.linear.x = abs(speed)

    #reset other attribute, we want only move in a straight line
    velMessage.linear.y = 0
    velMessage.linear.z = 0
    velMessage.angular.x = 0
    velMessage.angular.y = 0
    velMessage.angular.z = 0

    #remember current time
    t0 = rospy.Time.now().to_sec()
    currentDistance = 0

    #Loop to move the turtle
    r = rospy.Rate(100)
    while(currentDistance < distance):
        # Publish message and get current time 
        velPublisher.publish(velMessage)
        t1 = rospy.Time.now().to_sec()
        # Calculate the already moved distance (formula: distance = speed * time)
        currentDistance = speed * (t1 - t0)
        r.sleep()

    #Force the turtle to stop
    velMessage.linear.x = 0
    velPublisher.publish(velMessage)

#turn turtle 1 in degrees (counterclockwise). With the given speed(degree per second)
def turn(angle, speed = 60.0, clockwise = False):

    #convert angles to radians
    angularSpeed = speed * (3.14 / 180)
    RadianAngle = angle * (3.14 / 180)

    #setup message
    velMessage = Twist()
    if(clockwise == False):
        velMessage.angular.z = abs(angularSpeed)
    else:
        velMessage.angular.z = -abs(angularSpeed)

    #reset other attribute, we want to turn the 
    velMessage.linear.x = 0
    velMessage.linear.y = 0
    velMessage.linear.z = 0
    velMessage.angular.x = 0
    velMessage.angular.y = 0

    #remember current time
    t0 = rospy.Time.now().to_sec()
    currentAngle = 0

    #Loop to turn the turtle
    r = rospy.Rate(500)
    while(currentAngle < RadianAngle):
        #Publish message and get current time 
        velPublisher.publish(velMessage)
        t1 = rospy.Time.now().to_sec()
        #Calculate the already moved distance (formula: distance = speed * time)
        currentAngle = angularSpeed * (t1 - t0)
        r.sleep()

    #Force the turtle to stop
    velMessage.linear.x = 0
    velPublisher.publish(velMessage)


def draw():
    #port to beginning
    reset()
    turnPenOff()
    teleportTo(8,2, 90)
    turnPenOff(False)

    #draw the house
    #right to top and top roof line
    move(5)
    turn(90)
    move(5)

    #roof, from left to right
    turn(90 + 45, clockwise=True)
    move(5.0 * math.cos(math.pi/4))
    turn(90, clockwise=True)
    move(5.0 * math.cos(math.pi/4))

    #diagonal from top right to bottom left
    turn(90, clockwise=True)
    move(math.sqrt(math.pow(5,2) + math.pow(5,2)))

    #connect to the roof an the other diagonal line
    turn(90 + 45, clockwise=True)
    move(5)
    turn(90 + 45, clockwise=True)
    move(math.sqrt(math.pow(5,2) + math.pow(5,2)))

    #last lane at the bottom
    turn(45 + 90, clockwise=True)
    move(5)


    #drive away
    turnPenOff()
    turn(45)
    move(2.2)

#draw the house instant 
def drawByPort():
    #port to beginning
    reset()
    turnPenOff()
    teleportTo(8,1)
    turnPenOff(False)

    #draw house with the help of the teleport service
    teleportTo(8,1)
    teleportTo(8,6)
    teleportTo(3,6)
    teleportTo(5.5,10)
    teleportTo(8,6)
    teleportTo(3,1)
    teleportTo(3,6)
    teleportTo(8,1)
    teleportTo(3,1, 180)


if __name__ == "__main__":
    try:
        init()
    except rospy.ROSInterruptException: pass

    #drawByPort()
    draw()

