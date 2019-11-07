#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import numpy as np
import Tkinter
import math

ERROR_CODE=100.0

### Settings and global Variables###

#debug window settings
CANVAS_HEIGHT=500
CANVAS_WIDTH=500
POINT_SCALE_FACTOR=80

#steering velocity that is used if the robot turned to much relative to the right wall
ALIGN_STEERING = 0.2
#sterrig velocity that is used if the robot is aligned relative to the right wall but not centered between the left and the right wall
CENTER_STEERING = 0.1

IS_PERFORMING_TURN_RIGHT=False
IS_PERFORMING_TURN_LEFT=False
counter = 0 #turn right and left counter

#the lineare velocity that is used to drive foreward
DEFAULT_LINEAR_VELOCITY=0.15
#the length or duration of the turn routines. 10 means, the driving logic will be blocked until 10 new scan measurements arrived 
TURN_ROUTINE_LENGTH=42
#the turn routine contains 2 parts: 1st part-> turning, 2nd-> part: driving foreward. this constance descripes how long the 1st part is.
TURN_ROUTING_1st_PART_LENGTH=25
TURN_ROUTINE_TURN_VELOCITY=0.3

#are walls detected
wallRight=False
wallBottom=False
wallLeft=False
wallTop=False

#how detect walls
LOOKUP_RANGE=8
LOOKUP_RANGE_TOP=1
RANGE_UNTIL_DETECT_TOP=0.35
RANGE_UNTIL_DETECT_LEFT=0.8
RANGE_UNTIL_DETECT_BOTTOM=0.3
RANGE_UNTIL_DETECT_RIGHT=0.8

velPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
top = Tkinter.Tk()
canvas = Tkinter.Canvas(top, bg="blue", height=CANVAS_HEIGHT, width=CANVAS_WIDTH)

def callback(data):
    i = 0
    listXY = []
    #calculate the points
    while i < len(data.ranges):
        reverseIndex=len(data.ranges)-1-i
        listXY.append( 
            (data.ranges[i] * math.sin(math.radians(i))       #x
                , data.ranges[i] * math.cos(math.radians(i))  #y
            ))
        i=i+1
    # for l in listXY:
    #     if (not math.isnan(l[0]) and not math.isinf(l[0]) and not math.isnan(l[1]) and not math.isinf(l[1])):
    #         print(l)
    isWallNearby(data.ranges)
    drive(listXY)
    drawPoints(listXY)
    

#draw measured points in the window
def drawPoints(listXY):
    i = 0
    resetCanvas()
    #scale and draw points
    while i < len(listXY):
        #translate to the middle of the screen and draw points
        drawPoint(tranlateToCenterScreen(listXY[i][0]), tranlateToCenterScreen(listXY[i][1], True))
        i=i+5
    #draw drive to line
    canvas.create_line(CANVAS_WIDTH/2,CANVAS_HEIGHT/2,(CANVAS_WIDTH/5 * getSteeringVelocity(listXY)) + CANVAS_WIDTH/2,CANVAS_HEIGHT/3, fill="yellow")

    #draw walls
    if(wallTop):
        canvas.create_rectangle(50, 50, CANVAS_WIDTH-50, 55, fill="black")
    if(wallLeft):
        canvas.create_rectangle(50, 50, 55, CANVAS_HEIGHT-50, fill="black")
    if(wallBottom):
        canvas.create_rectangle(50, CANVAS_WIDTH-50, CANVAS_WIDTH-50, CANVAS_HEIGHT-45, fill="black")
    if(wallRight):
        canvas.create_rectangle(CANVAS_WIDTH-50, 50, CANVAS_WIDTH-55, CANVAS_HEIGHT-50, fill="black")

def tranlateToCenterScreen(x, isHeight=False):
    if isHeight:
        return (x * -POINT_SCALE_FACTOR) + (CANVAS_HEIGHT/2)
    else:
        return (x * -POINT_SCALE_FACTOR) + (CANVAS_WIDTH/2)

def drawPoint(x,y, color="white"):
    x1, y1 = (x - 1), (y - 1)
    x2, y2 = (x + 1), (y + 1)
    canvas.create_oval(x1, y1, x2, y2, fill=color)

def resetCanvas():
    canvas.delete("all")
    canvas.create_line(CANVAS_WIDTH/2,0,CANVAS_WIDTH/2,CANVAS_HEIGHT, fill="red")
    canvas.create_line(0,CANVAS_HEIGHT/2,CANVAS_WIDTH,CANVAS_HEIGHT/2, fill="red")


#calculates the steering velocity
def getSteeringVelocity(listXY, debug=False):
    steering = ERROR_CODE
    
    #check right site to align the robot
    if wallRight:
        #check which diff is smaller
        diffToRightTop = listXY[270 + 45][0] - listXY[270][0]
        diffToLeftBottom = listXY[270 - 45][0] - listXY[270][0]
        #check for infinite and not a number
        if(math.isnan(diffToRightTop) or math.isinf(diffToRightTop)):
            diffToRightTop = -ERROR_CODE
        if(math.isnan(diffToLeftBottom) or math.isinf(diffToLeftBottom)):
            diffToLeftBottom = -ERROR_CODE
        if(abs(diffToRightTop) < abs(diffToLeftBottom)):
            #use the rightTop diff
            if(abs(diffToRightTop) > 0.04):
                if(diffToRightTop > 0):
                    steering = ALIGN_STEERING
                else:
                    steering = -ALIGN_STEERING
            else:
                steering = 0.0
        else:
            if(diffToLeftBottom == -ERROR_CODE and diffToRightTop == -ERROR_CODE):
                print("Error in steering calculation! Drive straight...")
                return 0.0

            #use the rightBottom diff
            if(abs(diffToLeftBottom) > 0.04):
                if(diffToLeftBottom > 0):
                    steering =  -ALIGN_STEERING
                else:
                    steering = ALIGN_STEERING
            else:
                steering = 0.0

    #try to center the robot beetween the walls (align = 0)
    if(wallRight and wallLeft and steering == 0.0):
        if(abs(abs(listXY[90][0]) - abs(listXY[270][0])) > 0.2):
            if(abs(listXY[90][0]) > abs(listXY[270][0])):
                #right wall is nearer -> drive slightly left
                steering = CENTER_STEERING
            else:
                #left wall is nearer -> drive slightly right
                steering = -CENTER_STEERING

    if(debug):
        if(steering == 0.0):
            print("drive straight")
        else:
            if(steering > 0):
                if(abs(steering) == 0.1):
                    print("steering slightly left")
                else:
                    print("steering left")
            else:
                if(abs(steering) == 0.1):
                    print("steering slightly right")
                else:
                    print("steering right")
    return steering


def drive(listXY):
    global IS_PERFORMING_TURN_RIGHT, IS_PERFORMING_TURN_LEFT
    global counter

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
        print("win!")
        IS_PERFORMING_TURN_RIGHT = False
        IS_PERFORMING_TURN_LEFT = False
        velPublisher.publish(velMessage)
        return

    if(counter >= TURN_ROUTINE_LENGTH):
        IS_PERFORMING_TURN_RIGHT = False
        IS_PERFORMING_TURN_LEFT = False

    if(not IS_PERFORMING_TURN_RIGHT and not IS_PERFORMING_TURN_LEFT):
        #drive logic
        if(not wallTop):
            velMessage.linear.x = DEFAULT_LINEAR_VELOCITY
        if(wallRight):
            counter = 0
            velMessage.angular.z = getSteeringVelocity(listXY, True)
        else:
            #start turn right routine
            IS_PERFORMING_TURN_RIGHT = True
            counter = 0
        if(wallTop and not wallLeft):
            #start turn left routine
            IS_PERFORMING_TURN_LEFT = True
            counter = 0
    if IS_PERFORMING_TURN_RIGHT:
        #performing right turn (first turn right and then drive foreward)
        if(counter < TURN_ROUTING_1st_PART_LENGTH):
            velMessage.angular.z = -TURN_ROUTINE_TURN_VELOCITY
        else:
            velMessage.linear.x = DEFAULT_LINEAR_VELOCITY
        counter = counter +1
        print("Performing right Turn - "+ str(counter))

    if IS_PERFORMING_TURN_LEFT:
        #performing left turn (first turn left and then drive foreward)
        if(counter < TURN_ROUTING_1st_PART_LENGTH):
            velMessage.angular.z = TURN_ROUTINE_TURN_VELOCITY
        else:
            velMessage.linear.x = DEFAULT_LINEAR_VELOCITY
        counter = counter +1
        print("Performing left Turn - "+ str(counter))

    velPublisher.publish(velMessage)

#performas a linear regression on the given points. straightline equation: f(x) = a + b * x
#this function returns a tuple [0] = a, [1] = b
#Note: the regression is not used in this implementation of the maze solver
def linearRegression(listXY):
    #calculate average
    averageX=0
    averageY=0
    sum1=0
    sum2=0
    for XY in listXY:
        averageX=averageX+XY[0]
        averageY=averageY+XY[1]
    averageX = averageX / len(listXY)
    averageY = averageY / len(listXY)


    #calculate linear regession
    #b = sum((Xi-averageX)*(Yi-averageY)) / sum(Xi-averageX)*sum(Xi-averageX)
    #a = averageY-b*averageX

    #calculate sums
    for XY in listXY:
        sum1 = sum1 + ((XY[0]-averageX)*(XY[1]-averageY))
        sum2 = sum2 + ((XY[0]-averageX)*(XY[0]-averageX))

    #calculate a and b
    b = sum1 / sum2
    a = averageY - b * averageX

    return (a,b)

#performas a polynom regression on the given points. equation: f(x) = return[0] * x ^degree + return[degree+1 - n] * x ^degree-n + ... + return[n]
#this function returns a list of coefficients starting with the highest degree
#Note: the regression is not used in this implementation of the maze solver
def polynomRegression(listXY, degree = 2):
    global regessionRightAB
    X = []
    Y = []
    for XY in listXY:
        X.append(XY[0])
        Y.append(XY[1])
    
    return np.polyfit(X,Y,degree)


def isWallNearby(ranges):
    #split the data
    subLeft = ranges[90-LOOKUP_RANGE:90+LOOKUP_RANGE]
    subBottom = ranges[180-LOOKUP_RANGE:180+LOOKUP_RANGE]
    subRight = ranges[270-LOOKUP_RANGE:270+LOOKUP_RANGE]
    subTop = ranges[0:LOOKUP_RANGE_TOP] + ranges[360-LOOKUP_RANGE_TOP:359]

    global wallBottom, wallLeft, wallRight, wallTop

    #find min
    minBottom=min(subBottom)
    minLeft=min(subLeft)
    minRight=min(subRight)
    minTop=min(subTop)

    #save the walls globally
    if(minTop <= RANGE_UNTIL_DETECT_TOP):
        wallTop=True
    else:
        wallTop=False
    if(minLeft <= RANGE_UNTIL_DETECT_LEFT):
        wallLeft=True
    else:
        wallLeft=False
    if(minBottom <= RANGE_UNTIL_DETECT_BOTTOM):
        wallBottom=True
    else:
        wallBottom=False
    if(minRight <= RANGE_UNTIL_DETECT_RIGHT):
        wallRight=True
    else:
        wallRight=False

def init():
    print("Init wall detector")
    rospy.init_node('wall_detector', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback)

    #start window
    canvas.pack()    
    top.mainloop()

    #stop robot at end
    velMessage = Twist()
    velMessage.linear.x = 0
    velMessage.linear.y = 0
    velMessage.linear.z = 0
    velMessage.angular.x = 0
    velMessage.angular.y = 0
    velMessage.angular.z = 0
    velPublisher.publish(velMessage)
    


if __name__ == '__main__':
    init()