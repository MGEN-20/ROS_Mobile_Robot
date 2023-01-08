#!/usr/bin/python3
import rospy
from std_msgs.msg import Int32 
from robot.msg import Speed
from robot.msg import Coords
import numpy as np

pub = rospy.Publisher('robot_speed', Speed, queue_size=10)

speed = Speed()

XCoordVector = []


def PublishSpeed():

    global XCoordVector

    mean = np.mean(np.array(XCoordVector))

    XCoordVector = []

    print(f'Mean: {mean}')

    speeds = MotorRegulator(x=mean)

    speed.L = speeds[1]

    speed.R = speeds[0]

    rospy.loginfo(speed.L)
    rospy.loginfo(speed.R)
    pub.publish(speed)


def MotorRegulator(x, y=0, z=0):

    xr = 0
    xl = 0
    
    if x > 0 and x < 350:
        xr = 60
        xl = -60
    elif x > -350 and x < 0:
        xr = -60
        xl = 60
    elif x == 0:
        xl = 0
        xr = 0

    speed = (xr,xl)

    return speed


def OnShutdown():

    speed = Speed()

    speed.L = 0
    speed.R = 0

    pub.publish(speed)


def ChatterCallback(message):

    XCoordVector.append(message.X)

    if len(XCoordVector) >= 20:
        PublishSpeed()



def Listener():

    rospy.init_node('Regulator_listener', anonymous=True)

    rospy.Subscriber('robot_coords', Coords, ChatterCallback)

    rospy.spin()

    rospy.on_shutdown(OnShutdown) 

if __name__ == '__main__':
    Listener()