#!/usr/bin/python3
import rospy
from std_msgs.msg import Int32
from robot.msg import Speed
from robot.msg import Coords
import numpy as np

pub = rospy.Publisher('robot_speed', Speed, queue_size=10)

speed = Speed()

XCoordVector = []

ZCoordVector = []

neg_threshold = rospy.get_param("neg_threshold")
pos_threshold = rospy.get_param("pos_threshold")
Dkp = rospy.get_param("Dkp")
SteeringSpeed = rospy.get_param("SteeringSpeed")


def PublishSpeed():
    global XCoordVector, ZCoordVector

    meanX = np.mean(np.array(XCoordVector))
    meanZ = np.mean(np.array(ZCoordVector))

    XCoordVector = []
    ZCoordVector = []

    speeds = MotorRegulator(x=meanX, z=meanZ)

    speed.L = speeds[1]

    speed.R = speeds[0]

    rospy.loginfo(speed.L)
    rospy.loginfo(speed.R)
    pub.publish(speed)


def MotorRegulator(x, y=0, z=0):
    xr = 0
    xl = 0

    rospy.loginfo(x)

    if x > pos_threshold and x != 0:

        xr = -SteeringSpeed
        xl = SteeringSpeed

    elif x < neg_threshold and x != 0:
        xr = SteeringSpeed
        xl = -SteeringSpeed

    elif x == 0:
        xr = 0
        xl = 0

    else:
        xr = int(Dkp * z)
        xl = int(Dkp * z)

    speed = (xr, xl)

    return speed


def OnShutdown():
    speed = Speed()

    speed.L = 0
    speed.R = 0

    pub.publish(speed)


def ChatterCallback(message):
    global ZCoord
    XCoordVector.append(message.X)
    ZCoordVector.append(message.Z)

    if len(XCoordVector) >= 4:
        PublishSpeed()

    rospy.on_shutdown(OnShutdown)


def Listener():
    rospy.init_node('Regulator', anonymous=True)

    rospy.Subscriber('robot_coords', Coords, ChatterCallback)

    rospy.spin()


if __name__ == '__main__':
    Listener()