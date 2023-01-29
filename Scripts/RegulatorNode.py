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

ThresholdGain = rospy.get_param("ThresholdGain")
DistanceGain = rospy.get_param("DistanceGain")
SteeringSpeed = rospy.get_param("SteeringSpeed")

def PublishSpeed():

    global XCoordVector, ZCoordVector

    meanX = np.mean(np.array(XCoordVector))
    meanZ = np.mean(np.array(ZCoordVector))

    XCoordVector = []
    ZCoordVector = []

    # print(f'Mean: {mean}')

    speeds = MotorRegulator(x=meanX, z=meanZ)

    speed.L = speeds[1]

    speed.R = speeds[0]

    rospy.loginfo(speed.L)
    rospy.loginfo(speed.R)
    pub.publish(speed)


def MotorRegulator(x, y=0, z=0):

    RightVelocity = 0
    LeftVelocity = 0

    neg_threshold = -int(ThresholdGain * z)
    pos_threshold = int(ThresholdGain * z)

    rospy.loginfo(x)

    if x > pos_threshold and x != 0:

        RightVelocity = -SteeringSpeed
        LeftVelocity = SteeringSpeed

    elif x < neg_threshold and x != 0:
        RightVelocity = SteeringSpeed
        LeftVelocity = -SteeringSpeed
    
    elif x == 0:
        RightVelocity = 0
        LeftVelocity = 0

    else:
        RightVelocity = int(DistanceGain * z)
        LeftVelocity = int(DistanceGain * z)

    speed = (RightVelocity,LeftVelocity)

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