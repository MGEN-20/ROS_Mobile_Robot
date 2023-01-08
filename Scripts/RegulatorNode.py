#!/usr/bin/python3
import rospy
from std_msgs.msg import Int32 
from robot.msg import Speed
from robot.msg import Coords

pub = rospy.Publisher('robot_speed', Speed, queue_size=10)

def MotorRegulator(x, y, z):

    xr = 0
    xl = 0
    
    if x > 0 and x < 350:
        xr = 60
        xl = -60
    elif x > -350:
        xl = 60
        xr = -60
    else:
        xl = 0
        xr = 0

    speed = (xr,xl)

    return speed


def OnShutdown():

    speed = Speed()

    speed.L = 0
    speed.R = 0

    pub.publish(speed)


def chatter_callback(message):

    rate = rospy.Rate(1)

    speed=Speed()

    velocity = MotorRegulator(message.X, message.Y, message.Z)

    speed.L = velocity[0]

    speed.R = velocity[1]

    while not rospy.is_shutdown():
        rospy.loginfo(speed.L)
        rospy.loginfo(speed.R)
        pub.publish(speed)
        rate.sleep()
        rospy.on_shutdown(OnShutdown) 

def listener():

    rospy.init_node('Regulator_listener', anonymous=True)

    rospy.Subscriber('robot_coords', Coords, chatter_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()