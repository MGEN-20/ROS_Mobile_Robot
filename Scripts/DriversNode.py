#!/usr/bin/python3
import rospy
from a_star import AStar
from robot.msg import Speed

a_star = AStar()

def chatter_callback(message):
    print(f'L speed {message.L}')
    print(f'R speed {message.R}')
    a_star.motors(message.L,message.R)

def listener():


    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('robot_speed', Speed, chatter_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()