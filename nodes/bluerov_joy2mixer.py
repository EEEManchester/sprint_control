#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


def joy2cmd_callback(msg):

    # cmd_vel.linear.z = 0.8*msg.axes[4]
    cmd_vel.linear.y = -0.8*msg.axes[0]
    cmd_vel.linear.x = 0.8*msg.axes[1]

    # cmd_vel.angular.y = 0.8*msg.axes[7]
    cmd_vel.angular.z = -0.8*msg.axes[3]
    # cmd_vel.angular.x = 0.8*msg.axes[6]

    
    pub.publish(cmd_vel)

def joy2mix():
    global cmd_vel

    global pub
    global eu_angles
    cmd_vel = Twist()

    rospy.Subscriber("/bluerov/joy_bluerov", Joy, joy2cmd_callback)
    # rospy.Subscriber("/bluerov/twist", Twist, joy2cmd_callback)
    pub = rospy.Publisher("cmd_vel1", Twist, queue_size=2)

    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('Joy2mix')
    joy2mix()
