#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist


def vis2cmd_callback(msg):
   
    cmd_vel.linear.z = msg.linear.z
    cmd_vel.linear.y = msg.linear.z
    cmd_vel.linear.x = -msg.linear.y
    cmd_vel.angular.y = msg.angular.y 
    cmd_vel.angular.z = -msg.angular.x
    cmd_vel.angular.x = msg.angular.x

    pub.publish(cmd_vel)

def vis2cmd():
    global cmd_vel

    global pub

    cmd_vel = Twist()


    rospy.Subscriber("/mallard/twist", Twist, vis2cmd_callback)
    pub = rospy.Publisher("/mallard/cmd_vel", Twist, queue_size=2)

    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('vis2cmd')
    vis2cmd()
