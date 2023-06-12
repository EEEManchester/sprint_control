#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from tf import transformations
from std_msgs.msg import Float64




def depth_callback(msg):
    global depth
    current_time = rospy.Time.now().to_sec()

    if msg:
        depth = msg
        if prev_z is not None and prev_time is not None:
            dt = current_time - prev_time
            current_velocity = (msg - prev_z) / dt

        prev_z = msg
        prev_time = current_time

    else:
        depth = 0
    return depth, current_velocity

def desire_callback(msg):
    if msg:
        des_depth = msg
    else:
        des_depth = -1
    return des_depth

def joy2cmd():
    global cmd_vel
    global des_depth
    des_depth = -1
    global pub
    global prev_z 
    prev_z = None
    global prev_time
    prev_time = None

    cmd_vel = Twist()

    depth, current_velocity = rospy.Subscriber("/bluerov/mavros/global_position/rel_alt", Float64, depth_callback)
    rospy.Subscriber("/bluerov/depth_desire", Float64, desire_callback)
    # rospy.Subscriber("/bluerov/twist", Twist, joy2cmd_callback)

    cmd_vel.linear.z = (des_depth - depth)*0.5 -current_velocity
    if cmd_vel.linear.z > 0.8:
        cmd_vel.linear.z=0.8
    cmd_vel.linear.y = 0
    cmd_vel.linear.x = 0

    cmd_vel.angular.y = 0
    cmd_vel.angular.z = 0
    cmd_vel.angular.x = 0

    pub = rospy.Publisher("twist_depth", Twist, queue_size=2)
    pub.publish(cmd_vel)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('depth2cmd')
    joy2cmd()
