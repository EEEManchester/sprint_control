#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from tf import transformations
from std_msgs.msg import Float64

prev_z = None
prev_time = None
depth = 0
current_velocity = 0
des_depth = -1

def depth_callback(msg):
    global prev_z, prev_time, depth, current_velocity
    current_time = rospy.Time.now().to_sec()

    if msg:
        depth = msg.data
        if prev_z is not None and prev_time is not None:
            dt = current_time - prev_time
            current_velocity = (msg.data - prev_z) / dt

        prev_z = msg.data
        prev_time = current_time

    else:
        depth = 0

    # Publish the message each time a new one is received
    publish_message()

def desire_callback(msg):
    global des_depth
    if msg:
        des_depth = msg.data
    else:
        des_depth = -1

    # Publish the message each time a new one is received
    publish_message()

def publish_message():
    global depth, current_velocity, des_depth
    cmd_vel = Twist()

    # cmd_vel.linear.z = (des_depth - depth)*0.5 - current_velocity
    cmd_vel.linear.z = -(des_depth - depth)*0.5

    # cmd_vel.linear.z =0
    if abs(cmd_vel.linear.z) > 0.5:
        # cmd_vel.linear.z=0.8
        cmd_vel.linear.z = 0.5 if cmd_vel.linear.z > 0 else -0.5
    cmd_vel.linear.y = 0
    cmd_vel.linear.x = 0

    cmd_vel.angular.y = 0
    cmd_vel.angular.z = 0
    cmd_vel.angular.x = 0

    pub.publish(cmd_vel)

def joy2cmd():
    global pub

    rospy.init_node('depth2cmd')
    pub = rospy.Publisher("twist_depth", Twist, queue_size=1)
    rospy.Subscriber("/bluerov/mavros/global_position/rel_alt", Float64, depth_callback)
    rospy.Subscriber("/bluerov/depth_desire", Float64, desire_callback)

    rospy.spin()

if __name__ == '__main__':
    joy2cmd()


# #!/usr/bin/env python
# import rospy
# from sensor_msgs.msg import Joy
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import Imu
# from tf import transformations
# from std_msgs.msg import Float64


# global cmd_vel
# global des_depth
# global pub
# global prev_z 
# global prev_time
# global depth
# global current_velocity
# prev_z = None
# prev_time = None
# depth = 0
# current_velocity = 0
# des_depth = -1


# def depth_callback(msg):
#     global prev_z, prev_time, depth, current_velocity
#     current_time = rospy.Time.now().to_sec()

#     if msg:
#         depth = msg.data
#         if prev_z is not None and prev_time is not None:
#             dt = current_time - prev_time
#             current_velocity = (msg.data - prev_z) / dt

#         prev_z = msg.data
#         prev_time = current_time

#     else:
#         depth = 0
#     return depth, current_velocity

# def desire_callback(msg):
#     global des_depth
#     if msg:
#         des_depth = msg.data
#     else:
#         des_depth = -1
#     return des_depth

# def joy2cmd():
#     global cmd_vel
#     global des_depth
#     global pub
#     global prev_z 
#     global prev_time
#     global depth
#     global current_velocity
#     cmd_vel = Twist()

#     rospy.Subscriber("/bluerov/mavros/global_position/rel_alt", Float64, depth_callback)
#     rospy.Subscriber("/bluerov/depth_desire", Float64, desire_callback)
#     # rospy.Subscriber("/bluerov/twist", Twist, joy2cmd_callback)

#     cmd_vel.linear.z = (des_depth - depth)*0.5 -current_velocity
#     if cmd_vel.linear.z > 0.8:
#         cmd_vel.linear.z=0.8
#     cmd_vel.linear.y = 0
#     cmd_vel.linear.x = 0

#     cmd_vel.angular.y = 0
#     cmd_vel.angular.z = 0
#     cmd_vel.angular.x = 0

#     pub = rospy.Publisher("twist_depth", Twist, queue_size=2)
#     pub.publish(cmd_vel)
#     rospy.spin()

# if __name__ == '__main__':
#     rospy.init_node('depth2cmd')
#     joy2cmd()
