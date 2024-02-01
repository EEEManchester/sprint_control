#!/usr/bin/env python
import rospy
import threading
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Float64
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from scipy.spatial.transform import Rotation as R
import math

class x_y_z_check():
    def __init__(self):
        rospy.init_node("x_y_z_check")

        self.position_x_pub = rospy.Publisher("/bluerov/position_x", Float64, queue_size=1)
        self.position_y_pub = rospy.Publisher("/bluerov/position_y", Float64, queue_size=1)
        self.position_z_pub = rospy.Publisher("/bluerov/position_z", Float64, queue_size=1)
        self.desire_x_pub = rospy.Publisher("/bluerov/desire_x", Float64, queue_size=1)
        self.desire_y_pub = rospy.Publisher("/bluerov/desire_y", Float64, queue_size=1)
        self.desire_z_pub = rospy.Publisher("/bluerov/depth_desire", Float64, queue_size=1)
        self.yaw_pub = rospy.Publisher("/yaw", Float64, queue_size=1)
        self.desire_x_y_z_pub = rospy.Publisher("/desire_x_y_z", Pose, queue_size=1)

        self.data_lock = threading.RLock()

        # Define desired poses
        # self.desired_poses_xy = [(-0.3, 0.5), (-0.3, -0.5), (0.7, -0.5), (0.7, 0.5)]  # Square pattern
        # self.desired_poses_xy = [(-0.3, 0.4), (0.7, 0.4), (0.2, -0.6), (-0.3, 0.4), (0.7, 0.4), (0.2, 0.6)]  # 8 pattern
        self.desired_poses_xy = [(-0.2, 0.7), (-0.2, -0.7), (0.4, -0.7), (0.4, 0.7), (1, 0.7), (1, -0.7)]  # lawn-mover pattern

        # self.desired_depths = [-0.6, -1.4]  #  depths
        self.desired_depths = [-1.8]  #  depths
        self.current_pose_index = 0
        self.current_depth_index = 0

        self.current_x = 0
        self.current_y = 0
        self.current_yaw = 0
        self.current_z = 0

        rospy.Subscriber("/qualisys/bluerov_8_points/pose", PoseStamped, self.localisation_callback)
        # rospy.Subscriber("/dscap_sys", PoseStamped, self.localisation_callback)
        rospy.Subscriber("/bluerov/mavros/global_position/rel_alt", Float64, self.depth_callback)

    def localisation_callback(self, msg):
        with self.data_lock:
            self.current_x = msg.pose.position.x
            self.current_y = msg.pose.position.y
            self.current_yaw = msg.pose.orientation

            q_w2t = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
            # print(q_w2t)
            q = R.from_quat([q_w2t[0], q_w2t[1], q_w2t[2], q_w2t[3]])
            eul_w2t = q.as_euler('zyx', degrees=True)
            yaw_w2t_radians = math.radians(eul_w2t[0])

            # orientation_list = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
            # self.yaw = euler_from_quaternion (orientation_list)[2]
            self.yaw_pub.publish(Float64(yaw_w2t_radians))

    def depth_callback(self, msg):
        with self.data_lock:
            self.current_z = msg.data

    def check_x_y_distance(self):
        with self.data_lock:
            target_x, target_y = self.desired_poses_xy[self.current_pose_index]
            distance = np.sqrt((self.current_x - target_x) ** 2 + (self.current_y - target_y) ** 2)
            if distance < 0.1:  # Threshold distance to consider as reached
                self.current_pose_index = (self.current_pose_index + 1) % len(self.desired_poses_xy)

            self.position_x_pub.publish(Float64(self.current_x))
            self.position_y_pub.publish(Float64(self.current_y))
            self.desire_x_pub.publish(Float64(self.desired_poses_xy[self.current_pose_index][0]))
            self.desire_y_pub.publish(Float64(self.desired_poses_xy[self.current_pose_index][1]))
            # self.yaw_pub.publish(Float64(self.current_yaw))

    def check_z_distance(self):
        with self.data_lock:
            target_z = self.desired_depths[self.current_depth_index]
            if abs(self.current_z - target_z) < 0.2:  # Threshold depth to consider as reached
                self.current_depth_index = (self.current_depth_index + 1) % len(self.desired_depths)

            self.desire_z_pub.publish(Float64(self.desired_depths[self.current_depth_index]))

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.check_x_y_distance()
            self.check_z_distance()
            rate.sleep()

def main():
    node = x_y_z_check()
    node.run()

if __name__ == "__main__":
    main()
