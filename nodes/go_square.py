#!/usr/bin/env python
import rospy
import threading
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Float64
import numpy as np

class x_y_z_check():
    def __init__(self):
        rospy.init_node("x_y_z_check")

        self.position_x_pub = rospy.Publisher("/bluerov/position_x", Float64, queue_size=1)
        self.position_y_pub = rospy.Publisher("/bluerov/position_y", Float64, queue_size=1)
        self.position_z_pub = rospy.Publisher("/bluerov/position_z", Float64, queue_size=1)
        self.desire_x_pub = rospy.Publisher("/bluerov/desire_x", Float64, queue_size=1)
        self.desire_y_pub = rospy.Publisher("/bluerov/desire_y", Float64, queue_size=1)
        self.desire_z_pub = rospy.Publisher("/bluerov/depth_desire", Float64, queue_size=1)
        self.desire_x_y_z_pub = rospy.Publisher("/desire_x_y_z", Pose, queue_size=1)

        self.data_lock = threading.RLock()

        # Define desired poses
        self.desired_poses_xy = [(0, 0), (1, 0), (1, 1), (0, 1)]  # Square pattern
        self.desired_depths = [-0.6, -1.3]  #  depths
        self.current_pose_index = 0
        self.current_depth_index = 0

        self.current_x = 0
        self.current_y = 0
        self.current_z = 0

        rospy.Subscriber("/dscap_sys", PoseStamped, self.localisation_callback)
        rospy.Subscriber("/bluerov/mavros/global_position/rel_alt", Float64, self.depth_callback)

    def localisation_callback(self, msg):
        with self.data_lock:
            self.current_x = msg.pose.position.x
            self.current_y = msg.pose.position.y

    def depth_callback(self, msg):
        with self.data_lock:
            self.current_z = msg.data

    def check_x_y_distance(self):
        with self.data_lock:
            target_x, target_y = self.desired_poses_xy[self.current_pose_index]
            distance = np.sqrt((self.current_x - target_x) ** 2 + (self.current_y - target_y) ** 2)
            if distance < 0.1:  # Threshold distance to consider as reached
                self.current_pose_index = (self.current_pose_index + 1) % len(self.desired_poses_xy)

            self.desire_x_pub.publish(Float64(self.desired_poses_xy[self.current_pose_index][0]))
            self.desire_y_pub.publish(Float64(self.desired_poses_xy[self.current_pose_index][1]))

    def check_z_distance(self):
        with self.data_lock:
            target_z = self.desired_depths[self.current_depth_index]
            if abs(self.current_z - target_z) < 0.1:  # Threshold depth to consider as reached
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
