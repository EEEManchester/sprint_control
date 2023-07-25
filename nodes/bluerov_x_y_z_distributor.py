#!/usr/bin/env python
import rospy
import threading
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Float64


class x_y_z_distributor():
    def __init__(self):
        rospy.init_node("x_y_distributor")

        self.position_x_pub = rospy.Publisher("/bluerov/position_x",
                                            Float64,
                                            queue_size=1)
        self.position_y_pub = rospy.Publisher("/bluerov/position_y",
                                            Float64,
                                            queue_size=1)
        self.position_z_pub = rospy.Publisher("/bluerov/position_z",
                                            Float64,
                                            queue_size=1)
        self.desire_x_pub = rospy.Publisher("/bluerov/desire_x",
                                            Float64,
                                            queue_size=1)
        self.desire_y_pub = rospy.Publisher("/bluerov/desire_y",
                                            Float64,
                                            queue_size=1)
        self.desire_z_pub = rospy.Publisher("/bluerov/depth_desire",
                                            Float64,
                                            queue_size=1)
        self.data_lock = threading.RLock()
        self.position_x= None
        self.position_y = None
        self.desire_x= None
        self.desire_y= None

        self.position_x_y_z_sub = rospy.Subscriber("/dscap_sys", PoseStamped, self.x_y_z_callback)
        self.desire_x_y_z_sub = rospy.Subscriber("/desire_x_y_z", Pose, self.desire_x_y_z_callback)

    def x_y_z_callback(self, msg):
        with self.data_lock:
            self.position_x = msg.pose.position.x
            self.position_y = msg.pose.position.y
            self.position_z = msg.pose.position.z

    def desire_x_y_z_callback(self, msg):
        with self.data_lock:
            self.desire_x = msg.pose.position.x
            self.desire_y = msg.pose.position.y
            self.desire_z = msg.pose.position.z
   
    def run(self):

        while not rospy.is_shutdown():
            if self.position_x:
                self.position_x_pub(self.position_x)
            if self.position_y:
                self.position_x_pub(self.position_y)

            if self.desire_x:
                self.desire_x_pub(self.desire_x)
            if self.desire_y:
                self.desire_x_pub(self.desire_y)
            if self.desire_z:
                self.desire_z_pub(self.desire_z)

def main():
    node = x_y_z_distributor()
    node.run()

if __name__ == "__main__":
    main()
