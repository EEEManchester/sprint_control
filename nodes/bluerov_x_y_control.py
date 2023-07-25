#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import threading


class x_y_control():
    def __init__(self):
        rospy.init_node("x_y_control")

        self.twist_x_pub = rospy.Publisher("/bluerov/twist_setpoint_x",
                                            Twist,
                                            queue_size=1)
        self.twist_y_pub = rospy.Publisher("/bluerov/twist_setpoint_y",
                                            Twist,
                                            queue_size=1)        
        self.data_lock = threading.RLock()
        self.position_x= None
        self.position_y = None
        self.desire_x= None
        self.desire_y= None
        self.data_lock = threading.RLock()
        self.twist_x = Twist()
        self.twist_y = Twist()


        self.position_x_sub = rospy.Subscriber("/bluerov/position_x", Float64, self.x_callback)
        self.desire_x_sub = rospy.Subscriber("/bluerov/desire_x", Float64, self.desire_x_callback)
        self.position_y_sub = rospy.Subscriber("/bluerov/position_x", Float64, self.y_callback)
        self.desire_y_sub = rospy.Subscriber("/bluerov/desire_x", Float64, self.desire_y_callback)

    def x_callback(self, msg):
        with self.data_lock:
            if msg:
                self.position_x = msg.data

    def desire_x_callback(self, msg):
        with self.data_lock:
            if msg:
                self.desire_x = msg.data  

    def y_callback(self, msg):
        with self.data_lock:
            if msg:
                self.position_y = msg.data

    def desire_y_callback(self, msg):
        with self.data_lock:
            if msg:
                self.desire_y = msg.data

    def twist_x_cal(self,x,desire_x):
        self.twist_x.linear.x = (desire_x - x)

        if abs(self.twist_x.linear.x) > 0.2:
            self.twist_x.linear.x= 0.2 if self.twist_x.linear.x > 0 else -0.2

        self.twist_x.linear.y = 0
        self.twist_x.linear.z = 0
        self.twist_x.angular.y = 0
        self.twist_x.angular.z = 0
        self.twist_x.angular.x = 0
            
    def twist_y_cal(self,y,desire_y):
        self.twist_y.linear.y = (desire_y - y)

        if abs(self.twist_y.linear.y) > 0.2:
            self.twist_y.linear.y= 0.2 if self.twist_y.linear.y > 0 else -0.2

        self.twist_y.linear.x = 0
        self.twist_y.linear.z = 0
        self.twist_y.angular.y = 0
        self.twist_y.angular.z = 0
        self.twist_y.angular.x = 0

    def run(self):
        rate = rospy.Rate(50) # 50 Hz
        while not rospy.is_shutdown():
            self.twist_x_cal(self.position_x, self.desire_x)
            self.twist_y_cal(self.position_y, self.desire_y)
            self.twist_x_pub.publish(self.twist_x)
            self.twist_y_pub.publish(self.twist_y)

            rate.sleep()
        

    def main():
        node = x_y_control()
        node.run()
            
    if __name__ == "__main__":
        main()
