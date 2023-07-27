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
        self.position_x= 0.0
        self.position_y = 0.0
        self.desire_x= 0.0
        self.desire_y= 0.0
        self.vel_yaw = 0
        self.data_lock = threading.RLock()
        self.position_x_sub = rospy.Subscriber("/bluerov/position_x", Float64, self.x_callback)
        self.desire_x_sub = rospy.Subscriber("/bluerov/desire_x", Float64, self.desire_x_callback)
        self.position_y_sub = rospy.Subscriber("/bluerov/position_y", Float64, self.y_callback)
        self.desire_y_sub = rospy.Subscriber("/bluerov/desire_y", Float64, self.desire_y_callback)
        self.yaw_sub = rospy.Subscriber("/yaw", Float64, self.yaw_callback)

    def yaw_callback(self, msg):
        with self.data_lock:
                yaw = msg.data
                self.vel_yaw = -1 + ((yaw - (-3.14)) * (1 - (-1)) / (3.14 - (-3.14)))
                if abs(self.vel_yaw) > 0.5:
                    # If vel_yaw is positive, set it to 0.5; if it's negative, set it to -0.5
                    self.vel_yaw = 0.5 if self.vel_yaw > 0 else -0.5

    def x_callback(self, msg):
        with self.data_lock:
                self.position_x = msg.data
                print(self.position_x)

    def desire_x_callback(self, msg):
        with self.data_lock:
                self.desire_x = msg.data  

    def y_callback(self, msg):
        with self.data_lock:

                self.position_y = msg.data

    def desire_y_callback(self, msg):
        with self.data_lock:
 
                self.desire_y = msg.data

    def twist_x_cal(self,x,desire_x):
            linear_x= 0.0
            if desire_x == 0:
                return linear_x
            else:
                linear_x = (desire_x - x)

                if abs(linear_x) > 0.15:
                    linear_x= 0.15 if linear_x > 0 else -0.15

                # self.twist_x.linear.y = 0
                # self.twist_x.linear.z = 0
                # self.twist_x.angular.y = 0
                # self.twist_x.angular.z = 0
                # self.twist_x.angular.x = 0

                return linear_x
            
    def twist_y_cal(self,y,desire_y):
            
            linear_y= 0.0
            if desire_y == 0:
                return linear_y
            else:
            
                linear_y = -(desire_y - y)

                if abs(linear_y) > 0.15:
                    linear_y= 0.15 if linear_y > 0 else -0.15

                # self.twist_y.linear.x = 0
                # self.twist_y.linear.z = 0
                # self.twist_y.angular.y = 0
                # self.twist_y.angular.z = 0
                # self.twist_y.angular.x = 0

                return linear_y


    def run(self):
        rate = rospy.Rate(50) # 50 Hz
        while not rospy.is_shutdown():
            twist_x = Twist()
            twist_y = Twist()
            x=self.twist_x_cal(self.position_x, self.desire_x)
            y=self.twist_y_cal(self.position_y, self.desire_y)
            twist_x.linear.x = x
            twist_x.linear.y = 0
            twist_x.linear.z = 0
            twist_x.angular.y = 0
            twist_x.angular.z = self.vel_yaw 
            twist_x.angular.x = 0

            twist_y.linear.y = y
            twist_y.linear.x = 0
            twist_y.linear.z = 0
            twist_y.angular.y = 0
            twist_y.angular.z = 0
            twist_y.angular.x = 0

            self.twist_x_pub.publish(twist_x)
            self.twist_y_pub.publish(twist_y)

            rate.sleep()
        

def main():
    node = x_y_control()
    node.run()
            
if __name__ == "__main__":
    main()
