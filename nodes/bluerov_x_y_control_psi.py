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
        self.last_desire_x = None
        self.last_desire_y = None
        self.new_target_received_time_x = None
        self.new_target_received_time_y = None
        self.RAMP_DURATION = 2.0  # 2 seconds for ramping

        self.data_lock = threading.RLock()
        self.position_x_sub = rospy.Subscriber("/bluerov/position_x", Float64, self.x_callback)
        self.desire_x_sub = rospy.Subscriber("/bluerov/desire_x", Float64, self.desire_x_callback)
        self.position_y_sub = rospy.Subscriber("/bluerov/position_y", Float64, self.y_callback)
        self.desire_y_sub = rospy.Subscriber("/bluerov/desire_y", Float64, self.desire_y_callback)
        self.yaw_sub = rospy.Subscriber("/yaw_w2t", Float64, self.yaw_callback)

    def yaw_callback(self, msg):
        with self.data_lock:
                if msg.data != 0:
                     
                    yaw = msg.data
                    self.vel_yaw = -1 + ((yaw - (-3.14)) * (1 - (-1)) / (3.14 - (-3.14)))
                    if abs(self.vel_yaw) > 0.5:
                        # If vel_yaw is positive, set it to 0.5; if it's negative, set it to -0.5
                        self.vel_yaw = 0.5 if self.vel_yaw > 0 else -0.5
                else:
                     self.vel_yaw = 0
    def desire_x_callback(self, msg):
        with self.data_lock:
            if self.last_desire_x != msg.data:
                self.new_target_received_time_x = rospy.get_time()
            self.last_desire_x = msg.data

    def desire_y_callback(self, msg):
        with self.data_lock:
            if self.last_desire_y != msg.data:
                self.new_target_received_time_y = rospy.get_time()
            self.last_desire_y = msg.data

              

    def desire_x_callback(self, msg):
        with self.data_lock:
                self.desire_x = msg.data  

    def desire_y_callback(self, msg):
        with self.data_lock:
 
                self.desire_y = msg.data

    def twist_x_cal(self, x, desire_x):
        error_x = desire_x - x
        desired_velocity_x = 0.8 * error_x

        # If a new target has been received recently
        if self.new_target_received_time_x is not None:
            elapsed_time = rospy.get_time() - self.new_target_received_time_x
            if elapsed_time < self.RAMP_DURATION:
                ramping_factor = elapsed_time / self.RAMP_DURATION
                desired_velocity_x *= ramping_factor
            else:
                self.new_target_received_time_x = None

        # Clamp to max velocity
        if abs(desired_velocity_x) > 0.15:
            desired_velocity_x = 0.15 if desired_velocity_x > 0 else -0.15

        return desired_velocity_x

    def twist_y_cal(self, y, desire_y):
        error_y = desire_y - y
        desired_velocity_y = -0.8 * error_y

        # If a new target has been received recently
        if self.new_target_received_time_y is not None:
            elapsed_time = rospy.get_time() - self.new_target_received_time_y
            if elapsed_time < self.RAMP_DURATION:
                ramping_factor = elapsed_time / self.RAMP_DURATION
                desired_velocity_y *= ramping_factor
            else:
                self.new_target_received_time_y = None

        # Clamp to max velocity
        if abs(desired_velocity_y) > 0.15:
            desired_velocity_y = 0.15 if desired_velocity_y > 0 else -0.15

        return desired_velocity_y



    def run(self):
        # rate = rospy.Rate(50) # 50 Hz
        while not rospy.is_shutdown():
            twist_x = Twist()
            twist_y = Twist()
            x=self.twist_x_cal(self.position_x, self.desire_x)
            y=self.twist_y_cal(self.position_y, self.desire_y)
            if self.position_x ==0:
                 
                twist_x.linear.x = 0
                twist_x.linear.y = 0
                twist_x.linear.z = 0
                twist_x.angular.y = 0
                twist_x.angular.z = 0 
                twist_x.angular.x = 0

                twist_y.linear.y = 0
                twist_y.linear.x = 0
                twist_y.linear.z = 0
                twist_y.angular.y = 0
                twist_y.angular.z = 0
                twist_y.angular.x = 0
            else:
                twist_x.linear.x = 0.8*x
                twist_x.linear.y = 0
                twist_x.linear.z = 0
                twist_x.angular.y = 0
                twist_x.angular.z = self.vel_yaw 
                twist_x.angular.x = 0

                twist_y.linear.y = 0.8*y
                twist_y.linear.x = 0
                twist_y.linear.z = 0
                twist_y.angular.y = 0
                twist_y.angular.z = 0
                twist_y.angular.x = 0

                self.twist_x_pub.publish(twist_x)
                self.twist_y_pub.publish(twist_y)

                # rate.sleep()
            

def main():
    node = x_y_control()
    node.run()
            
if __name__ == "__main__":
    main()
