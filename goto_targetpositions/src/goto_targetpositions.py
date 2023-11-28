#!/usr/bin/env python

# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64

class PwmController:
    def __init__(self):
        # Subscribers
        self.drive_angle_sub = rospy.Subscriber('drive_angle', Float64, self.drive_angle_callback)
        self.steer_angle_sub = rospy.Subscriber('steer_angle', Float64, self.steer_angle_callback)

        # Publishers
        self.drive_pwm_pub = rospy.Publisher('drive_pwm', Float64, queue_size=10)
        self.steer_pwm_pub = rospy.Publisher('steer_pwm', Float64, queue_size=10)

    def drive_angle_callback(self, msg):
        # Convert drive angle to PWM (this is just a placeholder, you should replace this with your own logic)
        drive_pwm = Float64()
        drive_pwm.data = 1500 + msg.data * 100  # Neutral + drive angle
        self.drive_pwm_pub.publish(drive_pwm)

    def steer_angle_callback(self, msg):
        # Convert steering angle to PWM (this is just a placeholder, you should replace this with your own logic)
        steer_pwm = Float64()
        steer_pwm.data = 1500 + msg.data * 100  # Neutral + steer angle
        self.steer_pwm_pub.publish(steer_pwm)

if __name__ == "__main__":
    rospy.init_node('pwm_controller')
    pc = PwmController()
    rospy.spin()