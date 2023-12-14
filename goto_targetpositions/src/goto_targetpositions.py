#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
#from std_msgs.msg import Float64
from std_msgs.msg import UInt16MultiArray, Float64

class PwmController:
    def __init__(self):
        # Subscribers
        self.drive_angle_sub = rospy.Subscriber('speed_pwm', Float64, self.drive_speed_callback)
        self.steer_angle_sub = rospy.Subscriber('steer_angle', Float64, self.steer_angle_callback)

        # Publishers
        self.drive_pwm_pub = rospy.Publisher('speed_drive_pwm', UInt16MultiArray, queue_size=10)
        self.steer_pwm_pub = rospy.Publisher('steer_angle_pwm', UInt16MultiArray, queue_size=10)

        self.cmd_vel = UInt16MultiArray()
        self.cmd_vel.data = [0, 0]

    def drive_speed_callback(self, msg):
        # Convert drive angle to PWM (this is just a placeholder, you should replace this with your own logic)
        '''
        drive_pwm = UInt16MultiArray()
        drive_pwm.data = 1500 + msg.data * 100  # Neutral + drive angle
        self.drive_pwm_pub.publish(drive_pwm)
        '''
        drive_pwm = msg.data * 100  + 80

        self.cmd_vel.data[0] = drive_pwm
        self.drive_pwm_pub.publish(self.cmd_vel)
        print('Speed_pwm',drive_pwm)

    def steer_angle_callback(self, msg):
        # Convert steering angle to PWM (this is just a placeholder, you should replace this with your own logic)
        '''
        steer_pwm = UInt16MultiArray()
        steer_pwm.data = 1500 + msg.data * 100  # Neutral + steer angle
        self.steer_pwm_pub.publish(steer_pwm)
        '''
        #steer_pwm = 1500 + msg.data * 100  # Neutral + steer angle
        steer_pwm = msg.data*57.3 # radian 2 degree
        self.cmd_vel.data[1] = steer_pwm
        self.steer_pwm_pub.publish(self.cmd_vel)
        print('SteeringAngle_pwm', steer_pwm)
    
if __name__ == "__main__":
    rospy.init_node('pwm_controller')
    pc = PwmController()
    rospy.spin()
