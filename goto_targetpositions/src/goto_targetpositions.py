#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
#from std_msgs.msg import Float64
from std_msgs.msg import UInt16MultiArray, Float64,Float64MultiArray

class PwmController:
    def __init__(self):
        # Subscribers
        self.local_drive_angle_sub = rospy.Subscriber('local_speed_vel', Float64, self.loc_drive_speed_callback)
        self.local_steer_angle_sub = rospy.Subscriber('local_steer_angle', Float64, self.loc_steer_angle_callback)

        self.global_drive_angle_sub = rospy.Subscriber('global_speed_vel', Float64, self.glo_drive_speed_callback)
        self.global_steer_angle_sub = rospy.Subscriber('global_steer_angle', Float64, self.glo_steer_angle_callback)

        # Publishers
        self.local_speed_vel_pwm_pub = rospy.Publisher('local_speed_vel_pwm', Float64MultiArray, queue_size=10)
        self.local_steer_angle_pwm_pub = rospy.Publisher('local_steer_angle_pwm', Float64MultiArray, queue_size=10)

        self.global_speed_vel_pwm_pub = rospy.Publisher('global_speed_vel_pwm', Float64MultiArray, queue_size=10)
        self.global_steer_angle_pwm_pub = rospy.Publisher('global_steer_angle_pwm', Float64MultiArray, queue_size=10)

        self.cmd_vel = Float64MultiArray()
        self.cmd_vel.data = [0, 0]

    def loc_drive_speed_callback(self, msg):
        # Convert drive angle to PWM (this is just a placeholder, you should replace this with your own logic)
        '''
        speed_vel = UInt16MultiArray()
        speed_vel.data = 1500 + msg.data * 100  # Neutral + drive angle
        self.speed_vel_pub.publish(speed_vel)
        '''
        local_speed_vel_pwm = msg.data * 100  + 80

        self.cmd_vel.data[0] = local_speed_vel_pwm
        self.speed_vel_pwm_pub.publish(self.cmd_vel)
        print('loc-Speed_vel_pwm',local_speed_vel_pwm)

    def loc_steer_angle_callback(self, msg):
        # Convert steering angle to PWM (this is just a placeholder, you should replace this with your own logic)
        '''
        steer_pwm = UInt16MultiArray()
        steer_pwm.data = 1500 + msg.data * 100  # Neutral + steer angle
        self.steer_pwm_pub.publish(steer_pwm)
        '''
        #steer_pwm = 1500 + msg.data * 100  # Neutral + steer angle
        local_steer_angle_pwm = msg.data*57.3 -70 # radian 2 degree & 67: angle offset
        self.cmd_vel.data[1] = local_steer_angle_pwm
        self.steer_angle_pwm_pub.publish(self.cmd_vel)
        print('loc-Steering_Angle_pwm', local_steer_angle_pwm)

    def glo_drive_speed_callback(self, msg):
        # Convert drive angle to PWM (this is just a placeholder, you should replace this with your own logic)
        '''
        speed_vel = UInt16MultiArray()
        speed_vel.data = 1500 + msg.data * 100  # Neutral + drive angle
        self.speed_vel_pub.publish(speed_vel)
        '''
        global_speed_vel_pwm = msg.data * 100  + 80

        self.cmd_vel.data[0] = global_speed_vel_pwm
        self.speed_vel_pwm_pub.publish(self.cmd_vel)
        print('glo-Speed_vel_pwm',global_speed_vel_pwm)

    def glo_steer_angle_callback(self, msg):
        # Convert steering angle to PWM (this is just a placeholder, you should replace this with your own logic)
        '''
        steer_pwm = UInt16MultiArray()
        steer_pwm.data = 1500 + msg.data * 100  # Neutral + steer angle
        self.steer_pwm_pub.publish(steer_pwm)
        '''
        #steer_pwm = 1500 + msg.data * 100  # Neutral + steer angle
        global_steer_angle_pwm = msg.data*57.3 -70 # radian 2 degree & 67: angle offset
        self.cmd_vel.data[1] = global_steer_angle_pwm
        self.steer_angle_pwm_pub.publish(self.cmd_vel)
        print('glo-Steering_Angle_pwm', global_steer_angle_pwm)
    
if __name__ == "__main__":
    rospy.init_node('pwm_controller')
    pc = PwmController()
    rospy.spin()
