#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import UInt16MultiArray
from sensor_msgs.msg import NavSatFix
from rtcm_msgs.msg import Message
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from nav_msgs.msg import Odometry
import numpy as np

class SensorIntegrator:
    def __init__(self):
        self.pub = rospy.Publisher('control_key', UInt16MultiArray, queue_size=10)
        self.odom_pub = rospy.Publisher('/Sikick/odom', Odometry, queue_size=50)

        self.x_hat = np.array([0.0, 0.0, 0.0])
        self.P = np.eye(3)
        self.Q = np.eye(3) * 0.01
        self.R = np.eye(3) * 0.1

        self.rtk_signal_strong = True
        self.current_x = 0.0
        self.current_y = 0.0
        self.source = "Unknown"
        self.utm_x = 0.0
        self.utm_y = 0.0

    def callback1(self, msg):
        if not self.rtk_signal_strong:
            signal = 1

    def callback2(self, msg):
        self.rtk_signal_strong = True

    def callback3(self, msg):
        position_covariance = msg.position_covariance_type
        threshold_value = 1

        if position_covariance > threshold_value:
            self.rtk_signal_strong = False
            self.source = "Bad signal RTK"
        else:
            self.rtk_signal_strong = True
            self.source = "Active RTK"

    def callback4(self, msg):
        signal = 2
        self.utm_x = msg.pose.position.x
        self.utm_y = msg.pose.position.y

    def callback5(self, msg):
        if not self.rtk_signal_strong:
            dt = 1.0
            A = np.array([
                [1, 0, -self.x_hat[2] * dt],
                [0, 1, self.x_hat[2] * dt],
                [0, 0, 1]
            ])

            B = np.array([
                [dt * math.cos(self.x_hat[2]), 0],
                [dt * math.sin(self.x_hat[2]), 0],
                [0, dt]
            ])

            utm_x = msg.pose.position.x
            utm_y = msg.pose.position.y
            z = np.array([utm_x, utm_y, 0.0])  # Measurement vector

            # Compute the Kalman gain
            K = np.dot(np.dot(self.P, A.T), np.linalg.inv(np.dot(np.dot(A, self.P), A.T) + self.Q))

            # Update the estimate via z
            self.x_hat = np.dot(A, self.x_hat) + np.dot(K, (z - np.dot(A, self.x_hat)))

            # Update the error covariance
            self.P = np.dot((np.eye(3) - np.dot(K, A)), self.P)

            # Set the current position and source based on the fused estimate
            self.current_x = self.x_hat[0]
            self.current_y = self.x_hat[1]
            self.source = "Sensor Fusion (UTM)"

    def callback6(self, msg):
        if self.rtk_signal_strong:
            self.current_x = msg.pose.position.x
            self.current_y = msg.pose.position.y
            self.source = "GPS UTM (RTK applied)"
            # rospy.loginfo("Position Source: {self.source}, x={self.current_x}, y={self.current_y}")

    def goal_positions_callback(self, msg):
        self.goal_positions = [(pose.position.x, pose.position.y) for pose in msg.poses]

    def Current_Odom_publisher(self, event):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.current_x
        odom.pose.pose.position.y = self.current_y

        self.odom_pub.publish(odom)
        rospy.loginfo("Position Source: {}, x={}, y={}".format(self.source, self.current_x, self.current_y))

    def sensor_integrate(self):
        rospy.init_node('sensor_integrate', anonymous=True)
        rospy.Subscriber("/zed2/zed_node/odom", Odometry, self.callback1)
        rospy.Subscriber("/ublox_gps/rtcm", Message, self.callback2)
        rospy.Subscriber("/ublox_gps/fix", NavSatFix, self.callback3)
        rospy.Subscriber("/utm", PoseStamped, self.callback4)

        rospy.Subscriber("/utm", PoseStamped, self.callback5)

        rospy.Subscriber("/utm", PoseStamped, self.callback6)

        rospy.Subscriber("/goal_positions", Pose, self.goal_positions_callback)

        rospy.Timer(rospy.Duration(1.0), self.Current_Odom_publisher)  

if __name__ == '__main__':
    try:
        integrator = SensorIntegrator()
        integrator.sensor_integrate()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
