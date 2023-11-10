#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import UInt16MultiArray
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
from nav_msgs.msg import Odometry
import numpy as np

pub = rospy.Publisher('control_key', UInt16MultiArray, queue_size=10)
odom_pub = rospy.Publisher('/Sikick/odom', Odometry, queue_size=50)

# Initialize Kalman filter variables
x_hat = np.array([0.0, 0.0, 0.0])  # Initial state estimate [x, y, theta]
P = np.eye(3)  # Initial error covariance matrix
Q = np.eye(3) * 0.01  # Process noise covariance matrix (adjust as needed)
R = np.eye(3) * 0.1  # Measurement noise covariance matrix (adjust as needed)

rtk_signal_strong = True  # flag, tracking
current_x = 0.0
current_y = 0.0

def callback1(msg):  # zed2/zed_node/odom
    global rtk_signal_strong, x_hat, P, current_x, current_y

    if not rtk_signal_strong:
        signal = 1
        current_pose_zed = msg.pose.pose
        print(current_pose_zed)

def callback2(msg):  # ublox_gps/rtcm
    global rtk_signal_strong
    rtk_signal_strong = True

def callback3(msg):  # ublox_gps/fix
    global rtk_signal_strong
    position_covariance = msg.position_covariance_type
    threshold_value = 1

    if position_covariance > threshold_value:
        rtk_signal_strong = False
    else:
        rtk_signal_strong = True

def callback4(msg):  # utm
    global current_x, current_y
    utm_x = msg.pose.position.x
    utm_y = msg.pose.position.y
    signal = 2

def callback5(msg):
    global rtk_signal_strong, x_hat, P, current_x, current_y
    if not rtk_signal_strong:
        utm_x = msg.pose.position.x
        utm_y = msg.pose.position.y
        
        ################################################ Prediction ######################################################
        # Prediction step (assuming constant velocity model)
        A = np.eye(3)  # State transition matrix
        B = np.zeros(3)  # Control input matrix (not used here)
        u = np.zeros(3)  # Control input (not used here)

        # Predicted state estimate
        x_hat_minus = np.dot(A, x_hat) + np.dot(B, u)

        # Predicted error covariance
        P_minus = np.dot(np.dot(A, P), A.T) + Q
        ###################################################################################################################

        #################################################### Update #######################################################
        # Update step (incorporating UTM measurement)
        H = np.eye(3)  # Measurement matrix (identity for direct measurement)
        z = np.array([utm_x, utm_y, 0.0])  # Measurement vector

        # Calculate the Kalman gain
        K = np.dot(np.dot(P_minus, H.T), np.linalg.inv(np.dot(np.dot(H, P_minus), H.T) + R))

        # Update the state estimate
        x_hat = x_hat_minus + np.dot(K, (z - np.dot(H, x_hat_minus)))

        # Update the error covariance
        P = np.dot((np.eye(3) - np.dot(K, H)), P_minus)
        ###################################################################################################################


        # Set the current position based on the fused estimate
        current_x = x_hat[0]
        current_y = x_hat[1]

def callback6(msg):
    # Callback for /utm topic (GPS/UTM data)
    global current_x, current_y

    # If the RTK signal is strong, use the GPS/UTM data directly
    if rtk_signal_strong:
        current_x = msg.pose.position.x
        current_y = msg.pose.position.y

def sensor_integrate():
    rospy.init_node('sensor_integrate', anonymous=True)
    rospy.Subscriber("/zed2/zed_node/odom", Odometry, callback1)
    rospy.Subscriber("/ublox_gps/rtcm", Message, callback2)
    rospy.Subscriber("/ublox_gps/fix", NavSatFix, callback3)
    rospy.Subscriber("/utm", PoseStamped, callback4)
    rospy.Subscriber("/utm", PoseStamped, callback5)

if __name__ == '__main__':
    try:
        sensor_integrate()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
