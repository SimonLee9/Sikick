#!/usr/bin/env python

import math
from math import sin, cos, pi
import tf
import rospy
from std_msgs.msg import UInt16MultiArray
from sensor_msgs.msg import NavSatFix
from rtcm_msgs.msg import Message
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
import numpy as np

# Publisher for controlling key
pub = rospy.Publisher('control_key', UInt16MultiArray, queue_size=10)
odom_pub = rospy.Publisher('/Sikick/odom', Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()


# Initialize Kalman filter variables
x_hat = np.array([0.0, 0.0, 0.0])  # Initial state estimate [x, y, theta]
P = np.eye(3)  # Initial error covariance matrix
Q = np.eye(3) * 0.01  # Process noise covariance matrix (adjust as needed)
R = np.eye(3) * 0.1  # Measurement noise covariance matrix (adjust as needed)


rtk_signal_strong = True  # flag, tracking

current_x = 0.0
current_y = 0.0

source="Unkown"


def callback1(msg):  # zed2/zed_node/odom

    global rtk_signal_strong, x_hat, P, current_x, current_y
    if not rtk_signal_strong:
        #rospy.loginfo('zed_odometry')
        signal = 1
        #current_pose_zed = msg.pose.pose
        #print(current_pose_zed)
        #rospy.loginfo('current_pose_zed')
        #print(msg.pose.position.x)
        #print(msg.pose.position.y)
        #current_pose_zed = msg.pose.pose
        #print(current_pose_zed)
        

def callback2(msg):  # ublox_gps/rtcm
    global rtk_signal_strong
    rtk_signal_strong = True
    #rospy.loginfo('ublox_gps-RTK')

def callback3(msg):  # ublox_gps/fix
    global rtk_signal_strong
    #navsatfix_msg = NavSatFix()
    position_covariance = msg.position_covariance_type #sensor_msgs/NavSatFix Message
    threshold_value =1 #######################Critical#####################################

    # Check the value of position_covariance and update rtk_signal_strong    
    if  position_covariance > threshold_value: # Set your threshold value here
        rtk_signal_strong = False
        #rospy.loginfo('Bad signal RTK') # print Test OK
    else:
        rtk_signal_strong = True
        #rospy.loginfo('Active RTK') # print Test OK

        
    #rospy.loginfo('ublox_gps')
    #print(msg.status.status)

def callback4(msg):  # utm
    #rospy.loginfo('ublox_gps-UTM')
    
    #print(msg.pose.position)
    signal = 2
    

    global current_x, current_y, utm_x, utm_y
    utm_x = msg.pose.position.x
    utm_y = msg.pose.position.y
    
    #callback5(utm_x, utm_y)
    
def callback5(msg):
    global rtk_signal_strong, x_hat, P, current_x, current_y, source, utm_x, utm_y
    # print('TEst')
    if not rtk_signal_strong:
        """
        try:
            zed2_odom = rospy.wait_for_message('/zed2/zed_node/odom', Odometry, timeout=1.0)
            print('TEst')

        except rospy.ROSException:
            rospy.logwarn("Failed to get ZED2 Odometry data.")
            return
        print('111')
        """
        #zed2_x = msg.pose.pose.position.x
        #zed2_y = msg.pose.pose.position.y

        utm_x = msg.pose.position.x
        utm_y = msg.pose.position.y

        # Prediction step (assuming constant velocity model)
        A = np.eye(3)  # State transition matrix
        B = np.zeros(3)  # Control input matrix (not used here)
        u = np.zeros(3)  # Control input (not used here)

        # Predicted state estimate
        x_hat_minus = np.dot(A, x_hat) + np.dot(B, u)

        # Predicted error covariance
        P_minus = np.dot(np.dot(A, P), A.T) + Q

        # Update step (incorporating GPS measurement)
        H = np.eye(3)  # Measurement matrix (identity for direct measurement)
        #z = np.array([zed2_x, zed2_y])  # Measurement vector
        z = np.array([utm_x, utm_y, 0.0])  # Measurement vector

        # Calculate the Kalman gain
        K = np.dot(np.dot(P_minus, H.T), np.linalg.inv(np.dot(np.dot(H, P_minus), H.T) + R))

        # Update the state estimate
        x_hat = x_hat_minus + np.dot(K, (z - np.dot(H, x_hat_minus)))

        # Update the error covariance
        P = np.dot((np.eye(3) - np.dot(K, H)), P_minus)

        # Set the current position and source based on the fused estimate
        current_x = x_hat[0]
        current_y = x_hat[1]
        source = "Sensor Fusion (Camera Odometry)"
    else:
        # Handle the case when RTK signal is strong (use GPS/UTM data)
        pass

def callback6(msg):
    # Callback for /utm topic (GPS/UTM data)
    '''
    global current_x, current_y, source, utm_x, utm_y

    utm_x = msg.pose.position.x
    utm_y = msg.pose.position.y

    # If the RTK signal is strong, use the GPS/UTM data directly
    if rtk_signal_strong:
        current_x = utm_x
        current_y = utm_y
        source = "GPS UTM"
    '''
    global current_x, current_y

    # If the RTK signal is strong, use the GPS/UTM data directly
    if rtk_signal_strong:
        current_x = msg.pose.position.x
        current_y = msg.pose.position.y
        source = "GPS UTM"


def goal_positions_callback(msg):
    global goal_positions
    goal_positions = [(pose.position.x, pose.position.y) for pose in msg.poses]
    #rospy.loginfo("Received goal positions: {goal_positions}")

class Node:
    def __init__(self, x, y, parent=None, g=0, h=0):
        self.x = x
        self.y = y
        self.parent = parent
        self.g = g
        self.h = h
        self.f = g + h

def heuristic(node, goal):
    return math.sqrt((node.x - goal.x) ** 2 + (node.y - goal.y) ** 2)


def Current_Odom_publisher(event):
    global current_x, current_y, source

    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = 'odom'  # Adjust the frame_id as needed
    odom.child_frame_id = 'base_link'  # Adjust the child_frame_id as needed

    # Set the position in the Odometry message
    odom.pose.pose.position.x = current_x
    odom.pose.pose.position.y = current_y

    # Publish the Odometry message
    odom_pub.publish(odom)
    rospy.loginfo("Position Source: {}, x={}, y={}".format(source, current_x, current_y))


def sensor_integrate():
    rospy.init_node('sensor_integrate', anonymous=True)
    rospy.Subscriber("/zed2/zed_node/odom", Odometry, callback1)
    rospy.Subscriber("/ublox_gps/rtcm", Message, callback2)
    rospy.Subscriber("/ublox_gps/fix", NavSatFix, callback3)
    rospy.Subscriber("/utm", PoseStamped, callback4)

    rospy.Subscriber("/utm", PoseStamped, callback5)

    rospy.Subscriber("/goal_positions", Pose, goal_positions_callback) # PoseArray <-> Pose

    #rospy.Subscriber("/zed2/zed_node/odom", Odometry, callback5)
    #rospy.Subscriber("/utm", PoseStamped, test) # PoseArray <-> Pose

    #rospy.Publisher("/Test",Odometry, queue_size=10,Current_Odom_publisher)
    rospy.Timer(rospy.Duration(1.0), Current_Odom_publisher)  # Adjust the duration as needed


if __name__ == '__main__':
    try:
        sensor_integrate()
        #plan_and_execute_path()  # Execute the path using A* algorithm
        #Current_Odom_publisher()


        #test()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
