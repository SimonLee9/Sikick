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
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import heapq

# Publisher for controlling key
pub = rospy.Publisher('control_key', UInt16MultiArray, queue_size=10)
odom_pub = rospy.Publisher('/Sikick/odom', Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

rtk_signal_strong = True  # flag, tracking



goal_positions = []  # List to store goal positions A, B, C

current_x = 0.0
current_y = 0.0


def callback1(msg):  # zed2/zed_node/odom


    if not rtk_signal_strong:
        #rospy.loginfo('zed_odometry')
        signal = 1
        #current_pose_zed = msg.pose.pose
        #print(current_pose_zed)
        #rospy.loginfo('current_pose_zed')
        #print(msg.pose.position.x)
        #print(msg.pose.position.y)
        

def callback2(msg):  # ublox_gps/rtcm
    global rtk_signal_strong
    rtk_signal_strong = True
    #rospy.loginfo('ublox_gps-RTK')

def callback3(msg):  # ublox_gps/fix
    global rtk_signal_strong
    #navsatfix_msg = NavSatFix()
    position_covariance = msg.position_covariance_type #sensor_msgs/NavSatFix Message
    threshold_value =2 #######################Critical#####################################

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
    

    global current_x, current_y
    current_x = msg.pose.position.x
    current_y = msg.pose.position.y
    

def test(msg):
    rospy.loginfo('TT')
    print(msg.pose.position.x)
    print(msg.pose.position.y)

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
    #rospy.loginfo('TEST')
    #print('test')
    #rospy.logwarn("False")

    global rtk_signal_strong

    if rtk_signal_strong == False: # Get data from ZED2 Camera
        zed2_odom =None 
        try:
            zed2_odom = rospy.wait_for_message('/zed2/zed_node/odom', Odometry, timeout=1.0)
        except rospy.ROSException:
            rospy.logwarn("Failed to get zed2 Odometry data.")

        
        if zed2_odom:
            current_x = zed2_odom.pose.pose.position.x
            current_y = zed2_odom.pose.pose.position.y
            source = "Camera Odometry"
        else:
            current_x = 0.0
            current_y = 0.0
            
    else:
        gps_position = None
        try:
            gps_position = rospy.wait_for_message('/utm', PoseStamped, timeout=1.0)
        except rospy.ROSException:
            rospy.logwarn("Failed to get GPS data.")

        if gps_position:
            current_x = gps_position.pose.position.x
            current_y = gps_position.pose.position.y
            source = "GPS UTM"
        else:
            current_x = 0.0
            current_y = 0.0

        print('Strong')
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = 'odom'  # Change the frame_id to match your setup
    odom.child_frame_id = 'base_link'  # Change the child_frame_id as needed

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
    rospy.Subscriber("/goal_positions", Pose, goal_positions_callback) # PoseArray <-> Pose
    #rospy.Subscriber("/utm", PoseStamped, test) # PoseArray <-> Pose

    #rospy.Publisher("/Test",Odometry, queue_size=10,Current_Odom_publisher)

if __name__ == '__main__':
    try:
        sensor_integrate()
        #plan_and_execute_path()  # Execute the path using A* algorithm
        #Current_Odom_publisher()

        rospy.Timer(rospy.Duration(1.0), Current_Odom_publisher)  # Adjust the duration as needed

        #test()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
