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


#goal_positions = [(x_A, y_A), (x_B, y_B), (x_C, y_C)]  # List to store goal positions A, B, and C

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
    threshold_value =1

    # Check the value of position_covariance and update rtk_signal_strong    
    if  position_covariance > threshold_value: # Set your threshold value here
        rtk_signal_strong = False
        #rospy.loginfo('Bad signal RTK') # print Test OK
    else:
        rtk_signal_strong = True
        rospy.loginfo('Active RTK')

        
    #rospy.loginfo('ublox_gps')
    #print(msg.status.status)

def callback4(msg):  # utm
    #rospy.loginfo('ublox_gps-UTM')
    
    #print(msg.pose.position)
    signal = 2
    
    #print(msg.pose.position.x)
    #print(msg.pose.position.y)

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

'''
def astar(start, end):
    open_list = []
    closed_set = set()

    #heappush(open_list, (start.f, start))
    heapq.heappush(open_list, (start.f, start))

    while open_list:
        _, current_node = heapq.heappop(open_list) # heappush

        if current_node.x == end.x and current_node.y == end.y:
            path = []
            while current_node:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            return path[::-1]

        closed_set.add((current_node.x, current_node.y))

        neighbors = [(1, 0), (0, 1), (-1, 0), (0, -1)]

        for dx, dy in neighbors:
            neighbor_x = current_node.x + dx
            neighbor_y = current_node.y + dy

            if (
                0 <= neighbor_x < 100 and
                0 <= neighbor_y < 100 and
                (neighbor_x, neighbor_y) not in closed_set
            ):
                neighbor = Node(
                    neighbor_x,
                    neighbor_y,
                    current_node,
                    current_node.g + 1,
                    heuristic(Node(neighbor_x, neighbor_y), end)
                )

                heapq.heappush(open_list, (neighbor.f, neighbor)) # heappush

def plan_and_execute_path():
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    rospy.loginfo('TEst')

    for i in range(len(goal_positions) - 1):
        start_x, start_y = goal_positions[i]
        end_x, end_y = goal_positions[i + 1]

        path = astar(Node(start_x, start_y), Node(end_x, end_y))

        if path:
            for x, y in path:
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.pose.position.x = x
                goal.target_pose.pose.position.y = y
                goal.target_pose.pose.orientation.w = 1.0

                #rospy.loginfo("Target position: x={x}, y={y}")
                client.send_goal(goal)
                client.wait_for_result()
'''

'''
def Current_Odom_publisher(msg):
    rospy.loginfo('TEST')
    print('test')

    
    global rtk_signal_strong
    current_x = callback4().msg.pose.position.x
    current_x = msg.pose.position.x
    print(current_x)

    current_y = callback4().msg.pose.position.y
    
    current_x = 0.0
    current_y = 0.0


    if rtk_signal_strong == False:
        current_x = callback1().msg.pose.position.x
        current_y = callback1().msg.pose.position.y

    else:
       current_x = callback4().msg.pose.position.x
       current_y = callback4().msg.pose.position.y

    
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = 'odom'  # Change the frame_id to match your setup
    odom.child_frame_id = 'base_link'  # Change the child_frame_id as needed

    # Set the position in the Odometry message
    odom.pose.pose.position.x = current_x
    odom.pose.pose.position.y = current_y

    # Publish the Odometry message
    odom_pub.publish(odom)
'''

def Current_Odom_publisher(event):
    #rospy.loginfo('TEST')
    #print('test')
    #rospy.logwarn("False")

    global rtk_signal_strong

    #global current_x, current_y

    if rtk_signal_strong == False: # Get data from ZED2 Camera
        zed2_odom =None 
        #current_x = callback1().msg.pose.position.x
        #current_y = callback1().msg.pose.position.y
        try:
            zed2_odom = rospy.wait_for_message('/zed2/zed_node/odom', Odometry, timeout=1.0)
        except rospy.ROSException:
            rospy.logwarn("Failed to get zed2 Odometry data.")

        if zed2_odom:
            current_x = zed2_odom.pose.pose.position.x
            current_y = zed2_odom.pose.pose.position.y

            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = 'odom'  # Change the frame_id to match your setup
            odom.child_frame_id = 'base_link'  # Change the child_frame_id as needed

            # Set the position in the Odometry message
            odom.pose.pose.position.x = current_x
            odom.pose.pose.position.y = current_y

            # Publish the Odometry message
            odom_pub.publish(odom)
        rospy.loginfo("Camera - Current Odometry Position: x={}, y={}".format(odom.pose.pose.position.x, odom.pose.pose.position.y))
    else:
        gps_position = None
        try:
            gps_position = rospy.wait_for_message('/utm', PoseStamped, timeout=1.0)
        except rospy.ROSException:
            rospy.logwarn("Failed to get GPS data.")

        if gps_position:
            current_x = gps_position.position.x
            current_y = gps_position.position.y

            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = 'odom'  # Change the frame_id to match your setup
            odom.child_frame_id = 'base_link'  # Change the child_frame_id as needed

            # Set the position in the Odometry message
            odom.pose.pose.position.x = current_x
            odom.pose.pose.position.y = current_y

            # Publish the Odometry message
            odom_pub.publish(odom)
        rospy.loginfo("GPS_UTM - Current Odometry Position: x={}, y={}".format(odom.pose.pose.position.x, odom.pose.pose.position.y))


        print('Strong')


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
