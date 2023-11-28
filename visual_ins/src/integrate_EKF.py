#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import UInt16MultiArray, Float32
from sensor_msgs.msg import NavSatFix
from rtcm_msgs.msg import Message
from geometry_msgs.msg import PoseStamped, PoseArray, Twist, Pose, Quaternion
from nav_msgs.msg import Odometry
import numpy as np
from nav_msgs.msg import Path


from tf.transformations import euler_from_quaternion, quaternion_from_euler


class SensorIntegrator:
    #def __init__(self, goal_positions):
    def __init__(self): # Remove goal_positions as an argument
        self.pub = rospy.Publisher('control_key', UInt16MultiArray, queue_size=10)
        self.odom_pub = rospy.Publisher('/Sikick/odom', Odometry, queue_size=50)
        self.pose_pub = rospy.Publisher('/Sikick/pose',PoseStamped, queue_size=10)

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

        #self.goal_positions = [(x1, y1),(x2, y2),(x3, y3)]
        #self.goal_positions = goal_positions
        self.goal_positions = [] # # Initialize as an empty list

        # Subscribe to the topic publishing goal_positions
        # rospy.Subscriber("/goal_positions_topic", PoseArray, self.goal_positions_callback)
        rospy.Subscriber("/goal_positions_topic", Path, self.goal_positions_callback)

        self.distance_pub = rospy.Publisher('/distance_to_goal_topic', Float32, queue_size=10)

    def goal_positions_callback(self, msg):
        # Update goal_positions when a new message is received
        self.goal_positions = [(pose.position.x, pose.position.y) for pose in msg.pose]

    def get_next_goal(self):

        min_distance = float('inf')
        next_goal = None
        for goal in self.goal_positions:
            distance = ((self.current_x - goal[0])**2 + (self.current_y - goal[1])**2)**0.5
            if distance < min_distance:
                min_distance = distance
                next_goal = goal
        return next_goal

    def navigate_to_goal(self, goal):
        # Create a publisher for the cmd_vel topic
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Create a Twist message
        twist = Twist()

        # Calculate the difference between the current position and the goal
        dx = goal[0] - self.current_x
        dy = goal[1] - self.current_y

        # Calculate the distance to the goal
        distance_to_goal = math.sqrt(dx**2 + dy**2)

        # If the robot is close to the goal, stop and turn to the next goal
        if distance_to_goal < 0.1:  # 0.1 is a threshold value
            # Stop the robot
            twist.linear.x = 0.0

            # Determine the direction to the next goal
            if self.goal_positions:  # If there are still goals left
                next_goal = self.goal_positions[0]
                dx = next_goal[0] - self.current_x
                dy = next_goal[1] - self.current_y
                angle_to_next_goal = math.atan2(dy, dx)
                twist.angular.z = angle_to_next_goal
            else:  # If there are no more goals
                twist.angular.z = 0.0
        else:  # If the robot is not close to the goal, continue moving towards the goal
            # Calculate the angle to the goal
            angle_to_goal = math.atan2(dy, dx)

            # Set the angular speed in the z-axis (rotation around the z-axis)
            twist.angular.z = angle_to_goal

            # Set the linear speed in the x-axis (forward)
            twist.linear.x = distance_to_goal

        # Publish the Twist message
        pub.publish(twist)       
        
        # Publish the distance to the goal
        self.distance_pub.publish(distance_to_goal)

    #def callback1(self, msg):
    def odom_callback(self, msg):
        if not self.rtk_signal_strong:
            signal = 1

    #def callback2(self, msg):
    def rtcm_callback(self, msg):
        self.rtk_signal_strong = True

    #def callback3(self, msg):
    def navsatfix_callback(self, msg):
        position_covariance = msg.position_covariance_type
        threshold_value = 1

        if position_covariance > threshold_value:
            self.rtk_signal_strong = False
            self.source = "Bad signal RTK"
        else:
            self.rtk_signal_strong = True
            self.source = "Active RTK"

    #def callback4(self, msg):
    def posestamped_callback1(self, msg):
        signal = 2
        self.utm_x = msg.pose.position.x
        self.utm_y = msg.pose.position.y

    #def callback5(self, msg):
    def posestamped_callback2(self, msg):
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

    #def callback6(self, msg):
    def posestamped_callback3(self, msg):
        if self.rtk_signal_strong:
            self.current_x = msg.pose.position.x
            self.current_y = msg.pose.position.y
            self.source = "GPS UTM (RTK applied)"
            # rospy.loginfo("Position Source: {self.source}, x={self.current_x}, y={self.current_y}")

    def goal_positions_callback(self, msg):
        self.goal_positions = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

    def Current_Odom_publisher(self, event):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.current_x
        odom.pose.pose.position.y = self.current_y

        # Assuming the orientation is given in Euler angles
        euler = euler_from_quaternion([self.x_hat[2], 0, 0, 0])
        quaternion = quaternion_from_euler(0, 0, euler[2])
        odom.pose.pose.orientation = Quaternion(*quaternion)

        self.odom_pub.publish(odom)
        rospy.loginfo("Position Source: {}, x={}, y={}".format(self.source, self.current_x, self.current_y))

    def current_pose_publisher(self, event):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'odom'

        # Set the current position
        pose.pose.position.x = self.current_x
        pose.pose.position.y = self.current_y

        # Assuming the orientation is given in Euler angles
        euler = euler_from_quaternion([self.x_hat[2], 0, 0, 0])
        quaternion = quaternion_from_euler(0, 0, euler[2])
        pose.pose.orientation = Quaternion(*quaternion)

        # Publish the PoseStamped message
        self.pose_pub.publish(pose)

    def sensor_integrate(self):
        rospy.init_node('sensor_integrate', anonymous=True)
        rospy.Subscriber("/zed2/zed_node/odom", Odometry, self.odom_callback) # callback1
        rospy.Subscriber("/ublox_gps/rtcm", Message, self.rtcm_callback) # callback2
        rospy.Subscriber("/ublox_gps/fix", NavSatFix, self.navsatfix_callback) # callback3
        rospy.Subscriber("/utm", PoseStamped, self.posestamped_callback1) # callback4

        rospy.Subscriber("/utm", PoseStamped, self.posestamped_callback2) # callback5

        rospy.Subscriber("/utm", PoseStamped, self.posestamped_callback3) # callback6

        #rospy.Subscriber("/goal_positions", Pose, self.goal_positions_callback)
        rospy.Subscriber("/goal_positions", Path, self.goal_positions_callback)

        rospy.Timer(rospy.Duration(1.0), self.Current_Odom_publisher)

        rospy.Timer(rospy.Duration(1.0), self.navigate_to_goal_callback)

        rospy.Timer(rospy.Duration(1.0), self.current_pose_publisher)

        while self.goal_positions:
            next_goal = self.get_next_goal()
            self.navigate_to_goal(next_goal)
            self.goal_positions.remove(next_goal)

    # Add a callback function for the Timer
    def navigate_to_goal_callback(self, event):
        if self.goal_positions:
            next_goal = self.get_next_goal()
            self.navigate_to_goal(next_goal)
            self.goal_positions.remove(next_goal)
if __name__ == '__main__':
    try:
        #goal_positions = [(1, 2), (3, 4), (5, 6)]  # Replace with actual coordinates
        #integrator = SensorIntegrator(goal_positions)

        integrator = SensorIntegrator()
        integrator.sensor_integrate()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass