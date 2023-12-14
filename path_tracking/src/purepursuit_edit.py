#!/usr/bin/env python
# -*- coding: utf-8 -*- #한글이 포함된 경우에 필수
import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import UInt16MultiArray, Float64
import math
class PurePursuit:
    def __init__(self, look_ahead):
        self.look_ahead = look_ahead
        self.path= None 
        self.utm_x= None 
        self.utm_y = None
        #self.Sikick_VINS_path_sub = rospy.Subscriber('/Sikick/odom', Odometry, self.Sikick_VINS_odom_callback)
        self.ZED2_odom_sub = rospy.Subscriber("/zed2/zed_node/odom", Odometry, self.ZED2_odom_callback)
        self.Sikick_VINS_pose_sub = rospy.Subscriber('/Sikick/pose', PoseStamped, self.Sikick_VINS_pose_callback)

        self.target_position_sub = rospy.Subscriber('goal_path_positions_topic', Path, self.target_positions_callback)
        #self.SikickUTM_sub = rospy.Subscriber('/utm', PoseStamped, self.UTMonlyGPS_sikick)
        self.local_speed_vel_pub = rospy.Publisher('local_speed_vel',Float64,queue_size=10)
        self.local_steering_angle_pub = rospy.Publisher('local_steer_angle', Float64, queue_size=10)

        self.global_speed_vel_pub = rospy.Publisher('global_speed_vel',Float64,queue_size=10)
        self.global_steering_angle_pub = rospy.Publisher('global_steer_angle', Float64, queue_size=10)

    '''
    def UTMonlyGPS_sikick(self, msg):
        if self.path is not None:
            vehicle_position = (msg.pose.position.x, msg.pose.position.y)
            vehicle_angle = 2 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)  # Assuming quaternion  #math.atan2 : 라디안 결과
                               
            # Compute steering angle
            steering_angle = self.get_steering_angle(vehicle_position, vehicle_angle)
            steering_angle_degrees = math.degrees(steering_angle)
            print("Steering angle: ", steering_angle_degrees)
            self.steering_angle_pub.publish(steering_angle) # 라디안
            # Set drive angle (you should replace this with your own logic)
            speed_vel = Float64()
            speed_vel.data = 0.2 # Neutral (you should replace this with your own logic)
            self.speed_vel_pub.publish(speed_vel)
            print("Speed velocity:",speed_vel )
        else: # No path information is available. Stop the vehicle.
            speed_vel = Float64()
            speed_vel.data = 0.0 # Stop
            self.speed_vel_pub.publish(speed_vel)
            self.steering_angle_pub.publish(0.0)
            print("No path-Steering angle: ",speed_vel.data)
    '''
    def target_positions_callback(self, msg):
        self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses if pose.pose.position.x != msg.poses[0].pose.position.x and pose.pose.position.x != msg.poses[-1].pose.position.x]

    
    def ZED2_odom_callback(self, msg): # /Sikick/odom, 그리고 Path를 사용했을 때. 
        #self.path = [(msg.pose.pose.position.x, msg.pose.pose.position.y)]
        if self.path is not None:
            vehicle_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            vehicle_angle = 2 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)  # Assuming quaternion #math.atan2 : 라디안 결과
            local_steering_angle = self.get_steering_angle(vehicle_position, vehicle_angle) # Compute steering angle
            local_steering_angle_degrees = math.degrees(local_steering_angle)
            print("Loc-Steering angle by ZED2odom: ", local_steering_angle_degrees)

            self.local_steering_angle_pub.publish(local_steering_angle) # 라디안 # Publish steering angle
            local_speed_vel = Float64()
            local_speed_vel.data = 0.2 # Neutral (you should replace this with your own logic)
            self.local_speed_vel_pub.publish(local_speed_vel)
            print("Loc-Speed velocity by ZED2odom:",local_speed_vel )
        else:             # No path information is available. Stop the vehicle.
            local_speed_vel = Float64()
            local_speed_vel.data = 0.0 # Stop
            self.local_speed_vel_pub(local_speed_vel)
            self.local_steering_angle_pub.publish(0.0)
            print("Loc-No path-Steering angle by ZED2odom: ",local_speed_vel.data)

    '''
    def Sikick_VINS_odom_callback(self, msg): # /Sikick/odom, 그리고 Path를 사용했을 때. 
        self.path = [(msg.pose.pose.position.x, msg.pose.pose.position.y)]
        if self.path is not None:
            vehicle_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            vehicle_angle = 2 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)  # Assuming quaternion #math.atan2 : 라디안 결과
            steering_angle = self.get_steering_angle(vehicle_position, vehicle_angle) # Compute steering angle
            steering_angle_degrees = math.degrees(steering_angle)
            print("Steering angle by Odom: ", steering_angle_degrees)

            self.steering_angle_pub.publish(steering_angle) # 라디안 # Publish steering angle
            speed_vel_odom = Float64()
            speed_vel_odom.data = 0.2 # Neutral (you should replace this with your own logic)
            self.speed_vel_pub.publish(speed_vel_odom)
            print("Speed velocity by Odom:",speed_vel_odom )
        else:             # No path information is available. Stop the vehicle.
            speed_vel_odom = Float64()
            speed_vel_odom.data = 0.0 # Stop
            self.speed_vel_pub.publish(speed_vel_odom)
            self.steering_angle_pub.publish(0.00)
            print("No path-Steering angle by Odom: ",speed_vel_odom.data)
    '''
    
    
    def Sikick_VINS_pose_callback(self, msg):
        if self.path is not None:
            vehicle_position = (msg.pose.position.x, msg.pose.position.y)
            vehicle_angle = 2 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)  # Assuming quaternion #math.atan2 : 라디안 결과
            global_steering_angle = self.get_steering_angle(vehicle_position, vehicle_angle) # Compute steering angle
            global_steering_angle_degrees = math.degrees(global_steering_angle)
            print("Glo-Steering angle by Pose: ", global_steering_angle_degrees)

            self.global_steering_angle_pub.publish(global_steering_angle) # 라디안 # Publish steering angle
            global_speed_vel = Float64()
            global_speed_vel.data = 0.2 # Neutral (you should replace this with your own logic)
            self.global_speed_vel_pub.publish(global_speed_vel)
            print("Glo-Speed velocity by Pose:",global_speed_vel )
        else:             # No path information is available. Stop the vehicle.
            global_speed_vel = Float64()
            global_speed_vel.data = 0.0 # Stop
            self.global_speed_vel_pub.publish(global_speed_vel)
            self.global_steering_angle_pub.publish(0.0)
            print("Glo-No path-Steering angle by Pose: ",global_speed_vel.data)
    
    def get_steering_angle(self, vehicle_position, vehicle_angle):
        # Find the path point closest to the vehicle
        distance = [math.hypot(vp[0]-vehicle_position[0], vp[1]-vehicle_position[1]) for vp in self.path]
        nearest_index = distance.index(min(distance))
        # Find the path point within the look ahead distance
        look_ahead_point = None
        for i in range(nearest_index, len(self.path)):
            if math.hypot(self.path[i][0]-vehicle_position[0], self.path[i][1]-vehicle_position[1]) > self.look_ahead:
                look_ahead_point = self.path[i]
                break
        # Compute the angle between the vehicle's position and the look ahead point
        if look_ahead_point is not None:
            angle = math.atan2(look_ahead_point[1]-vehicle_position[1], look_ahead_point[0]-vehicle_position[0])
            steering_angle = angle - vehicle_angle
        else:
            steering_angle = 0
        return steering_angle

if __name__ == "__main__":
    rospy.init_node('pure_pursuit')
    pp = PurePursuit(look_ahead=1)
    rospy.spin()