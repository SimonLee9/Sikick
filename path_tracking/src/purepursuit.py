#!/usr/bin/env python
# -*- coding: utf-8 -*- #한글이 포함된 경우에 필수


import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import Float64
import math

class PurePursuit:
    def __init__(self, look_ahead):
        self.look_ahead = look_ahead
        self.path = None

        # Subscribers
        #self.path_sub = rospy.Subscriber('/Sikick/path', Path, self.system_path_callback)
        self.path_sub = rospy.Subscriber('/Sikick/odom', Path, self.system_path_callback)
        self.pose_sub = rospy.Subscriber('/Sikick/pose', PoseStamped, self.system_pose_callback)
        #self.target_position_sub = rospy.Subscriber('goal_positions_topic', Path, self.get_path_callback)
        self.target_position_sub = rospy.Subscriber('goal_path-positions_topic', Path, self.target_positions_callback)

        #'path'는 차량이 따라가야 할 경로를, 
        #'pose'는 차량의 현재 위치와 방향(자세)을 나타냅니다. 

        # PurePursuit 클래스는 이 두 가지 정보를 바탕으로 Pure Pursuit 알고리즘을 실행하고, 조향 각도를 계산합니다.

        self.speed_pwm_pub = rospy.Publisher('speed_pwm',Float64,queue_size=10)
        self.steering_angle_pub = rospy.Publisher('steer_angle', Float64, queue_size=10)

    def target_positions_callback(self, msg):
        self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
    
    def system_path_callback(self, msg):
        self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

    def system_pose_callback(self, msg):
        if self.path is not None:
            vehicle_position = (msg.pose.position.x, msg.pose.position.y)
            vehicle_angle = 2 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)  # Assuming quaternion
                                #math.atan2 : 라디안 결과

            # Compute steering angle
            steering_angle = self.get_steering_angle(vehicle_position, vehicle_angle)
            #print("Steering angle: ", steering_angle)

            steering_angle_degrees = math.degrees(steering_angle)
            print("Steering angle: ", steering_angle_degrees)

            # Publish steering angle
            self.steering_angle_pub.publish(steering_angle) # 라디안

            # Set drive angle (you should replace this with your own logic)
            speed_pwm = Float64()
            speed_pwm.data = 0.2 # Neutral (you should replace this with your own logic)
            self.speed_pwm_pub.publish(speed_pwm)
        else:
            # No path information is available. Stop the vehicle.
            speed_pwm = Float64()
            speed_pwm.data = 0.0 # Stop
            self.speed_pwm_pub.publish(speed_pwm)
            self.steering_angle_pub.publish(0.0)
            print("No path-Steering angle: ",speed_pwm.data)

    
    #def get_path_callback(self, msg):
    #    self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

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
    pp = PurePursuit(look_ahead=8)
    rospy.spin()
