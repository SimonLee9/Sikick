#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray, Pose

def publish_goal_positions():
    # Initialize the node
    rospy.init_node('goal_positions_publisher')

    # Create a publisher
    pub = rospy.Publisher('goal_positions_topic', PoseArray, queue_size=10)

    # Define the goal positions
    goal_positions = [(1, 2), (3, 4), (5, 6)]  # Replace with actual coordinates

    # Create a PoseArray message
    pose_array = PoseArray()

    # Fill the PoseArray message with the goal positions
    for goal in goal_positions:
        pose = Pose()
        pose.position.x = goal[0]
        pose.position.y = goal[1]
        pose_array.poses.append(pose)

    # Set the rate
    rate = rospy.Rate(10)

    # Keep publishing the goal_positions
    while not rospy.is_shutdown():
        # Publish the PoseArray message
        pub.publish(pose_array)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_goal_positions()
    except rospy.ROSInterruptException:
        pass