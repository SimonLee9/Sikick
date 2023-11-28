#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

def publish_goal_positions():
    # Initialize the node
    rospy.init_node('goal_positions_publisher')

    # Create a publisher
    pub = rospy.Publisher('goal_positions_topic', Path, queue_size=10)

    # Define the goal positions
    goal_positions = [(0, 0), (10, 0), (10, 10)]  # Replace with actual coordinates

    # Create a Path message
    path = Path()
    path.header.frame_id = 'map'  # Set the frame_id for the Path message

    # Fill the Path message with the goal positions
    for i, goal in enumerate(goal_positions):
        pose_stamped = PoseStamped()
        pose_stamped.header.seq = i
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.pose.position.x = goal[0]
        pose_stamped.pose.position.y = goal[1]
        path.poses.append(pose_stamped)

    # Set the rate
    rate = rospy.Rate(10)

    # Keep publishing the goal_positions
    while not rospy.is_shutdown():
        # Publish the Path message
        pub.publish(path)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_goal_positions()
    except rospy.ROSInterruptException:
        pass
