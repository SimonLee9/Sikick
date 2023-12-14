#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

def publish_goal_positions():
    # Initialize the node
    rospy.init_node('goal_positions_publisher')

    # Create a publisher
    pub = rospy.Publisher('goal_path_positions_topic', Path, queue_size=10)

    # Define the goal positions
    goal_positions = [(351763.901, 4025749.144), (351783.3108, 4025748.981), (351782.6192, 4025739.505)]  # Replace with actual coordinates
    #goal_positions = [(10, 0),(10, 10),(20,10)]  # Replace with actual coordinates

    print("Goal positions: ", goal_positions)

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
        pose_stamped.pose.orientation.w = 1  # Set the orientation to be valid
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

