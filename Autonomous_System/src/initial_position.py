#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped

A_POSITION = (0.35, 1.85, 0.0)

def send_goal(x, y, yaw):
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()
    
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0
    
    rospy.loginfo(f"Sending goal: ({x}, {y})")
    client.send_goal(goal)
    client.wait_for_result()

def in_area(current, target, threshold=0.3):
    return abs(current[0] - target[0]) < threshold and abs(current[1] - target[1]) < threshold

def pose_callback(msg):
    current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y, 0)
    if not in_area(current_position, A_POSITION):
        send_goal(*A_POSITION)

if __name__ == "__main__":
    rospy.init_node('send_goal_node')

    # Subscribe to the robot's current pose topic
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, pose_callback)

    # Spin the node to keep it alive
    rospy.spin()

