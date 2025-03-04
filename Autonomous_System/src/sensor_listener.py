#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

# 目标位置（A 和 B 的坐标）
A_POSITION = (-0.1, 0.0, 0.0)
B_POSITION = (-0.1, -2.4, 0.0)
    
def send_goal(x, y, yaw):
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()
    
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    
    goal.target_pose.pose.orientation.w = 1.0
    # goal.target_pose.pose.orientation.z = 0.707
    
    rospy.loginfo(f"Sending goal: ({x}, {y})")
    client.send_goal(goal)
    client.wait_for_result()

def callback(msg):
    global current_position
    if msg.data == "Bin is full":
        if in_area(current_position, A_POSITION):
            send_goal(*B_POSITION)
        elif in_area(current_position, B_POSITION):
            send_goal(*A_POSITION)
        else:
            rospy.loginfo(f"Current position is not at A or B. Going to A.")
            send_goal(*A_POSITION)

def in_area(current, target, threshold=0.3):
    return abs(current[0] - target[0]) < threshold and abs(current[1] - target[1]) < threshold

def position_callback(msg):
    global current_position
    current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y, 0)
    rospy.loginfo(f"current position: ({msg.pose.pose.position.x},{msg.pose.pose.position.y})")

def listener():

    rospy.init_node("sensor_listener")

    # 订阅机器人位置的消息
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, position_callback)

    # 订阅传感器的消息（这里假设传感器数据的主题是 /sensor_topic）
    rospy.Subscriber("/sensor_topic", String, callback)

    rospy.loginfo("Waiting for initial position...")

    # 给机器人时间来进行定位
    rospy.sleep(2)

    rospy.loginfo("Starting navigation...")

    # 启动机器人并等待
    rospy.spin()

if __name__ == '__main__':
    current_position = (0.0, 0.0, 0.0)  # 初始位置
    listener()
