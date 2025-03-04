#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import math
from tf.transformations import quaternion_from_euler

# 目标位置（A 和 B 的坐标）
A_POSITION = (-0.1, 0.0, 0.0)
B_POSITION = (-0.1, -2.4, 0.0)

# 记录初始朝向（即 yaw）
initial_yaw = 0.0

# 发布过滤后的激光数据
laser_pub = rospy.Publisher('/laser_scan_filtered', LaserScan, queue_size=10)

def laser_callback(data):
    # 获取雷达扫描数据，判断距离小于 5cm 的点（可以根据需要调整距离阈值）
    threshold_distance = 0.05  # 忽略小于 5cm 的障碍物
    filtered_ranges = [r if r > threshold_distance else float('inf') for r in data.ranges]

    # 创建新的 LaserScan 消息，并填充数据
    filtered_scan = LaserScan()
    filtered_scan.header = data.header
    filtered_scan.ranges = filtered_ranges
    filtered_scan.angle_min = data.angle_min
    filtered_scan.angle_max = data.angle_max
    filtered_scan.angle_increment = data.angle_increment
    filtered_scan.time_increment = data.time_increment
    filtered_scan.scan_time = data.scan_time
    filtered_scan.range_min = data.range_min
    filtered_scan.range_max = data.range_max

    # 发布过滤后的数据
    laser_pub.publish(filtered_scan)

def send_goal(x, y, yaw):
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()
    
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    
    # 使用初始 yaw 角度（转换为四元数）
    quaternion = quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]
    
    rospy.loginfo(f"Sending goal: ({x}, {y}, {yaw})")
    client.send_goal(goal)
    client.wait_for_result()

def callback(msg):
    global current_position
    if msg.data == "Bin is full":
        if in_area(current_position, A_POSITION):
            send_goal(*B_POSITION, initial_yaw)
        elif in_area(current_position, B_POSITION):
            send_goal(*A_POSITION, initial_yaw)
        else:
            rospy.loginfo(f"Current position is not at A or B. Going to A.")
            send_goal(*A_POSITION, initial_yaw)

def in_area(current, target, threshold=0.3):
    return abs(current[0] - target[0]) < threshold and abs(current[1] - target[1]) < threshold

def position_callback(msg):
    global current_position, initial_yaw
    current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y, 0)
    
    # 记录初始朝向（如果没有记录）
    if initial_yaw == 0.0:
        initial_yaw = msg.pose.pose.orientation.z  # 假设这是机器人的初始朝向，实际应该通过四元数转换
    rospy.loginfo(f"current position: ({msg.pose.pose.position.x},{msg.pose.pose.position.y})")

def listener():
    # 只初始化一次节点
    rospy.init_node("sensor_listener", anonymous=True)

    # 订阅激光扫描数据，并调用 laser_callback 进行处理
    rospy.Subscriber("/scan", LaserScan, laser_callback)
    
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
