#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

def move_to_goal(x, y, frame_id="map"):
    # 初始化ROS节点
    rospy.init_node('move_to_goal', anonymous=True)
    
    # 创建一个action客户端
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
    # 等待move_base服务器连接
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base action server")
    
    # 设置目标位置
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame_id
    goal.target_pose.header.stamp = rospy.Time.now()

    # 设置目标位置的坐标
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0  # 定义方向（这里是正方向）

    # 发送目标并等待结果
    rospy.loginfo("Sending goal")
    client.send_goal(goal)
    client.wait_for_result()

    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("The robot reached the goal!")
    else:
        rospy.loginfo("The robot failed to reach the goal.")

if __name__ == '__main__':
    try:
        # 目标位置可以从 launch 文件传入，或者在脚本中直接定义
        move_to_goal(2.0, 2.0)  # 将机器人移动到 (2.0, 2.0) 坐标
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
