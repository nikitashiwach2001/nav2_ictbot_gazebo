#!/usr/bin/env python3

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
# from tf_transformations import quaternion_from_euler
import math
import time

def main():
    rclpy.init()

    navigator = BasicNavigator()

    print("Waiting for Nav2 to become active...")
    navigator.waitUntilNav2Active()
    print("Nav2 is active!")

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()

    # Hardcoded Goal Values 
    # x = 3.3
    # y = 1.2
    # yaw = 1.57  # radians (90 degrees)

    x = float(input("Enter X (meters): "))
    y = float(input("Enter Y (meters): "))
    yaw_deg = float(input("Enter Yaw (degrees): "))

    # Convert degrees to radians
    yaw = math.radians(yaw_deg)

    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.position.z = 0.0

    # q = quaternion_from_euler(0.0, 0.0, yaw)
    # goal_pose.pose.orientation.x = q[0]
    # goal_pose.pose.orientation.y = q[1]
    # goal_pose.pose.orientation.z = q[2]
    # goal_pose.pose.orientation.w = q[3]

    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal_pose.pose.orientation.w = math.cos(yaw / 2.0)

    print(f"Sending goal: x={x}, y={y}, yaw={yaw}")

    navigator.goToPose(goal_pose)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        time.sleep(0.5)

    result = navigator.getResult()

    print("Navigation completed!")

    rclpy.shutdown()

if __name__ == '__main__':
    main()