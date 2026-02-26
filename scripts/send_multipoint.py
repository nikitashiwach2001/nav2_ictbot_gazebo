#!/usr/bin/env python3

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import math
import time

def create_pose(navigator, x, y, yaw_deg):
    yaw = math.radians(yaw_deg)

    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0

    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)

    return pose


def main():
    rclpy.init()
    navigator = BasicNavigator()

    print("Waiting for Nav2...")
    navigator.waitUntilNav2Active()

    waypoints = []

    num = int(input("How many waypoints? "))

    for i in range(num):
        print(f"\nWaypoint {i+1}")
        x = float(input("Enter X: "))
        y = float(input("Enter Y: "))
        yaw = float(input("Enter Yaw (deg): "))

        waypoints.append(create_pose(navigator, x, y, yaw))

    print("Sending waypoints...")
    navigator.followWaypoints(waypoints)

    while not navigator.isTaskComplete():
        time.sleep(0.5)

    print("All waypoints completed!")
    rclpy.shutdown()


if __name__ == '__main__':
    main()