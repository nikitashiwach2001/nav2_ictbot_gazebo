#!/usr/bin/env python3
"""
Publishes /robot_description as a latched topic only — no TF published.
Use this instead of robot_state_publisher when Isaac Sim already handles TF.
"""
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


class URDFPublisher(Node):
    def __init__(self):
        super().__init__('urdf_publisher')
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(String, '/robot_description', qos)
        pkg = get_package_share_directory('ict_bot_nav')
        urdf_path = os.path.join(pkg, 'urdf', 'ict_bot.urdf')
        with open(urdf_path, 'r') as f:
            urdf_content = f.read()
        msg = String()
        msg.data = urdf_content
        self.pub.publish(msg)
        self.get_logger().info('Published /robot_description (latched, no TF)')


def main():
    rclpy.init()
    node = URDFPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
