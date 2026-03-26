#!/usr/bin/env python3
"""
Publishes sensor_msgs/CameraInfo for Isaac Sim's depth camera.

Isaac Sim does not publish camera_info alongside image_raw / depth/image_raw.
This node subscribes to the RGB image, mirrors its header stamp/frame_id, and
re-publishes a matching CameraInfo so that RTAB-Map's approx_sync can pair all
three topics.

Camera intrinsics: 1280x720, horizontal FoV = 90 degrees (default Isaac Sim).
Adjust HFOV_DEG below if your Isaac Sim camera uses a different field of view.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

HFOV_DEG = 82.6   # Isaac Sim camera horizontal field-of-view (degrees)

class CameraInfoPub(Node):
    def __init__(self):
        super().__init__('camera_info_pub')

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._pub = self.create_publisher(
            CameraInfo,
            '/camera/depth_camera/camera_info',
            qos,
        )

        self._sub = self.create_subscription(
            Image,
            '/camera/depth_camera/image_raw',
            self._image_cb,
            qos,
        )

        self._msg = None  # built once on first image
        self.get_logger().info('camera_info_pub started — waiting for first image...')

    # ------------------------------------------------------------------
    def _build_msg(self, width: int, height: int, frame_id: str) -> CameraInfo:
        hfov_rad = math.radians(HFOV_DEG)
        fx = (width / 2.0) / math.tan(hfov_rad / 2.0)
        fy = fx
        cx = width  / 2.0
        cy = height / 2.0

        msg = CameraInfo()
        msg.header.frame_id = frame_id
        msg.width  = width
        msg.height = height
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [fx, 0.0, cx,
                 0.0, fy, cy,
                 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0,
                 0.0, fy, cy, 0.0,
                 0.0, 0.0, 1.0, 0.0]
        self.get_logger().info(
            f'Camera intrinsics: {width}x{height}, '
            f'fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f} '
            f'(HFoV={HFOV_DEG}°)'
        )
        return msg

    def _image_cb(self, img_msg: Image):
        if self._msg is None:
            self._msg = self._build_msg(img_msg.width, img_msg.height,
                                        img_msg.header.frame_id)

        self._msg.header.stamp = img_msg.header.stamp
        self._pub.publish(self._msg)


def main():
    rclpy.init()
    node = CameraInfoPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
