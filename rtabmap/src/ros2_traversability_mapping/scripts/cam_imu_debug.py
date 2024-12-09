#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu, Image


class CamImuDebugNode(Node):

    def __init__(self):
        super().__init__('cam_imu_debug_node')
        self._last_camera_msg_sec = None
        self._last_camera_msg_nanosec = None
        self._last_imu_msg_sec = None
        self._last_imu_msg_nanosec = None
        
        # Subscribers
        self.camera_sub = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.camera_cb,
            10)
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/zed/zed_node/imu/data',
            self.imu_cb,
            10)
        
        
    def camera_cb(self, msg):
        self._last_camera_msg_sec = msg.header.stamp.sec
        self._last_camera_msg_nanosec = msg.header.stamp.nanosec
        if self._last_imu_msg_sec is not None and self._last_imu_msg_nanosec is not None:
            time_diff_sec = self._last_camera_msg_sec - self._last_imu_msg_sec
            time_diff_nanosec = self._last_camera_msg_nanosec - self._last_imu_msg_nanosec
            time_diff_ms = time_diff_sec * 1000 + time_diff_nanosec / 1e6
            self.get_logger().info(f'Difference between last camera and imu timestamps: {time_diff_ms} ms')
        # Process message from topic_1

    def imu_cb(self, msg):
        self._last_imu_msg_sec = msg.header.stamp.sec
        self._last_imu_msg_nanosec = msg.header.stamp.nanosec


def main(args=None):
    rclpy.init(args=args)
    node = CamImuDebugNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()