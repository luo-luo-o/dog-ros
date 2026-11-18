"""启动实物占位节点。"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def main():
    rclpy.init()
    node = Node("dog_hardware")
    node.get_logger().warning("Hardware driver is pending. IK calibration, servo protocol, feedback and control rate are required.")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
