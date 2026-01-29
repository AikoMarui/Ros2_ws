#!/usr/bin/env python
import rclpy
from rclpy.node import Node

class ObstacleDetectorNode(Node):
    def __init__(self, node_name="obstacle_detector_node"):
        self._node_name = node_name
        super().__init__(self._node_name)
        self.get_logger().info(self._node_name + " Ready...")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()