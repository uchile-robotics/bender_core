#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import copy


class LaserFilter(Node):
    def __init__(self):
        super().__init__('laser_filter_node')
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.scan_callback,
            10
        )
        self.scan_pub = self.create_publisher(
            LaserScan,
            '/scan',
            10
        )
        self.half_window_size = 100 

    def scan_callback(self, msg: LaserScan):
        filtered_scan = copy.deepcopy(msg)
        n = len(filtered_scan.ranges)
        center = n // 2
        start = max(0, center - self.half_window_size)
        end = min(n, center + self.half_window_size)

        for i in range(start, end):
            filtered_scan.ranges[i] = float('inf')

        self.scan_pub.publish(filtered_scan)


def main(args=None):
    rclpy.init(args=args)
    node = LaserFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
