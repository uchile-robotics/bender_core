#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LaserFilter(Node):
    def __init__(self):
        super().__init__('laser_filter_node')
        self._scan_sub = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self._callback,
            10 
        )
        self._filtered_scan_pub = self.create_publisher(
            LaserScan,
            '/scan',
            10
        )
        self._filtered_fov = (3.8397,2.4435)
        
        self._scan_data = None
        self._angle_increment = None
        self._angle_max = None  
        self._angle_min = None
        
    def _callback(self,msg):
        self._angle_increment = msg.angle_increment
        filtered_fov_max, filtered_fov_min = self._filtered_fov
        range_index_max, range_index_min = (
            int(filtered_fov_max // self._angle_increment),
            int(filtered_fov_min // self._angle_increment))
        
        # self.get_logger().info(f'\n{range_index_max = }, \n{range_index_min = }')
        for i in range(range_index_min,range_index_max):
           msg.ranges[i] = float('inf')
        self._filtered_scan_pub.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    lf = LaserFilter()
    try:
        rclpy.spin(lf)
        lf.get_logger().info('Laser filter ran succesfully')
    except KeyboardInterrupt:
        lf.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()