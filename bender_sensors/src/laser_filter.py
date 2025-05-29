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
        self._scan_data = None
        self._angle_increment = None
        self._angle_max = None
        self._angle_min = None
        
    def _callback(self,msg:LaserScan):
        self._angle_max       = msg.angle_max
        self._angle_min       = msg.angle_min
        self._scan_data       = msg.ranges
        self._angle_increment = msg.angle_increment
        
    def _scan_publisher(self):
        # calculate the index of the self._scan_data array
        # corresponding to the given angle
        
        msg = LaserScan()
        
        self._filtered_scan_pub.publish(msg)