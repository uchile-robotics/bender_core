#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion, Point
from sensor_msgs.msg import Joy
from tf_transformations import quaternion_from_euler

class GoalPosePublisher(Node):

    def __init__(self, x, y, theta):
        super().__init__('goal_pose_publisher')

        self._goal_pose = (x, y, theta)
        self._goal_sent = False
        self._origin_sent = False

        self.sub_joy = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.publisher_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.get_logger().info("Usa Start, Select, D-pad y X para enviar metas")
        
        self.get_logger().info("D-pad arriba -> (2.95, 5.64, 0)")
        self.get_logger().info("D-pad bajo -> (3.36, 2.28, 2.237)")
        self.get_logger().info("D-pad izquierda -> (0.27, 4.97, -2.375)")
        self.get_logger().info("D-pad derecha -> (1.61, 2.14, 1.57)")
        self.get_logger().info("Botón X -> (3.41, 0.21, -1.57)")

    def joy_callback(self, msg: Joy):
        start_pressed = msg.buttons[7] == 1
        select_pressed = msg.buttons[6] == 1

        if start_pressed and not self._goal_sent:
            self.publish_goal(*self._goal_pose, label="meta definida")
            self._goal_sent = True
            self._origin_sent = False  
        elif select_pressed and not self._origin_sent:
            self.publish_goal(0.0, 0.0, 0.0, label="origen")
            self._origin_sent = True
            self._goal_sent = False

        elif msg.buttons[11] == 1:  # D-pad arriba
            self.publish_goal(2.95, 5.64, 0, label="D-pad ↑")
        elif msg.buttons[12] == 1:  # D-pad abajo
            self.publish_goal(3.36, 2.28, 2.237, label="D-pad ↓")
        elif msg.buttons[13] == 1:  # D-pad izquierda
            self.publish_goal(0.27, 4.97, -2.375, label="D-pad ←")
        elif msg.buttons[14] == 1:  # D-pad derecha
            self.publish_goal(1.61, 2.14, 1.57, label="D-pad →")
        elif msg.buttons[2] == 1:  # Botón X
            self.publish_goal(3.41, 0.21, -1.57, label="botón X")

    def publish_goal(self, x, y, theta, label=""):
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = "odom"

        orientation = Quaternion()
        orientation.x, orientation.y, orientation.z, orientation.w = quaternion_from_euler(0, 0, theta)

        position = Point()
        position.x = x
        position.y = y
        position.z = 0.0

        goal_pose.pose.position = position
        goal_pose.pose.orientation = orientation

        self.publisher_goal.publish(goal_pose)
        self.get_logger().info(f"Pose publicada hacia {label} → x={x:.2f}, y={y:.2f}, θ={theta:.2f}")

def main(args=None):
    rclpy.init(args=args)
    x = 3.38
    y = 3.48
    theta = 2.0
    node = GoalPosePublisher(x, y, theta)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
