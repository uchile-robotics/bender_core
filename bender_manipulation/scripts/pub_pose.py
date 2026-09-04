#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

# ==============================================================================
# CONFIGURACIÓN DE LA POSE A PROBAR
# ==============================================================================
FRAME_ID = "base_link"
TOPIC_NAME = "/grasp_candidate_pose"

# Modifica los valores de x, y, z según la posición que quieras probar
POSITION = {
    "x": 0.5,
    "y": -0.2,
    "z": 0.4
}

# Orientación extraída de tu log
ORIENTATION = {
    "x": -0.609510162214212,
    "y": 0.26277153622129734,
    "z": -0.40581920107560043,
    "w": 0.6282987012145304
}
# ==============================================================================


class TestPosePublisher(Node):
    def __init__(self):
        super().__init__("test_pose_publisher")
        self.pub = self.create_publisher(PoseStamped, TOPIC_NAME, 10)

        # Pausa para dar tiempo a la sincronización de ROS 2 DDS con el suscriptor
        time.sleep(1.0)
        self.publish_pose()

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.frame_id = FRAME_ID

        msg.pose.position.x = float(POSITION["x"])
        msg.pose.position.y = float(POSITION["y"])
        msg.pose.position.z = float(POSITION["z"])

        msg.pose.orientation.x = float(ORIENTATION["x"])
        msg.pose.orientation.y = float(ORIENTATION["y"])
        msg.pose.orientation.z = float(ORIENTATION["z"])
        msg.pose.orientation.w = float(ORIENTATION["w"])

        # Se publica varias veces seguidas para asegurar que el nodo receptor lo capture
        for _ in range(5):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            time.sleep(0.1)

        self.get_logger().info(f"Pose enviada a {TOPIC_NAME}:")
        self.get_logger().info(f" Posición: x={msg.pose.position.x:.4f}, y={msg.pose.position.y:.4f}, z={msg.pose.position.z:.4f}")
        self.get_logger().info(f" Orientación: x={msg.pose.orientation.x:.4f}, y={msg.pose.orientation.y:.4f}, z={msg.pose.orientation.z:.4f}, w={msg.pose.orientation.w:.4f}")


def main():
    rclpy.init()
    node = TestPosePublisher()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
