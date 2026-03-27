#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time


class SaludoNode(Node):
    def __init__(self):
        super().__init__('saludo_node')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'goal_pos', 10)

        # Lista de configuraciones (8 valores: hombro + 7 Dynamixel)
        self.configs = [
            [2.0, 0.0, 0.2, 1.57, 0.0, 0.25, 0.0, 0.0],   # posición base
            [2.0, 0.1, 0.2, 1.57, 0.0, 0.25, 0.0, 0.0],   # mueve un poco
            [2.0, -0.1, 0.2, 1.57, 0.0, 0.25, 0.0, 0.0],  # mueve otro poco
        ]

        # Timer para empezar la secuencia
        self.timer = self.create_timer(2.0, self.sequence)
        self.step = 0
        self.saludando = False

    def sequence(self):
        if self.step < len(self.configs):
            cfg = self.configs[self.step]
            self.publish_config(cfg)
            self.get_logger().info(f"Posición {self.step+1}: {cfg}")
            self.step += 1
        else:
            if not self.saludando:
                self.get_logger().info("Comenzando saludo con el gripper...")
                self.saludando = True
                self.saludo()
                self.get_logger().info("Secuencia terminada.")
                self.timer.cancel()

    def publish_config(self, cfg):
        msg = Float64MultiArray()
        msg.data = cfg
        self.publisher_.publish(msg)

    def saludo(self):
        # abre y cierra gripper varias veces (últimos dos valores son el gripper)
        for i in range(6):
            open_cfg = [2.0, 0.0, 0.2, 1.57, -0.2, 0.25, 0.5, -0.5]  # abre
            close_cfg = [2.0, 0.0, 0.2, 1.57, 0.2, 0.1, -0.2, 0.0] # cierra

            self.publish_config(open_cfg)
            self.get_logger().info("Gripper abierto 👋")
            time.sleep(1.0)

            self.publish_config(close_cfg)
            self.get_logger().info("Gripper cerrado ✊")
            time.sleep(1.0)


def main(args=None):
    rclpy.init(args=args)
    node = SaludoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
