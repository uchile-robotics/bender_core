#!/usr/bin/env python3
import odrive
from odrive.enums import *
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

class ODriveVelocityNode(Node):
    def __init__(self):
        super().__init__('odrive_velocity_node')

        self.get_logger().info("Buscando ODrive...")
        try:
            self.odrv = odrive.find_any()
            self.get_logger().info(f"Conectado a ODrive: {self.odrv.serial_number}")
            self.connected = True
        except Exception as e:
            self.get_logger().error("No se encontró ODrive")
            self.connected = False

        # Suscripción al tópico de velocidad
        self.subscription = self.create_subscription(
            Float64,
            'set_velocity',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        if not self.connected:
            return

        target_rad_s = msg.data

        # Enviar comando directo al ODrive
        self.odrv.axis0.controller.input_vel = target_rad_s

    def stop_motor(self):
        if self.connected:
            self.odrv.axis0.controller.input_vel = 0


def main(args=None):
    rclpy.init(args=args)
    node = ODriveVelocityNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motor() # Seguridad: detener al cerrar
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
