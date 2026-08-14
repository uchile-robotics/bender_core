#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Re-timestampa la nube de puntos de la cámara real al reloj de la simulación.

La RealSense es hardware real: sus mensajes llegan con reloj de pared (epoch
del sistema). El resto del árbol TF (robot_state_publisher, controladores)
usa el tiempo simulado de Gazebo (use_sim_time=True, requerido por
grasp_pick_node/trajectory_execution_manager). Como son dos dominios de
tiempo distintos, tf2 no puede transformar la nube al buscarla en el instante
de su header original (le falta esa época en el buffer).

Este nodo solo cambia el header.stamp al 'ahora' de su propio reloj (que sí
es tiempo simulado, porque se lanza con use_sim_time:=true) y republica en
otro tópico. No toca el contenido de la nube ni el frame_id.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class CameraTimeBridgeNode(Node):
    def __init__(self):
        super().__init__('camera_time_bridge_node')

        self.declare_parameter('input_topic', '/camera/depth/color/points')
        self.declare_parameter('output_topic', '/camera/depth/color/points_stamped')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.pub = self.create_publisher(PointCloud2, output_topic, 10)
        self.create_subscription(PointCloud2, input_topic, self._cb, 10)

        self.get_logger().info(f"Re-timestampando '{input_topic}' -> '{output_topic}'")

    def _cb(self, msg: PointCloud2):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraTimeBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
