#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray, Float64
import math


class ShoulderController(Node):
    def __init__(self):
        super().__init__('shoulder_controller')

        # --- CONFIGURACIÓN ---
        self.kp = 5.0
        self.deadband_deg = 1.0
        self.gear_ratio = 250.0
        self.max_vel_motor_turns_s = 10.0

        # Límites articulares válidos en [0, 360)
        self.max_forward_deg = 95.0
        self.max_backward_deg = 315.0   # equivale a -45°

        self.current_angle_deg = None
        self.target_angle_deg = None

        self.sub_feedback = self.create_subscription(
            Float32,
            '/shoulder/angle',
            self.feedback_callback,
            10
        )

        self.sub_goal = self.create_subscription(
            Float64MultiArray,
            '/goal_pos',
            self.goal_callback,
            10
        )

        self.pub_vel = self.create_publisher(Float64, '/set_velocity', 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Shoulder Controller iniciado. Esperando datos...")

    def normalize_angle_deg(self, angle_deg):
        return angle_deg % 360.0

    def is_valid_angle(self, angle_deg):
        angle_deg = self.normalize_angle_deg(angle_deg)
        return (0.0 <= angle_deg <= self.max_forward_deg) or (self.max_backward_deg <= angle_deg < 360.0)

    def shortest_angular_error_deg(self, target_deg, current_deg):
        """
        Error angular mínimo con wrap-around.
        Resultado en [-180, 180).
        """
        return (target_deg - current_deg + 180.0) % 360.0 - 180.0

    def feedback_callback(self, msg):
        self.current_angle_deg = self.normalize_angle_deg(msg.data)

    def goal_callback(self, msg):
        if len(msg.data) > 0:
            target_rad = msg.data[0]
            target_deg = self.normalize_angle_deg(math.degrees(target_rad))

            # Si el target cae en la zona prohibida [95, 315], lo mandamos a 0°
            if not self.is_valid_angle(target_deg):
                self.get_logger().warn(
                    f"Target {target_deg:.2f}° fuera de rango permitido. Se reemplaza por 0°."
                )
                target_deg = 0.0

            self.target_angle_deg = target_deg

    def control_loop(self):
        if self.current_angle_deg is None or self.target_angle_deg is None:
            return

        # Seguridad: si la lectura actual está en la zona prohibida, detener
        if not self.is_valid_angle(self.current_angle_deg):
            self.get_logger().error(
                f"Ángulo actual inválido: {self.current_angle_deg:.2f}°. Deteniendo motor."
            )
            self.send_velocity(0.0)
            return

        # Error angular mínimo considerando wrap-around
        error_deg = self.shortest_angular_error_deg(
            self.target_angle_deg,
            self.current_angle_deg
        )

        # Zona muerta
        if abs(error_deg) < self.deadband_deg:
            self.send_velocity(0.0)
            return

        # Control P -> velocidad deseada del brazo [deg/s]
        vel_arm_deg_s = self.kp * error_deg

        # Convertir velocidad del brazo a vueltas/s del motor
        vel_motor_turns_s = (vel_arm_deg_s / 360.0) * self.gear_ratio

        # Saturación de seguridad
        vel_motor_turns_s = max(
            min(vel_motor_turns_s, self.max_vel_motor_turns_s),
            -self.max_vel_motor_turns_s
        )

        self.send_velocity(vel_motor_turns_s)

    def send_velocity(self, velocity):
        msg = Float64()
        msg.data = velocity
        self.pub_vel.publish(msg)

    def destroy_node(self):
        self.send_velocity(0.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ShoulderController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
