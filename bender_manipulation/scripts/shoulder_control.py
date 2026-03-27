#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray, Float64
import math

class ShoulderController(Node):
    def __init__(self):
        super().__init__('shoulder_controller')

        # --- CONFIGURACIÓN ---
        self.kp = 8.0   # Ganancia Proporcional (Ajustar según fuerza requerida)
        self.deadband_deg = 1.0  # Zona muerta en grados (para evitar oscilaciones)
        self.max_vel_rad_s = 7.0 # Velocidad máxima de seguridad (rad/s)

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

    def feedback_callback(self, msg):
        self.current_angle_deg = msg.data

    def goal_callback(self, msg):

        if len(msg.data) > 0:
            target_rad = msg.data[0]
            # Convertimos a grados para trabajar en la misma unidad que el encoder
            self.target_angle_deg = math.degrees(target_rad)


    def control_loop(self):
        if self.current_angle_deg is None or self.target_angle_deg is None:
            return

        # 1. Calcular Error
        error = self.target_angle_deg - self.current_angle_deg

        # 2. Zona muerta (Si estamos cerca, paramos para no vibrar)
        if abs(error) < self.deadband_deg:
            self.send_velocity(0.0)
            return

        # 3. Control Proporcional (P)
        # Velocidad = Error * Kp
        # El resultado estará en grados/segundo
        vel_deg_s = error * self.kp

        # 4. Convertir a Radianes/segundo (que es lo que pide el ODrive)
        vel_rad_s = math.radians(vel_deg_s)

        # 5. Saturación (Seguridad)
        # Limitamos la velocidad máxima
        vel_rad_s = max(min(vel_rad_s, self.max_vel_rad_s), -self.max_vel_rad_s)

        # 6. Enviar comando
        self.send_velocity(vel_rad_s)

    def send_velocity(self, velocity):
        msg = Float64()
        msg.data = velocity
        self.pub_vel.publish(msg)

    def destroy_node(self):
        # Parar motor al salir
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