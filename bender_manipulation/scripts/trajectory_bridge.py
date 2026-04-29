#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray, Float64
import math

class ShoulderController(Node):
    def __init__(self):
        super().__init__('shoulder_controller')

        # --- CONFIGURACIÓN ---
        self.kp = 2.0         # Ganancia Proporcional (Corrige el error de posición)
        self.deadband_deg = 0.5 # Zona muerta pequeña
        self.max_vel_rad_s = 2.0 # Velocidad máxima absoluta permitida

        # --- ESTADO ---
        self.current_angle_deg = None # Dónde está realmente (Encoder)
        self.target_angle_deg = None  # Dónde debería estar (MoveIt)
        self.feedforward_vel = 0.0    # Velocidad que MoveIt dice que llevemos (Rad/s)

        # --- SUSCRIPCIONES ---
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

        # Loop de control rápido (20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Shoulder Controller (FF + P) iniciado.")

    def feedback_callback(self, msg):
        # El encoder manda grados
        self.current_angle_deg = msg.data

    def goal_callback(self, msg):
        """
        Recibe [Pos0...Pos7, Vel0...Vel7]
        Pos0 = Hombro Pos
        Vel0 está en index 8 (porque hay 8 posiciones antes)
        """
        # Verificamos que el mensaje tenga el tamaño esperado
        # Mínimo necesitamos llegar al índice 8
        if len(msg.data) >= 9:
            # 1. Posición Objetivo (Viene en Radianes -> Convertimos a Grados para comparar con encoder)
            target_rad = msg.data[0]
            self.target_angle_deg = math.degrees(target_rad)
            
            # 2. Velocidad Feedforward (Viene en Rad/s -> La usamos directo)
            # El índice es 8 porque: 6 joints brazo + 2 joints gripper = 8 posiciones (0-7).
            # Las velocidades empiezan en el índice 8.
            self.feedforward_vel = msg.data[8]
            
        elif len(msg.data) > 0:
            # Fallback por si llega un mensaje viejo sin velocidades
            target_rad = msg.data[0]
            self.target_angle_deg = math.degrees(target_rad)
            self.feedforward_vel = 0.0

    def control_loop(self):
        if self.current_angle_deg is None or self.target_angle_deg is None:
            return

        # --- 1. Calcular Error de Posición ---
        error_deg = self.target_angle_deg - self.current_angle_deg

        # --- 2. Calcular Velocidad de Corrección (Feedback) ---
        # Si el error es muy pequeño, ignoramos la corrección P para no oscilar,
        # PERO seguimos aplicando la velocidad de MoveIt si existe.
        if abs(error_deg) < self.deadband_deg:
            feedback_vel_rad_s = 0.0
        else:
            # P Controller: Grados de error * Kp
            # Convertimos el resultado a Radianes/s para poder sumarlo
            feedback_vel_deg_s = error_deg * self.kp
            feedback_vel_rad_s = math.radians(feedback_vel_deg_s)

        # --- 3. FUSIÓN: Feedforward + Feedback ---
        # Esta es la magia: El motor se mueve porque MoveIt lo dice (FF), 
        # y si se atrasa, el error (FB) le da un empujón extra.
        total_vel_rad_s = self.feedforward_vel + feedback_vel_rad_s

        # --- 4. Saturación (Seguridad) ---
        total_vel_rad_s = max(min(total_vel_rad_s, self.max_vel_rad_s), -self.max_vel_rad_s)

        # --- 5. Enviar ---
        self.send_velocity(total_vel_rad_s)

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
