#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Bool  # <--- IMPORTANTE: Importar Bool
from ament_index_python.packages import get_package_share_directory

# Importar tu clase Commander corregida
from dynamixel_command import DynamixelCommander

class DynamixelJointPublisher(Node):
    def __init__(self):
        super().__init__('dynamixel_joint_publisher')

        # --- 1. Subscriptor de Trayectoria del Brazo (Goal) ---
        self.position_sub = self.create_subscription(
            Float64MultiArray,
            'goal_pos',
            self.position_callback,
            10
        )
        
        # --- 2. Nuevo Subscriptor para el Gripper (Bool) ---
        # True = Cerrar, False = Abrir
        self.gripper_sub = self.create_subscription(
            Bool,
            '/gripper/close',
            self.gripper_callback,
            10
        )

        # --- 3. Publicador de Estados y Timers ---
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        # --- 4. Subscripción al Hombro ---
        self.shoulder_sub = self.create_subscription(
            Float32,
            '/shoulder/angle',
            self.shoulder_callback,
            10
        )

        # Configuración del Hombro
        self.joint_name = "l1r_to_base_link"
        self.declare_parameter('shoulder_in_degrees', True)
        self._shoulder_in_degrees = self.get_parameter('shoulder_in_degrees').get_parameter_value().bool_value
        self._shoulder_angle_rad = 0.0

        # --- 5. Inicialización de Hardware ---
        pkg_share = get_package_share_directory('bender_manipulation')
        config_path = os.path.join(pkg_share, 'config', 'params.yaml')
        self.get_logger().info(f'Usando config: {config_path}')

        try:
            self.dynamixel = DynamixelCommander(config_path)
        except Exception as e:
            self.get_logger().error(f"Error fatal iniciando DynamixelCommander: {e}")
            raise e

        # --- 6. ESTADO INICIAL DEL ROBOT ---
        # Guardamos la posición de los 5 motores del brazo (inicialmente en 0 o home)
        self.last_arm_pos = [0.0, 0.0, 0.0, 0.0, 0.0] 
        
        # Guardamos la posición del gripper (Por defecto ABIERTO: 0.6, -0.6)
        self.last_gripper_pos = [0.6, -0.6]
        
        # Mandar comando inicial para asegurar que la pinza se abra al iniciar el programa
        self.get_logger().info("Inicializando Gripper en posición ABIERTA (0.6, -0.6)")
        initial_joints = self.last_arm_pos + self.last_gripper_pos
        # Usamos una velocidad moderada para el inicio
        initial_vels = [0.5] * 7 
        self.dynamixel.set_joints_with_velocity(initial_joints, initial_vels)

    def publish_joint_states(self):
        """Lee los motores y publica en /joint_states"""
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        # Mapear IDs a nombres de joints (Dynamixel)
        joint_name_map = {
            0: "l2r_to_l1r",
            1: "l3r_to_l2r",
            2: "l4r_to_l3r",
            3: "l5r_to_l4r",
            4: "l6r_to_l5r",
            5: "g2ra_to_g1r",
            6: "g2rb_to_g1r"
        }

        names = [self.joint_name]
        positions = [self._shoulder_angle_rad]
        velocities = [0.0]

        try:
            pos, vel = self.dynamixel.get_joints_data()
        except Exception as e:
            self.get_logger().warn(f"Error leyendo joints: {e}")
            return

        for i in range(len(pos)):
            # Determinar resolución por tipo
            if i in [0, 2]:  # Asumiendo MX-106 o similar
                resolution = 4095
                max_radians = 2 * math.pi
            else:            # Asumiendo RX-28
                resolution = 1023
                max_radians = math.radians(300)

            pos_raw = pos[i]
            vel_raw = vel[i]

            pos_rad = (pos_raw / resolution) * max_radians
            pos_rad = (pos_rad + math.pi) % (2 * math.pi) - math.pi

            if vel_raw > 1023:
                velocity = (vel_raw - 1024) * -1
            else:
                velocity = vel_raw

            if i in joint_name_map:
                names.append(joint_name_map[i])
                positions.append(pos_rad)
                velocities.append(velocity)

        joint_state_msg.name = names
        joint_state_msg.position = positions
        joint_state_msg.velocity = velocities
        joint_state_msg.effort = [0.0] * len(names)

        self.joint_pub.publish(joint_state_msg)

    def shoulder_callback(self, msg: Float32):
        angle = float(msg.data)
        if self._shoulder_in_degrees:
            angle = math.radians(angle)
        angle = (angle + math.pi) % (2 * math.pi) - math.pi
        self._shoulder_angle_rad = angle

    def gripper_callback(self, msg: Bool):
        """
        Callback para el tópico booleano del gripper.
        True  -> Cerrar (0.325, -0.325)
        False -> Abrir  (0.6, -0.6)
        """
        if msg.data:
            self.get_logger().info("Orden recibida: CERRAR Gripper")
            target_gripper = [0.225, -0.225]
        else:
            self.get_logger().info("Orden recibida: ABRIR Gripper")
            target_gripper = [0.6, -0.6]

        # 1. Actualizamos la memoria interna del gripper
        self.last_gripper_pos = target_gripper

        # 2. Construimos el comando completo:
        #    (Última posición conocida del brazo) + (Nueva posición del gripper)
        full_pose = self.last_arm_pos + self.last_gripper_pos
        
        # 3. Velocidad para el movimiento del gripper (y mantener brazo quieto)
        #    Podemos usar una velocidad fija segura para la acción de pinza
        gripper_action_vel = [0.5] * 7

        # 4. Enviar al hardware
        self.dynamixel.set_joints_with_velocity(full_pose, gripper_action_vel)

    def position_callback(self, msg: Float64MultiArray):
        """
        Recibe trayectoria del brazo desde MoveIt.
        Estructura esperada original: [Hombro, Arm1...Arm5, Grip1, Grip2, Vel_Hombro...]
        
        NUEVA LÓGICA:
        - Leemos Arm1...Arm5.
        - IGNORAMOS Grip1 y Grip2 que vienen de MoveIt.
        - Usamos self.last_gripper_pos (controlado por el Bool).
        """
        num_arm_joints = 5 # Motores Dynamixel dedicados al brazo (sin gripper)
        
        # Validación básica de longitud (debe traer al menos hombro + 5 brazo)
        if len(msg.data) < 6:
            return

        # --- Extracción de Posiciones ---
        # Índice 0: Hombro (Ignorado aquí)
        # Índice 1 a 5: Motores del brazo (Arm1 a Arm5)
        # Índice 6 a 7: Grippers de MoveIt (LOS IGNORAMOS)
        
        # Extraemos solo los 5 del brazo
        current_arm_cmd = list(msg.data[1 : 1 + num_arm_joints])
        
        # Actualizamos la memoria interna del brazo
        self.last_arm_pos = current_arm_cmd

        # --- Extracción de Velocidades ---
        # MoveIt manda: [Pos Hombro, Pos DXL(7), Vel Hombro, Vel DXL(7)]
        # Vel Hombro está en index: 1 + 7 = 8
        # Vel Arm1 empieza en: 9
        start_vel_index = 9 
        
        # Extraemos velocidades del brazo si existen en el mensaje
        if len(msg.data) >= start_vel_index + num_arm_joints:
            current_arm_vel = list(msg.data[start_vel_index : start_vel_index + num_arm_joints])
        else:
            # Fallback si no hay velocidades
            current_arm_vel = [0.5] * num_arm_joints

        # --- FUSIÓN DE COMANDOS ---
        # Combinamos: [Brazo Nuevo] + [Gripper Guardado]
        final_positions = self.last_arm_pos + self.last_gripper_pos
        
        # Velocidades: [Velocidad Brazo Nueva] + [Velocidad Gripper 0]
        # Ponemos velocidad 0 (o muy baja) al gripper para que mantenga fuerza/posición
        # pero sin intentar moverse si ya está ahí.
        final_velocities = current_arm_vel + [0.5, 0.5] 

        # Enviar al commander
        self.dynamixel.set_joints_with_velocity(final_positions, final_velocities)

    def destroy_node(self):
        try:
            self.dynamixel.shutdown()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelJointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()