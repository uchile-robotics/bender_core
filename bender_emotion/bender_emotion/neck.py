#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from dynamixel_sdk import *  
from std_msgs.msg import Float64MultiArray
import math 

class DynamixelJointPublisher(Node):
    def __init__(self):
        super().__init__('dynamixel_joint_publisher')

        # Crear publicador
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz
        self.counter = 0

        # Subscriptor a posiciones deseadas
        self.position_sub = self.create_subscription(
            Float64MultiArray,
            'goal_pos',
            self.position_callback,
            10
        )

        # Configuración general
        self.PROTOCOL_VERSION = 1.0
        self.BAUDRATE = 1000000
        self.DEVICE_NAME = '/dev/ttyUSB1'

        self.DXL_IDS = [19,20,21]

        # Direcciones de los RX-28, RX-64 y MX-106 
        self.ADDR_PRESENT_POSITION = 36
        self.ADDR_PRESENT_SPEED = 38
        self.ADDR_TORQUE_ENABLE = 24

        self.ADDR_GOAL_POSITION = 30
        self.ADDR_MOVING_SPEED = 32


        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0

        # Tamaños de datos
        self.LEN_POSITION = 2
        self.LEN_SPEED = 2

        #self.offsets = [190,198,642]
        self.offsets = [0,0,0]
        # Inicializa puerto y handler
        self.portHandler = PortHandler(self.DEVICE_NAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        if not self.portHandler.openPort():
            print("Error al abrir el puerto")
            exit()

        if not self.portHandler.setBaudRate(self.BAUDRATE):
            print("Error al establecer la velocidad de baudios")
            exit()

        # Habilita el torque (opcional)
        for dxl_id in self.DXL_IDS:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)

        print("Iniciando lectura rápida...")

    def publish_joint_states(self):
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        # Lista completa de joints del robot (según URDF o echo)
        all_joint_names = [
            'l2l_to_l1l', 'l3l_to_l2l', 'l4l_to_l3l', 'l5l_to_l4l', 'l6l_to_l5l',
            'l2r_to_l1r', 'l3r_to_l2r', 'l4r_to_l3r', 'l5r_to_l4r', 'l6r_to_l5r',
            'lh2_to_lh1', 'g2ra_to_g1r', 'g2rb_to_g1r', 'g2lb_to_g1l', 'g2la_to_g1l',
            'l1r_to_base_link', 'l1l_to_base_link', 'l1h_to_base_link',
            'neck_1', 'neck_2'  # Agrega los joints que controlas directamente
        ]

        # Inicializa todos los joints en 0.0
        joint_positions = {name: 0.0 for name in all_joint_names}
        joint_velocities = {name: 0.0 for name in all_joint_names}

        # Mapeo de IDs a nombres de joints
        joint_name_map = {
            19: "l1h_to_base_link",
            20: "lh2_to_lh1",
            21: "neck_2"
        }

        for dxl_id in self.DXL_IDS:
            if dxl_id not in joint_name_map:
                continue

            data, result, error = self.packetHandler.readTxRx(
                self.portHandler, dxl_id, self.ADDR_PRESENT_POSITION, 4)

            if result != COMM_SUCCESS or error != 0:
                self.get_logger().warn(f"⚠️ Fallo al leer motor {dxl_id}")
                continue

            offset = self.offsets[dxl_id - 20]
            resolution = 1023
            max_radians = math.radians(300)

            pos_raw = data[0] + (data[1] << 8)
            vel_raw = data[2] + (data[3] << 8)

            pos_rad = ((pos_raw - offset) / resolution) * max_radians

            if vel_raw > 1023:
                velocity = (vel_raw - 1024) * -1.0
            else:
                velocity = vel_raw

            joint_name = joint_name_map[dxl_id]
            joint_positions[joint_name] = pos_rad
            joint_velocities[joint_name] = float(velocity)

        # Construir mensaje final
        joint_state_msg.name = all_joint_names
        joint_state_msg.position = [joint_positions[name] for name in all_joint_names]
        joint_state_msg.velocity = [joint_velocities[name] for name in all_joint_names]
        joint_state_msg.effort = [0.0] * len(all_joint_names)

        self.joint_pub.publish(joint_state_msg)

        self.counter += 1
        if self.counter % 10 == 0:  # Imprime cada segundo
            for dxl_id in self.DXL_IDS:
                joint_name = joint_name_map.get(dxl_id)
                if joint_name:
                    pos_rad = joint_positions[joint_name]
                    pos_deg = math.degrees(pos_rad)  # CONVERSIÓN A GRADOS
                    self.get_logger().info(f"{joint_name} ({dxl_id}): {pos_rad:.4f} rad / {pos_deg:.2f}°")



    def position_callback(self, msg: Float64MultiArray):
        if len(msg.data) != 3:
            self.get_logger().warn("Se esperaban exactamente 3 posiciones (en radianes)")
            return

        VELOCIDAD_LENTA = 175

        for i, dxl_id in enumerate(self.DXL_IDS):
            degrees = msg.data[i]
            radians = math.radians(degrees)
            
            resolution = 1023
            max_radians = math.radians(300)  # ≈ 5.23599

            # Calcular posición cruda
            pos_raw = int((radians / max_radians) * resolution)

            # Aplicar offset
            pos_raw += self.offsets[i]

            # Limitar al rango válido
            pos_raw = max(0, min(resolution, pos_raw))

            # 1. Establecer velocidad lenta
            result_speed, error_speed = self.packetHandler.write2ByteTxRx(
                self.portHandler, dxl_id, 32, VELOCIDAD_LENTA)
            
            if result_speed != COMM_SUCCESS or error_speed != 0:
                self.get_logger().warn(f"⚠️ Error al fijar velocidad del motor {dxl_id}")

            # Enviar posición
            result, error = self.packetHandler.write2ByteTxRx(
                self.portHandler, dxl_id, self.ADDR_GOAL_POSITION, pos_raw)

            if result != COMM_SUCCESS or error != 0:
                self.get_logger().warn(f"Error al mover motor {dxl_id} a {pos_raw}")


    def destroy_node(self):
        pass


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


main()
