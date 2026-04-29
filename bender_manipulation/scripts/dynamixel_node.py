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
from std_msgs.msg import Bool
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

from dynamixel_command import DynamixelCommander


class DynamixelJointPublisher(Node):
    def __init__(self):
        super().__init__('dynamixel_joint_publisher')

        # -------- Parámetros de seguridad --------
        self.load_limit_percent = 80.0
        self.overload_active = False

        # -------- Subscripciones --------
        self.position_sub = self.create_subscription(
            Float64MultiArray,
            'goal_pos',
            self.position_callback,
            10
        )

        self.gripper_sub = self.create_subscription(
            Bool,
            '/gripper/close',
            self.gripper_callback,
            10
        )

        self.shoulder_sub = self.create_subscription(
            Float32,
            '/shoulder/angle',
            self.shoulder_callback,
            10
        )

        # -------- Publicadores --------
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # carga de TODOS los motores (7 Dynamixel)
        self.load_all_pub = self.create_publisher(
            Float64MultiArray,
            '/dynamixel/present_load_all',
            10
        )

        # carga solo del gripper [motor_5, motor_6]
        self.load_gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/gripper/present_load',
            10
        )

        # estado de protección
        self.overload_pub = self.create_publisher(
            Bool,
            '/gripper/overload',
            10
        )

        self.overload_text_pub = self.create_publisher(
            String,
            '/gripper/overload_text',
            10
        )

        # -------- Timer --------
        self.timer = self.create_timer(0.05, self.publish_joint_states_and_loads)

        # -------- Hombro --------
        self.joint_name = "l1r_to_base_link"
        self.declare_parameter('shoulder_in_degrees', True)
        self._shoulder_in_degrees = self.get_parameter(
            'shoulder_in_degrees'
        ).get_parameter_value().bool_value
        self._shoulder_angle_rad = 0.0

        # -------- Hardware --------
        pkg_share = get_package_share_directory('bender_manipulation')
        config_path = os.path.join(pkg_share, 'config', 'params.yaml')
        self.get_logger().info(f'Usando config: {config_path}')

        try:
            self.dynamixel = DynamixelCommander(config_path)
        except Exception as e:
            self.get_logger().error(f"Error fatal iniciando DynamixelCommander: {e}")
            raise e

        # -------- Estado interno --------
        self.last_arm_pos = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.last_gripper_pos = [0.6, -0.6]

        self.get_logger().info("Inicializando Gripper en posición ABIERTA (0.6, -0.6)")
        initial_joints = self.last_arm_pos + self.last_gripper_pos
        initial_vels = [0.5] * 7
        self.dynamixel.set_joints_with_velocity(initial_joints, initial_vels)

    def publish_joint_states_and_loads(self):
        """
        Lee joints, publica joint_states, publica carga y activa protección
        si cualquier motor supera 80%.
        """
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

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
        efforts = [0.0]

        try:
            pos, vel, loads_signed, loads_abs = self.dynamixel.get_joints_data()
        except Exception as e:
            self.get_logger().warn(f"Error leyendo joints/load: {e}")
            return

        # -------- Publicar cargas --------
        all_load_msg = Float64MultiArray()
        all_load_msg.data = loads_abs
        self.load_all_pub.publish(all_load_msg)

        gripper_load_msg = Float64MultiArray()
        if len(loads_abs) >= 7:
            gripper_load_msg.data = [loads_abs[5], loads_abs[6]]
        else:
            gripper_load_msg.data = []
        self.load_gripper_pub.publish(gripper_load_msg)

        # -------- Protección por sobrecarga --------
        max_load = max(loads_abs) if loads_abs else 0.0
        overload_now = max_load >= self.load_limit_percent

        overload_bool = Bool()
        overload_bool.data = overload_now
        self.overload_pub.publish(overload_bool)

        if overload_now and not self.overload_active:
            self.overload_active = True
            self.get_logger().warn(
                f"SOBRECARGA detectada: {max_load:.1f}% >= {self.load_limit_percent:.1f}%. "
                "Congelando posición actual."
            )

            try:
                current_angles = self.dynamixel.get_current_joint_angles_rad()
                self.last_arm_pos = current_angles[:5]
                self.last_gripper_pos = current_angles[5:7]
                self.dynamixel.hold_current_position(hold_speed=0.2)
            except Exception as e:
                self.get_logger().error(f"No se pudo congelar pose actual: {e}")

            txt = String()
            txt.data = f"OVERLOAD: max_load={max_load:.1f}%"
            self.overload_text_pub.publish(txt)

        elif not overload_now and self.overload_active:
            self.overload_active = False
            self.get_logger().info("Sobrecarga liberada. Se aceptan comandos nuevamente.")

            txt = String()
            txt.data = "OVERLOAD CLEARED"
            self.overload_text_pub.publish(txt)

        # -------- Publicar joint_states --------
        for i in range(len(pos)):
            if i in [0, 2]:
                resolution = 4095
                max_radians = 2 * math.pi
            else:
                resolution = 1023
                max_radians = math.radians(300)

            pos_raw = pos[i]
            vel_raw = vel[i]

            pos_rad = (pos_raw / resolution) * max_radians
            pos_rad = (pos_rad + math.pi) % (2 * math.pi) - math.pi

            if i in joint_name_map:
                names.append(joint_name_map[i])
                positions.append(pos_rad)
                velocities.append(float(vel_raw))
                efforts.append(loads_signed[i] if i < len(loads_signed) else 0.0)

        joint_state_msg.name = names
        joint_state_msg.position = positions
        joint_state_msg.velocity = velocities
        joint_state_msg.effort = efforts
        self.joint_pub.publish(joint_state_msg)

    def shoulder_callback(self, msg: Float32):
        angle = float(msg.data)
        if self._shoulder_in_degrees:
            angle = math.radians(angle)
        angle = (angle + math.pi) % (2 * math.pi) - math.pi
        self._shoulder_angle_rad = angle

    def gripper_callback(self, msg: Bool):
        """
        True  -> Cerrar
        False -> Abrir

        Si hay sobrecarga activa, se ignora el comando y se mantiene la pose.
        """
        if self.overload_active:
            self.get_logger().warn("Comando de gripper ignorado por sobrecarga activa.")
            return

        if msg.data:
            self.get_logger().info("Orden recibida: CERRAR Gripper")
            target_gripper = [0.225, -0.225]
        else:
            self.get_logger().info("Orden recibida: ABRIR Gripper")
            target_gripper = [0.6, -0.6]

        self.last_gripper_pos = target_gripper
        full_pose = self.last_arm_pos + self.last_gripper_pos
        gripper_action_vel = [0.5] * 7
        self.dynamixel.set_joints_with_velocity(full_pose, gripper_action_vel)

    def position_callback(self, msg: Float64MultiArray):
        """
        Si hay sobrecarga activa, ignora nuevos comandos y mantiene la pose actual.
        """
        if self.overload_active:
            self.get_logger().warn("goal_pos ignorado por sobrecarga activa.")
            return

        num_arm_joints = 5

        if len(msg.data) < 6:
            return

        current_arm_cmd = list(msg.data[1:1 + num_arm_joints])
        self.last_arm_pos = current_arm_cmd

        start_vel_index = 9
        if len(msg.data) >= start_vel_index + num_arm_joints:
            current_arm_vel = list(msg.data[start_vel_index:start_vel_index + num_arm_joints])
        else:
            current_arm_vel = [0.5] * num_arm_joints

        final_positions = self.last_arm_pos + self.last_gripper_pos
        final_velocities = current_arm_vel + [0.5, 0.5]

        self.dynamixel.set_joints_with_velocity(final_positions, final_velocities)

    def destroy_node(self):
        try:
            self.dynamixel.shutdown()
        except Exception:
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
