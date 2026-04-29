#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from control_msgs.action import FollowJointTrajectory
import time


class DynamixelTrajectoryBridge(Node):
    def __init__(self):
        super().__init__('dynamixel_trajectory_bridge')

        # Publicador al mismo tópico que usa tu nodo real
        self.goal_pub = self.create_publisher(Float64MultiArray, 'goal_pos', 10)

        # Action server que MoveIt usará
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/right_arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback
        )

        # --- NUEVO: Definimos el orden EXACTO que espera el nodo que controla los motores ---
        # Si los nombres de tu gripper son diferentes, cámbialos aquí.
        self.expected_joints = [
            "l1r_to_base_link", # Hombro (Índice 0)
            "l2r_to_l1r",       # DXL 0  (Índice 1)
            "l3r_to_l2r",       # DXL 1  (Índice 2)
            "l4r_to_l3r",       # DXL 2  (Índice 3)
            "l5r_to_l4r",       # DXL 3  (Índice 4)
            "l6r_to_l5r",       # DXL 4  (Índice 5)
            "g2ra_to_g1r",      # Gripper 1 (Índice 6)
            "g2rb_to_g1r"       # Gripper 2 (Índice 7)
        ]

        self.get_logger().info("DynamixelTrajectoryBridge listo: escuchando FollowJointTrajectory")

    async def execute_callback(self, goal_handle):
        traj = goal_handle.request.trajectory
        joint_names = traj.joint_names
        n_points = len(traj.points)
        self.get_logger().info(f"Recibida trayectoria con {n_points} puntos")

        for i, point in enumerate(traj.points):
            msg = Float64MultiArray()
            
            # Inicializamos el arreglo con 8 posiciones en 0.0
            pos_array = [0.0] * 8  
            
            # --- NUEVO: Mapeo por nombre en lugar de cortar el arreglo ---
            for j, name in enumerate(self.expected_joints):
                if name in joint_names:
                    # Buscamos en qué posición del mensaje de MoveIt viene este joint
                    idx = joint_names.index(name)
                    pos_array[j] = point.positions[idx]
                else:
                    # Si MoveIt no mandó este joint (ej. gripper), mantenemos 0.0 o un valor seguro
                    pass 
            
            msg.data = pos_array
            self.goal_pub.publish(msg)

            # Esperar hasta el próximo punto
            dt = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            if i < n_points - 1:
                next_dt = traj.points[i+1].time_from_start.sec + traj.points[i+1].time_from_start.nanosec * 1e-9
                sleep_time = max(0.0, next_dt - dt)
                time.sleep(sleep_time)

        goal_handle.succeed()
        self.get_logger().info("Trayectoria completada")
        return FollowJointTrajectory.Result()


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelTrajectoryBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
