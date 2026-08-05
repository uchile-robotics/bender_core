#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class RobotRoutine(Node):
    def __init__(self):
        super().__init__('robot_routine_node')

        # --- Inicializar Action Clients ---
        self.get_logger().info("Inicializando Action Clients...")

        # Hombros (JointTrajectoryController)
        self.left_shoulder_client = ActionClient(self, FollowJointTrajectory, '/left_shoulder_controller/follow_joint_trajectory')
        self.right_shoulder_client = ActionClient(self, FollowJointTrajectory, '/right_shoulder_controller/follow_joint_trajectory')

        # Brazos y Cabeza (JointTrajectoryController)
        self.left_arm_client = ActionClient(self, FollowJointTrajectory, '/left_arm_controller/follow_joint_trajectory')
        self.right_arm_client = ActionClient(self, FollowJointTrajectory, '/right_arm_controller/follow_joint_trajectory')
        self.head_client = ActionClient(self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory')

        # Grippers (GripperActionController) - Agregué el derecho también por completitud
        self.left_gripper_client = ActionClient(self, GripperCommand, '/left_gripper_controller/gripper_cmd')
        self.right_gripper_client = ActionClient(self, GripperCommand, '/right_gripper_controller/gripper_cmd')

        # Esperar a que los servidores estén listos
        self.wait_for_servers()

    def wait_for_servers(self):
        servers = [
            self.left_shoulder_client, self.right_shoulder_client,
            self.left_arm_client, self.right_arm_client,
            self.head_client,
            self.left_gripper_client, self.right_gripper_client
        ]
        for server in servers:
            if not server.wait_for_server(timeout_sec=5.0):
                self.get_logger().error(f"Action server {server._action_name} no disponible.")
        self.get_logger().info("¡Todos los Action Servers están listos!")

    def send_trajectory(self, action_client, joint_names, positions, time_from_start_sec):
        """Envía una trayectoria de un solo punto al controlador especificado."""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=time_from_start_sec, nanosec=0)

        goal_msg.trajectory.points.append(point)

        self.get_logger().info(f"Enviando trayectoria a {action_client._action_name}...")
        return action_client.send_goal_async(goal_msg)

    def send_gripper_cmd(self, action_client, position, max_effort=0.0):
        """Envía un comando al gripper (posición)."""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort

        self.get_logger().info(f"Moviendo gripper a posición {position}...")
        return action_client.send_goal_async(goal_msg)

    def execute_routine(self):
        """Define la secuencia de movimientos de la rutina."""

        # Definición de nombres de joints
        left_shoulder_joint = ['l1l_to_base_link']
        right_shoulder_joint = ['l1r_to_base_link']
        left_arm_joints = ['l2l_to_l1l', 'l3l_to_l2l', 'l4l_to_l3l', 'l5l_to_l4l', 'l6l_to_l5l']
        right_arm_joints = ['l2r_to_l1r', 'l3r_to_l2r', 'l4r_to_l3r', 'l5r_to_l4r', 'l6r_to_l5r']



        # 2. Mover TODO (Hombros + Brazos) a una posición de inicio (Zero)
        self.send_trajectory(self.left_shoulder_client, left_shoulder_joint, [0.0], 2)
        self.send_trajectory(self.right_shoulder_client, right_shoulder_joint, [1.5], 2)
        self.send_trajectory(self.left_arm_client, left_arm_joints, [0.0, 0.0, 0.0, 0.0, 0.0], 2)
        self.send_trajectory(self.right_arm_client, right_arm_joints, [-0.5, 0.0, -1.5, 0.3, 0.2], 2)
        time.sleep(10.0)


        self.send_trajectory(self.left_shoulder_client, left_shoulder_joint, [0.0], 2)
        self.send_trajectory(self.right_shoulder_client, right_shoulder_joint, [0.0], 2)
        self.send_trajectory(self.left_arm_client, left_arm_joints, [0.0, 0.0, 0.0, 0.0, 0.0], 2)
        self.send_trajectory(self.right_arm_client, right_arm_joints, [0.0, 0.0, 0.0, 0.0, 0.0], 2)

        self.get_logger().info("Rutina completada con éxito.")

def main(args=None):
    rclpy.init(args=args)
    routine_node = RobotRoutine()

    try:
        # Ejecutar la rutina secuencial
        routine_node.execute_routine()
    except KeyboardInterrupt:
        routine_node.get_logger().info("Rutina interrumpida por el usuario.")
    finally:
        routine_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
