#!/usr/bin/env python3
"""
Nodo para monitorear los errores (posicion/velocidad) de los
controladores del brazo derecho:
  - right_shoulder_controller
  - right_arm_controller

Se suscribe al topico ~/controller_state que publican los
joint_trajectory_controller en ROS2 Jazzy
(control_msgs/msg/JointTrajectoryControllerState)
e imprime el arreglo de errores por cada joint.
"""

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState


class RightArmErrorMonitor(Node):
    def __init__(self):
        super().__init__('right_arm_error_monitor')


        self.create_subscription(
            JointTrajectoryControllerState,
            '/right_arm_controller/controller_state',
            self.arm_callback,
            10
        )

        self.get_logger().info(
            'Right arm error monitor iniciado. Escuchando '
            '/right_shoulder_controller/controller_state y '
            '/right_arm_controller/controller_state'
        )


    def arm_callback(self, msg: JointTrajectoryControllerState):
        self.print_error('right_arm_controller', msg)

    def print_error(self, controller_name: str, msg: JointTrajectoryControllerState):
        joint_names = list(msg.joint_names)
        pos_error = list(msg.error.positions) if msg.error.positions else []
        # vel_error = list(msg.error.velocities) if msg.error.velocities else []
        formatted = [f"{x:.5f}" for x in pos_error]
        print(joint_names)
        print(formatted)


def main(args=None):
    rclpy.init(args=args)
    node = RightArmErrorMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
