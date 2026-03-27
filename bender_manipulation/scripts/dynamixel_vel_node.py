#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float64MultiArray
import math 

from dynamixel_command import DynamixelCommander


class DynamixelJointPublisher(Node):
    def __init__(self):
        super().__init__('dynamixel_joint_publisher')

        # Crear publicador
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz

        # Subscriptor a posiciones deseadas
        self.position_sub = self.create_subscription(
            Float64MultiArray,
            'goal_pos',
            self.position_callback,
            10
        )

        self.dynamixel_commander = DynamixelCommander()
        self.offsets = [2048,512,10,512,512,512,512]

    def publish_joint_states(self):
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        # Mapear IDs a nombres de joints
        joint_name_map = {
            0: "l2l_to_l1l",
            1: "l3l_to_l2l",
            2: "l4l_to_l3l",
            3: "l5l_to_l4l",
            4: "l6l_to_l5l",
            5: "griper_1",
            6: "griper_2"
        }

        names = ["l1l_to_base_link"]
        positions = [0.0]  # valor fijo
        velocities = [0.0]  # valor fijo

        pos,vel = self.dynamixel_commander.get_joints_data()

        for i in range(len(pos)):

            offset = self.offsets[i]

            # Determinar resolución por tipo
            if i in [0, 2]:  # MX-106
                resolution = 4095
                max_radians = 2 * math.pi
            else:  # RX-28
                resolution = 1023
                max_radians = math.radians(300)

            pos_raw = pos[i]
            vel_raw = vel[i]
            
            pos_rad = ((pos_raw - offset) / resolution) * max_radians
            pos_rad = (pos_rad + math.pi) % (2 * math.pi) - math.pi
            # Velocidad
            if vel_raw > 1023:
                velocity = (vel_raw - 1024) * -1
            else:
                velocity = vel_raw
            
            joint_name = joint_name_map[i]
            names.append(joint_name)
            positions.append(pos_rad)
            velocities.append(velocity)

        joint_state_msg.name = names
        joint_state_msg.position = positions
        joint_state_msg.velocity = velocities
        joint_state_msg.effort = [0.0] * len(names)

        self.joint_pub.publish(joint_state_msg)



    def position_callback(self, msg: Float64MultiArray):
        if len(msg.data) != 7:
            self.get_logger().warn("Se esperaban exactamente 7 posiciones (en radianes)")
            return

        


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