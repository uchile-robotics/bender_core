#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import BatteryState
from std_msgs.msg import UInt8MultiArray


def percentage_to_rgb(percentage: float) -> list[int]:
    soc = percentage * 100.0
    if soc < 20.0:
        return [255, 0, 0]
    if soc < 50.0:
        return [255, 128, 0]
    if soc < 80.0:
        return [255, 255, 0]
    return [0, 255, 0]


class BatteryStateToChestLed(Node):
    def __init__(self) -> None:
        super().__init__('battery_state_to_chest_led')
        self.declare_parameter('battery_state_topic', '/battery_state')
        self.declare_parameter(
            'led_color_topic', '/bender/chest_led_controller/color')

        battery_topic = self.get_parameter('battery_state_topic').value
        led_color_topic = self.get_parameter('led_color_topic').value
        if not battery_topic or not led_color_topic:
            raise ValueError('battery_state_topic and led_color_topic must not be empty')

        self.led_color_pub = self.create_publisher(
            UInt8MultiArray, led_color_topic, qos_profile_system_default)
        self.battery_state_sub = self.create_subscription(
            BatteryState,
            battery_topic,
            self.on_battery_state,
            qos_profile_system_default,
        )
        self.get_logger().info(
            f'Converting {battery_topic} to RGB commands on {led_color_topic}')

    def on_battery_state(self, battery: BatteryState) -> None:
        percentage = battery.percentage
        if (
            not battery.present
            or not math.isfinite(percentage)
            or percentage < 0.0
            or percentage > 1.0
        ):
            self.get_logger().warning(
                'Ignorando estado de batería ausente o con porcentaje inválido')
            return

        color = UInt8MultiArray()
        color.data = percentage_to_rgb(percentage)
        self.led_color_pub.publish(color)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BatteryStateToChestLed()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
