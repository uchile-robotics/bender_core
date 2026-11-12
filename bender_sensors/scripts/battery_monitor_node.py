#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32


BMS_COMMAND = bytes((0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77))


def decode_bms_response(data: bytes) -> Optional[Tuple[float, float]]:
    if len(data) < 8:
        return None

    voltage_raw = (data[4] << 8) | data[5]
    current_raw = (data[6] << 8) | data[7]
    if current_raw > 32767:
        current_raw -= 65536

    return voltage_raw / 100.0, current_raw / 100.0


def estimate_soc(voltage: float, voltage_min: float, voltage_max: float) -> float:
    return max(0.0, min(100.0, (voltage - voltage_min) * 100.0 / (voltage_max - voltage_min)))


class BatteryMonitor(Node):
    def __init__(self) -> None:
        super().__init__('battery_monitor')

        self.declare_parameter('bms_port', '/dev/ttyUSB0')
        self.declare_parameter('pico_port', '/dev/ttyACM0')
        self.declare_parameter('bms_baud_rate', 9600)
        self.declare_parameter('pico_baud_rate', 115200)
        self.declare_parameter('publish_period', 1.0)
        self.declare_parameter('voltage_min', 42.0)
        self.declare_parameter('voltage_max', 54.6)
        self.declare_parameter('battery_topic', '/battery_state')
        self.declare_parameter('power_topic', '/battery_power')

        self.bms_port = self.get_parameter('bms_port').value
        self.pico_port = self.get_parameter('pico_port').value
        self.bms_baud_rate = self.get_parameter('bms_baud_rate').value
        self.pico_baud_rate = self.get_parameter('pico_baud_rate').value
        self.voltage_min = float(self.get_parameter('voltage_min').value)
        self.voltage_max = float(self.get_parameter('voltage_max').value)
        publish_period = float(self.get_parameter('publish_period').value)
        battery_topic = self.get_parameter('battery_topic').value
        power_topic = self.get_parameter('power_topic').value

        if self.voltage_max <= self.voltage_min:
            raise ValueError('voltage_max must be greater than voltage_min')
        if publish_period <= 0.0:
            raise ValueError('publish_period must be greater than zero')

        self.battery_pub = self.create_publisher(BatteryState, battery_topic, 10)
        self.power_pub = self.create_publisher(Float32, power_topic, 10)
        self.pico_sub = self.create_subscription(
            BatteryState, battery_topic, self.forward_soc_to_pico, 10)

        self.bms_serial: Optional[serial.Serial] = None
        self.pico_serial: Optional[serial.Serial] = None
        self.timer = self.create_timer(publish_period, self.read_and_publish)

        self.get_logger().info(
            f'Reading BMS on {self.bms_port}; publishing {battery_topic} and {power_topic}; '
            f'forwarding SOC to Pico on {self.pico_port}')

    def _open_serial(self, port: str, baud_rate: int) -> Optional[serial.Serial]:
        try:
            return serial.Serial(port, baud_rate, timeout=0.25, write_timeout=0.25)
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warning(f'Cannot open serial port {port}: {exc}')
            return None

    def read_and_publish(self) -> None:
        if self.bms_serial is None:
            self.bms_serial = self._open_serial(self.bms_port, self.bms_baud_rate)
            if self.bms_serial is None:
                return

        try:
            self.bms_serial.reset_input_buffer()
            self.bms_serial.write(BMS_COMMAND)
            self.bms_serial.flush()
            response = self.bms_serial.read(40)
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'BMS serial read failed: {exc}')
            self.bms_serial.close()
            self.bms_serial = None
            return

        decoded = decode_bms_response(response)
        if decoded is None:
            self.get_logger().warning('BMS returned an incomplete response; no sample published')
            return

        voltage, current = decoded
        soc_percent = estimate_soc(voltage, self.voltage_min, self.voltage_max)
        battery = BatteryState()
        battery.header.stamp = self.get_clock().now().to_msg()
        battery.voltage = voltage
        battery.current = current
        battery.percentage = soc_percent / 100.0
        battery.present = True
        self.battery_pub.publish(battery)

        power = Float32()
        power.data = voltage * current
        self.power_pub.publish(power)

    def forward_soc_to_pico(self, battery: BatteryState) -> None:
        if not battery.present or not math.isfinite(battery.percentage):
            return

        if self.pico_serial is None:
            self.pico_serial = self._open_serial(self.pico_port, self.pico_baud_rate)
            if self.pico_serial is None:
                return

        soc_percent = max(0.0, min(100.0, battery.percentage * 100.0))
        line = f'B,{soc_percent:.1f}\n'.encode('ascii')
        try:
            self.pico_serial.write(line)
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'Pico serial write failed: {exc}')
            self.pico_serial.close()
            self.pico_serial = None

    def destroy_node(self) -> bool:
        if self.bms_serial is not None and self.bms_serial.is_open:
            self.bms_serial.close()
        if self.pico_serial is not None and self.pico_serial.is_open:
            self.pico_serial.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BatteryMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
