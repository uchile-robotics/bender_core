# Bender Sensors

## Battery monitor (Raspberry Pi 5)

`battery_monitor_node.py` reads the BMS over serial, estimates state of charge
from the battery voltage (42.0–54.6 V by default), and publishes:

- `/battery_state` (`sensor_msgs/msg/BatteryState`): voltage, current and
  estimated `percentage` in the ROS range 0.0–1.0.
- `/battery_power` (`std_msgs/msg/Float32`): power in watts (`voltage * current`).

The node communicates only with the BMS. The chest LED controller subscribes
to `/battery_state` and sends RGB commands through the existing
`ros2_control`/Pico interface, so this node does not open a second serial
connection to the Pico.

On the Raspberry Pi 5, install the package in the ROS 2 workspace and ensure
`python3-serial` is installed. Connect the BMS adapter, then run:

```bash
ros2 run bender_sensors battery_monitor_node.py --ros-args \
  -p bms_port:=/dev/bms
```

The BMS port, baud rate, publish period, voltage limits and topic names are ROS
parameters. For a stable BMS device name, use its path under
`/dev/serial/by-id/`.
