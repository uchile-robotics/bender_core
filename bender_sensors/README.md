# Bender Sensors

## Battery monitor (Raspberry Pi 5 and Pico)

`battery_monitor_node.py` reads the BMS over serial, estimates state of charge
from the battery voltage (42.0–54.6 V by default), and publishes:

- `/battery_state` (`sensor_msgs/msg/BatteryState`): voltage, current and
  estimated `percentage` in the ROS range 0.0–1.0.
- `/battery_power` (`std_msgs/msg/Float32`): power in watts (`voltage * current`).

The node subscribes to `/battery_state` and forwards the SOC to the Pico over
USB serial as `B,<percentage>\n` (percentage from 0 to 100). The Pico firmware
is provided in `firmware/battery_status_pico.ino`; it displays the SOC as a
color on its NeoPixel strip and blinks red if it stops receiving updates for
three seconds. The Pico uses USB serial rather than ROS 2 or micro-ROS.

Flash that firmware to the Pico and connect it to the Pi 5 by USB. The sketch
uses GPIO 14 for the NeoPixel data line, 16 LEDs, and the Adafruit NeoPixel
library.

On the Raspberry Pi 5, install the package in the ROS 2 workspace and ensure
`python3-serial` is installed. Connect the BMS adapter and Pico USB serial port,
then run:

```bash
ros2 run bender_sensors battery_monitor_node --ros-args \
  -p bms_port:=/dev/ttyUSB0 \
  -p pico_port:=/dev/ttyACM0
```

Port names, baud rates, publish period, voltage limits and topic names are ROS
parameters. For stable device names, set `bms_port` and `pico_port` to the
appropriate paths under `/dev/serial/by-id/`.
