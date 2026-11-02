# Bender Hardware Interfaces

## Table of contents

- [Bender Hardware Interfaces](#bender-hardware-interfaces)
  - [Table of contents](#table-of-contents)
  - [Introduction](#introduction)
  - [Requirements](#requirements)
  - [Interfaces](#interfaces)
    - [EncoderInterface](#encoderinterface)
    - [ChestLedController](#chestledcontroller)
    - [PioneerInterface](#pioneerinterface)
  - [Development](#development)
    - [Directory structure](#directory-structure)
    - [Adding a new hardware interface](#adding-a-new-hardware-interface)

## Introduction

This directory contains the `ros2_control` [`SystemInterface`](https://control.ros.org/jazzy/doc/api/classhardware__interface_1_1SystemInterface.html) and [`SensorInterface`](https://control.ros.org/jazzy/doc/api/classhardware__interface_1_1SensorInterface.html) implementations that let Bender talk to hardware that has no off-the-shelf ROS 2 driver: the shoulder encoders and the Pioneer 3-AT mobile base. Each class here is compiled into `libbender_hardware_interfaces.so` and loaded dynamically by `controller_manager` through `pluginlib`, as declared in [`bender_hardware_interfaces.xml`](./bender_hardware_interfaces.xml).

If you are looking for how these plugins are wired into the robot's URDF or the controller configuration, see `bender_description/urdf/ros2_control.xacro` and `bender_description/config/controllers/controllers.yaml`, and the general [Bender Core README](../README.md).

## Requirements

- The [`uchile_system`](https://github.com/uchile-robotics/uchile_system) Docker environment, which already provides ROS 2 Jazzy, `ros2_control` and AriaCoda pre-installed — see its README for setup instructions. Everything below assumes you are working inside that container.
- ROS 2 Jazzy, `hardware_interface` and `pluginlib` (installed as part of `ros-jazzy-ros2-control`).
- [AriaCoda](https://github.com/reedhedges/AriaCoda) (only needed by `PioneerInterface`). It does not ship a CMake config, so the package's `CMakeLists.txt` links against it directly at `/usr/local/{include,lib}`, matching how the `bender_core` Docker image installs it:
  ```bash
  cd AriaCoda
  make -j$(nproc) && make install
  ```
  If AriaCoda is installed somewhere else, add that prefix to `CMAKE_PREFIX_PATH` instead of editing `CMakeLists.txt`.

## Interfaces

### EncoderInterface

[`encoder_interface.hpp`](include/bender_hardware_interfaces/encoder_interface.hpp) / [`encoder_interface.cpp`](src/encoder_interface.cpp)

A `hardware_interface::SystemInterface` for the chest Raspberry Pi Pico W, which shares a single serial line for two jobs:

- `read()`: parses the shoulder encoders, line protocol `{"left": {"pos": ..., "vel": ...}, "right": {"pos": ..., "vel": ...}}\n`. Requires exactly 2 `<sensor>` entries in the URDF (left, then right).
- `write()`: sends the chest light state `L,<estop 0|1>,<r>,<g>,<b>\n`. Only enabled if the URDF declares `<gpio name="chest_led">` with the command interfaces `r`, `g`, `b`, `estop`. The line is sent when it changes and re-sent every `led_resend_period` seconds; the Pico blinks red if it receives nothing for 500 ms. Nothing is sent until a controller writes the commands (they start as NaN).

| Hardware parameter | Default | Description |
| --- | --- | --- |
| `serial_device` | `/dev/encoders` | Serial port of the Pico. |
| `baud_rate` | `115200` | Baud rate for the serial connection. |
| `led_resend_period` | `0.1` | Re-send period of the light line, in seconds. Must be below the Pico timeout (0.5 s). |

Exported state interfaces per `<sensor>`: `position`, `velocity`. Exported command interfaces: `chest_led/r`, `chest_led/g`, `chest_led/b` (0-255) and `chest_led/estop` (0/1).

### ChestLedController

[`chest_led_controller.hpp`](include/bender_hardware_interfaces/chest_led_controller.hpp) / [`chest_led_controller.cpp`](src/chest_led_controller.cpp)

A controller that claims the `chest_led/*` command interfaces. The color comes from `~/color` (`std_msgs/UInt8MultiArray`, `data = [r, g, b]`) and the e-stop is read directly from the `estop/pressed` state interface exported by `PioneerInterface`, so it does not depend on any topic. The Pico gives the e-stop priority over the color (solid red). If the e-stop interface cannot be read it is assumed pressed.

| Parameter | Default | Description |
| --- | --- | --- |
| `led_name` | `chest_led` | Name of the `<gpio>` in the URDF. |
| `estop_interface` | `estop/pressed` | E-stop state interface. Set to `""` when running without the base. |
| `default_color` | `[0, 0, 0]` | Color used until something is published on `~/color`. |

```bash
ros2 run controller_manager spawner chest_led_controller
ros2 topic pub --once /chest_led_controller/color std_msgs/msg/UInt8MultiArray "{data: [0, 120, 255]}"
```

### PioneerInterface

[`pioneer_interface.hpp`](include/bender_hardware_interfaces/pioneer_interface.hpp) / [`pioneer_interface.cpp`](src/pioneer_interface.cpp)

A `hardware_interface::SystemInterface` that drives the Pioneer 3-AT base through AriaCoda. It expects exactly 2 `<joint>` entries in the URDF, the first treated as the left wheel and the second as the right wheel.

| Hardware parameter | Default | Description |
| --- | --- | --- |
| `serial_device` | `/dev/pioneer` | Serial port to the robot's onboard controller. |
| `baud_rate` | `9600` | Baud rate for the serial connection. |
| `wheel_separation` | *(required, no default)* | Track width in meters, used by the kinematics below. `on_init()` fails if it is missing. |


- Command interface: `velocity` (rad/s at the wheel).
- State interface: `velocity` (rad/s at the wheel), derived from AriaCoda's `getVel()`/`getRotVel()`.

**Lifecycle**

- `on_init()`: reads the parameters above and validates them.
- `on_activate()`: builds the ARIA argument list (`-robotPort`/`-robotBaud`), connects via `ArRobotConnector`, starts the async communication thread (`runAsync()`) and enables the motors.
- `read()`/`write()`: convert between ROS-style `(v, w)` and AriaCoda's `(transVel, rotVel)` using `inverse_kinematics()`/`forward_kinematics()`, which need `wheel_separation_` to split/combine the per-wheel velocities.
- `on_deactivate()`: stops the robot, disables the motors, stops the async thread and shuts ARIA down.

## Development

### Directory structure

```bash
bender_hardware_interfaces/
├── bender_hardware_interfaces.xml
├── CMakeLists.txt
├── include
│   └── bender_hardware_interfaces
│       ├── encoder_interface.hpp
│       └── pioneer_interface.hpp
├── package.xml
├── README.md
└── src
    ├── encoder_interface.cpp
    └── pioneer_interface.cpp
```

### Adding a new hardware interface

1. Add the header under `include/bender_hardware_interfaces/` and the implementation under `src/`, deriving from `hardware_interface::SystemInterface` or `SensorInterface` as appropriate.
2. Register the new `.cpp` file in `add_library(${PROJECT_NAME} SHARED ...)` in `CMakeLists.txt`.
3. Add a `<class>` entry to `bender_hardware_interfaces.xml` — double check the `type` and `base_class_type` attributes (a mismatched pair throws `pluginlib::CreateClassException` at load time).
4. Reference the plugin from a `<ros2_control>` block in `bender_description/urdf/ros2_control.xacro`, and its interfaces from a controller in `bender_description/config/controllers/controllers.yaml`.
5. Document the new interface's parameters and exported interfaces in this file.
