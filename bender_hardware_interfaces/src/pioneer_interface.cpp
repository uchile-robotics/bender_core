#include "bender_hardware_interfaces/pioneer_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include <array>
#include <utility>

namespace bender_hardware_interfaces {

hardware_interface::CallbackReturn PioneerInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
    // TODO:
    // 1. Ejecutar el on_init() de la clase base
    // (hardware_interface::SystemInterface::on_init(params)).
    // 2. Leer/parsear los parámetros desde el mapa 'info_.hardware_parameters'.

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
PioneerInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    Aria::init();
    // 1. Instanciar los objetos de ARIA (ArRobot, ArRobotConnector, etc.).
    // 2. Pasar los parámetros del puerto serie y baudrate.
    // 3. Conectar al robot real (connectRobot()), activar motores y lanzar el hilo
    // asíncrono (runAsync()).

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
PioneerInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    // TODO:
    // 1. Enviar comando de detención y desactivar motores en ARIA.
    // 2. Detener el hilo de procesamiento (stopRunning()) y apagar la librería
    // (Aria::shutdown()).
    Aria::shutdown();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
PioneerInterface::read(const rclcpp::Time& /*time*/,
                       const rclcpp::Duration& /*period*/) {

    double aria_v = robot_->getVel();
    double aria_w = robot_->getRotVel();
    std::array<double, 2> velocities = this->inverse_kinematics(aria_v, aria_w);
    right_wheel_vel_ = velocities[0];
    left_wheel_vel_ = velocities[1];

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
PioneerInterface::write(const rclcpp::Time& /*time*/,
                        const rclcpp::Duration& /*period*/) {
    double v_left = hw_cmd_wheel_left_;
    double v_right = hw_cmd_wheel_right_;

    std::array<double, 2> velocities = this->forward_kinematics(v_left, v_right);
    double aria_v = velocities[0];
    double aria_w = velocities[1];

    robot_->lock();
    robot_->setVel(aria_v);
    robot_->setRotVel(aria_w);
    robot_->unlock();

    return hardware_interface::return_type::OK;
}
std::array<double, 2> PioneerInterface::inverse_kinematics(const double& v,
                                                           const double& w) {

    double ros_v = v / 1000.0;
    double ros_w = w * (M_PI / 180);

    double v_r = (2 * ros_v + wheel_separation_ * ros_w) / 2;
    double v_l = (2 * ros_v - wheel_separation_ * ros_w) / 2;
    std::array<double, 2> val_pair = {v_r, v_l};
    return val_pair;
}

std::array<double, 2>
PioneerInterface::forward_kinematics(const double& left_wheel_speed,
                                     const double& right_wheel_speed) {
    std::array<double, 2> speeds = {0.0, 0.0};
    double v_linear = (right_wheel_speed + left_wheel_speed) / 2.0;
    double v_angular = (right_wheel_speed - left_wheel_speed) / wheel_separation_;
    double aria_v = v_linear * 1000.0;
    double aria_w = v_angular * (180.0 / M_PI);

    speeds = {aria_v, aria_w};
    return speeds;
}
} // namespace bender_hardware_interfaces

// Exportación como plugin para que pluginlib y ros2_control puedan cargar esta clase
// dinámicamente
PLUGINLIB_EXPORT_CLASS(bender_hardware_interfaces::PioneerInterface,
                       hardware_interface::SystemInterface)
