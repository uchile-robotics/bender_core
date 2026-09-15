#include "bender_hardware_interfaces/pioneer_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include <array>
#include <cmath>
#include <utility>

namespace bender_hardware_interfaces {

hardware_interface::CallbackReturn PioneerInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
    if (hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Parámetros de conexión serial (con valores por defecto razonables).
    if (info_.hardware_parameters.find("serial_device") != info_.hardware_parameters.end()) {
        serial_port_ = info_.hardware_parameters["serial_device"];
    } else {
        serial_port_ = "/dev/pioneer";
    }

    if (info_.hardware_parameters.find("baud_rate") != info_.hardware_parameters.end()) {
        serial_baud_ = std::stoi(info_.hardware_parameters["baud_rate"]);
    } else {
        serial_baud_ = 9600;
    }

    // La separación entre ruedas es obligatoria: sin ella la cinemática
    // (inverse_kinematics/forward_kinematics) no puede convertir entre v/w
    // de ROS y las velocidades individuales de cada rueda para ARIA.
    if (info_.hardware_parameters.find("wheel_separation") != info_.hardware_parameters.end()) {
        wheel_separation_ = std::stod(info_.hardware_parameters["wheel_separation"]);
    } else {
        RCLCPP_ERROR(get_logger(),
                     "Falta el parámetro obligatorio 'wheel_separation' en el URDF "
                     "(ros2_control.xacro, bloque Pioneer).");
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (info_.joints.size() != 2) {
        RCLCPP_ERROR(get_logger(),
                     "Se esperaban 2 joints (rueda izq y der), se detectaron: %zu",
                     info_.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // NOTA: se asume que info_.joints[0] es la rueda izquierda y
    // info_.joints[1] la derecha, según el orden declarado en el xacro
    // (joint1 -> izquierda, joint2 -> derecha). Verificar si el mapeo real
    // del robot es al revés.
    hw_cmd_wheel_left_ = 0.0;
    hw_cmd_wheel_right_ = 0.0;
    left_wheel_pos_ = 0.0;
    left_wheel_vel_ = 0.0;
    right_wheel_pos_ = 0.0;
    right_wheel_vel_ = 0.0;

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> PioneerInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Solo se exportan interfaces de velocidad: es lo único que declara el
    // URDF para "Pioneer" (joint1/joint2) y lo único que ARIA nos entrega
    // directamente vía getVel()/getRotVel() en read(). Si más adelante se
    // necesita posición (odometría por rueda), habría que agregar aquí el
    // HW_IF_POSITION y calcularla integrando left_wheel_vel_/right_wheel_vel_
    // en read().
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &left_wheel_vel_));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &right_wheel_vel_));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> PioneerInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_cmd_wheel_left_));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &hw_cmd_wheel_right_));

    return command_interfaces;
}

hardware_interface::CallbackReturn
PioneerInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    Aria::init();

    // 1. Construir los argumentos que ARIA usa para conectar al robot,
    // equivalentes a pasar "-robotPort <serial_port_> -robotBaud <serial_baud_>"
    // por línea de comandos.
    args_ = std::make_shared<ArArgumentBuilder>();
    args_->add("-robotPort");
    args_->add("%s", serial_port_.c_str());
    args_->add("-robotBaud");
    args_->add("%d", serial_baud_);

    argparser_ = std::make_shared<ArArgumentParser>(args_.get());
    argparser_->loadDefaultArguments();

    robot_ = std::make_shared<ArRobot>();
    conn_ = std::make_shared<ArRobotConnector>(argparser_.get(), robot_.get());

    // 2. Conectar al robot real.
    if (!conn_->connectRobot()) {
        RCLCPP_ERROR(get_logger(),
                     "No se pudo conectar al robot Pioneer en %s (%d baudios).",
                     serial_port_.c_str(), serial_baud_);
        conn_.reset();
        argparser_.reset();
        args_.reset();
        robot_.reset();
        Aria::shutdown();
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 3. Lanzar el hilo asíncrono de ARIA (se detiene si se pierde la conexión)
    // y habilitar los motores.
    robot_->runAsync(true);

    robot_->lock();
    robot_->enableMotors();
    robot_->unlock();

    RCLCPP_INFO(get_logger(), "Robot Pioneer conectado en %s (%d baudios).",
                serial_port_.c_str(), serial_baud_);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
PioneerInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    if (robot_) {
        // 1. Detener el robot y desactivar motores antes de cortar la conexión.
        robot_->lock();
        robot_->stop();
        robot_->disableMotors();
        robot_->unlock();

        // 2. Detener el hilo de procesamiento de ARIA y esperar a que termine.
        robot_->stopRunning();
        robot_->waitForRunExit();
    }

    conn_.reset();
    argparser_.reset();
    args_.reset();
    robot_.reset();

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
