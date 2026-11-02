#include "bender_hardware_interfaces/chest_led_controller.hpp"

#include <cmath>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bender_hardware_interfaces {

controller_interface::CallbackReturn ChestLedController::on_init() {
    // Nombre del <gpio> de EncoderInterface en el URDF
    auto_declare<std::string>("led_name", "chest_led");
    // State interface de la parada de emergencia ("" = no usar e-stop, p.ej. sin base)
    auto_declare<std::string>("estop_interface", "estop/pressed");
    // Color mientras nadie publique en ~/color: [r, g, b]
    auto_declare<std::vector<int64_t>>("default_color", {0, 0, 0});
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ChestLedController::command_interface_configuration() const {
    // El orden importa: update() escribe por índice (0=r, 1=g, 2=b, 3=estop)
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            {led_name_ + "/r", led_name_ + "/g", led_name_ + "/b", led_name_ + "/estop"}};
}

controller_interface::InterfaceConfiguration
ChestLedController::state_interface_configuration() const {
    if (estop_interface_.empty()) {
        return {controller_interface::interface_configuration_type::NONE, {}};
    }
    return {controller_interface::interface_configuration_type::INDIVIDUAL, {estop_interface_}};
}

controller_interface::CallbackReturn
ChestLedController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    led_name_ = get_node()->get_parameter("led_name").as_string();
    estop_interface_ = get_node()->get_parameter("estop_interface").as_string();

    const auto dc = get_node()->get_parameter("default_color").as_integer_array();
    if (dc.size() != 3) {
        RCLCPP_ERROR(get_node()->get_logger(), "default_color debe tener 3 valores [r, g, b]");
        return controller_interface::CallbackReturn::ERROR;
    }
    auto ch = [](int64_t v) { return static_cast<uint8_t>(v < 0 ? 0 : (v > 255 ? 255 : v)); };
    default_color_ = pack(ch(dc[0]), ch(dc[1]), ch(dc[2]));
    color_.store(default_color_);

    color_sub_ = get_node()->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "~/color", rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
            if (msg->data.size() != 3) {
                RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                    "~/color espera exactamente 3 valores [r, g, b], llegaron %zu", msg->data.size());
                return;
            }
            color_.store(pack(msg->data[0], msg->data[1], msg->data[2]));
        });

    RCLCPP_INFO(get_node()->get_logger(), "Luz del pecho: '%s', e-stop: '%s'",
        led_name_.c_str(), estop_interface_.empty() ? "(deshabilitado)" : estop_interface_.c_str());
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ChestLedController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
ChestLedController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    bool estop = false;
    if (!estop_interface_.empty()) {
        const auto value = state_interfaces_[0].get_optional<double>();
        // Si no se puede leer el e-stop, por seguridad se asume presionado
        estop = !value || std::isnan(*value) || *value > 0.5;
    }

    const uint32_t c = color_.load();
    (void)command_interfaces_[0].set_value(static_cast<double>((c >> 16) & 0xFF));
    (void)command_interfaces_[1].set_value(static_cast<double>((c >> 8) & 0xFF));
    (void)command_interfaces_[2].set_value(static_cast<double>(c & 0xFF));
    (void)command_interfaces_[3].set_value(estop ? 1.0 : 0.0);

    return controller_interface::return_type::OK;
}

} // namespace bender_hardware_interfaces

PLUGINLIB_EXPORT_CLASS(bender_hardware_interfaces::ChestLedController,
                       controller_interface::ControllerInterface)
