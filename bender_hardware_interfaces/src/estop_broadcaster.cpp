#include "bender_hardware_interfaces/estop_broadcaster.hpp"

#include <cmath>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bender_hardware_interfaces {

controller_interface::CallbackReturn EstopBroadcaster::on_init() {
    auto_declare<std::string>("interface_name", "estop/pressed");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
EstopBroadcaster::command_interface_configuration() const {
    return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration
EstopBroadcaster::state_interface_configuration() const {
    return {controller_interface::interface_configuration_type::INDIVIDUAL, {interface_name_}};
}

controller_interface::CallbackReturn
EstopBroadcaster::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    interface_name_ = get_node()->get_parameter("interface_name").as_string();
    // transient_local: quien se suscriba tarde recibe igual el último estado.
    publisher_ = get_node()->create_publisher<std_msgs::msg::Bool>(
        "~/pressed", rclcpp::QoS(1).transient_local());
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
EstopBroadcaster::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    last_published_.reset();
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
EstopBroadcaster::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    const auto value = state_interfaces_[0].get_optional<double>();
    if (!value || std::isnan(*value)) {
        return controller_interface::return_type::OK;
    }

    const bool pressed = *value > 0.5;
    if (last_published_ != pressed) {
        std_msgs::msg::Bool msg;
        msg.data = pressed;
        publisher_->publish(msg);
        last_published_ = pressed;
    }

    return controller_interface::return_type::OK;
}

} // namespace bender_hardware_interfaces

PLUGINLIB_EXPORT_CLASS(bender_hardware_interfaces::EstopBroadcaster,
                       controller_interface::ControllerInterface)
