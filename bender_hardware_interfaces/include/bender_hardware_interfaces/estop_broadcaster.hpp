#ifndef BENDER_HARDWARE_INTERFACES__ESTOP_BROADCASTER_HPP_
#define BENDER_HARDWARE_INTERFACES__ESTOP_BROADCASTER_HPP_

#include <optional>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/bool.hpp"

namespace bender_hardware_interfaces {
    // Publica en ~/pressed (std_msgs/Bool, latched) el valor de una state
    // interface de parada de emergencia. Solo publica cuando el valor cambia.
    class EstopBroadcaster : public controller_interface::ControllerInterface {
        public:
            controller_interface::CallbackReturn on_init() override;

            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

            controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        private:
            std::string interface_name_;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
            std::optional<bool> last_published_;
    };

} // namespace bender_hardware_interfaces

#endif // BENDER_HARDWARE_INTERFACES__ESTOP_BROADCASTER_HPP_
