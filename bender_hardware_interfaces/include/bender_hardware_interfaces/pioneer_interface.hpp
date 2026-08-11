#ifndef BENDER_HARDWARE_INTERFACES__PIONEER_INTERFACE_HPP_
#define BENDER_HARDWARE_INTERFACES__PIONEER_INTERFACE_HPP_

#include <array>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <Aria/Aria.h>
#include <Aria/ArRobot.h>

namespace bender_hardware_interfaces {
    class PioneerInterface : public hardware_interface::SystemInterface {
        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(PioneerInterface)

            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;

            hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

            hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

            hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

            hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            std::array<double, 2> inverse_kinematics(const double &v, const double &w);
            std::array<double, 2> forward_kinematics(const double &left_wheel_speed, const double &right_wheel_speed);

        private:
            // params
            std::string serial_port_;
            int serial_baud_;
            double wheel_separation_;

            // ariacoda stuff
            std::shared_ptr<ArRobot> robot_;
            std::shared_ptr<ArRobotConnector> conn_;
            std::shared_ptr<ArArgumentBuilder> args_;
            std::shared_ptr<ArArgumentParser> argparser_;

            // command variables
            double hw_cmd_wheel_left_;
            double hw_cmd_wheel_right_;
            // state variables
            double left_wheel_pos_;
            double left_wheel_vel_;
            double right_wheel_pos_;
            double right_wheel_vel_;
    };

} // namespace bender_hardware_interfaces

#endif // BENDER_HARDWARE_INTERFACES__PIONEER_INTERFACE_HPP_
