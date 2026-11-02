#ifndef BENDER_HARDWARE_INTERFACES__CHEST_LED_CONTROLLER_HPP_
#define BENDER_HARDWARE_INTERFACES__CHEST_LED_CONTROLLER_HPP_

#include <atomic>
#include <cstdint>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace bender_hardware_interfaces {
    // Controla la luz del pecho (Pico W) a través de las command interfaces
    // <led_name>/r, /g, /b, /estop que exporta EncoderInterface.
    //
    //  - El color viene del tópico ~/color (std_msgs/UInt8MultiArray, data = [r, g, b]).
    //  - El estado de la parada de emergencia se lee DIRECTO de la state interface
    //    del Pioneer (estop/pressed), dentro del mismo ciclo de control, sin pasar
    //    por tópicos. La Pico le da prioridad al e-stop sobre el color.
    class ChestLedController : public controller_interface::ControllerInterface {
        public:
            controller_interface::CallbackReturn on_init() override;

            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

            controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        private:
            static uint32_t pack(uint8_t r, uint8_t g, uint8_t b) {
                return (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8) | b;
            }

            std::string led_name_;
            std::string estop_interface_;
            uint32_t default_color_ = 0;

            // Color pedido, empaquetado 0x00RRGGBB. Atómico porque lo escribe el
            // callback del tópico y lo lee update() (hilo de tiempo real).
            std::atomic<uint32_t> color_{0};

            rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr color_sub_;
    };

} // namespace bender_hardware_interfaces

#endif // BENDER_HARDWARE_INTERFACES__CHEST_LED_CONTROLLER_HPP_
