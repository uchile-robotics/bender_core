#ifndef BENDER_HARDWARE_INTERFACES__ENCODER_INTERFACE_HPP_
#define BENDER_HARDWARE_INTERFACES__ENCODER_INTERFACE_HPP_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"

namespace bender_hardware_interfaces
{
// Interfaz con la Raspberry Pi Pico W del pecho (un solo puerto serial):
//  - read():  parsea el JSON de los encoders de los hombros
//             {"left":{"pos":..,"vel":..},"right":{"pos":..,"vel":..}}
//  - write(): envía el estado de la luz del pecho "L,<estop 0|1>,<r>,<g>,<b>\n"
// Es SystemInterface (y no SensorInterface) porque necesita command interfaces
// para la luz. Los comandos de la luz se exponen como un <gpio> del URDF.
class EncoderInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(EncoderInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::string serial_device_ = "/dev/encoders";
  int baud_rate_;
  int serial_fd_ = -1;
  std::string rx_buffer_;

  // Variables para almacenar los estados de los encoders
  double left_pos_ = 0.0;
  double left_vel_ = 0.0;
  double right_pos_ = 0.0;
  double right_vel_ = 0.0;

  // ---- Luz del pecho (opcional: solo si el URDF declara el <gpio>)
  bool has_led_ = false;
  std::string led_gpio_name_ = "chest_led";
  // Periodo de reenvío aunque no cambie nada. Debe ser menor que el
  // COMMS_TIMEOUT_MS de la Pico (500 ms), si no la Pico parpadea en rojo.
  double led_resend_period_s_ = 0.1;

  static constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
  double led_cmd_r_ = NaN;
  double led_cmd_g_ = NaN;
  double led_cmd_b_ = NaN;
  double led_cmd_estop_ = NaN;

  std::string last_led_line_;
  double time_since_led_tx_ = 0.0;
};

}  // namespace bender_hardware_interfaces

#endif  // BENDER_HARDWARE_INTERFACES__ENCODER_INTERFACE_HPP_
