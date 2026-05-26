
#ifndef BENDER_HARDWARE_INTERFACES__ENCODER_INTERFACE_HPP_
#define BENDER_HARDWARE_INTERFACES__ENCODER_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"

namespace bender_hardware_interfaces
{
class EncoderInterface : public hardware_interface::SensorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(EncoderInterface)

  // Inicialización del componente desde el archivo URDF
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::string serial_device_ = "/dev/encoders";
  int baud_rate_;
  std::vector<double> hw_states_;
  int serial_fd_ = -1;
};

}  // namespace bender_hardware_interfaces

#endif  // BENDER_HARDWARE_INTERFACES__ENCODER_INTERFACE_HPP_
