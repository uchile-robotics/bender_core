
#include "bender_hardware_interfaces/encoder_interface.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bender_hardware_interfaces {

hardware_interface::CallbackReturn EncoderInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams &params) {

  if (hardware_interface::SensorInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.find("serial_device") != info_.hardware_parameters.end()) {
    serial_device_ = info_.hardware_parameters["serial_device"];
  } else {
    serial_device_ = "/dev/encoders";
  }

  if (info_.hardware_parameters.find("baud_rate") != info_.hardware_parameters.end()) {
    baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
  } else {
    baud_rate_ = 115200;
  }

  if (info_.sensors.size() != 2) {
    RCLCPP_ERROR(get_logger(), "Expected 2 sensors in urdf, got: %zu", info_.sensors.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EncoderInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {

  RCLCPP_INFO(get_logger(), "Conectando al encoder en %s a %d baudios...", serial_device_.c_str(), baud_rate_);

  serial_fd_ = open(serial_device_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(get_logger(), "Error crítico: No se pudo abrir el puerto %s", serial_device_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  struct termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "Error getting serial port attributes");
    close(serial_fd_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  speed_t speed = B115200;
  if (baud_rate_ == 9600) speed = B9600;
  else if (baud_rate_ == 57600) speed = B57600;

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag &= ~IGNBRK;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "Error al aplicar la configuración termios.");
    close(serial_fd_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  tcflush(serial_fd_, TCIOFLUSH);
  RCLCPP_INFO(get_logger(), "Puerto serial inicializado con éxito.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EncoderInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {

  RCLCPP_INFO(get_logger(), "Desactivando interfaz de hardware...");
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
  RCLCPP_INFO(get_logger(), "Puerto serial cerrado. Desactivación exitosa.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type EncoderInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {

  if (serial_fd_ < 0) {
    return hardware_interface::return_type::ERROR;
  }

  char buf[256];
  int bytes_leidos;
  std::string datos_crudos = "";

  while ((bytes_leidos = ::read(serial_fd_, buf, sizeof(buf) - 1)) > 0) {
    buf[bytes_leidos] = '\0';
    datos_crudos += buf;
  }

  std::string ultima_linea = "";
  if (!datos_crudos.empty()) {
    size_t ultimo_salto = datos_crudos.rfind('\n');
    if (ultimo_salto != std::string::npos) {
      size_t previo_salto = datos_crudos.rfind('\n', ultimo_salto - 1);
      if (previo_salto == std::string::npos) {
        ultima_linea = datos_crudos.substr(0, ultimo_salto);
      } else {
        ultima_linea = datos_crudos.substr(previo_salto + 1, ultimo_salto - previo_salto - 1);
      }
    }
  }

  // Si logramos aislar una línea completa, la decodificamos
  if (!ultima_linea.empty()) {
    float angulo1 = 0.0f;
    float angulo2 = 0.0f;

    // Parseamos el string "angulo1,angulo2" enviado desde la Pico W
    if (sscanf(ultima_linea.c_str(), "%f,%f", &angulo1, &angulo2) == 2) {
      // Mapear los datos numéricos a los nombres de los sensores declarados en tu URDF
      std::string interfaz_sensor1 = info_.sensors[0].name + "/" + hardware_interface::HW_IF_POSITION;
      std::string interfaz_sensor2 = info_.sensors[1].name + "/" + hardware_interface::HW_IF_POSITION;

      set_state(interfaz_sensor1, static_cast<double>(angulo1));
      set_state(interfaz_sensor2, static_cast<double>(angulo2));
    }
  }

  std::string interfaz_sensor1 = info_.sensors[0].name + "/" + hardware_interface::HW_IF_POSITION;
  std::string interfaz_sensor2 = info_.sensors[1].name + "/" + hardware_interface::HW_IF_POSITION;
  // logging every 500 ms
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                       "%s: %.2f deg | %s: %.2f deg",
                       info_.sensors[0].name.c_str(), get_state(interfaz_sensor1),
                       info_.sensors[1].name.c_str(), get_state(interfaz_sensor2));

  return hardware_interface::return_type::OK;
}

} // namespace bender_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(bender_hardware_interfaces::EncoderInterface,
                       hardware_interface::SensorInterface)
