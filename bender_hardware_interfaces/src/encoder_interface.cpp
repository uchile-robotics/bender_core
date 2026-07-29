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
    RCLCPP_ERROR(get_logger(), "Se esperaban 2 sensores en el URDF (izq y der), se detectaron: %zu", info_.sensors.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> EncoderInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Vinculamos las variables internas a ROS 2 Control usando los nombres del URDF
  // Sensor 0: Izquierdo ("left")
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[0].name, hardware_interface::HW_IF_POSITION, &left_pos_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[0].name, hardware_interface::HW_IF_VELOCITY, &left_vel_));

  // Sensor 1: Derecho ("right")
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[1].name, hardware_interface::HW_IF_POSITION, &right_pos_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[1].name, hardware_interface::HW_IF_VELOCITY, &right_vel_));

  return state_interfaces;
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
    RCLCPP_ERROR(get_logger(), "Error al obtener atributos del puerto serial");
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
  tty.c_cflag &= ~HUPCL;

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
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type EncoderInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {

  if (serial_fd_ < 0) {
    return hardware_interface::return_type::ERROR;
  }

  char buf[256]; // Búfer de lectura en memoria
  int bytes_leidos;
  static std::string buffer_acumulador = "";

  // 1. Leer todo lo disponible en el puerto de una sola vez
  while ((bytes_leidos = ::read(serial_fd_, buf, sizeof(buf))) > 0) {

    // 2. Iterar sobre los bytes en RAM (mucho más eficiente)
    for (int i = 0; i < bytes_leidos; ++i) {
      char c = buf[i];

      if (c == '\n') {
        std::string ultima_linea = buffer_acumulador;
        buffer_acumulador.clear();

        if (!ultima_linea.empty()) {
          try {
            size_t left_start = ultima_linea.find("\"left\"");
            size_t right_start = ultima_linea.find("\"right\"");

            if (left_start != std::string::npos && right_start != std::string::npos) {
              std::string left_sub = ultima_linea.substr(left_start, right_start - left_start);
              std::string right_sub = ultima_linea.substr(right_start);

              // Parseo Izquierdo
              size_t lp = left_sub.find("\"pos\":");
              size_t lv = left_sub.find("\"vel\":");
              if (lp != std::string::npos) left_pos_ = std::stod(left_sub.substr(lp + 6, left_sub.find_first_of(",}", lp) - (lp + 6)));
              if (lv != std::string::npos) left_vel_ = std::stod(left_sub.substr(lv + 6, left_sub.find_first_of(",}", lv) - (lv + 6)));

              // Parseo Derecho
              size_t rp = right_sub.find("\"pos\":");
              size_t rv = right_sub.find("\"vel\":");
              if (rp != std::string::npos) right_pos_ = std::stod(right_sub.substr(rp + 6, right_sub.find_first_of(",}", rp) - (rp + 6)));
              if (rv != std::string::npos) right_vel_ = std::stod(right_sub.substr(rv + 6, right_sub.find_first_of(",}", rv) - (rv + 6)));
            }
          } catch (const std::exception &e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Error decodificando JSON: %s", e.what());
          }
        }
      } else if (c != '\r') {
        buffer_acumulador += c;
      }

      // Evitar desbordamiento de memoria por ruido en el serial
      if (buffer_acumulador.length() > 500) {
        buffer_acumulador.clear();
      }
    }
  }

  return hardware_interface::return_type::OK;
}

} // namespace bender_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(bender_hardware_interfaces::EncoderInterface,
                       hardware_interface::SensorInterface)
