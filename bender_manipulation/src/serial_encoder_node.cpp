#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdexcept>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class SerialEncoderReader : public rclcpp::Node {
public:
    SerialEncoderReader() : Node("serial_encoder_node"), serial_fd_(-1) {
        // Parámetros de conexión
        this->declare_parameter<std::string>("port", "/dev/encoders");
        this->declare_parameter<int>("baudrate", 115200);

        std::string port = this->get_parameter("port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();

        // Inicializar puerto serial
        init_serial(port, baudrate);

        // Publisher: Solo publicamos el ángulo leído
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("shoulder/angle", 10);

        // Timer: Revisar el puerto serial cada 0.01 segundos (100 Hz)
        timer_ = this->create_wall_timer(
            10ms, std::bind(&SerialEncoderReader::serial_callback, this));
    }

    ~SerialEncoderReader() {
        // Cerrar el puerto serial de forma segura al destruir el nodo
        if (serial_fd_ != -1) {
            close(serial_fd_);
        }
    }

private:
    int serial_fd_;
    std::string buffer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void init_serial(const std::string& port, int baudrate) {
        // Abrir puerto en modo lectura/escritura, sin ser controlador de terminal, y no bloqueante (O_NDELAY)
        serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo abrir el puerto serial: %s", port.c_str());
            exit(1);
        }

        struct termios options;
        tcgetattr(serial_fd_, &options);

        // Asignar los baudios correspondientes. Por simplicidad manejamos 115200 y 9600.
        speed_t speed = B115200;
        if (baudrate == 9600) speed = B9600;
        // Puedes agregar más casos (B57600, B38400, etc.) si los necesitas

        cfsetispeed(&options, speed);
        cfsetospeed(&options, speed);

        // Configuración 8N1 (8 bits de datos, sin paridad, 1 bit de parada), modo "raw"
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_oflag &= ~OPOST;

        tcsetattr(serial_fd_, TCSANOW, &options);
        fcntl(serial_fd_, F_SETFL, FNDELAY); // Asegurarse de que las lecturas no sean bloqueantes

        RCLCPP_INFO(this->get_logger(), "Conectado al encoder en %s a %d baudios.", port.c_str(), baudrate);
    }

    void serial_callback() {
        if (serial_fd_ == -1) return;

        char buf[256];
        // Leer todo lo que esté disponible en el buffer del sistema
        int bytes_read = read(serial_fd_, buf, sizeof(buf) - 1);

        if (bytes_read > 0) {
            buf[bytes_read] = '\0';
            buffer_ += buf; // Acumular en nuestro buffer interno

            // Buscar saltos de línea (simula readline)
            size_t pos;
            while ((pos = buffer_.find('\n')) != std::string::npos) {
                std::string line = buffer_.substr(0, pos);
                buffer_.erase(0, pos + 1);

                // Quitar '\r' y espacios (simula strip())
                if (!line.empty() && line.back() == '\r') {
                    line.pop_back();
                }

                if (!line.empty()) {
                    try {
                        // Convertimos a float y publicamos
                        float angle_value = std::stof(line);

                        auto msg = std_msgs::msg::Float32();
                        msg.data = angle_value;
                        publisher_->publish(msg);

                        // Opcional: Debug para ver qué llega
                        // RCLCPP_INFO(this->get_logger(), "Encoder: %f", angle_value);

                    } catch (const std::invalid_argument& e) {
                        // A veces llega basura por el serial (Equivalente a ValueError en Python)
                    } catch (const std::out_of_range& e) {
                        // Ignorar si el número excede los límites de float
                    } catch (const std::exception& e) {
                        RCLCPP_WARN(this->get_logger(), "Error leyendo serial: %s", e.what());
                    }
                }
            }
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialEncoderReader>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
