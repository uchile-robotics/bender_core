#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

// Mapeo del control Xbox 360 con joy_linux:
//   botones: 0 A, 1 B, 2 X, 3 Y
//   ejes:    0 stick izq. X, 1 stick izq. Y, 2 LT, 3 stick der. X, 4 stick der. Y
//
// Modo base (por defecto): stick izq. Y = avance, stick der. X = giro.
// Modo cabeza (se alterna con X): stick izq. X = yaw, stick der. Y = pitch,
//   stick der. X = roll, Y = centrar la cabeza. La base no recibe comandos.
// En ambos modos B frena todo hasta apretar A + LT.

class TeleopJoy : public rclcpp::Node
{
public:
  TeleopJoy()
  : Node("teleop_node")
  {
    scale_linear_ = declare_parameter<double>("scale_linear", 1.0);
    scale_angular_ = declare_parameter<double>("scale_angular", 1.0);

    head_mode_button_ = declare_parameter<int>("head_mode_button", 2);
    head_center_button_ = declare_parameter<int>("head_center_button", 3);
    head_joints_ = declare_parameter<std::vector<std::string>>(
      "head_joints", {"l1h_to_base_link", "l2h_to_l1h", "l3h_to_l1h"});
    // rad/s con el stick al tope, en el orden de head_joints (yaw, pitch, roll)
    head_rates_ = declare_parameter<std::vector<double>>("head_rates", {1.5, 1.0, 1.0});
    head_min_ = declare_parameter<std::vector<double>>("head_min", {-3.229, -0.70, -0.70});
    head_max_ = declare_parameter<std::vector<double>>("head_max", {1.948, 0.70, 0.70});

    if (head_joints_.size() != 3 || head_rates_.size() != 3 ||
      head_min_.size() != 3 || head_max_.size() != 3)
    {
      throw std::runtime_error("head_joints, head_rates, head_min and head_max need 3 values");
    }

    head_measured_.assign(3, 0.0);
    head_target_.assign(3, 0.0);

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel_joy", rclcpp::QoS(10));
    head_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/bender/head_controller/joint_trajectory", rclcpp::QoS(10));

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::QoS(10),
      std::bind(&TeleopJoy::joyCallback, this, std::placeholders::_1));
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/bender/joint_states", rclcpp::QoS(10),
      std::bind(&TeleopJoy::jointStateCallback, this, std::placeholders::_1));

    head_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(kHeadPeriod * 1000)),
      std::bind(&TeleopJoy::headTimerCallback, this));
  }

private:
  static constexpr double kHeadPeriod = 0.05;  // s
  static constexpr double kDeadzone = 1e-3;
  // joy_linux repite el último estado a 20 Hz (autorepeat_rate)
  static constexpr double kJoyTimeout = 0.3;  // s

  static int button(const sensor_msgs::msg::Joy & msg, int i)
  {
    return (i >= 0 && i < static_cast<int>(msg.buttons.size())) ? msg.buttons[i] : 0;
  }

  static double axis(const sensor_msgs::msg::Joy & msg, int i)
  {
    return (i >= 0 && i < static_cast<int>(msg.axes.size())) ? msg.axes[i] : 0.0;
  }

  bool pressed(const sensor_msgs::msg::Joy & msg, int i) const
  {
    const int prev = (i >= 0 && i < static_cast<int>(prev_buttons_.size())) ? prev_buttons_[i] : 0;
    return button(msg, i) == 1 && prev == 0;
  }

  void publishStop()
  {
    geometry_msgs::msg::TwistStamped v;
    v.header.stamp = now();
    v.header.frame_id = "base_link";
    cmd_vel_pub_->publish(v);
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t j = 0; j < head_joints_.size(); ++j) {
      auto it = std::find(msg->name.begin(), msg->name.end(), head_joints_[j]);
      if (it == msg->name.end()) {
        continue;
      }
      const size_t idx = std::distance(msg->name.begin(), it);
      if (idx < msg->position.size()) {
        head_measured_[j] = msg->position[idx];
        head_state_received_ = true;
      }
    }
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    last_joy_ = *msg;
    last_joy_time_ = now();
    have_joy_ = true;

    if (latched_) {
      publishStop();
      if (button(*msg, 0) == 1 && axis(*msg, 2) < -0.9) {
        latched_ = false;
        RCLCPP_INFO(get_logger(), "E-stop released");
      }
      prev_buttons_ = msg->buttons;
      return;
    }

    if (button(*msg, 1) == 1) {
      latched_ = true;
      publishStop();
      RCLCPP_WARN(get_logger(), "E-stop latched (A + LT to release)");
      prev_buttons_ = msg->buttons;
      return;
    }

    if (pressed(*msg, head_mode_button_)) {
      toggleHeadMode();
    }

    if (head_mode_) {
      if (pressed(*msg, head_center_button_)) {
        std::fill(head_target_.begin(), head_target_.end(), 0.0);
        publishHead(head_target_, 2.0);
      }
    } else {
      driveBase(*msg);
    }

    prev_buttons_ = msg->buttons;
  }

  void toggleHeadMode()
  {
    if (!head_mode_ && !head_state_received_) {
      RCLCPP_WARN(get_logger(), "No head joint states yet, staying in base mode");
      return;
    }
    head_mode_ = !head_mode_;
    if (head_mode_) {
      // Parte desde donde está la cabeza, no desde el último objetivo
      head_target_ = head_measured_;
      publishStop();
    }
    RCLCPP_INFO(get_logger(), "Teleop mode: %s", head_mode_ ? "HEAD" : "BASE");
  }

  void driveBase(const sensor_msgs::msg::Joy & msg)
  {
    const double lin = axis(msg, 1) * scale_linear_;
    const double ang = axis(msg, 3) * scale_angular_;

    if (std::abs(lin) > kDeadzone || std::abs(ang) > kDeadzone) {
      geometry_msgs::msg::TwistStamped v;
      v.header.stamp = now();
      v.header.frame_id = "base_link";
      v.twist.linear.x = lin;
      v.twist.angular.z = ang;
      cmd_vel_pub_->publish(v);
    }
  }

  // Integra la velocidad pedida con los sticks a una posición objetivo para la cabeza
  void headTimerCallback()
  {
    if (!head_mode_ || latched_ || !have_joy_) {
      return;
    }
    // Si se corta el joystick con el stick apretado, que la cabeza no siga moviéndose
    if ((now() - last_joy_time_).seconds() > kJoyTimeout) {
      return;
    }

    // yaw: stick izq. a la izquierda gira a la izquierda (+z)
    // pitch: stick der. arriba mira hacia arriba (-y)
    // roll: stick der. a la derecha inclina a la derecha (+x)
    const double input[3] = {
      axis(last_joy_, 0),
      -axis(last_joy_, 4),
      -axis(last_joy_, 3),
    };

    const std::vector<double> previous_target = head_target_;
    bool moved = false;
    for (size_t j = 0; j < 3; ++j) {
      if (std::abs(input[j]) <= kDeadzone) {
        continue;
      }
      head_target_[j] = std::clamp(
        head_target_[j] + input[j] * head_rates_[j] * kHeadPeriod, head_min_[j], head_max_[j]);
      moved = true;
    }

    if (moved) {
      // Con un solo punto el controlador interpola desde la posición medida, que va
      // atrasada, y cada mensaje nuevo vuelve a partir desde atrás (respuesta muy suave).
      // Partiendo del objetivo anterior en t=0 la referencia avanza a velocidad constante.
      publishHead(head_target_, kHeadPeriod, &previous_target);
    }
  }

  void publishHead(
    const std::vector<double> & target, double duration,
    const std::vector<double> * start = nullptr)
  {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = head_joints_;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    if (start) {
      point.positions = *start;
      point.time_from_start = rclcpp::Duration::from_seconds(0.0);
      traj.points.push_back(point);
    }
    point.positions = target;
    point.time_from_start = rclcpp::Duration::from_seconds(duration);
    traj.points.push_back(point);
    head_pub_->publish(traj);
  }

  bool latched_ = false;
  bool head_mode_ = false;
  bool head_state_received_ = false;
  bool have_joy_ = false;

  double scale_linear_;
  double scale_angular_;
  int head_mode_button_;
  int head_center_button_;
  std::vector<std::string> head_joints_;
  std::vector<double> head_rates_;
  std::vector<double> head_min_;
  std::vector<double> head_max_;
  std::vector<double> head_measured_;
  std::vector<double> head_target_;

  sensor_msgs::msg::Joy last_joy_;
  rclcpp::Time last_joy_time_;
  std::vector<int> prev_buttons_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr head_pub_;
  rclcpp::TimerBase::SharedPtr head_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleopJoy>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
