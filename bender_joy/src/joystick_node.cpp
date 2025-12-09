#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TeleopJoy : public rclcpp::Node
{
public:
  TeleopJoy()
  : Node("teleop_node"),
    latched_(false)
  {
    // Declare parameters
    this->declare_parameter<double>("scale_linear", 1.0);
    this->declare_parameter<double>("scale_angular", 1.0);

    scale_linear_ = this->get_parameter("scale_linear").as_double();
    scale_angular_ = this->get_parameter("scale_angular").as_double();

    // Publisher
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel_joy", rclcpp::QoS(10));

    // Subscriber
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy",
      rclcpp::QoS(10),
      std::bind(&TeleopJoy::joyCallback, this, std::placeholders::_1)
    );
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    geometry_msgs::msg::Twist v;
    if (latched_) {
      cmd_vel_pub_->publish(v);
      if (msg->buttons[0] == 1 && msg->axes[2] < -0.9) {
        latched_ = false;
      }
      return;
    }

    if (msg->buttons[1] == 1) {
      latched_ = true;
      cmd_vel_pub_->publish(v);
      return;
    }

    v.linear.x  = msg->axes[1] * scale_linear_;
    v.angular.z = msg->axes[3] * scale_angular_;

    cmd_vel_pub_->publish(v);
  }

  bool latched_;
  double scale_linear_;
  double scale_angular_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleopJoy>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
