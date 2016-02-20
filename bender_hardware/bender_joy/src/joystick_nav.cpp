#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>
#include <bender_joy/xbox_joy.h>

using namespace xbox_joy;

// - - - - - buttons keys - - - - -

// face: emotions, neck
const uint16_t PAUSE    = _START_;
const uint16_t CANCEL_GOAL = _B_;


// TODO: Tal vez sea necesario disminuir la velocidad de publicación al probarlo con bender

class Joystick {

public:
  Joystick();
  void spin();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  // ros stuff
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  ros::ServiceClient cancel_goal_client;

  // INDEXES
  // axes index
  int a_idx_base_linear_, a_idx_base_angular_;

  // buttons index
  int b_idx_pause_;

  // STATES
  // axes level
  double factor_linear_, factor_angular_;

  // button state
  bool is_paused_;

  // latch behavior stuff
  geometry_msgs::Twist _last_twist;
  ros::Duration _latch_timeout;
  ros::Time _last_cmd_vel_time;
};

Joystick::Joystick():
  a_idx_base_linear_(_LS_VERT_),
  a_idx_base_angular_(_LS_HORZ_),
  b_idx_pause_(PAUSE),
  factor_angular_(2),
  factor_linear_(2)
{

  ros::NodeHandle priv("~");

  // Axes Index params
  priv.param("a_idx_base_linear", a_idx_base_linear_, a_idx_base_linear_);
  priv.param("a_idx_base_angular", a_idx_base_angular_,a_idx_base_angular_);
  
  // Buttons Index params
  priv.param("b_idx_pause", b_idx_pause_, b_idx_pause_);

  // other params
  priv.param("factor_angular", factor_angular_, factor_angular_);
  priv.param("factor_linear", factor_linear_, factor_angular_);

  vel_pub_ = priv.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  joy_sub_ = priv.subscribe<sensor_msgs::Joy>("joy", 10, &Joystick::joyCallback, this);
  cancel_goal_client = priv.serviceClient<std_srvs::Empty>("/bender/nav/goal_server/cancel");

  is_paused_ = false;

  // [important!] wait for timer initialization!
  while (ros::ok() && ros::Time::now().isZero());

  _latch_timeout = ros::Duration(1.0);
  _last_cmd_vel_time = ros::Time::now();
}

/**
* joyCallback features:
* - up, down, left, right
* - diferent velocities (depends on angle of stick)
* - pause/resume
*/
void Joystick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {

  geometry_msgs::Twist vel;
  uint16_t buttons = get_button_mask(joy);

  // check for pause
  if ( buttons == PAUSE ) {

      // Stop the robot before pause!
      vel.angular.z = 0;
      vel.linear.x = 0;
      vel_pub_.publish(vel);

      is_paused_ = !is_paused_;
      if (is_paused_) { 
        ROS_WARN("\nControlling PAUSED!, press start button to resume it\n");
      } else {
        ROS_WARN("Controlling RESUMED, press start button to pause it");
      }

      // very important sleep!
      sleep(1); // it must be >= 1;

  }
  if (is_paused_) return;

  // - - - - - - - - - - - - AXES - - - - - - - - - - - - -

  // - - - - handle navigation - - - -
  vel.angular.z = factor_angular_*joy->axes[a_idx_base_angular_];
  vel.linear.x = factor_linear_*joy->axes[a_idx_base_linear_];
  vel_pub_.publish(vel);
  // latch behavior
  _last_twist = vel;
  _last_cmd_vel_time = ros::Time::now();

  // - - - - - - - - - - - BUTTONS - - - - - - - - - - - - -
  switch (buttons) {

	case CANCEL_GOAL: {

		ROS_INFO_THROTTLE(1.0,"Sending cancel navigation goal request...");
		std_srvs::Empty srv;
		if (!cancel_goal_client.call(srv)) {
			ROS_WARN_THROTTLE(1.0,"Failed to send 'cancel' navigation goal");
		}
		break;
	}
	default:
		break;
  }
}

void Joystick::spin() {

	ros::Time now = ros::Time::now();
	if (now - _last_cmd_vel_time > _latch_timeout) {
		return;
	}
	vel_pub_.publish(_last_twist);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "joystick_nav");
  Joystick joystick;

  ros::Rate rate(30);
  while (ros::ok()) {
	  joystick.spin();
	  ros::spinOnce();
	  rate.sleep();
  }
}
