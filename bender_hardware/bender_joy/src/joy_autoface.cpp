#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <bender_msgs/Emotion.h>
#include <bender_srvs/synthesize.h>

#define MAX_FACE_ANGLE 45
#define MIN_FACE_ANGLE -45
#define CENTER_FACE_ANGLE 0


class Joystick {

public:
	Joystick();

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

	// ros stuff
	ros::NodeHandle nh_;
	ros::Publisher face_pub_;
	ros::Subscriber joy_sub_;
	ros::ServiceClient speech_serv_;

	int face_intensity;
	int face_yaw_position;

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

};

Joystick::Joystick():
		  a_idx_base_linear_(0),
		  a_idx_base_angular_(1),
		  b_idx_pause_(7),
		  factor_angular_(2.0),
		  factor_linear_(2.0),
		  face_intensity(1),
		  face_yaw_position(CENTER_FACE_ANGLE)
{


	// Axes Index params
	nh_.param("/joystick/a_idx_base_linear", a_idx_base_linear_, a_idx_base_linear_);
	nh_.param("/joystick/a_idx_base_angular", a_idx_base_angular_,a_idx_base_angular_);

	// Buttons Index params
	nh_.param("/joystick/b_idx_pause", b_idx_pause_, b_idx_pause_);

	// other params
	nh_.param("/joystick/factor_angular", factor_angular_, factor_angular_);
	nh_.param("/joystick/factor_linear", factor_linear_, factor_angular_);

	face_pub_ = nh_.advertise<bender_msgs::Emotion>("/head", 1);
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 1, &Joystick::joyCallback, this);
	speech_serv_ = nh_.serviceClient<bender_srvs::synthesize>("/speech_synthesizer/synthesize");

	is_paused_ = false;
}

/**
 * joyCallback features:
 * - up, down, left, right
 * - diferent velocities (depends on angle of stick)
 * - pause/resume
 */
void Joystick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {

	bender_msgs::Emotion emotion;
	bender_srvs::synthesize speech_text;

	// check for pause
	if ( joy->axes[2]==-1) {

		if ( joy->buttons[8] == 1 ) {
			emotion.Order = "changeFace";
			emotion.Action = "happy1";
			emotion.X = 0;
			face_pub_.publish(emotion);
			ros::Duration(3).sleep();

			emotion.Order = "MoveX";
			emotion.Action = " ";
			emotion.X = MAX_FACE_ANGLE;
			face_pub_.publish(emotion);
			ros::Duration(1).sleep();

			emotion.Order = "MoveX";
			emotion.Action = " ";
			emotion.X = MIN_FACE_ANGLE;
			face_pub_.publish(emotion);
			ros::Duration(2).sleep();

			emotion.Order = "changeFace";
			emotion.Action = "surprise";
			emotion.X = 0;
			face_pub_.publish(emotion);
			ros::Duration(4).sleep();

			emotion.Order = "MoveX";
			emotion.Action = " ";
			emotion.X = MAX_FACE_ANGLE;
			face_pub_.publish(emotion);
			ros::Duration(2).sleep();

			emotion.Order = "changeFace";
			emotion.Action = "serious";
			emotion.X = 0;
			face_pub_.publish(emotion);
			ros::Duration(2).sleep();

			emotion.Order = "MoveX";
			emotion.Action = " ";
			emotion.X = CENTER_FACE_ANGLE;
			face_pub_.publish(emotion);
			ros::Duration(1).sleep();

			emotion.Order = "changeFace";
			emotion.Action = "happy2";
			emotion.X = 0;
			face_pub_.publish(emotion);

			ros::Duration(2).sleep();

		}
	}

}

int main(int argc, char** argv) {

	ros::init(argc, argv, "joystick_autoface");
	Joystick joystick;

	ros::spin();
}
