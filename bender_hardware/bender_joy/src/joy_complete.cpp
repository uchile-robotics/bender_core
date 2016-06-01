
// TODO!: agregar soporte para los brazos

// C++
#include <unistd.h>

// ROS
#include <ros/ros.h>
#include <ros/package.h>

// messages
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>  // navigation
#include <sensor_msgs/Joy.h>      // joystick
#include <bender_msgs/Command.h>  // arm
#include <bender_msgs/Emotion.h>  // face

// services
#include <bender_srvs/synthesize.h>  // speech

/*# # # # # # # # # # # # # # # # # # # # # # # # # # #
# Xbox Controller Button List:                        #
# - - - - - - - - - - - - - - - - - - - - - - - - - - #
# Axis:                                               #
# 0.- LS left, right   # 1.- LS up, down   # 2.- LT   #
# 3.- RS left, right   # 4.- RS up, down   # 5.- RT   #
#                                                     #
# Buttons:                                            #
# 0.- A   # 5.- RB     # 10.- RS                      #
# 1.- B   # 6.- Back   # 11.- Left                    #
# 2.- X   # 7.- Start  # 12.- Right                   #
# 3.- Y   # 8.- Xbox   # 13.- Up                      #
# 4.- LB  # 9.- LS     # 14.- Down                    #
# # # # # # # # # # # # # # # # # # # # # # # # # # #*/

// - - - - BUTTON DEFINITIONS - - - -
const uint16_t _A_       = 1<<0;
const uint16_t _B_       = 1<<1;
const uint16_t _X_       = 1<<2;
const uint16_t _Y_       = 1<<3;
const uint16_t _LB_      = 1<<4;
const uint16_t _RB_      = 1<<5;
const uint16_t _BACK_    = 1<<6;
const uint16_t _START_   = 1<<7;
const uint16_t _XBOX_    = 1<<8;
const uint16_t _LS_      = 1<<9;
const uint16_t _RS_      = 1<<10;
const uint16_t _LEFT_    = 1<<11;
const uint16_t _RIGHT_   = 1<<12;
const uint16_t _UP_      = 1<<13;
const uint16_t _DOWN_    = 1<<14;

// - - - - AXES NUMBERS - - - -
const uint16_t _LS_HORZ_ = 0;
const uint16_t _LS_VERT_ = 1;
const uint16_t _LT_      = 2;
const uint16_t _RS_HORZ_ = 3;
const uint16_t _RS_VERT_ = 4;
const uint16_t _RT_      = 5;

// - - - - - buttons keys - - - - -

// face: emotions, neck
const uint16_t PAUSE    = _START_;
const uint16_t HAPPY    = _A_;
const uint16_t ANGRY    = _B_;
const uint16_t SAD      = _X_;
const uint16_t SURPRISE = _Y_;
const uint16_t FACE_DECREMENT = _LB_;
const uint16_t FACE_INCREMENT = _RB_;
const uint16_t NECK_ACTUATE = _RS_;
const int MAX_FACE_INTENSITY = 3;
const int MIN_FACE_INTENSITY = 1;
const int NECK_ANGLE_LIMIT = 60;

// arm
const uint16_t ARM_INITIAL_POSE = _A_;
const uint16_t ARM_PREPARE      = _B_;
const uint16_t ARM_GRAB         = _X_;
const uint16_t ARM_GRIP         = _Y_;
const uint16_t ARM_TORQUE_OFF   = _BACK_;
const int ARM_IS_SELECTED       = -1;
const int ARM_IS_NOT_SELECTED   = +1;

// speech
const uint16_t SPEECH_INCREMENT = _UP_;
const uint16_t SPEECH_DECREMENT = _DOWN_;
const uint16_t SPEECH_PHRASE_A_	= _LEFT_;
const uint16_t SPEECH_PHRASE_B_	= _RIGHT_;

class Joystick {

public:
	Joystick();
	void synthesize(std::string);
	void speak_Off();
	void show_emotion(std::string emotion);
	void move_head(int angle);
	uint16_t get_button_mask(const sensor_msgs::Joy::ConstPtr& joy);
	void show_button_mask(uint16_t x);

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

	// - - - -  Service Clients - - - -

	// speech
	ros::ServiceClient speech_serv_;

	// - - - - subscribers - - - -
	ros::Subscriber joy_sub_;

	// - - - - publishers - - - -

	// navigation
	ros::Publisher nav_pub_;

	// speech
	ros::Publisher speech_status;

	// face
	ros::Publisher face_pub_;


	// - - - - Button/Axes Indexes - - - -

	// neck
	int axe_idx_neck_sides_;
	int axe_idx_neck_front_;

	// navigation
	int axe_idx_nav_linear_;
	int axe_idx_nav_angular_;

	// arm
	int axe_idx_arm_left_;
	int axe_idx_arm_right_;

	// system
	int button_idx_pause_;


	// - - - - control variables - - - -

	// joystick
	bool is_paused_;

	// face
	int face_intensity;

	// axes level
	double nav_factor_linear_;
	double nav_factor_angular_;

	// speech
	int speech_level;
	std::vector<std::string> phrases;
};


Joystick::Joystick():
	axe_idx_nav_linear_(_LS_VERT_),
	axe_idx_nav_angular_(_LS_HORZ_),
	axe_idx_neck_sides_(_RS_HORZ_),
	axe_idx_neck_front_(_RS_VERT_),
	axe_idx_arm_left_(_LT_),
	axe_idx_arm_right_(_RT_),
	button_idx_pause_(_BACK_),
	nav_factor_angular_(2),
	nav_factor_linear_(2)
	{

	// ros stuff
	ros::NodeHandle priv("~");

	// - - - - params - - - -

	// nav
	priv.param("nav_factor_angular", nav_factor_angular_, nav_factor_angular_);
	priv.param("nav_factor_linear", nav_factor_linear_, nav_factor_angular_);

	// speech
	priv.param("speech_phrases", phrases, phrases);

	if (phrases.size() == 0) {
		ROS_WARN_STREAM("Using Zero phrases for speech!");
	}

	//  - - - - - subscriptions - - - -
	joy_sub_ = priv.subscribe<sensor_msgs::Joy>("joy", 1, &Joystick::joyCallback, this);


	//  - - - - - service clients - - - -

	// speech
	speech_serv_ = priv.serviceClient<bender_srvs::synthesize>("/bender/speech/synthesizer/synthesize");

	// face
	face_pub_ = priv.advertise<bender_msgs::Emotion>("/bender/face/head", 1);


	// - - - - publishers - - -

	// navigation
	nav_pub_ = priv.advertise<geometry_msgs::Twist>("/bender/joy/joystick_nav/cmd_vel", 1);


	// - - - - control variables - - - -

	// joystick
	is_paused_ = false;

	// face
	face_intensity = 1;

	// speech
	speech_level = 0;

}

void Joystick::show_emotion(std::string emotion){
	bender_msgs::Emotion msg;
	msg.Order = "changeFace";
	msg.Action = emotion;
	msg.X = 0;
	face_pub_.publish(msg);
}

void Joystick::move_head(int angle) {
	bender_msgs::Emotion msg;
	msg.Order = "MoveX";
	msg.Action = "";
	msg.X = angle;
	face_pub_.publish(msg);
}

/*void  Joystick::speak_Off(){
	bender_msgs::Emotion emotion;
	emotion.Order = "changeFace";
	emotion.Action = "speakOff";
	emotion.X = 0;
	face_pub_.publish(emotion);
}*/


void  Joystick::synthesize(std::string text){
	bender_srvs::synthesize speech_text;
	std::string talk = text;
	std::string text_evaluate = text;

	std::size_t found_enter,found_emotion;
	bool end = false;
	std::string emo;
	while(!end){
		found_enter = text_evaluate.find("//");//Enter
		found_emotion = text_evaluate.find("@");//Enter
		talk = text_evaluate;
		if (found_enter!=std::string::npos && found_enter<found_emotion){
			talk =  text_evaluate.substr (0,found_enter);
			text_evaluate = text_evaluate.substr (found_enter+3);
		}
		if (found_emotion!=std::string::npos && found_emotion<found_enter){
			talk =  text_evaluate.substr (0,found_emotion);

			std::size_t found_space = text_evaluate.find(" ",found_emotion);
			if (found_space!=std::string::npos){
				emo = text_evaluate.substr (found_emotion+1,found_space - found_emotion -1);
				text_evaluate = text_evaluate.substr (found_space);
			}else{
				emo = text_evaluate.substr (found_emotion+1);
				text_evaluate = text_evaluate.substr (found_emotion);
				end=true;
			}
			show_emotion(emo);
		}
		if (found_enter==std::string::npos && found_emotion==std::string::npos ) end=true;
		speech_text.request.text = talk;
		speech_serv_.call(speech_text);
		ros::Duration(talk.size()/9).sleep();

	}

}

/**
 * Displays <x> as a 16 bit chain
 */
void Joystick::show_button_mask(uint16_t x) {

	for(int i=(sizeof(uint16_t)*8)-1; i>=0; i--) {
		(x&(1<<i))?putchar('1'):putchar('0');
	}
	printf("\n");
}

uint16_t Joystick::get_button_mask(const sensor_msgs::Joy::ConstPtr& joy) {

	uint16_t mask = 1;
	uint16_t result = 0;

	//printf("\n----\n");
	for (int i = 0; i < joy->buttons.size(); i++) {

		//printf("mask:   "); show_button_mask(mask);

		if (joy->buttons[i]) {
			result |= mask;
		}
		mask <<= 1;

	}
	//printf("result: "); show_button_mask(result);
	return result;
}

/**
 * joyCallback features:
 * - emotions    : happy{123}, angry{123}, sad{123}, surprise
 * - arm         : pick & place style motion
 * - talk        : synthesizes a bunch of predefined phrases
 * - navigation  : velocity depends on angle of stick
 * - pause/resume: pauses the joystick
 */
void Joystick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {

	std_srvs::Empty dum;

	uint16_t buttons = get_button_mask(joy);

	// check for pause
	if (buttons == PAUSE) {

		// Stop the robot before pause!
		geometry_msgs::Twist vel;
		vel.angular.z = 0;
		vel.linear.x = 0;
		nav_pub_.publish(vel);

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
	geometry_msgs::Twist vel;
	vel.angular.z = nav_factor_angular_*joy->axes[axe_idx_nav_angular_];
	vel.linear.x = nav_factor_linear_*joy->axes[axe_idx_nav_linear_];
	nav_pub_.publish(vel);

	// - - - - handle neck - - - -
	float neck_side  = joy->axes[axe_idx_neck_sides_];
	float neck_front = joy->axes[axe_idx_neck_front_];
	int angle = (int)(180*atan2f(neck_side,neck_front)/M_PI);
	angle = std::min(std::max(-NECK_ANGLE_LIMIT,angle),NECK_ANGLE_LIMIT);
	ROS_INFO_STREAM_THROTTLE(0.1, " - (obs) neck angle: " << angle << "[deg]");
	if (buttons == NECK_ACTUATE) {

		// command is valid only for axe limits
		if ( neck_side*neck_side + neck_front*neck_front > 0.5 ) {
			move_head(angle);
			ROS_INFO_STREAM("Setting neck angle: " << angle << "[deg]");
		}
	}


	//  - - - - both arms - - - -
	if ( joy->axes[axe_idx_arm_left_] == ARM_IS_SELECTED && joy->axes[axe_idx_arm_right_] == ARM_IS_SELECTED ) {
		switch (buttons) {
			case ARM_TORQUE_OFF: {

				// TODO
				ROS_WARN("TORQUE OFF for both arms: NOT IMPLEMENTED");
				break;
			}
			default:
				break;
		}

		return;
	}

	// - - - - left arm - - - -
	if ( joy->axes[axe_idx_arm_left_] == ARM_IS_SELECTED && joy->axes[axe_idx_arm_right_] == ARM_IS_NOT_SELECTED ) {
		switch (buttons) {
			case ARM_TORQUE_OFF: {
				ROS_WARN("TORQUE OFF for left arm: NOT IMPLEMENTED");
				break;
			}
			case ARM_INITIAL_POSE:{
				ROS_WARN("ARM_INITIAL_POSE for left arm is NOT IMPLEMENTED");
				break;
			}
			case ARM_PREPARE: {
				ROS_WARN("ARM_PREPARE for left arm is NOT IMPLEMENTED");
				break;
			}
			case ARM_GRAB: {

				ROS_WARN("ARM_GRAB for left arm is NOT IMPLEMENTED");
				break;
			}
			case ARM_GRIP: {

				ROS_WARN("ARM_GRIP usage for left arm is NOT IMPLEMENTED");
				break;
			}
			default:
				break;
		}
		return;
	}

	// right arm axis(5) + command
	if ( joy->axes[axe_idx_arm_left_] == ARM_IS_NOT_SELECTED && joy->axes[axe_idx_arm_right_] == ARM_IS_SELECTED ) {
		switch (buttons) {
			case ARM_TORQUE_OFF: {
				ROS_WARN("ARM_TORQUE_OFF for right arm is NOT IMPLEMENTED");
				break;
			}
			case ARM_INITIAL_POSE:{
				ROS_WARN("ARM_INITIAL_POSE for right arm is NOT IMPLEMENTED");
				break;
			}
			case ARM_PREPARE: {
				ROS_WARN("ARM_PREPARE for right arm is NOT IMPLEMENTED");
				break;
			}
			case ARM_GRAB: {
				ROS_WARN("ARM_GRAB for right arm is NOT IMPLEMENTED");
				break;
			}
			case ARM_GRIP: {
				ROS_WARN("ARM_GRIP usage for right arm is NOT IMPLEMENTED");
				break;
			}
			default:
				break;
		}
		return;
	}

	// - - - - - - - - - - - BUTTONS - - - - - - - - - - - - -
	switch (buttons) {

		// - - - - face emotions - - - -
		case HAPPY: {
			std::stringstream sstm;
			sstm << "happy" << face_intensity;
			ROS_INFO_STREAM("happy" << face_intensity);
			show_emotion(sstm.str());

			break;
		}
		case ANGRY: {
			std::stringstream sstm;
			sstm << "angry" << face_intensity;
			ROS_INFO_STREAM("angry" << face_intensity);
			show_emotion(sstm.str());

			break;
		}
		case SAD: {
			std::stringstream sstm;
			sstm << "sad" << face_intensity;
			ROS_INFO_STREAM("sad" << face_intensity);
			show_emotion(sstm.str());

			break;
		}
		case SURPRISE: {
			ROS_INFO_STREAM("surprise");
			show_emotion("surprise");

			break;
		}
		case FACE_DECREMENT: {
			face_intensity = std::max(MIN_FACE_INTENSITY,face_intensity-1);
			ROS_INFO_STREAM("Emotion intensity decrement: " << face_intensity);

			break;
		}
		case FACE_INCREMENT: {
			face_intensity = std::min(MAX_FACE_INTENSITY,face_intensity+1);
			ROS_INFO_STREAM("Emotion intensity increment: " << face_intensity);

			break;
		}
		case SPEECH_INCREMENT: {

			if (!phrases.empty()) {
				int levels = ceil(phrases.size()/2.0);
				speech_level = (speech_level+1)%levels;
				ROS_INFO_STREAM("Speech level: " << speech_level + 1 << "/" << levels);
			}
			break;
		}
		case SPEECH_DECREMENT: {

			if (!phrases.empty()) {
				int levels = ceil(phrases.size()/2.0);
				speech_level--;
				if (speech_level < 0) {
					speech_level = levels-1;
				}
				ROS_INFO_STREAM("Speech level: " << speech_level + 1 << "/" << levels);
			}

			break;

		}
		case SPEECH_PHRASE_A_: {

			if (!phrases.empty()) {
				int phrase_no = speech_level*2;
				std::string text = phrases[phrase_no];
				synthesize(text);
				ROS_INFO_STREAM("Synthesizing text: " << text);
			}


			break;
		}
		case SPEECH_PHRASE_B_: {

			if (!phrases.empty()) {
				int phrase_no = (speech_level*2 + 1)%phrases.size();
				std::string text = phrases[phrase_no];
				synthesize(text);
				ROS_INFO_STREAM("Synthesizing text: " << text);
			}

			break;
		}
		default:
			break;
	}





}

int main(int argc, char** argv) {

	ros::init(argc, argv, "joystick_complete");

	Joystick joystick;


//	joystick.inauguracion();
	ros::spin();
}
