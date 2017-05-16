#ifndef BENDER_TTS_SYNTHESIZER_H_
#define BENDER_TTS_SYNTHESIZER_H_

// C, C++
#include <stdio.h>
#include <string>
#include <boost/scoped_ptr.hpp>
#include <map>

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <sound_play/sound_play.h>
#include <diagnostic_msgs/DiagnosticArray.h>

// Bender
#include <uchile_srvs/String.h>
#include <bender_utils/ParameterServerWrapper.h>

namespace bender_tts {

class synthesizer {

private:

	ros::NodeHandle nh_;
	std::string _current_language;
	std::string _voice_name;
	std::string _soundplay_node_name;
	sound_play::SoundClient sc;
	bool _advertised_talking;
	bool _advertised_not_talking;

	// dictionary: language -> voice
	std::map<std::string, std::string> _language_dictionary;

	// Base msgs
	std_msgs::String text_msg_;
	std_msgs::Bool is_talking_msg_;

	// - - - - - P u b l i s h e r s - - - - - - -
	ros::Publisher _is_talking_pub;
	ros::Publisher _text_pub;

	// - - - - - L i s t e n e r s - - - - - - - -
	ros::Subscriber _voice_diagnostics_sub;

	// - - - - - - S e r v i c e s - - - - - - - -
	ros::ServiceServer _synthesize_server;
	ros::ServiceServer _stop_server;
	ros::ServiceServer _set_language_server;

	// - - - - - Plugins - - - - -
	// pronunciation
	std::map<std::string, std::string> _pronunciation_map;
	std::string pronunciation_plugin(std::string input);
	void pronunciation_plugin_update();	

public:
	synthesizer();
	~synthesizer();

private:

	bool synthesize_server(uchile_srvs::String::Request &req, uchile_srvs::String::Response &res);

	bool stop_server(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

	bool set_language_server(uchile_srvs::String::Request &req, uchile_srvs::String::Response &res);

	void status_calculation_callback(const diagnostic_msgs::DiagnosticArray &msg);

	bool update_language(std::string desired_language);

};

} /* namespace bender_tts */
#endif /* BENDER_TTS_SYNTHESIZER_H_ */
