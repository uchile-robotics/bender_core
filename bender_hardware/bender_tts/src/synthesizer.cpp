#include "bender_tts/synthesizer.h"

namespace bender_tts {

synthesizer::synthesizer(): nh_("~") {

	// setup control variables
	_advertised_talking = false;
	_advertised_not_talking = true;

	// available languages
	_language_dictionary["spanish"] = "voice_el_diphone";
	_language_dictionary["english"] = "voice_us2_mbrola";

	// default language and voice name
	_current_language = "english";
	_voice_name = _language_dictionary[_current_language];

	// - - - - - - P a r a m e t e r s - - - - - - - - - - - -
	// used to signal synthesis status
	nh_.setParam("talking", false);

	// utility for parameter handling
	bender_utils::ParameterServerWrapper psw;

	// used to know if there is any sound playing
	psw.getParameter("sound_play_node_name", _soundplay_node_name, "sound_play");

	// setup language
	std::string desired_language;
	psw.getParameter("language", desired_language, "english");   // english, spanish
	update_language(desired_language);


	// - - - - - P u b l i s h e r s - - - - - - -
	_is_talking_pub = nh_.advertise<std_msgs::Bool>("is_talking",1);
	_text_pub = nh_.advertise<std_msgs::String>("text",1);

	// - - - - - L i s t e n e r s - - - - - - - -
	_voice_diagnostics_sub = nh_.subscribe("diagnostics",1,&synthesizer::status_calculation_callback,this);

	// - - - - - - S e r v i c e s - - - - - - - -
	_synthesize_server = nh_.advertiseService("say",&synthesizer::synthesize_server,this);
	_set_language_server = nh_.advertiseService("set_language", &synthesizer::set_language_server,this);
	_stop_server = nh_.advertiseService("stop_speech",&synthesizer::stop_server,this);

}

synthesizer::~synthesizer() {

}

bool synthesizer::update_language(std::string desired_language) {

	std::map<std::string,std::string>::iterator it;
	it = _language_dictionary.find(desired_language);

	// valid language --> update values
	if (it != _language_dictionary.end()) {
		_current_language = desired_language;
		_voice_name = it->second;

		// plugins
		pronunciation_plugin_update();

		ROS_INFO_STREAM("Synthesizer language set to: '" << desired_language << "'");
		return true;
	}

	// unknown language :(
	ROS_WARN_STREAM("Failed to set synthesizer language to '" << desired_language
				<< "'. Current active language is: '" << _current_language << "'");
	return false;
}

bool synthesizer::set_language_server(uchile_srvs::String::Request &req, uchile_srvs::String::Response &res) {

	std::string desired_language = req.data;
	return update_language(desired_language);
}

void synthesizer::pronunciation_plugin_update() {

	// TODO
	_pronunciation_map.clear();
	_pronunciation_map["facebook"] = "feisbuc";
	_pronunciation_map["bender"]   = "b'ender";
}

std::string synthesizer::pronunciation_plugin(std::string input) {

	//ROS_WARN_STREAM("plugin input: '" << input << "', for language: " << _current_language);
	std::string output = input;

	std::map<std::string, std::string>::iterator it;
	for (it=_pronunciation_map.begin(); it!=_pronunciation_map.end(); ++it) {

		//ROS_INFO_STREAM("current key: " << it->first);

		size_t idx;
		while ( (idx = output.find(it->first)) != std::string::npos) {
			output.replace(idx, it->first.length(), it->second);
			//ROS_INFO_STREAM("-> replacing on idx: " << idx);
		}
	}
	//ROS_WARN_STREAM("plugin output: '" << output << "', for language: " << _current_language);
	return output;
}

bool synthesizer::synthesize_server(uchile_srvs::String::Request &req,	uchile_srvs::String::Response &res) {

	//ROS_WARN_STREAM("synthesize: " << req.data);
	text_msg_.data = pronunciation_plugin(req.data);
	//ROS_WARN_STREAM("synthesize updated: " << text);

	// synthesize with modified text (ugly text, specialized for synthesis)
	sc.say(text_msg_.data, _voice_name);
	is_talking_msg_.data = true;
	_is_talking_pub.publish(is_talking_msg_);

	// publish original text! (nice text)
	std_msgs::String msg;
	msg.data = req.data;
	_text_pub.publish(msg);

	res.data = "Talking";

	nh_.setParam("talking",true);

	return true;
}

bool synthesizer::stop_server(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
	sc.stopAll();
	return true;
}

void synthesizer::status_calculation_callback(const diagnostic_msgs::DiagnosticArray &msg) {

	bool valid_msg = false;
	bool sound_playing = false;
	diagnostic_msgs::DiagnosticStatus status;
	for (int i=0; i<msg.status.size(); ++i) {

		status = msg.status[i];
		if (status.name.find(_soundplay_node_name) != std::string::npos && // correct diagnostic provider
			status.message.find("sounds playing") != std::string::npos )   // correct message data
		{
			valid_msg = true;

			diagnostic_msgs::KeyValue item;
			for (int j=0; j<status.values.size(); ++j) {
				item = status.values[j];

				if (item.key == "Active sounds") {

					int active_sounds = std::atoi(item.value.c_str());
					sound_playing = active_sounds > 0 ? true : false;
					break;
				}
			}
			break;
		}
	}

	// skip invalid messages
	if (!valid_msg) {
		return;
	}



	// set 'talking' to true/false only the first time!
	if(sound_playing) {

		is_talking_msg_.data = true;

		if (!_advertised_talking) {

			_advertised_talking = true;
			_advertised_not_talking = false;
			ROS_INFO("Talking");
		}
	} else {

		is_talking_msg_.data = false;

		if (!_advertised_not_talking) {

			nh_.setParam("talking",false);
			_advertised_not_talking = true;
			_advertised_talking = false;
			ROS_INFO("Not Talking");
		}
	}

	// publish always
	_is_talking_pub.publish(is_talking_msg_);
}

} /* namespace bender_speech */

int main(int argc, char **argv) {
	ros::init(argc, argv, "tts");

	bender_tts::synthesizer s;

	ros::AsyncSpinner spinner(2);
	spinner.start();

	while(ros::ok()) {
		ros::waitForShutdown();
	}

	return 0;
}
