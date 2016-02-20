#include "bender_pose_server/PoseServer.h"

// TODO: maps folder as parameter 

void operator >> (const YAML::Node& node, geometry_msgs::Point & point) {

	node[0] >> point.x;
	node[1] >> point.y;
	node[2] >> point.z;
}

void operator >> (const YAML::Node& node, geometry_msgs::Quaternion & quaternion) {

	node[0] >> quaternion.w;
	node[1] >> quaternion.x;
	node[2] >> quaternion.y;
	node[3] >> quaternion.z;
}

void operator >> (const YAML::Node& node, bender_msgs::SemanticObject& data) {

	node["id"] >> data.id;
	node["type"] >> data.type;
	node["frame"] >> data.frame_id;
	node["position"] >> data.pose.position;
	node["orientation"] >> data.pose.orientation;
}

namespace bender_knowledge {

PoseServer::PoseServer(std::string name): _name(name){

	ros::NodeHandle priv("~");

	this->_last_loaded_map = "";

	// load default map if necessary
	bool load_map;
	bender_utils::ParameterServerWrapper psw;
	psw.getParameter("load_default_map",load_map, true);
	if (load_map) {
		bender_srvs::String str_srv;
		str_srv.request.data = "map.sem_map";
		this->load(str_srv.request, str_srv.response);
	}

	// Advertise services
	_which_service = priv.advertiseService("which", &PoseServer::which,this);
	_set_service = priv.advertiseService("set", &PoseServer::set,this);
	_get_service = priv.advertiseService("get", &PoseServer::get,this);
	_get_all_service = priv.advertiseService("get_all", &PoseServer::get_all,this);
	_delete_service = priv.advertiseService("delete", &PoseServer::del,this);
	_print_service = priv.advertiseService("print", &PoseServer::print,this);
	_save_service = priv.advertiseService("save", &PoseServer::save,this);
	_load_service = priv.advertiseService("load", &PoseServer::load,this);

	ROS_INFO("Ready to Work");
}

PoseServer::~PoseServer(){

}

bool PoseServer::which(bender_srvs::String::Request  &req, bender_srvs::String::Response &res) {

	res.data = this->_last_loaded_map;

	return true;
}

bool PoseServer::set(bender_srvs::SemMap::Request  &req, bender_srvs::SemMap::Response &res) {

	bender_msgs::SemanticObject data = req.new_data;

	boost::algorithm::trim(data.id);
	boost::algorithm::trim(data.type);
	boost::algorithm::trim(data.frame_id);

	if(data.id == "" || data.type == "" || data.frame_id == "") {

		ROS_ERROR_STREAM("'id', 'type' nor 'frame_id' should be empty");
		return false;
	}

	_map[data.id] = data;

	return true;
}

bool PoseServer::get(bender_srvs::SemMap::Request  &req, bender_srvs::SemMap::Response &res) {

	SemanticMap::iterator it = _map.find(req.id);

	// key = 'id' does not exist
	if( it == _map.end() ) {

		return false;
	}

	res.data.reserve(1);
	res.data.push_back(it->second);

	return true;
}

bool PoseServer::get_all(bender_srvs::SemMap::Request  &req, bender_srvs::SemMap::Response &res) {

	SemanticMap::iterator map_it;

	res.data.reserve(_map.size());
	for(map_it = _map.begin(); map_it != _map.end(); map_it++) {

		res.data.push_back(map_it->second);
	}

	return true;
}

bool PoseServer::del(bender_srvs::String::Request  &req, bender_srvs::String::Response &res) {

	if( _map.erase(req.data) == 0 ) {

		return false;
	}

	return true;
}

bool PoseServer::print(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res) {

	SemanticMap::iterator it;

	std::cout << "- - - - - - - - - - - - -" << std::endl;
	std::cout << "Map contents: " << std::endl;
	for(it = _map.begin(); it != _map.end(); it++){

		std::cout << " - " <<  it->second.id.data() << std::endl;
	}
	std::cout << "- - - - - - - - - - - - -" << std::endl;

	return true;

}

bool PoseServer::save(bender_srvs::String::Request &req, bender_srvs::String::Response &res) {

	YAML::Emitter yaml;
	SemanticMap::iterator it;
	std::string map_path;

	// yaml file
	map_path += ros::package::getPath("bender_maps");
	map_path += "/maps/" + req.data;
	std::ofstream file(map_path.c_str(), std::ios::trunc);

	if( !file.is_open() ){

		ROS_ERROR_STREAM("Cannot open file '" << map_path << "' for save");

		return false;
	}

	ROS_INFO_STREAM("Saving to " << map_path << " . . . ");

	yaml << YAML::BeginSeq;

	for(it = _map.begin(); it != _map.end(); it++) {

		yaml << YAML::BeginMap;
		yaml << YAML::Key << "id" << YAML::Value << it->second.id;
		yaml << YAML::Key << "type" << YAML::Value << it->second.type;
		yaml << YAML::Key << "frame" << YAML::Value << it->second.frame_id;

		yaml << YAML::Key << "position" << YAML::Value;
		yaml << YAML::Flow << YAML::BeginSeq;
		yaml << it->second.pose.position.x;
		yaml << it->second.pose.position.y;
		yaml << it->second.pose.position.z;
		yaml << YAML::EndSeq;

		yaml << YAML::Key << "orientation" << YAML::Value;
		yaml << YAML::Flow << YAML::BeginSeq;
		yaml << it->second.pose.orientation.w;
		yaml << it->second.pose.orientation.x;
		yaml << it->second.pose.orientation.y;
		yaml << it->second.pose.orientation.z;
		yaml << YAML::EndSeq;

		yaml << YAML::EndMap << YAML::Newline;
	}
	yaml << YAML::EndSeq;

	file << yaml.c_str();
	file.close();

	ROS_INFO_STREAM("Map Saved");

	return true;
}

bool PoseServer::load(bender_srvs::String::Request &req, bender_srvs::String::Response &res) {

	std::string map_path;

	// load yaml file
	map_path += ros::package::getPath("bender_maps");
	map_path += "/maps/" + req.data;
	std::ifstream file(map_path.data());

	if(!file.is_open()){
		ROS_ERROR_STREAM("Cannot open file '" << map_path << "' for reading");

		return false;
	}
	ROS_INFO_STREAM("Reading map from: " << map_path << " . . . ");


	// Save previous map (prevention)
	SemanticMap tmp_map = _map;
	_map.clear();

    try {

    	// parse yaml file
    	YAML::Node doc;
#ifdef HAVE_NEW_YAMLCPP
		doc = YAML::Load(file);
#else
		YAML::Parser parser(file);
		parser.GetNextDocument(doc);
#endif

		for (unsigned int k=0; k < doc.size(); k++) {

			bender_msgs::SemanticObject data;
			doc[k] >> data;
			_map[data.id] = data;
		}

	} catch (YAML::ParserException  &ex) {

		ROS_ERROR_STREAM("Error while parsing data: " << ex.what());
		_map = tmp_map;
		file.close();
		return false;

	} catch (YAML::Exception &ex) {
		ROS_ERROR_STREAM("Error while loading the yaml file: " << ex.what());
		_map = tmp_map;
		file.close();
		return false;
	}

	file.close();
	ROS_INFO("Map loaded");

	_last_loaded_map = req.data;

	return true;
}

} /* namespace bender_knowledge */


int main(int argc, char **argv) {

	ros::init(argc, argv, "pose_server");

	boost::scoped_ptr<bender_knowledge::PoseServer> node(
			new bender_knowledge::PoseServer(ros::this_node::getName())
	);

	ros::spin();

	printf("\nQuitting... \n\n");

	return 0;
}
