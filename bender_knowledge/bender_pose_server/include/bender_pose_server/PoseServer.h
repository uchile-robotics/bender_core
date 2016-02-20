/*
 * PoseServer.h
 *
 *  Created on: Nov , 2013
 *      Author: matias.pavez.b@gmail.com
 */

#ifndef POSESERVER_H_
#define POSESERVER_H_

// C, C++
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <vector>
#include <map>
#include <fstream>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include <boost/scoped_ptr.hpp>

// ROS
#include "ros/ros.h"
#include <ros/package.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Empty.h>

// Bender
#include <bender_utils/ParameterServerWrapper.h>
#include <bender_msgs/SemanticObject.h>
#include <bender_srvs/SemMap.h>
#include <bender_srvs/String.h>

// YAML Parsing & Writing
#include <yaml-cpp/yaml.h>


#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i) {
	i = node.as<T>();
}
#endif


typedef std::map<std::string, bender_msgs::SemanticObject> SemanticMap;
typedef std::vector<bender_msgs::SemanticObject> SemanticObjectArray;

namespace bender_knowledge {

class PoseServer {

private:

	std::string _name;
	std::string _last_loaded_map;

	// Services
	ros::ServiceServer _which_service;
	ros::ServiceServer _set_service;
	ros::ServiceServer _get_service;
	ros::ServiceServer _get_all_service;
	ros::ServiceServer _delete_service;
	ros::ServiceServer _print_service;
	ros::ServiceServer _save_service;
	ros::ServiceServer _load_service;

	// Map
	SemanticMap _map;

public:
	PoseServer(std::string name);
	virtual ~PoseServer();

	/**
	 * Returns the name of the last loaded map
	 */
	bool which(bender_srvs::String::Request  &req, bender_srvs::String::Response &res);

	/**
	 * Inserts a SemanticObject in memory.
	 * If the object already exists, then it is replaced.
	 * It does not modifies the map file
	 *
	 * returns false: if 'id', 'type' or 'frame_id' fields are empty
	 * returns true: otherwise
	 */
	bool set(bender_srvs::SemMap::Request  &req, bender_srvs::SemMap::Response &res);

	/**
	 * Gets a SemanticObject with the specified 'id' from memory.
	 * It does not uses the map file
	 *
	 * returns false: if a object with that 'id' does not exist
	 * returns true: otherwise
	 */
	bool get(bender_srvs::SemMap::Request  &req, bender_srvs::SemMap::Response &res);

	/**
	 * Gets all the SemanticObject(s) from memory.
	 * It does not uses the map file
	 */
	bool get_all(bender_srvs::SemMap::Request  &req, bender_srvs::SemMap::Response &res);

	/**
	 * Deletes a SemanticObject with the specified 'id' from memory.
	 * It does not modifies the map file
	 *
	 * returns false: if a object with that 'id' does not exist
	 * returns true: otherwise
	 */
	bool del(bender_srvs::String::Request  &req, bender_srvs::String::Response &res);

	/**
	 * Prints current map contents on screen
	 */
	bool print(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);

	/**
	 * Saves the current map contents on the specified file
	 */
	bool save(bender_srvs::String::Request &req, bender_srvs::String::Response &res);

	/**
	 * Loads the current map contents from the specified file
	 */
	bool load(bender_srvs::String::Request &req, bender_srvs::String::Response &res);

};

} /* namespace bender_knowledge */

#endif /* POSESERVER_H_ */
