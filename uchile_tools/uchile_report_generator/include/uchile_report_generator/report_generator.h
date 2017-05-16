/*
 * ReportGenerator.h
 */

#ifndef REPORT_GENERATOR_H_
#define REPORT_GENERATOR_H_

// C, C++
#include <string>
#include <fstream>
#include <boost/scoped_ptr.hpp>

// opencv2
#include <opencv2/opencv.hpp>

// ROS
#include "ros/ros.h"
#include <ros/package.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <std_srvs/Empty.h>
#include <cv_bridge/cv_bridge.h>

// Bender
#include <uchile_util/ParameterServerWrapper.h>
#include <uchile_srvs/ReportGenerator.h>


namespace uchile_report_generator {

class ReportGenerator {

private:
	std::string _name;

	std::string _pkg_path;

	// tex templates
	std::string _main_template;
	std::string _map_template;
	std::string _img_template;

	// compilation command
	std::string _compile_cmd;

	// gaps
	std::string _gap_img_path;
	std::string _gap_img_caption;
	std::string _gap_img_section;
	std::string _gap_map_section;
	std::string _gap_test_name;
	std::string _gap_try_number;

	// other
	std::string _map_filename;

	// service
	ros::ServiceServer _image_srv;
	ros::ServiceServer _map_srv;	
	ros::ServiceServer _create_srv;

	//Document information
	std::stringstream report_file;
	std::string _test_name;
	int try_number;
	
	std::string tex_code;
	int loaded_images;

	// map_parameters
    float _res;
    int _img_width;
    int _img_height;
    float _map_x0;
    float _map_y0;


public:
	ReportGenerator(std::string name);
	virtual ~ReportGenerator();

	std::string saveMapImage();
	void loadParametersFile();
	void getMapParameters();
	void compile(std::string output_filename);
	void replaceGap(std::string& subject, const std::string& key, const std::string& value);
	bool readFile(std::string filename, std::string& result);
	bool writeFile(std::string filename, std::string text);
	bool generate(uchile_srvs::ReportGenerator::Request &req, uchile_srvs::ReportGenerator::Response &res);
	bool generateMapImgs(uchile_srvs::ReportGenerator::Request &req, uchile_srvs::ReportGenerator::Response &res);
	bool add_image(uchile_srvs::ReportGenerator::Request &req, uchile_srvs::ReportGenerator::Response &res) ;
	bool add_map(uchile_srvs::ReportGenerator::Request &req, uchile_srvs::ReportGenerator::Response &res) ;
};

} /* namespace uchile_report_generator */

#endif /* REPORT_GENERATOR_H_ */
