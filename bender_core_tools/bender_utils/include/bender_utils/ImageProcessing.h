/*
 * ImageProcessing.h
 *
 *      Author: LuzMartinez
 */

#ifndef IMAGEPROCESSING_H_
#define IMAGEPROCESSING_H_

// C, C++
#include <cstdio> // for EOF
#include <string>
#include <sstream>
#include <vector>

// ROS
#include <ros/ros.h>
#include <bender_srvs/Onoff.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <sensor_msgs/image_encodings.h>

//OpenCV
#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

namespace bender_utils {

class ImageProcessing {

private:

	ros::Subscriber _subs_rgb;
	ros::Subscriber _subs_depth;

protected:
  ros::NodeHandle priv;
	bool display;
	bool ready_rgb;
	bool ready_depth;

	std::string _imageRGB_topic;
	std::string _imageDepth_topic;

	sensor_msgs::ImageConstPtr image_in;
	sensor_msgs::ImageConstPtr depth_in;

  static const float consPCL = 0.0019047619;


public:
	ImageProcessing(std::string name = "~");
	virtual ~ImageProcessing();
  bool _is_on;

protected:
	/** - - - - ..... - - - - **/
	void equalizeImage(const cv::Mat& input, cv::Mat &output);
	geometry_msgs::Point get_point(cv::Mat src, cv::Point p);
  cv::Mat get_mat (sensor_msgs::ImageConstPtr sensor_img, std::string encoding);

	/** - - - - ..... - - - - **/
  void _process_image(const sensor_msgs::ImageConstPtr& img);
  void _process_depth(const sensor_msgs::ImageConstPtr& img);

public:
	bool _active_service_RGB(bender_srvs::Onoff::Request  &req, bender_srvs::Onoff::Response &res);
	bool _active_service_RGBD(bender_srvs::Onoff::Request  &req, bender_srvs::Onoff::Response &res);
  bool _active_service_D(bender_srvs::Onoff::Request  &req, bender_srvs::Onoff::Response &res);

};


inline ImageProcessing::ImageProcessing(std::string name) {

	priv = ros::NodeHandle(name);
	display = true;
	_is_on = false;
	ready_rgb = false;
	ready_depth = false;
}

inline ImageProcessing::~ImageProcessing() {

}


inline void ImageProcessing::equalizeImage(const cv::Mat& input, cv::Mat &output) {
	cv::cvtColor(input, output, CV_BGR2GRAY);
	cv::equalizeHist(output, output);
}

inline geometry_msgs::Point ImageProcessing::get_point(cv::Mat src, cv::Point p){
    geometry_msgs::Point point;
    int img_width=src.cols;
    int img_height=src.rows;

    point.z = ((float)src.at<uchar>(p.y,p.x))/255.0;
    point.x = (p.x - img_width/2)*(point.z*consPCL) ;
    point.y = (p.y - img_height/2)*(point.z*consPCL) ;

    return point;
}

inline cv::Mat ImageProcessing::get_mat (sensor_msgs::ImageConstPtr sensor_img, std::string encoding){

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy((sensor_img), encoding);

    return cv_ptr->image;
}

inline bool ImageProcessing::_active_service_RGB(bender_srvs::Onoff::Request  &req, bender_srvs::Onoff::Response &res) {

    if(req.select == true) {
        if (!_is_on) {
          _subs_rgb = priv.subscribe(_imageRGB_topic, 1, &ImageProcessing::_process_image, this);
            _is_on = true;
          
            ROS_INFO_STREAM("Turning on "+_imageRGB_topic+" . . . OK");
        } else ROS_DEBUG_STREAM("Already turned on");
    }
    else{
        if (_is_on) {
              _subs_rgb.shutdown();
              _is_on = false;
              ready_rgb = false;
              ROS_INFO_STREAM(" Turning off "+_imageRGB_topic+" . . . OK");
        } else  ROS_DEBUG_STREAM("Already turned off"); 
    }
    return true;
}

inline bool ImageProcessing::_active_service_RGBD(bender_srvs::Onoff::Request  &req, bender_srvs::Onoff::Response &res) {

    if(req.select == true) {
        if (!_is_on) {
            _subs_rgb = priv.subscribe(_imageRGB_topic, 1, &ImageProcessing::_process_image, this);
            _subs_depth = priv.subscribe(_imageDepth_topic, 1, &ImageProcessing::_process_depth, this); 
            _is_on = true;

            ROS_INFO_STREAM("Turning on "+_imageRGB_topic+". . . OK");
            ROS_INFO_STREAM("Turning on "+_imageDepth_topic+". . . OK");
        } else ROS_DEBUG_STREAM("Already turned on");
    }
    else{
        if (_is_on) {
              _subs_rgb.shutdown();
              _subs_depth.shutdown();
              _is_on = false;
              ready_rgb = false;
              ready_depth = false;
              ROS_INFO_STREAM(" Turning off . . . OK");
        } else  ROS_DEBUG_STREAM("Already turned off"); 
    }
    return true;
}

inline bool ImageProcessing::_active_service_D(bender_srvs::Onoff::Request  &req, bender_srvs::Onoff::Response &res) {

    if(req.select == true) {
        if (!_is_on) {
            _subs_depth = priv.subscribe(_imageDepth_topic, 1, &ImageProcessing::_process_depth, this); 
            _is_on = true;

            ROS_INFO_STREAM("Turning on "+_imageDepth_topic+". . . OK");
        } else ROS_DEBUG_STREAM("Already turned on");
    }
    else{
        if (_is_on) {
              _subs_depth.shutdown();
              _is_on = false;
              ready_depth = false;
              ROS_INFO_STREAM(" Turning off . . . OK");
        } else  ROS_DEBUG_STREAM("Already turned off"); 
    }
    return true;
}

inline void ImageProcessing::_process_image(const sensor_msgs::ImageConstPtr& img){
    if(!_is_on) return;
    image_in=img;
    ready_rgb = true;
}

inline void ImageProcessing::_process_depth(const sensor_msgs::ImageConstPtr& img){
    if(!_is_on) return;
    depth_in=img;
    ready_depth = true;
}

} /* namespace bender_utils */
#endif /* ImageProcessing_H_ */
