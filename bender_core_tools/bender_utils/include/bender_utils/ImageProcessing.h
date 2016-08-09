/*
 * ImageProcessing.h
 *
 *      Author: LuzMartinez
 */

#ifndef IMAGEPROCESSING_H_
#define IMAGEPROCESSING_H_

#include <ros/ros.h> 
 #include <ros/console.h>

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

public:
  ros::NodeHandle priv;
	bool display;
	bool ready_rgb;
	bool ready_depth;

	std::string _imageRGB_topic;
	std::string _imageDepth_topic;
  std::vector < std::string > win_name;

	sensor_msgs::ImageConstPtr image_in;
	sensor_msgs::ImageConstPtr depth_in;

  static const float consPCL = 0.0019047619;


public:
	ImageProcessing(std::string name = "~");
	virtual ~ImageProcessing();
  bool _is_on;
  bool _is_on_depth;

protected:
	/** - - - - ..... - - - - **/
	void equalizeImage(const cv::Mat& input, cv::Mat &output);
	geometry_msgs::Point get_point(cv::Mat src, cv::Point p);
  cv::Mat get_mat (sensor_msgs::ImageConstPtr sensor_img, std::string encoding);

	/** - - - - ..... - - - - **/
  void _process_image(const sensor_msgs::ImageConstPtr& img);
  void _process_depth(const sensor_msgs::ImageConstPtr& img);

  void Display(std::string win, cv::Mat frame, std::vector<cv::Rect> faces, cv::Scalar s);
  void Display(  std::string win, cv::Mat frame);

public:
	bool _active_service_RGB(bender_srvs::Onoff::Request  &req, bender_srvs::Onoff::Response &res);
	bool _active_service_RGBD(bender_srvs::Onoff::Request  &req, bender_srvs::Onoff::Response &res);
  bool _active_service_D(bender_srvs::Onoff::Request  &req, bender_srvs::Onoff::Response &res);
  bool _active_service_RGB_D(bender_srvs::Onoff::Request  &req, bender_srvs::Onoff::Response &res); 
  void set_topics (std::string rgb, std::string depth);
  bool set_display(bender_srvs::Onoff::Request  &req, bender_srvs::Onoff::Response &res);
};


  void ImageProcessing::set_topics(std::string rgb, std::string depth) {   
    _imageRGB_topic = rgb;
    _imageDepth_topic = depth;
  }

  bool ImageProcessing::set_display(bender_srvs::Onoff::Request  &req, bender_srvs::Onoff::Response &res) {   
    if(req.select==display)return true;
    display = req.select;

    if(display){
         cv::startWindowThread();
         for (int i = 0; i < win_name.size(); ++i)
            cv::namedWindow(win_name[i], cv::WINDOW_AUTOSIZE );
    }
    else{
        cv::destroyAllWindows();
        cv::waitKey(10);
    }
    
    return true;
  }

 ImageProcessing::ImageProcessing(std::string name) {

	priv = ros::NodeHandle(name);
	display = true;
	_is_on = false;
  _is_on_depth =false;
	ready_rgb = false;
	ready_depth = false;
}

 ImageProcessing::~ImageProcessing() {

}


 void ImageProcessing::equalizeImage(const cv::Mat& input, cv::Mat &output) {
	cv::cvtColor(input, output, CV_BGR2GRAY);
	cv::equalizeHist(output, output);
}

 geometry_msgs::Point ImageProcessing::get_point(cv::Mat src, cv::Point p){
    geometry_msgs::Point point;
    int img_width=src.cols;
    int img_height=src.rows;

    point.z = ((float)src.at<uchar>(p.y,p.x))/255.0;
    point.x = (p.x - img_width/2)*(point.z*consPCL) ;
    point.y = (p.y - img_height/2)*(point.z*consPCL) ;

    return point;
}

 cv::Mat ImageProcessing::get_mat (sensor_msgs::ImageConstPtr sensor_img, std::string encoding){

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy((sensor_img), encoding);

    return cv_ptr->image;
}


 bool ImageProcessing::_active_service_RGB(bender_srvs::Onoff::Request  &req, bender_srvs::Onoff::Response &res) {

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

 bool ImageProcessing::_active_service_RGBD(bender_srvs::Onoff::Request  &req, bender_srvs::Onoff::Response &res) {

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

 bool ImageProcessing::_active_service_D(bender_srvs::Onoff::Request  &req, bender_srvs::Onoff::Response &res) {

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

bool ImageProcessing::_active_service_RGB_D(bender_srvs::Onoff::Request  &req, bender_srvs::Onoff::Response &res) {

    std::string _image_topic = _imageRGB_topic;

    if(req.select == true) {
        if (!_is_on) {
            _subs_rgb = priv.subscribe(_imageRGB_topic, 1, &ImageProcessing::_process_image, this);
            _is_on = true;
            ROS_INFO_STREAM("Turning on "+_imageRGB_topic+". . . OK");

            if (_image_topic.find("rgbd") !=std::string::npos) {
                _is_on_depth = true;
                std::string _depth_topic = _image_topic, rgb = "rgb/";
                _depth_topic.replace(_depth_topic.find(rgb), rgb.length(),"depth/");
                _imageDepth_topic = _depth_topic;
                _subs_depth = priv.subscribe(_imageDepth_topic, 1, &ImageProcessing::_process_depth, this); 
                ROS_INFO_STREAM("Turning on "+_imageDepth_topic+". . . OK");
            }
        } else ROS_DEBUG_STREAM("Already turned on");
    }
    else{
        if (_is_on) {
              _subs_rgb.shutdown();
              if (_is_on_depth){
                  _subs_depth.shutdown();
                  _is_on_depth = false;
                  ready_depth = false;
                }
              _is_on = false;
              ready_rgb = false;
              ROS_INFO_STREAM(" Turning off . . . OK");
        } else  ROS_DEBUG_STREAM("Already turned off"); 
    }
    return true;
}

 void ImageProcessing::_process_image(const sensor_msgs::ImageConstPtr& img){
    if(!_is_on) return;
    image_in=img;
    ready_rgb = true;
}

 void ImageProcessing::_process_depth(const sensor_msgs::ImageConstPtr& img){
    if(!_is_on) return;
    depth_in=img;
    ready_depth = true;
}



void ImageProcessing::Display(std::string win, cv::Mat img_in, std::vector<cv::Rect> faces, cv::Scalar s = cv::Scalar(255, 0, 0)) {

  cv::Mat frame = img_in.clone();
    for (unsigned int i = 0; i < faces.size(); i++) {
        cv::rectangle(frame, cv::Point(faces[i].x, faces[i].y),
                cv::Point(faces[i].x + faces[i].width,
                        faces[i].y + faces[i].height), s ,5);
    }

    imshow(win, frame);
    cv::waitKey(10);
}

void ImageProcessing::Display(  std::string win, cv::Mat frame) {
    imshow(win, frame);
    cv::waitKey(10);
}


} /* namespace bender_utils */
#endif /* ImageProcessing_H_ */
