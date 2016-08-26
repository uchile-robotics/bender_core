/*
 * PointProcessing.h
 *
 *      Author: LuzMartinez
 */

#ifndef POINTPROCESSING_H_
#define POINTPROCESSING_H_

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

class PointProcessing {

public:

  typedef pcl::PointXYZ pcl_point;
  typedef pcl::PointXYZRGB pcl_point_rbg;
  typedef pcl::PointCloud<pcl_point> pcl_point_cloud;
  typedef pcl::PointCloud<pcl_point_rbg> pcl_point_rgb_cloud;

  typedef std::map<std::string, ros::Publisher> PubMap;

  ros::NodeHandle priv;
	bool display;
	bool ready_point;

	std::string _imagePoint_topic;

  pcl::PCLPointCloud2::ConstPtr cloud_in;
  static const float consPCL = 0.0019047619;

private:

  ros::Subscriber _subs_point;
  PubMap _cloud_pubs;

public:
	PointProcessing(std::string name = "~");
	virtual ~PointProcessing();
  bool _is_on;

protected:
	/** - - - - FUNCTIONS. - - - - **/
  void register_cloud_rgb(std::string name);
  void register_cloud(std::string name);
  void publish_cloud(std::string name, pcl_point_cloud::Ptr cloud);
  void publish_cloud_rgb(std::string name, pcl_point_rgb_cloud::Ptr cloud);

	/** - - - - CALLBACKS - - - - **/
  void _process_point(const pcl::PCLPointCloud2::ConstPtr &point_cloud_in);

  // void Display(cv::Mat frame, std::vector<cv::Rect> faces);

public:
	bool _active_service_Point(bender_srvs::Onoff::Request  &req, bender_srvs::Onoff::Response &res);
  void set_topic (std::string topic);

};


  void PointProcessing::set_topic(std::string topic) {   
    _imagePoint_topic = topic;
  }


 PointProcessing::PointProcessing(std::string name) {

	priv = ros::NodeHandle(name);
	display = true;
	_is_on = false;
	ready_point = false;
}

 PointProcessing::~PointProcessing() {

}


 bool PointProcessing::_active_service_Point(bender_srvs::Onoff::Request  &req, bender_srvs::Onoff::Response &res) {

    if(req.select == true) {
        if (!_is_on) {
            _subs_point = priv.subscribe(_imagePoint_topic, 1, &PointProcessing::_process_point, this); 
            _is_on = true;

            ROS_INFO_STREAM("Turning on "+_imagePoint_topic+". . . OK");
        } else ROS_DEBUG_STREAM("Already turned on");
    }
    else{
        if (_is_on) {
              _subs_point.shutdown();
              _is_on = false;
              ready_point = false;
              ROS_INFO_STREAM(" Turning off . . . OK");
        } else  ROS_DEBUG_STREAM("Already turned off"); 
    }
    return true;
}

void PointProcessing::_process_point(const pcl::PCLPointCloud2::ConstPtr &point_cloud_in){

    if(!_is_on) return;
    cloud_in=point_cloud_in;
    ready_point = true;
}


  void PointProcessing::register_cloud(std::string name) {

    PubMap::iterator it;
    it = _cloud_pubs.find(name);
    if (it != _cloud_pubs.end()) {
      ROS_WARN_STREAM("Sorry, a cloud publisher named " << name << "already exists");
      return;
    }
    _cloud_pubs[name] = priv.advertise<pcl_point_cloud>(name, 1);
  }

  void PointProcessing::publish_cloud(std::string name, pcl_point_cloud::Ptr cloud) {

    cloud->header.stamp = ros::Time::now().toNSec();
    ROS_INFO_STREAM("Frame " << cloud->header.frame_id );
    ROS_INFO_STREAM("Time " << cloud->header.stamp );
    // todo: check existence
    ros::Publisher pub = priv.advertise<pcl_point_cloud>(name, 1);

    if (pub.getNumSubscribers() > 0) {
      ROS_INFO_STREAM("Publish in " << name );
      pub.publish(cloud);
    }
  }


  void PointProcessing::register_cloud_rgb(std::string name) {

    PubMap::iterator it;
    it = _cloud_pubs.find(name);
    if (it != _cloud_pubs.end()) {
      ROS_WARN_STREAM("Sorry, a cloud publisher named " << name << "already exists");
      return;
    }

    _cloud_pubs[name] = priv.advertise<pcl_point_rgb_cloud>(name, 1);
  }


  void PointProcessing::publish_cloud_rgb(std::string name, pcl_point_rgb_cloud::Ptr cloud) {

    // todo: check existence
    ros::Publisher pub = _cloud_pubs[name];

    if (pub.getNumSubscribers() > 0) {
      pub.publish(cloud);
    }
  }

} /* namespace bender_utils */
#endif /* POINTPROCESSING_H_ */
