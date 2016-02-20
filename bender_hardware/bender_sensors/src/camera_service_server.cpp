#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "bender_srvs/ImageService.h"
#include "bender_srvs/IsOn.h"

#include <stdio.h>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <pthread.h>

// default node/camera parameters
int cam_number = -1;
int cam_width = 640;
int cam_height = 480;

// node variables
bool is_on = false;        // current camera connection state
CvCapture* capture;         // cv device
IplImage* src;              // current image
sensor_msgs::Image img_;    // current ros image
pthread_mutex_t cam_mutex;  // mutex for image frame usage/retrieving
pthread_mutex_t on_mutex;   // mutex for on/off stuff

bool init_camera() {

	pthread_mutex_lock(&on_mutex);

	// camera already turned on
	if (is_on) {
		ROS_WARN("Camera already turned on");
		pthread_mutex_unlock(&on_mutex);
		return true;
	}

	if( NULL == ( capture=cvCaptureFromCAM(cam_number) ) ) {
		ROS_WARN("\nError on cvCaptureFromCAM");
		return false;
	}
	
	cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_WIDTH,cam_width);
	cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_HEIGHT,cam_height);
	
	if( NULL == ( src=cvQueryFrame(capture) ) ) {
		ROS_WARN("\nError on cvQueryFrame");
	}
	//cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_WIDTH,cam_width); // Error: se movio arriba
	//cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_HEIGHT,cam_height);

	ROS_INFO("Acquired image. size:(%d/%d), channels:(%d), depth:(%d)\n",src->width,src->height,src->nChannels,src->depth);

	// set flag
	is_on = true;
	ROS_INFO_STREAM("Camera from device /dev/video" << cam_number << " turned on");

	pthread_mutex_unlock(&on_mutex);

	return true;
}

bool take_image() {

	pthread_mutex_lock(&cam_mutex);
	src=cvRetrieveFrame(capture);

	if( NULL==(src) ) {

		ROS_WARN("\nError on cvQueryFrame");
		pthread_mutex_unlock(&cam_mutex);

		return false;

	} else {

		fillImage(img_, sensor_msgs::image_encodings::BGR8, src->height,src->width, src->nChannels  * src->width, src->imageData);  //free?
		pthread_mutex_unlock(&cam_mutex);

		return true;
	}
}

void get_image_frame() {

	pthread_mutex_lock(&on_mutex);
	if (is_on) {
		pthread_mutex_lock(&cam_mutex);
		cvGrabFrame(capture); //free?
		pthread_mutex_unlock(&cam_mutex);
	}
	pthread_mutex_unlock(&on_mutex);
}


// - - - - - - - - R O S    S E R V I C E S - - - - - - - - -

bool turn_on(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

	return init_camera();
}

bool turn_off(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

	pthread_mutex_lock(&on_mutex);
	if (!is_on) {
		ROS_WARN("Camera already turned off");
		pthread_mutex_unlock(&on_mutex);
		return true;
	}

	cvReleaseCapture(&capture);
	is_on = false;

	pthread_mutex_unlock(&on_mutex);

	ROS_INFO_STREAM("Camera turned off");

	return true;
}

bool is_on_cb(bender_srvs::IsOn::Request &req, bender_srvs::IsOn::Response &res) {

	res.is_on = is_on;
	return true;
}

bool get_image(bender_srvs::ImageService::Request  &req, bender_srvs::ImageService::Response &res) {

	pthread_mutex_lock(&on_mutex);
	if (!is_on) {
		pthread_mutex_unlock(&on_mutex);
		return false;
	}

	if(!take_image())
	{
		ROS_WARN("take_image failed.");
		pthread_mutex_unlock(&on_mutex);
		return false;
	}
	//img_.header.stamp = ros::Time::now();
	res.im = img_;

	pthread_mutex_unlock(&on_mutex);

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "camera_service");
	ros::NodeHandle n("~");

	// get parameters
	bool connect_at_start = true;
	if(!n.getParam("cam_number",cam_number)) n.setParam("cam_number",cam_number);
	if(!n.getParam("width",cam_width))   n.setParam("width",cam_width);
	if(!n.getParam("height",cam_height)) n.setParam("height",cam_height);
	if(!n.getParam("connect_at_start",connect_at_start)) n.setParam("connect_at_start",connect_at_start);

	// begin camera image acquisition if necessary
	if(connect_at_start && !init_camera()) {
		return 0;
	}

	ros::AsyncSpinner spinner(2);

	// advertise services
	ros::ServiceServer get_image_srv = n.advertiseService("get_image", get_image);
	ros::ServiceServer turn_on_srv = n.advertiseService("turn_on", turn_on);
	ros::ServiceServer rurn_off_srv = n.advertiseService("turn_off", turn_off);
	ros::ServiceServer is_on_srv = n.advertiseService("is_on", is_on_cb);

	ROS_INFO("Ready to serve images.");

	spinner.start();
	while(ros::ok())
	{
		// process image frame
		get_image_frame();

		// sleep
		cvWaitKey(10);
	}

	return 0;
}
