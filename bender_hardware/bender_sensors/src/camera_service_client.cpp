#include "ros/ros.h"
#include "bender_srvs/ImageService.h"
#include "bender_srvs/IsOn.h"
#include <cstdlib>
#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "camera_service_client");

	ros::NodeHandle n("~");

	// parameter server: camera name
	std::string cam_name = "default_camera";
	if(!n.getParam("cam_name",cam_name)) n.setParam("cam_name",cam_name);

	// window name/title
	std::string window = "Camera display: " + cam_name;

	// camera clients
	ros::ServiceClient client = n.serviceClient<bender_srvs::ImageService>("/bender/sensors/" + cam_name + "/get_image");
	ros::ServiceClient is_on_client = n.serviceClient<bender_srvs::IsOn>("/bender/sensors/" + cam_name + "/is_on");

	// wait services
	while ( ros::ok() && !client.waitForExistence(ros::Duration(3.0)) );
	while ( ros::ok() && !is_on_client.waitForExistence(ros::Duration(3.0)) );

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	ROS_INFO("Working ...");

	bender_srvs::IsOn is_on_srv;
	bender_srvs::ImageService srv;
	cv_bridge::CvImagePtr cv_ptr;

	// control variables
	bool was_on = false;


	// this is necessary for window closing
	cv::startWindowThread();

	// work ......
	while(ros::ok()) {

		// verify camera status
		while ( ros::ok() && !is_on_client.waitForExistence(ros::Duration(3.0)) );
		if (!is_on_client.call(is_on_srv)) {
			ros::Duration(1).sleep();
			continue;
		}

		// if is-on, then work.
		if (!is_on_srv.response.is_on) {

				// wait a while
				ros::Duration(1).sleep();

				// destroy/close window if necessary
				if (was_on == true) {
					cv::destroyWindow(window);
					cv::waitKey(1);
				}
				was_on = false;

				continue;
		}

		// get image
		while ( ros::ok() && !client.waitForExistence(ros::Duration(3.0)) );
		if (client.call(srv)) {

			//ROS_DEBUG("imageIn encoding = %s", srv.response.im.encoding.c_str() );

			// show image
			try {
				cv_ptr = cv_bridge::toCvCopy((srv.response.im), sensor_msgs::image_encodings::BGR8);

				// create window if necessary
				if(was_on == false) {
					cv::namedWindow(window);
				}

				// show images
				cv::imshow(window, cv_ptr->image);
				cv::waitKey(20);

			} catch (cv_bridge::Exception& e) {
				ROS_ERROR("cv_bridge exception: %s", e.what());
			}
		}
		was_on = true;
	}

	return 0;
}
