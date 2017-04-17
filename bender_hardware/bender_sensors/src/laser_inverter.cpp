
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <bender_utils/ParameterServerWrapper.h>
#include <vector>
#include <math.h>

//************ Variables ***********//

// ROS
ros::Publisher pub;
ros::Subscriber sub;

// parameters
std::string _frame_out;

void callback(const sensor_msgs::LaserScan &in){

	//ROS_WARN_STREAM("cb");
	sensor_msgs::LaserScan out;
	out.angle_increment = in.angle_increment;
	out.angle_max = -in.angle_min;
	out.angle_min = -in.angle_max;
	//ROS_WARN_STREAM("rmin:" << out.angle_min*180.0/M_PI << "  rmax: " << out.angle_max*180.0/M_PI);
	out.range_max = in.range_max;
	out.range_min = in.range_min;
	out.scan_time = in.scan_time;
	out.time_increment = in.time_increment;

	//ROS_WARN_STREAM("copying");
	int n_readings = in.ranges.size();
	for (int i=n_readings-1; i>=0; i--) {
		out.ranges.push_back(in.ranges[i]);
	}
	out.header.frame_id = _frame_out;
	out.header.stamp = in.header.stamp;

	//ROS_WARN_STREAM("pub");
	pub.publish(out);
}

int main(int argc, char **argv){

	ROS_INFO("Configuring...");
	ros::init(argc, argv, "laser_inverter");
	ros::NodeHandle priv("~");

	// parameters
	bender_utils::ParameterServerWrapper psw;
	psw.getParameter("scan_out_frame", _frame_out, "");
	if (_frame_out == "") {
		ROS_ERROR_STREAM("This node requires the parameter '~scan_out_frame' to be set");
		exit(1);
	}

	/* Topics */
	pub = priv.advertise<sensor_msgs::LaserScan>("scan_out", 1);
	sub = priv.subscribe("scan_in", 1, callback);

	// [important!] wait for timer initialization!
	//while (ros::ok() && ros::Time::now().isZero());

	ROS_INFO("Config. Done");
	ros::spin();

	ROS_INFO("Quitting... \n");
	return 0;
}
