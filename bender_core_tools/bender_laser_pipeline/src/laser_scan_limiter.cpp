#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <vector>

ros::Publisher pub;
ros::Subscriber sub;
sensor_msgs::LaserScan scan_out;
std::vector<float>::iterator r_it;

void callback(const sensor_msgs::LaserScan &msg){
	scan_out = msg;
	for (r_it = scan_out.ranges.begin(); r_it != scan_out.ranges.end(); ++r_it)
	{
		if (*r_it < msg.range_min) {
			*r_it = std::max(0.0f, msg.range_min - 1);
		} else if (*r_it > msg.range_max) {
			*r_it = msg.range_max + 1;
		}
	}
	pub.publish(scan_out);
}

int main(int argc, char **argv){

	ros::init(argc, argv, "laser_scan_limiter");

	ros::NodeHandle n("~");
	sub = n.subscribe("input", 1, callback);
	pub = n.advertise<sensor_msgs::LaserScan>("output", 10);

	ROS_INFO("[laser_scan_limiter]: Working ... ");
	ros::spin();
	ROS_INFO("[laser_scan_limiter]: Quitting... \n");
	return 0;
}
