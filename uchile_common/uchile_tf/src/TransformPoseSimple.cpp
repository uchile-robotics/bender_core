/*
 * By Matias Pavez Bahamondes
 * 10 May, 2014
 */

#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <uchile_srvs/Transformer.h>

namespace uchile_tf {

class TransformPoseSimple {

private:
	ros::ServiceServer _transform_srv;
    tf::TransformListener _tf_listener;
    std::string _name;

private:
	bool transform(uchile_srvs::Transformer::Request &req, uchile_srvs::Transformer::Response &res);

public:
	TransformPoseSimple(std::string name);
	~TransformPoseSimple();
};

TransformPoseSimple::TransformPoseSimple(std::string name): _name(name) {

	ros::NodeHandle priv("~");
	priv = ros::NodeHandle("~");

	//this->_tf_listener.setExtrapolationLimit(ros::Duration(0.1));
	_transform_srv = priv.advertiseService("transform",&TransformPoseSimple::transform,this);

}

TransformPoseSimple::~TransformPoseSimple() {
}

bool TransformPoseSimple::transform(uchile_srvs::Transformer::Request &req, uchile_srvs::Transformer::Response &res) {

	tf::StampedTransform transform;
	int max_connectivity_exceptions = 5;
	int connectivity_exceptions = 0;

	while (ros::ok()) {

		try {
			_tf_listener.waitForTransform(req.frame_out, req.pose_in.header.frame_id,req.pose_in.header.stamp,ros::Duration(2.0));
			_tf_listener.transformPose(req.frame_out, req.pose_in, res.pose_out);
			break;

		} catch (tf::ConnectivityException &e) {

			ROS_WARN_STREAM("Requested a transform between unconnected trees!: " << e.what());
			connectivity_exceptions++;

			if (connectivity_exceptions == max_connectivity_exceptions) {
				return false;
			}
			continue;

		} catch (tf::TransformException &e) {

			ROS_WARN_STREAM("Transform Exception: " << e.what());
			return false;
		}

	}
	return true;
}

} /* namespace uchile_tf */

int main(int argc, char** argv) {

	ros::init(argc, argv, "simple_pose_transformer");

	uchile_tf::TransformPoseSimple *node = new uchile_tf::TransformPoseSimple(ros::this_node::getName());

	ros::MultiThreadedSpinner spinner(2);
	spinner.spin();

	ROS_INFO("Quitting ... \n");
	return 0;
}
