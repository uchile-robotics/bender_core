/*
 * By Matias Pavez Bahamondes
 * 5 June, 2014
 */

#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <bender_srvs/Float.h>

namespace bender_tf {

class NeckTfServer {

private:
	ros::ServiceServer _update_angle_srv;
	std::string _name;

	// tf data & broadcaster
	tf::TransformBroadcaster _br;
	tf::Transform _transform;
	tf::Quaternion _quaternion;
	std::string _parent_link;
	std::string _neck_link;

	// current angle
	float _angle;



private:
	bool update(bender_srvs::Float::Request &req, bender_srvs::Float::Response &res);

public:
	NeckTfServer(std::string name);
	~NeckTfServer();
	void publishTransform();
};

NeckTfServer::NeckTfServer(std::string name): _name(name) {

	ros::NodeHandle priv("~");
	priv = ros::NodeHandle("~");

	_transform.setOrigin(tf::Vector3(0, 0, 0.50));
	_angle = 0;

	_parent_link = "/bender/torso_link";
	_neck_link = "/bender/neck_link";

	_update_angle_srv = priv.advertiseService("update",&NeckTfServer::update,this);

}

NeckTfServer::~NeckTfServer() {
}

bool NeckTfServer::update(bender_srvs::Float::Request &req, bender_srvs::Float::Response &res) {

	_angle = req.x1;
	return true;
}

void NeckTfServer::publishTransform() {

	_quaternion.setRPY(0, 0, _angle*M_PI/180.0);
	_transform.setRotation(_quaternion);

	_br.sendTransform(
		tf::StampedTransform(
			_transform,
			ros::Time::now(),
			_parent_link,
			_neck_link
		)
	);
}

} /* namespace bender_tf */

int main(int argc, char** argv) {

	ros::init(argc, argv, "neck_tf");

	bender_tf::NeckTfServer *node = new bender_tf::NeckTfServer(ros::this_node::getName());

	ros::Rate r(30); // 30 hz
	while (ros::ok()) {

 		ros::spinOnce();
 		node->publishTransform();

		r.sleep();
	}

	ROS_INFO("Quitting ... \n");
	return 0;
}
