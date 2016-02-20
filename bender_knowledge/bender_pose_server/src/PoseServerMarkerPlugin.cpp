/*
 * PoseServerMarkerPlugin.cpp
 *
 *   matias.pavez.b@gmail.com
 */

// C, C++
#include <vector>
#include <string>
#include <boost/scoped_ptr.hpp>

// ROS
#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/Empty.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <std_msgs/ColorRGBA.h>

// bender
#include <bender_msgs/SemanticObject.h>
#include <bender_srvs/SemMap.h>
#include <bender_srvs/String.h>

typedef std::vector<bender_msgs::SemanticObject> SemanticObjectArray;

/*
 * TODO: new pose: rviz plugin
 * TODO: id
 */

namespace bender_nav {

visualization_msgs::Marker make_map_arrow_marker( visualization_msgs::InteractiveMarker &msg )
{
	visualization_msgs::Marker marker;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::MODIFY;

	marker.scale.x = 0.60; marker.scale.y = 0.08; marker.scale.z = 0.08;
	marker.color.a = 1.0; marker.color.r = 1.0;
	marker.color.g = 0.0; marker.color.b = 0.0;

	marker.ns = msg.name;
	marker.id = 0;
	marker.header = msg.header;
	marker.pose = msg.pose;

	return marker;
}

visualization_msgs::Marker make_map_ball_marker( visualization_msgs::InteractiveMarker &msg )
{
	visualization_msgs::Marker ball;
	ball.type = visualization_msgs::Marker::SPHERE;
	ball.action = visualization_msgs::Marker::MODIFY;
	ball.scale.x = 0.20; ball.scale.y = 0.20; ball.scale.z = 0.20;
	ball.color.a = 1.0; ball.color.r = 1.0;
	ball.color.g = 0.0; ball.color.b = 0.0;
	ball.ns = msg.name;
	ball.id = 1;
	ball.header = msg.header;
	ball.pose = msg.pose;

	return ball;
}

visualization_msgs::Marker make_object_marker( visualization_msgs::InteractiveMarker &msg )
{
	visualization_msgs::Marker marker;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::MODIFY;
	marker.scale.x = 0.20; marker.scale.y = 0.20; marker.scale.z = 0.20;
	marker.color.r = 0.0; marker.color.g = 1.0;
	marker.color.b = 0.0; marker.color.a = 1.0;

	marker.ns = msg.name;
	marker.id = 1;
	marker.header = msg.header;
	marker.pose = msg.pose;

	return marker;
}

visualization_msgs::Marker make_unknown_marker( visualization_msgs::InteractiveMarker &msg )
{
	visualization_msgs::Marker marker;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::MODIFY;
	marker.scale.x = 0.45; marker.scale.y = 0.45; marker.scale.z = 0.45;
	marker.color.r = 0.5; marker.color.g = 0.5;
	marker.color.b = 0.5; marker.color.a = 1.0;

	marker.ns = msg.name;
	marker.id = 2;
	marker.header = msg.header;
	marker.pose = msg.pose;

	return marker;
}

void make_map_marker_control( visualization_msgs::InteractiveMarker &msg )
{
	visualization_msgs::InteractiveMarkerControl ball_control;
	ball_control.name = "move_ball";
	ball_control.always_visible = true;
	ball_control.orientation.w = 1;	ball_control.orientation.x = 0;
	ball_control.orientation.y = 1; ball_control.orientation.z = 0;
	ball_control.markers.push_back( make_map_ball_marker(msg) );
	ball_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
	msg.controls.push_back( ball_control );

	visualization_msgs::InteractiveMarkerControl arrow_control;
	arrow_control.name = "rotate_arrow";
	arrow_control.always_visible = true;
	arrow_control.markers.push_back( make_map_arrow_marker(msg) );
	arrow_control.orientation.w = 1; arrow_control.orientation.x = 0;
	arrow_control.orientation.y = 1; arrow_control.orientation.z = 0;
	arrow_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	msg.controls.push_back( arrow_control );

	visualization_msgs::InteractiveMarkerControl control;
	control.name = "move_x";
	control.orientation.w = 1; control.orientation.x = 1; control.orientation.y = 0; control.orientation.z = 0;
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	msg.controls.push_back(control);

	control.name = "move_y";
	control.orientation.w = 1; control.orientation.x = 0; control.orientation.y = 0; control.orientation.z = 1;
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	msg.controls.push_back(control);

	control.name = "rotate_z";
	control.orientation.w = 1; control.orientation.x = 0; control.orientation.y = 1; control.orientation.z = 0;
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	msg.controls.push_back(control);
}

void make_object_marker_control( visualization_msgs::InteractiveMarker &msg )
{
	visualization_msgs::InteractiveMarkerControl object_control;
	object_control.name = "move_object";
	object_control.always_visible = true;
	object_control.markers.push_back( make_object_marker(msg) );
	object_control.orientation.w = 1;
	object_control.orientation.x = 0;
	object_control.orientation.y = 1;
	object_control.orientation.z = 0;
	object_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
	msg.controls.push_back( object_control );

	visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    msg.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    msg.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    msg.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	msg.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    msg.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    msg.controls.push_back(control);

}

void make_unknown_marker_control( visualization_msgs::InteractiveMarker &msg )
{
	visualization_msgs::InteractiveMarkerControl unknown_control;
	unknown_control.name = "move_object";
	unknown_control.always_visible = true;
	unknown_control.markers.push_back( make_unknown_marker(msg) );
	unknown_control.orientation.w = 1;
	unknown_control.orientation.x = 0;
	unknown_control.orientation.y = 1;
	unknown_control.orientation.z = 0;
	unknown_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
	msg.controls.push_back( unknown_control );
}

void add_menu_marker_control( visualization_msgs::InteractiveMarker &msg ) {

	visualization_msgs::InteractiveMarkerControl menu_control;
	menu_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
	menu_control.name = "menu";
	msg.controls.push_back(menu_control);
}



class PoseServerMarkerHandler {

private:

	std::string _current_map_name;

	// map clients
	ros::ServiceClient _which_map_client;
	ros::ServiceClient _set_client;
	ros::ServiceClient _save_client;
	ros::ServiceClient _get_all_client;
	ros::ServiceClient _get_client;

public:

	PoseServerMarkerHandler() {};
	virtual ~PoseServerMarkerHandler() {};

	bool initialize() {

		ros::NodeHandle priv("~");

		// -- prepare clients --
		_which_map_client = priv.serviceClient<bender_srvs::String>("/bender/knowledge/pose_server/which");
		_set_client = priv.serviceClient<bender_srvs::SemMap>("/bender/knowledge/pose_server/set");
		_get_all_client = priv.serviceClient<bender_srvs::SemMap>("/bender/knowledge/pose_server/get_all");
		_get_client = priv.serviceClient<bender_srvs::SemMap>("/bender/knowledge/pose_server/get");
		_save_client = priv.serviceClient<bender_srvs::String>("/bender/knowledge/pose_server/save");

		ROS_INFO("Waiting for services");
		while ( ros::ok() && !_which_map_client.waitForExistence(ros::Duration(3.0)) );
		while ( ros::ok() && !_set_client.waitForExistence(ros::Duration(3.0)) );
		while ( ros::ok() && !_get_all_client.waitForExistence(ros::Duration(3.0)) );
		while ( ros::ok() && !_get_client.waitForExistence(ros::Duration(3.0)) );
		while ( ros::ok() && !_save_client.waitForExistence(ros::Duration(3.0)) );

		if (ros::ok()) {
			_current_map_name = getMapName();
		}

		return ros::ok() ? true : false;
	};

	std::string getMapName() {

		bender_srvs::String str_srv;
		_which_map_client.call(str_srv);
		return str_srv.response.data;
	};

	std::string getType(const std::string& key) {

		bender_srvs::SemMap get_srv;
		get_srv.request.id = key;
		if ( !_get_client.call(get_srv) ) {
			ROS_ERROR_STREAM("Failed to call service " << _get_client.getService());
			return "";
		}
		return get_srv.response.data[0].type;
	};

	void getAll(std::vector<bender_msgs::SemanticObject> &map_data) {

		map_data.clear();

		bender_srvs::SemMap all_srv;
		if ( !_get_all_client.call(all_srv) ) {
			ROS_ERROR_STREAM("Failed to call service " << _get_all_client.getService());
			return;
		}

		map_data = all_srv.response.data;
	};

	bool add(bender_msgs::SemanticObject data) {

		bender_srvs::SemMap srv;
		srv.request.new_data = data;

		if (!_set_client.call(srv)) {
			ROS_ERROR_STREAM("Failed to call service" << _set_client.getService());
		}

		bender_srvs::String str_srv;
		str_srv.request.data = _current_map_name;
		if (!_save_client.call(str_srv)) {

			ROS_ERROR_STREAM("Failed to save map '" << _current_map_name << "'");
			return true;
		}
		ROS_INFO_STREAM("Map file saved: " << _current_map_name);

		return true;
	};

	bool add(const std::string &marker_name, const std::string &marker_type,
			const std::string &frame_id, const geometry_msgs::Pose& pose) {

		bender_msgs::SemanticObject data;
		data.id = marker_name;
		data.type = marker_type;
		data.frame_id = frame_id;
		data.pose = pose;
		return this->add(data);
	};

};

// marker server
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> _server;
PoseServerMarkerHandler map_server_handler;
interactive_markers::MenuHandler _menu_handler;

void process_marker_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {

	switch (feedback->event_type) {

		case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE: {

			ros::NodeHandle priv("~");
			ros::Publisher marker_names_pub = priv.advertise<visualization_msgs::MarkerArray>("pose_names", 1);

			// update text marker
			visualization_msgs::MarkerArray marker_names;
			visualization_msgs::Marker name;
			name.header.stamp = ros::Time::now();
			name.id = 0;
			name.lifetime = ros::Duration(10.0);
			name.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			name.action = visualization_msgs::Marker::MODIFY;
			name.color.a = 1.0; name.color.g = 1.0;
			name.color.r = 1.0; name.color.b = 1.0;
			name.scale.z = 0.3;

			// -- create marker name for displaying --
			name.ns = "smap_" + feedback->marker_name;
			name.text = feedback->marker_name;
			name.header.frame_id = feedback->header.frame_id;
			name.pose = feedback->pose;
			name.pose.position.z += 0.5;
			marker_names.markers.push_back(name);
			marker_names_pub.publish(marker_names);
			break;
		}
		case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP: {

			ROS_INFO_STREAM("Saving map pose: " << feedback->marker_name);
			std::string object_type = map_server_handler.getType(feedback->marker_name);
			map_server_handler.add(feedback->marker_name, object_type, feedback->header.frame_id, feedback->pose);

			break;
		}
		case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT: {

			if (feedback->menu_entry_id == 1) {
				ROS_INFO_STREAM("Setting name: (TODO)");

			} else if (feedback->menu_entry_id == 3) {

				ROS_INFO_STREAM("Setting type: 'map_pose' for marker " << feedback->marker_name);
				map_server_handler.add(feedback->marker_name, "map_pose", feedback->header.frame_id, feedback->pose);

				// -- modify interactive marker --

				// create interactive marker
				visualization_msgs::InteractiveMarker int_marker;
				int_marker.header = feedback->header;
				int_marker.name = feedback->marker_name;
				int_marker.description = ""; // this description is too small! (as we want scale=0.5)
				int_marker.pose = feedback->pose;
				int_marker.scale = 0.5;

				// append controls
				make_map_marker_control(int_marker);
				add_menu_marker_control(int_marker);

				// modify
				_server->insert(int_marker);
				_menu_handler.apply(*_server, int_marker.name);


			} else if (feedback->menu_entry_id == 4) {

				ROS_INFO_STREAM("Setting type: 'object_pose' for marker " << feedback->marker_name);
				map_server_handler.add(feedback->marker_name, "object_pose", feedback->header.frame_id, feedback->pose);

				// -- modify interactive marker --

				// create interactive marker
				visualization_msgs::InteractiveMarker int_marker;
				int_marker.header = feedback->header;
				int_marker.name = feedback->marker_name;
				int_marker.description = ""; // this description is too small! (as we want scale=0.5)
				int_marker.pose = feedback->pose;
				int_marker.scale = 0.5;

				// append controls
				make_object_marker_control(int_marker);
				add_menu_marker_control(int_marker);

				// modify
				_server->insert(int_marker);
				_menu_handler.apply(*_server, int_marker.name);
			}
			break;

		}
		default: {
			break;
		}

	}

	_server->applyChanges();
}

class PoseServerMarkerPlugin {

private:

	int _new_point_cnt;

	// new marker subscriber
	ros::Subscriber _new_marker_sub;

	// marker names publisher
	ros::Publisher _marker_names_pub;

public:

	PoseServerMarkerPlugin();
	virtual ~PoseServerMarkerPlugin();

private:

	void loadMarkers();

	// -- callbacks --
	void newMarker_callback(const geometry_msgs::PointStamped msg);

};

PoseServerMarkerPlugin::PoseServerMarkerPlugin() {

	ros::NodeHandle priv("~");
	_new_point_cnt = 0;

	// -- prepare server --
	map_server_handler.initialize();
	_server.reset( new interactive_markers::InteractiveMarkerServer("pose_server_markers") );

	// marker menu setup
	_menu_handler.insert("set name(TODO)", &process_marker_feedback);
	interactive_markers::MenuHandler::EntryHandle handler_select_type = _menu_handler.insert("select type");
	_menu_handler.insert(handler_select_type, "map_pose", &process_marker_feedback);
	_menu_handler.insert(handler_select_type, "object_pose", &process_marker_feedback);

	// Get & publish saved poses
	_marker_names_pub = priv.advertise<visualization_msgs::MarkerArray>("pose_names", 1, this);
	loadMarkers();

	// -- new marker subscription --
	_new_marker_sub = priv.subscribe("set",10, &PoseServerMarkerPlugin::newMarker_callback, this);

	ROS_INFO("Ready to Work");
}

PoseServerMarkerPlugin::~PoseServerMarkerPlugin() {
	_server.reset();
}

void PoseServerMarkerPlugin::loadMarkers() {

	// get map data
	std::vector<bender_msgs::SemanticObject> map_data;
	map_server_handler.getAll(map_data);

	// marker names
	visualization_msgs::MarkerArray marker_names;
	visualization_msgs::Marker name;
	name.header.stamp = ros::Time::now();
	name.id = 0;
	name.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	name.action = visualization_msgs::Marker::MODIFY;
	name.lifetime = ros::Duration(10.0);
	name.color.a = 1.0; name.color.g = 1.0;
	name.color.r = 1.0; name.color.b = 1.0;
	name.scale.z = 0.3;

	// append data
	std::vector<bender_msgs::SemanticObject>::const_iterator it;
	for( it = map_data.begin(); it != map_data.end(); ++it) {

		// -- create interactive marker --
		visualization_msgs::InteractiveMarker int_marker;
		int_marker.header.stamp = ros::Time::now();
		int_marker.header.frame_id = it->frame_id;
		int_marker.name = it->id;
		int_marker.description = ""; // this description is too small! (as we want scale=0.5)
		int_marker.pose = it->pose;
		int_marker.scale = 0.5;

		// marker motion controls and type depends on data type
		if (it->type == "map_pose")   { make_map_marker_control(int_marker);    }
		else if (it->type == "object_pose") { make_object_marker_control(int_marker); }
		else { make_unknown_marker_control(int_marker); }

		add_menu_marker_control(int_marker);
		_server->insert(int_marker);
		_server->setCallback(int_marker.name, &process_marker_feedback);
		_menu_handler.apply(*_server, int_marker.name);

		// -- create marker name for displaying --
		name.ns = "smap_" + it->id;
		name.text = it->id;
		name.header.frame_id = it->frame_id;
		name.pose = it->pose;
		name.pose.position.z += 0.5;
		marker_names.markers.push_back(name);

	}
	_server->applyChanges();
	_marker_names_pub.publish(marker_names);
}

void PoseServerMarkerPlugin::newMarker_callback(const geometry_msgs::PointStamped msg) {

	// -- generate marker name --
	std::string marker_name;
	std::stringstream ss;
	ss << "ID_" << _new_point_cnt;
	_new_point_cnt++;
	marker_name = ss.str();

	// -- create marker name for displaying --
	visualization_msgs::MarkerArray names_array;
	visualization_msgs::Marker name;
	name.header.stamp = ros::Time::now();
	name.ns = "smap_" + marker_name;
	name.id = 0;
	name.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	name.action = visualization_msgs::Marker::MODIFY;
	name.text = marker_name;
	name.lifetime = ros::Duration(10.0);
	name.color.a = 1.0; name.color.g = 1.0;
	name.color.r = 1.0; name.color.b = 1.0;
	name.scale.z = 0.3;
	name.header = msg.header;
	name.pose.orientation.w = 1.0;
	name.pose.position = msg.point;
	name.pose.position.z += 0.5;
	names_array.markers.push_back(name);

	// -- create interactive marker --
	visualization_msgs::InteractiveMarker int_marker;
	int_marker.header = msg.header;
	int_marker.name = marker_name;
	int_marker.description = ""; // this description is too small! (as we want scale=0.5)
	int_marker.pose.orientation.w = 1.0;
	int_marker.pose.position = msg.point;
	int_marker.scale = 0.5;
	make_unknown_marker_control(int_marker);
	add_menu_marker_control(int_marker);

	// -- add data to map server --
	map_server_handler.add(marker_name, "unknown", msg.header.frame_id, int_marker.pose);

	_server->insert(int_marker);
	_server->setCallback(int_marker.name, &process_marker_feedback);
	_menu_handler.apply(*_server, int_marker.name);

	// apply changes and display
	_server->applyChanges();
	_marker_names_pub.publish(names_array);
}

} /* namespace bender_nav */

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pose_server_marker_plugin");

	boost::scoped_ptr<bender_nav::PoseServerMarkerPlugin> node(
		new bender_nav::PoseServerMarkerPlugin()
	);

	ros::spin();

	printf("\nQuitting... \n\n");

	return 0;
}
