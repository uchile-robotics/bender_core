/*
 * MapAnalyzer.cpp
 *
 *  Created on: January, 2015
 *      Author: matias.pavez.b@gmail.com
 */

#include "bender_maps/MapAnalyzer.h"

namespace bender_knowledge {

// TODO: compute bounding boxes from map image, this way the node configuration is easier.
// TODO: checkPointCloud2Service() with XYZ& vs. XYZPtr&. choose the best one!.

MapAnalyzer::MapAnalyzer(std::string) {

	ros::NodeHandle priv("~");

	// - - - - parameters - - - -
	bender_utils::ParameterServerWrapper psw;
	psw.getParameter("tf_map_frame", _map_frame, "/map");
	psw.getParameter("map_polygon", _map_polygon, _map_polygon);
	psw.getParameter("map_bounding_box", _map_bounding_box, _map_bounding_box);

	if (_map_bounding_box.points.size() != 4 ) {
		ROS_ERROR_STREAM("Bounding box must have 4 points!... Bye!");
		exit(1);
	}
	if (_map_polygon.points.size() < 4 ) {
		ROS_WARN_STREAM("Map polygon must have at least 4 points!... Bye!");
		exit(1);
	}

	// rooms
	psw.getParameter("rooms", _rooms, _rooms);
	for (int idx=0; idx < _rooms.size(); idx++) {
		std::string room_name = _rooms[idx];
		geometry_msgs::Polygon room_polygon;
		psw.getParameter(room_name, room_polygon, room_polygon);

		if (room_polygon.points.size() < 4 ) {
			ROS_ERROR_STREAM("Room " << room_name << " polygon must have at least 4 points!... Bye!");
			exit(1);
		}
		_room_polygons[room_name] = room_polygon;
	}
	psw.getParameter("debug", _debug_enabled, false);


	// - - - - publishers - - - -
	_rviz_pub =  priv.advertise<visualization_msgs::MarkerArray>("map_data",1,this);

	// - - - - Servers - - - -
	_check_point_server = priv.advertiseService("check_point_inside_map", &MapAnalyzer::checkPointService,this);
	_mask_point_cloud2_server = priv.advertiseService("mask_pointcloud2_inside_map", &MapAnalyzer::checkPointCloud2Service,this);

	// - - - - (testing) - - - -
	if (_debug_enabled) {
		_cloud_testing_sub = priv.subscribe("test_cloud", 1, &MapAnalyzer::testCloudCallback, this);
		_cloud_testing_pub = priv.advertise<sensor_msgs::PointCloud2>("test_cloud_masked", 1, this);
		ROS_WARN_STREAM("Using Map analyzer in Testing Mode:\n" <<
				" - Subscribed to clouds on topic: '" << _cloud_testing_sub.getTopic() << "'\n" <<
				" - Publishing masked cloud on topic: '" << _cloud_testing_pub.getTopic() << "'");
	}
	
	// [important!] wait for timer initialization!
	while (ros::ok() && ros::Time::now().isZero());
	ros::Duration(1.0).sleep();

	// publish polygons
	publishPolygons();

	ROS_INFO("Ready to work");
}

MapAnalyzer::~MapAnalyzer() {

}

void MapAnalyzer::testCloudCallback(const sensor_msgs::PointCloud2& cloud) {

	if (_cloud_testing_pub.getNumSubscribers() > 0) {

		bender_srvs::PointCloud2Trasnform srv;
		srv.request.cloud_in = cloud;
		srv.request.frame_out = cloud.header.frame_id;

		checkPointCloud2Service(srv.request, srv.response );

		ROS_INFO_STREAM("publishing masked cloud with header.seq = " << srv.response.cloud_out.header.seq << "and output frame: " << srv.response.cloud_out.header.frame_id);
		_cloud_testing_pub.publish(srv.response.cloud_out);
	}
}

bool MapAnalyzer::checkPointCloud2Service(bender_srvs::PointCloud2Trasnform::Request &req, bender_srvs::PointCloud2Trasnform::Response& res) {

	// desired == ("" or "/map") --> don't apply transform
	bool should_apply_final_tf = (req.frame_out != _map_frame && req.frame_out != "");

	// transform frame
	sensor_msgs::PointCloud2 cloud_tf, cloud_tf2;
	try {
		_tf_listener.waitForTransform(_map_frame, req.cloud_in.header.frame_id, ros::Time(0), ros::Duration(1.0));
		pcl_ros::transformPointCloud(_map_frame, req.cloud_in, cloud_tf, _tf_listener);
	} catch (tf::ConnectivityException &e) {
		ROS_WARN_STREAM("Requested a transform between unconnected trees!: " << e.what());
		return false;
	} catch (tf::TransformException &e) {
		ROS_WARN_STREAM("Transform Exception: " << e.what());
		return false;
	} catch (std::exception &e) {
		ROS_WARN_STREAM("Exception thrown while trying to transform the PointCloud2 data: " << e.what());
		return false;
	}

	try {
		// transform ROS PointCloud2 to pcl PointCloud
		pcl::PointCloud< pcl::PointXYZ > pcl_cloud;
		pcl::fromROSMsg(cloud_tf, pcl_cloud);

		// mask points
		int cloudsize = (pcl_cloud.width) * (pcl_cloud.height);
		for (int i=0; i< cloudsize; i++){

			if ( checkPoint(pcl_cloud.points[i].x, pcl_cloud.points[i].y) == false ) {
				pcl_cloud.points[i].x = pcl_cloud.points[i].y = pcl_cloud.points[i].z = invalid_pcl_point;
			}
		}

		// transform back to ROS PointCloud2 message
		if (should_apply_final_tf) {
			pcl::toROSMsg(pcl_cloud, cloud_tf2);
		} else {
			pcl::toROSMsg(pcl_cloud, res.cloud_out);
		}

	} catch (std::exception &e) {
		ROS_ERROR_STREAM("Unknown error while trying to mask a pointcloud: " << e.what());
		return false;
	}

	// we are ready
	if (!should_apply_final_tf) {
		return true;
	}

	// else: transform to the desired frame
	try {
		_tf_listener.waitForTransform(req.frame_out, _map_frame, ros::Time::now(), ros::Duration(1.0));
		pcl_ros::transformPointCloud(req.frame_out, cloud_tf2, res.cloud_out, _tf_listener);
	} catch (tf::ConnectivityException &e) {
		ROS_WARN_STREAM("Requested a transform between unconnected trees!: " << e.what());
		return false;
	} catch (tf::TransformException &e) {
		ROS_WARN_STREAM("Transform Exception: " << e.what());
		return false;
	} catch (std::exception &e) {
		ROS_WARN_STREAM("Exception thrown while trying to transform the PointCloud2 data: " << e.what());
		return false;
	}
	
	return true;
}

// Transforms a polygon into a marker, only modifying the 'points' field.
void MapAnalyzer::polygonToMarker(const geometry_msgs::Polygon &polygon, visualization_msgs::Marker &marker) {

	marker.points.clear();
	int n_points = polygon.points.size();
	for (int i=0; i<=n_points; i++) {
		geometry_msgs::Point p;
		p.x = polygon.points[i%n_points].x;
		p.y = polygon.points[i%n_points].y;
		p.z = polygon.points[i%n_points].z;
		marker.points.push_back(p);
	}
}

void MapAnalyzer::publishPolygons() {

	visualization_msgs::MarkerArray rviz;
	visualization_msgs::Marker marker;

	// polygon marker
	marker.header.frame_id = _map_frame;
	marker.header.stamp = ros::Time::now();
	marker.action = visualization_msgs::Marker::MODIFY;
	marker.type   = visualization_msgs::Marker::LINE_STRIP;
	marker.lifetime = ros::Duration(0);
	marker.scale.x = 0.05;
	marker.color.a = 1.0;

	// map bbox
	marker.ns = "map_bbox";
	marker.id = 0;
	polygonToMarker(_map_bounding_box, marker);
	marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;
	rviz.markers.push_back(marker);

	// map polygon
	marker.ns = "map_polygon";
	marker.id = 0;
	polygonToMarker(_map_polygon, marker);
	marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
	rviz.markers.push_back(marker);

	// rooms
	marker.ns = "map_rooms";
	marker.id = 0;
	marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0;
	std::map<std::string,geometry_msgs::Polygon>::iterator it;
	for (it = _room_polygons.begin(); it != _room_polygons.end(); it++) {

		polygonToMarker(it->second, marker);
		rviz.markers.push_back(marker);
		marker.id++;
	}

	// room names
	marker.ns = "map_room_names";
	marker.id = 0;
	marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 1.0;
	marker.points.clear();
	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker.scale.z = 0.6;
	for (it = _room_polygons.begin(); it != _room_polygons.end(); it++) {

		marker.pose.position.x = it->second.points[0].x;
		marker.pose.position.y = it->second.points[0].y;
		marker.pose.position.z = it->second.points[0].z;
		marker.pose.orientation.w = 1.0;
		marker.text = it->first;
		rviz.markers.push_back(marker);
		marker.id++;
	}

	// publish
	_rviz_pub.publish(rviz);
}

bool MapAnalyzer::checkPointService(bender_srvs::ValidPoint::Request &req, bender_srvs::ValidPoint::Response& res) {

	geometry_msgs::PointStamped ps_in;
	geometry_msgs::PointStamped ps_out;
	ps_in.header.frame_id = req.frame_id;
	ps_in.header.stamp = ros::Time(0);
	ps_in.point = req.point;

	try {
		_tf_listener.waitForTransform(_map_frame, req.frame_id, ros::Time(0), ros::Duration(1.0));
		_tf_listener.transformPoint(_map_frame, ps_in, ps_out);
	} catch (tf::ConnectivityException &e) {
		ROS_WARN_STREAM("Requested a transform between unconnected trees!: " << e.what());
		return false;
	} catch (tf::TransformException &e) {
		ROS_WARN_STREAM("Transform Exception: " << e.what());
		return false;
	}

	try {
		// inside map
		res.is_valid = checkPoint(ps_out.point.x, ps_out.point.y);

		// related rooms
		std::map<std::string,geometry_msgs::Polygon>::iterator it;
		for (it = _room_polygons.begin(); it != _room_polygons.end(); it++) {

			if (checkInsidePolygon(ps_out.point.x, ps_out.point.y,it->second)) {
				res.data.push_back(it->first);
			}
		}

	} catch (std::exception &e) {
		ROS_ERROR_STREAM("Unknown exception while trying to check point validity: " << e.what());
		return false;
	}
	return true;
}

bool MapAnalyzer::checkInsideBoundingBox(float x, float y) {

	return checkInsidePolygon(x, y, _map_bounding_box);
}

bool MapAnalyzer::checkInsidePolygon(float x, float y, geometry_msgs::Polygon &polygon) {

	// map polygon
	int _n_vert = polygon.points.size();

	int i, j;
	bool c = false;
	for (i = 0, j = _n_vert-1; i < _n_vert; j = i++) {
		if (
				( (polygon.points[i].y > y) != (polygon.points[j].y > y) )
				&&
				( x < (polygon.points[j].x -polygon.points[i].x) * (y -polygon.points[i].y) /
						(polygon.points[j].y - polygon.points[i].y) + polygon.points[i].x) ) {
			c = !c;
		}
	}
	return c;
}

bool MapAnalyzer::checkPoint(float x, float y) {

	if ( checkInsideBoundingBox(x ,y) == false ) {
		return false;
	}
	return checkInsidePolygon(x, y, _map_polygon);
}

} /* namespace bender_knowledge */


int main(int argc, char **argv) {

	ros::init(argc, argv, "map_analyzer");

	bender_knowledge::MapAnalyzer node(ros::this_node::getName());

	ros::spin();

	printf("\nQuitting... \n\n");
	return 0;
}


