#include "uchile_laser_pipeline/self_filter.h"

uchile_laser_pipeline::LaserScanSelfFilter::LaserScanSelfFilter() {

}

// TODO: wait for tfs a few seconds while configuring the pipeline
bool uchile_laser_pipeline::LaserScanSelfFilter::configure() {

    bool succeeded = true;
    if (!getParam("target_frames", _target_frames)) {
        ROS_ERROR(
        "The 'target_frames' parameter is not set. It corresponds to a list of the frames to consider when"
                " filtering the laser scan data for self collisions. e.g.: ['left_hand', 'right_hand'].");
        succeeded = false;
    }

    if (!getParam("inflation_radius_list", _inflation_radius_list)) {
        ROS_ERROR("The 'inflation_radius_list' parameter is not set. It corresponds to a list inflation radius for each"
                          "provided target frame. They must be constrained to [0, 0.5] meters."
                          "The inflation is used to create target spheres to be removed from laser scans."
                          " e.g.: ['0.3', 0.3].");
        succeeded = false;
    } else {
        std::vector<double>::const_iterator it;
        for(it = _inflation_radius_list.begin(); it != _inflation_radius_list.end(); ++it) {
            if (*it < 0 || *it > 0.5) {
                ROS_ERROR_STREAM("The 'inflation_radius_list' values must be in range [0, 0.5] meters. Provided: " << *it);
                succeeded = false;
            }
        }

        if (_inflation_radius_list.size() != _target_frames.size()) {
            ROS_ERROR_STREAM("The 'inflation_radius_list' size ("
                                     << _inflation_radius_list.size()
                                     << ") does not match the 'target_frames' size ("
                                     << _target_frames.size()
                                     << ").");
            succeeded = false;
        }
    }
    ros::Duration(3).sleep();
    return succeeded;
}

bool uchile_laser_pipeline::LaserScanSelfFilter::update(
        const sensor_msgs::LaserScan &input_scan,
        sensor_msgs::LaserScan &output_scan) {

    ros::Time target_time = input_scan.header.stamp + ros::Duration().fromSec(input_scan.ranges.size()*input_scan.time_increment);

    // point centered at required frame
    geometry_msgs::PointStamped link_point;
    link_point.header.stamp = target_time;

    // copy
    output_scan = input_scan;

    std::string error_msg;
    std::vector<std::string>::const_iterator it;
    std::vector<double>::const_iterator r_it;
    for(it = _target_frames.begin(), r_it = _inflation_radius_list.begin(); it != _target_frames.end(); ++it, ++r_it) {

        std::string target_frame = *it;
        float radius = (float)*r_it;

        link_point.header.frame_id = target_frame;

        bool success = tf_listener.waitForTransform(
                target_frame,
                input_scan.header.frame_id,
                target_time,
                ros::Duration(1.0),
                ros::Duration(0.01),
                &error_msg
        );
        if(!success){
            ROS_WARN_STREAM_THROTTLE(0.2 , "Could not get transform for frame '"
                    << target_frame.c_str()
                    << "'. Ignoring this frame. TF Error: "
                    << error_msg.c_str()
            );
            continue;
        }

        geometry_msgs::PointStamped link_point_transformed;
        try {
            tf_listener.transformPoint(input_scan.header.frame_id, link_point, link_point_transformed);
        }
        catch(tf::TransformException& ex) {
            ROS_WARN_STREAM_THROTTLE(1, "Ignoring frame: '"
                    << target_frame.c_str()
                    << "'. Transform Exception: " << ex.what());
            continue;
        }

        // link does not collides with the laser scan plane
        if (fabsf((float)link_point_transformed.point.z) > radius) {
            continue;
        }

        // angular limits
        float theta_min;
        float theta_max;
        getAngularLimits(
                (float) link_point_transformed.point.x,
                (float) link_point_transformed.point.y,
                (float) link_point_transformed.point.z,
                radius, theta_min, theta_max);

        // remove ranges which collide with our projection
        float cur_angle;
        std::vector<float>::iterator range_it;
        for (range_it = output_scan.ranges.begin(), cur_angle = output_scan.angle_min;
             range_it != output_scan.ranges.end();
             ++range_it, cur_angle += output_scan.angle_increment) {

            if (theta_min < cur_angle  && cur_angle < theta_max) {
                *range_it = output_scan.range_max + 1;
            }
        }

    }
    return true;
}

/**
 * The algorithm works as follows:
 * Each considered joint/frame on 'target_frames' is inflated and
 * considered as a sphere centered at (0,0,0) with radius given by
 * 'inflation_radius_list'.
 *
 * Then, the sphere center is transformed into the laser scan frame,
 * centered at (x,y,z). If the sphere is too high or too low, then
 * there can't be a collision.
 *
 * The goal is to remove/invalidate laser points which collides with
 * our sphere, so we project it into the laser X-Y plane, resulting
 * in a circle on (xm, ym), z=0 and radius rm.
 *
 * Then we compute the min/max angles of the scan sweep to be removed,
 * by finding the tangent lines to the projection which intersects
 * the laser center at (0,0)
 */
void uchile_laser_pipeline::LaserScanSelfFilter::getAngularLimits(
        float x, float y, float z, float r, float &theta_min, float &theta_max) {

    // sphere projection into the laser plane
    float xm = x;
    float ym = y;
    float rm = sqrtf(r*r - z*z);

    // angle from laser to projection center
    float theta_m = atan2f(ym, xm);

    // distance to the laser
    float d = sqrtf(xm*xm + ym*ym);
    // TODO: check whether d ~< rm, this means the link is colliding to the laser

    // length of tangent segment to the laser
    float td = sqrtf(d*d - rm*rm);
    td = fmaxf(td, 0.01); // min 1 [cm].

    // angle/2 between the 2 tangents
    float delta_theta = atanf(rm/td);

    // angular limits
    theta_min = theta_m - delta_theta;
    theta_max = theta_m + delta_theta;
}
