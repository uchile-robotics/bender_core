#include "bender_laser_pipeline/self_filter.h"

bender_laser_pipeline::LaserScanSelfFilter::LaserScanSelfFilter() {

}

bool bender_laser_pipeline::LaserScanSelfFilter::configure() {
    up_and_running_ = true;

    bender_utils::ParameterServerWrapper psw;
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
                ROS_ERROR_STREAM("The 'inflation_radius' must be in range [0, 0.5] meters. Provided: " << *it);
                succeeded = false;
            }
        }
    }
    return succeeded;
}

bool bender_laser_pipeline::LaserScanSelfFilter::update(
        const sensor_msgs::LaserScan &input_scan,
        sensor_msgs::LaserScan &output_scan) {
    output_scan = input_scan;



    up_and_running_ = true;
    return true;
}
