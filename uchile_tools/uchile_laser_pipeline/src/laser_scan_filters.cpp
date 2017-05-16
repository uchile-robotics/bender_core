#include "uchile_laser_pipeline/self_filter.h"
#include "sensor_msgs/LaserScan.h"
#include "filters/filter_base.h"

#include "pluginlib/class_list_macros.h"

PLUGINLIB_DECLARE_CLASS(uchile_laser_pipeline, LaserScanSelfFilter, uchile_laser_pipeline::LaserScanSelfFilter, filters::FilterBase<sensor_msgs::LaserScan>)
