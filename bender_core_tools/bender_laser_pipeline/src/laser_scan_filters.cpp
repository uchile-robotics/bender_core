#include "bender_laser_pipeline/dyn_polygon_filter.h"
#include "sensor_msgs/LaserScan.h"
#include "filters/filter_base.h"

#include "pluginlib/class_list_macros.h"

PLUGINLIB_DECLARE_CLASS(bender_laser_pipeline, LaserScanDynamicPolygonFilter, bender_laser_pipeline::LaserScanDynamicPolygonFilter, filters::FilterBase<sensor_msgs::LaserScan>)
