#ifndef DYN_POLYGON_FILTER_H
#define DYN_POLYGON_FILTER_H

#include <filters/filter_base.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_geometry/laser_geometry.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <bender_utils/ParameterServerWrapper.h>

namespace bender_laser_pipeline
{
/**
 * @brief This is a filter that removes points in a laser scan inside of a cartesian box.
 */
class LaserScanDynamicPolygonFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
  public:
    LaserScanDynamicPolygonFilter();
    bool configure();

    bool update(
      const sensor_msgs::LaserScan& input_scan,
      sensor_msgs::LaserScan& filtered_scan);

  private:
    bool inBox(tf::Point &point);
    std::string box_frame_;
    laser_geometry::LaserProjection projector_;
    
    // tf listener to transform scans into the box_frame
    tf::TransformListener tf_; 
    
    // defines two opposite corners of the box
    tf::Point min_, max_; 
    bool up_and_running_;
};

}


#endif /* dyn_polygon_filter.h */
