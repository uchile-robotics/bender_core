#ifndef BENDER_SELF_LASER_FILTER_H
#define BENDER_SELF_LASER_FILTER_H

#include <filters/filter_base.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_geometry/laser_geometry.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>


namespace uchile_laser_pipeline
{

class LaserScanSelfFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
  public:
    LaserScanSelfFilter();
    bool configure();

    bool update(
      const sensor_msgs::LaserScan& input_scan,
      sensor_msgs::LaserScan& filtered_scan);

  private:

    void getAngularLimits(float x, float y, float z, float r, float &theta_min, float &theta_max);

    std::vector<std::string> _target_frames;
    std::vector<double> _inflation_radius_list;

    laser_geometry::LaserProjection projector_;
    
    // tf listener to transform scans into the box_frame
    tf::TransformListener tf_listener;
};

}


#endif /* self_filter.h */
