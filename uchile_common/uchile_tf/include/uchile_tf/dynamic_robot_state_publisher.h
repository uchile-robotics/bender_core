/*
 * Rodrigo Munoz
 * Mayo 2015
 * Basado en robot_state_publisher de Wim Meeussen
 */

#ifndef DYNAMIC_ROBOT_STATE_PUBLISHER_H
#define DYNAMIC_ROBOT_STATE_PUBLISHER_H

#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <kdl/segment.hpp>
#include <kdl/tree.hpp>
#include <uchile_tf/dynamic_tf.h>

namespace robot_state_publisher{

class SegmentPair
{
public:
  SegmentPair(const KDL::Segment& p_segment, const std::string& p_root, const std::string& p_tip):
    segment(p_segment), root(p_root), tip(p_tip){}

  KDL::Segment segment;
  std::string root, tip;
};



class RobotStatePublisher
{
public:
  /** Constructor
   * \param tree The kinematic model of a robot, represented by a KDL Tree 
   */
  RobotStatePublisher(const KDL::Tree& tree, ros::NodeHandle& nh);

  /// Destructor
  ~RobotStatePublisher(){};

  /** Publish transforms to tf 
   * \param joint_positions A map of joint names and joint positions. 
   * \param time The time at which the joint positions were recorded
   */
  void publishTransforms(const std::map<std::string, double>& joint_positions, const ros::Time& time, const std::string& tf_prefix);
  void publishFixedTransforms(const std::string& tf_prefix);

private:
  void addChildren(const KDL::SegmentMap::const_iterator segment);


  std::map<std::string, SegmentPair> segments_, segments_fixed_;
  tf::TransformBroadcaster tf_broadcaster_;
  ros::NodeHandle nh_;
  std::map<std::string, uchile_tf::DynamicTfPtr> dynamic_tf_;
};

}

#endif
