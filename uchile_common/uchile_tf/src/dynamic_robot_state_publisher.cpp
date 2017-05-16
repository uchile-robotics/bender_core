/*
 * Rodrigo Munoz
 * Mayo 2015
 * Basado en robot_state_publisher de Wim Meeussen
 */

#include <uchile_tf/dynamic_robot_state_publisher.h>
#include <kdl/frames_io.hpp>
#include <tf_conversions/tf_kdl.h>


using namespace std;
using namespace ros;


namespace robot_state_publisher{

  RobotStatePublisher::RobotStatePublisher(const KDL::Tree& tree, ros::NodeHandle& nh):
    nh_(nh)
  {
    // walk the tree and add segments to segments_
    addChildren(tree.getRootSegment());
  }


  // Add children to correct maps
  void RobotStatePublisher::addChildren(const KDL::SegmentMap::const_iterator segment)
  {
    const std::string& root = GetTreeElementSegment(segment->second).getName();

    const std::vector<KDL::SegmentMap::const_iterator>& children = GetTreeElementChildren(segment->second);
    for (unsigned int i=0; i<children.size(); i++){
      const KDL::Segment& child = GetTreeElementSegment(children[i]->second);
      SegmentPair s(GetTreeElementSegment(children[i]->second), root, child.getName());
      if (child.getJoint().getType() == KDL::Joint::None){
        // Add dynamic TF
        uchile_tf::DynamicTfPtr tf_ptr(new uchile_tf::DynamicTf(child.getJoint().getName(), nh_.getNamespace()));
        dynamic_tf_.insert(make_pair(child.getJoint().getName(), tf_ptr));

        segments_fixed_.insert(make_pair(child.getJoint().getName(), s));
        ROS_DEBUG("Adding fixed segment from %s to %s", root.c_str(), child.getName().c_str());
      }
      else{
        segments_.insert(make_pair(child.getJoint().getName(), s));
        ROS_DEBUG("Adding moving segment from %s to %s", root.c_str(), child.getName().c_str());
      }
      addChildren(children[i]);
    }
  }

  // Publish moving transforms
  void RobotStatePublisher::publishTransforms(const map<string, double>& joint_positions, const Time& time, const std::string& tf_prefix)
  {
    ROS_DEBUG("Publishing transforms for moving joints");
    std::vector<tf::StampedTransform> tf_transforms;
    tf::StampedTransform tf_transform;
    tf_transform.stamp_ = time;

    // Loop over all joints
    for (std::map<std::string, double>::const_iterator jnt=joint_positions.begin(); jnt != joint_positions.end(); jnt++){
      std::map<std::string, SegmentPair>::const_iterator seg = segments_.find(jnt->first);
      if (seg != segments_.end()){
        tf::transformKDLToTF(seg->second.segment.pose(jnt->second), tf_transform);    
        tf_transform.frame_id_ = tf::resolve(tf_prefix, seg->second.root);
        tf_transform.child_frame_id_ = tf::resolve(tf_prefix, seg->second.tip);
        tf_transforms.push_back(tf_transform);
      }
    }
    tf_broadcaster_.sendTransform(tf_transforms);
  }


  // Publish fixed transforms using dynamic TF data
  void RobotStatePublisher::publishFixedTransforms(const std::string& tf_prefix)
  {
    ROS_DEBUG("Publishing transforms for fixed joints");
    std::vector<tf::StampedTransform> tf_transforms;
    tf::StampedTransform tf_transform;
    tf_transform.stamp_ = ros::Time::now();

    // loop over all fixed segments
    for (std::map<std::string, SegmentPair>::const_iterator seg=segments_fixed_.begin(); seg != segments_fixed_.end(); seg++){
      // Get KDL Frame
      KDL::Frame frame = seg->second.segment.pose(0);
      // Apply dynamic TF
      std::map<std::string,uchile_tf::DynamicTfPtr>::const_iterator tf_config = dynamic_tf_.find(seg->first);
      if (tf_config !=  dynamic_tf_.end())
      {
        tf_config->second->applyTransform(frame);
      }
      tf::transformKDLToTF(frame, tf_transform);    
      tf_transform.frame_id_ = tf::resolve(tf_prefix, seg->second.root);
      tf_transform.child_frame_id_ = tf::resolve(tf_prefix, seg->second.tip);
      tf_transforms.push_back(tf_transform);
    }
    tf_broadcaster_.sendTransform(tf_transforms);
  }
}