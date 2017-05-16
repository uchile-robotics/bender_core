/*
 * Rodrigo Munoz
 * Mayo 2015
 * Basado en robot_state_publisher de Wim Meeussen
 */

// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
// URDF parse utils
#include <urdf/model.h>
// KDL
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <uchile_tf/dynamic_tf.h>
#include <uchile_tf/dynamic_robot_state_publisher.h>

// Util para almacenar joints mimics
typedef std::map<std::string, boost::shared_ptr<urdf::JointMimic> > MimicMap;

using namespace std;

namespace uchile_tf {

class DynamicJointStateListener {
  
public:
  /**
   * @brief Constructor
   *  
   * @param tree Representacion del robot usando KDL, usualmente se obtiene a partir del URDF.
   * @param m Joints de tipo mimic.
   */
  DynamicJointStateListener(const KDL::Tree& tree, const MimicMap& m, ros::NodeHandle& node);

  // Destructor
  ~DynamicJointStateListener();

private:
  // Suscriber callback
  void callbackJointState(const sensor_msgs::JointStateConstPtr& state);
  // Timer callback for fixed joints
  void callbackFixedJoint(const ros::TimerEvent& e);

  // TF prefix
  std::string tf_prefix_;
  // Timer rate
  ros::Duration publish_interval_;
  robot_state_publisher::RobotStatePublisher state_publisher_;
  // Suscriber for joint state
  ros::Subscriber joint_state_sub_;
  // Timer for publish static TF
  ros::Timer timer_;
  ros::Time last_callback_time_;
  std::map<std::string, ros::Time> last_publish_time_;
  MimicMap mimic_;
  
};

// Constructor implementation
DynamicJointStateListener::DynamicJointStateListener(const KDL::Tree& tree, const MimicMap& m, ros::NodeHandle& node):
  state_publisher_(tree, node),
  mimic_(m)
{
  // Nodehandles
  ros::NodeHandle n_tilde("~");

  // Set publish frequency
  double publish_freq;
  n_tilde.param("publish_frequency", publish_freq, 50.0);
  
  // Get the tf_prefix parameter from the closest namespace
  std::string tf_prefix_key;
  n_tilde.searchParam("tf_prefix", tf_prefix_key);
  n_tilde.param(tf_prefix_key, tf_prefix_, std::string(""));
  
  publish_interval_ = ros::Duration(1.0/std::max(publish_freq,1.0));

  // Subscribe to joint state
  joint_state_sub_ = node.subscribe("joint_states", 1, &DynamicJointStateListener::callbackJointState, this);

  // trigger to publish fixed joints
  timer_ = n_tilde.createTimer(publish_interval_, &DynamicJointStateListener::callbackFixedJoint, this);
};

// Destructor
DynamicJointStateListener::~DynamicJointStateListener(){};

// Timer callback for fixed joints
void DynamicJointStateListener::callbackFixedJoint(const ros::TimerEvent& e)
{
  state_publisher_.publishFixedTransforms(tf_prefix_);
}

// Joint states callback
void DynamicJointStateListener::callbackJointState(const sensor_msgs::JointStateConstPtr& state)
{
  if (state->name.size() != state->position.size()){
    ROS_ERROR("Robot state publisher received an invalid joint state vector");
    return;
  }

  // check if we moved backwards in time (e.g. when playing a bag file)
  ros::Time now = ros::Time::now();
  if(last_callback_time_ > now) {
    // force re-publish of joint transforms
    ROS_WARN("Moved backwards in time (probably because ROS clock was reset), re-publishing joint transforms!");
    last_publish_time_.clear();
  }
  last_callback_time_ = now;

  // determine least recently published joint
  ros::Time last_published = now;
  for (unsigned int i=0; i<state->name.size(); i++)
  {
    ros::Time t = last_publish_time_[state->name[i]];
    last_published = (t < last_published) ? t : last_published;
  }
  // note: if a joint was seen for the first time,
  //       then last_published is zero.


  // check if we need to publish
  if (state->header.stamp >= last_published + publish_interval_){
    // get joint positions from state message
    map<string, double> joint_positions;
    for (unsigned int i=0; i<state->name.size(); i++)
      joint_positions.insert(make_pair(state->name[i], state->position[i]));

    for(MimicMap::iterator i = mimic_.begin(); i != mimic_.end(); i++){
      if(joint_positions.find(i->second->joint_name) != joint_positions.end()){
        double pos = joint_positions[i->second->joint_name] * i->second->multiplier + i->second->offset;
        joint_positions.insert(make_pair(i->first, pos));
      }
    }

    state_publisher_.publishTransforms(joint_positions, state->header.stamp, tf_prefix_);

    // store publish time in joint map
    for (unsigned int i=0; i<state->name.size(); i++)
      last_publish_time_[state->name[i]] = state->header.stamp;
  }
}


} // namespace uchile_tf

int main(int argc, char** argv) {
  
  // Init ROS
	ros::init(argc, argv, "dynamic_robot_state_publisher");
  ros::NodeHandle node;
  
  // Get the location of the robot description on the parameter server
  urdf::Model model;
  model.initParam("robot_description");
  // Use KDL for parse URDF
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)){
    ROS_ERROR("Failed to extract KDL tree from URDF robot description");
    ROS_BREAK();
  }
  
  // Mimic joints
  MimicMap mimic;
  for(std::map< std::string, boost::shared_ptr< urdf::Joint > >::iterator i = model.joints_.begin(); i != model.joints_.end(); i++){
    // Check if current joint(i) is mimic type
    if(i->second->mimic){
      // Add to mimic map
      mimic.insert(make_pair(i->first, i->second->mimic));
    }
  }
  
  // Wait for calibration parameters
  double param_timeout = 5.0;
  bool param_founded = false;
  node.param<double>("calibration_parameter_timeout", param_timeout, 5.0);
  ROS_INFO("Waiting for calibration parameters");
  ros::Time init_time = ros::Time::now();
  while(ros::Time::now() - init_time < ros::Duration(param_timeout))
  {
    if (node.hasParam("/bender/dynamic_tf/"))
    {
      ROS_INFO("Calibration parameters found.");
      param_founded = true;
      break;
    }
    ros::Duration(0.1).sleep();
  }
  // Print warning if calibration parameters were not found
  if (!param_founded)
  {
    ROS_WARN("Calibration parameters not found in parameter server, using default values.");
  }

  uchile_tf::DynamicJointStateListener state_publisher(tree, mimic, node);

  ros::spin();

  return 0;
}
