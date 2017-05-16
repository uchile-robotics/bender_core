/*
 * Rodrigo Munoz
 * Mayo 2015
 */

#include <uchile_tf/dynamic_tf.h>
 
namespace uchile_tf {
  
  // Constructor   
  DynamicTf::DynamicTf(const std::string& name, const std::string& name_space):
    name_("dynamic_tf_" + name),
    nh_(name_space + "/dynamic_tf/" + name),
    dynamic_reconfig_initialized_(false),
    t_(0,0,0),
    r_(0),p_(0),y_(0)
  {
    ROS_DEBUG_NAMED(name_,"Initializing dynamic reconfigure in namespace %s", nh_.getNamespace().c_str());

    // Start dynamic reconfigure server
    param_reconfig_server_.reset(new DynamicReconfigServer(param_reconfig_mutex_, nh_));
    dynamic_reconfig_initialized_ = true;

    // Set callback
    param_reconfig_callback_ = boost::bind(&DynamicTf::dynamicReconfigCallback, this, _1, _2);
    param_reconfig_server_->setCallback(param_reconfig_callback_);
  }

  // Destructor
  DynamicTf::~DynamicTf(){
  }
  
  // Dynamic reconfigure callback
  void DynamicTf::dynamicReconfigCallback(uchile_tf::tfParametersConfig& config, uint32_t level){
    // Update translation
    t_ = KDL::Vector(config.x, config.y, config.z);
    // Update rotation
    r_ = config.roll; p_ = config.pitch; y_ = config.yaw;

    ROS_DEBUG_NAMED(name_,"Update values to [%.2f,%.2f,%.2f] [%.2f,%.2f,%.2f]",
      t_.x(), t_.y(), t_.z(), r_, p_, y_);
  }



} // namespace uchile_tf