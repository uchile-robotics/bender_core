#ifndef BENDER_TF__DYNAMIC_TF_H
#define BENDER_TF__DYNAMIC_TF_H

#include <ros/ros.h>
// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <uchile_tf/tfParametersConfig.h>
#include <boost/thread/mutex.hpp>

// KDL
#include <kdl/frames.hpp>
#include <tf_conversions/tf_kdl.h>

namespace uchile_tf {
  
  class DynamicTf{
    public:
     
      DynamicTf(const std::string& name, const std::string& name_space);

      ~DynamicTf();
      
      void dynamicReconfigCallback(uchile_tf::tfParametersConfig &config, uint32_t level);

      inline void applyTransform(KDL::Frame& frame)
      {
        frame.p += t_; // Apply translation
        double r, p, y;
        frame.M.GetRPY(r,p,y); // Get RPY angles
        frame.M = KDL::Rotation::RPY(r+r_, p+p_, y+y_); // Apply rotation
      }
     
    private:
      const std::string name_;

      ros::NodeHandle nh_;
      // Dynamic reconfigure
      bool dynamic_reconfig_initialized_;
      typedef dynamic_reconfigure::Server<uchile_tf::tfParametersConfig> DynamicReconfigServer;                             
      boost::shared_ptr<DynamicReconfigServer> param_reconfig_server_;
      DynamicReconfigServer::CallbackType param_reconfig_callback_;

      boost::recursive_mutex param_reconfig_mutex_;
      
      KDL::Vector t_;     // Translation
      double r_, p_, y_;  // Rotation RPY

    
  };

  typedef boost::shared_ptr<uchile_tf::DynamicTf> DynamicTfPtr;
} // namespace uchile_tf
  
#endif