/// \author Rodrigo Munoz

#ifndef BENDER_GRIPPER_CONTROLLER_EFFORT_HARDWARE_INTERFACE_ADAPTER_H
#define BENDER_GRIPPER_CONTROLLER_EFFORT_HARDWARE_INTERFACE_ADAPTER_H

#include <cassert>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <ros/node_handle.h>
#include <ros/time.h>

#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>

/**
 * \brief
 *
 * Helper class to simplify integrating the GripperActionController effort-controlled hardware
 * interface. Maps position and velocity errors to effort commands through a position PID loop.
 *
 * The following is an example configuration of a controller that uses this adapter. Notice the \p gains entry:
 * \code
 * gripper_controller:
 *   type: "gripper_action_controller/GripperActionController"
 *   joints: gripper_joint
 *   goal_tolerance: 0.01
 *   stalled_velocity_threshold: 0.01
 *   stall_timeout: 0.2
 *   gains:
 *     gripper_joint: {p: 200, d: 1, i: 5, i_clamp: 1}
 * \endcode
 */
class HardwareInterfaceAdapter
{
public:
  HardwareInterfaceAdapter() : joint_handles_ptr_(0), num_joints_(0) {}

  bool init(std::vector<hardware_interface::JointHandle>& joint_handles, ros::NodeHandle& controller_nh)
  {
    // Store pointer to joint handles
    joint_handles_ptr_ = &joint_handles;
    num_joints_ = joint_handles_ptr_->size();
    // Initialize PIDs
    pids_.resize(num_joints_);
    for (unsigned int i = 0; i < num_joints_; ++i)
    {
      ROS_DEBUG_STREAM("Joint " << (*joint_handles_ptr_)[i].getName());
      // Node handle to PID gains
      ros::NodeHandle joint_nh(controller_nh, std::string("gains/") + (*joint_handles_ptr_)[i].getName());

      // Init PID gains from ROS parameter server
      pids_[i].reset(new control_toolbox::Pid());
      if (!pids_[i]->init(joint_nh))
      {
        ROS_WARN_STREAM("Failed to initialize PID gains from ROS parameter server.");
        return false;
      }
    }
    return true;
  }

  void starting(const ros::Time& time)
  {
    // Reset PIDs, zero effort commands
    for (unsigned int i = 0; i < num_joints_; ++i)
    {
      pids_[i]->reset();
      (*joint_handles_ptr_)[i].setCommand(0.0);
    }
    
  }

  void stopping(const ros::Time& time) {}

  double updateCommand(const ros::Duration&       period,
                       const std::vector<double>& error_position,
                       const std::vector<double>& error_velocity,
                       double                     max_allowed_effort)
  {
    // Preconditions
    if (!joint_handles_ptr_) {return 0.0;}
    
    // Check large
    assert(num_joints_ == error_position.size());
    assert(num_joints_ == error_velocity.size());

    double mean_effort = 0.0;
    // Update PIDs
    for (unsigned int i = 0; i < num_joints_; ++i)
    {
      // Compute PID Command
      double command = pids_[i]->computeCommand(error_position[i], error_velocity[i], period);
      command = std::min<double>(fabs(max_allowed_effort), std::max<double>(-fabs(max_allowed_effort), command));
      (*joint_handles_ptr_)[i].setCommand(command);
      mean_effort += command;
    }
    return mean_effort/num_joints_;
  }

private:
  typedef boost::shared_ptr<control_toolbox::Pid> PidPtr;
  std::vector<PidPtr> pids_;
  std::vector<hardware_interface::JointHandle>* joint_handles_ptr_;
  unsigned int num_joints_;
};

#endif // header guard