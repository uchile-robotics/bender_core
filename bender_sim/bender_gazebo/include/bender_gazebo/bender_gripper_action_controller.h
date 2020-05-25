/// \author Rodrigo Munoz

#ifndef BENDER_GAZEBO_BENDER_GRIPPER_ACTION_CONTROLLER_H
#define BENDER_GAZEBO_BENDER_GRIPPER_ACTION_CONTROLLER_H

// C++ standard
#include <cassert>
#include <iterator>
#include <stdexcept>
#include <string>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/algorithm/string/join.hpp>

// ROS
#include <ros/node_handle.h>
#include <log4cxx/logger.h>

// URDF
#include <urdf/model.h>

// ROS messages
#include <control_msgs/GripperCommandAction.h>

// actionlib
#include <actionlib/server/action_server.h>

// ros_controls
#include <controller_interface/controller.h>
#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_server_goal_handle.h>

// Project
#include <bender_gazebo/effort_hardware_interface_adapter.h>

namespace bender_gazebo
{

/**
 * \brief Controller for executing a gripper command action for simple single-dof grippers.
 *
 * \tparam HardwareInterface Controller hardware interface. Currently \p hardware_interface::PositionJointInterface and
 * \p hardware_interface::EffortJointInterface are supported out-of-the-box.
 */
class MultiGripperActionController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:

  /**
   * \brief Store position and max effort in struct to allow easier realtime buffer usage
   */
  struct Commands
  {
    double position_; // Last commanded position
    double max_effort_; // Max allowed effort
  };
  // @TODO
  struct State
  {
    double desired_position_;
    double current_position_;
    double current_velocity_;
    double error_position_;
    double error_velocity_;
  };
  
  MultiGripperActionController();
  
  /** \name Non Real-Time Safe Functions
   *\{*/
  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  /*\}*/
  
  /** \name Real-Time Safe Functions
   *\{*/
  /** \brief Holds the current position. */
  void starting(const ros::Time& time);

  /** \brief Cancels the active action goal, if any. */
  void stopping(const ros::Time& time);
  
  void update(const ros::Time& time, const ros::Duration& period);
  /*\}*/

  // RT Buffer
  realtime_tools::RealtimeBuffer<std::vector<Commands> > commands_;

  // pre-allocated memory that is re-used to set the realtime buffer
  std::vector<Commands> command_struct_, command_struct_rt_;

private:
  // Typedefs
  typedef actionlib::ActionServer<control_msgs::GripperCommandAction>                   ActionServer;
  typedef std::shared_ptr<ActionServer>                                               ActionServerPtr;
  typedef ActionServer::GoalHandle                                                      GoalHandle;
  typedef realtime_tools::RealtimeServerGoalHandle<control_msgs::GripperCommandAction>  RealtimeGoalHandle;
  typedef boost::shared_ptr<RealtimeGoalHandle>                                         RealtimeGoalHandlePtr;

  bool        update_hold_position_; 
  bool        verbose_;            // Hard coded verbose flag to help in debugging.
  std::string name_;               // Controller name.
  
  // Joint members
  std::vector<std::string>                     joint_names_; // Controlled joint names.
  std::vector<hardware_interface::JointHandle> joints_;      // Handles to controlled joints.
  unsigned int                                 num_joints_;  // Joints number                    

  HardwareInterfaceAdapter                     hw_iface_adapter_;   // Adapts desired goal state to HW interface.
  RealtimeGoalHandlePtr                        rt_active_goal_;     // Currently active action goal, if any.
  control_msgs::GripperCommandResultPtr        pre_alloc_result_;

  ros::Duration action_monitor_period_;

  // ROS API
  ros::NodeHandle    controller_nh_;
  ActionServerPtr    action_server_;

  ros::Timer         goal_handle_timer_;

  // Action server functions
  void goalCB(GoalHandle gh);
  void cancelCB(GoalHandle gh);
  void preemptActiveGoal();
  void setHoldPosition(const ros::Time& time);

  ros::Time last_movement_time_;  // Store stall time
  double computed_command_;       // Computed command

  // Action Parameters
  double stall_timeout_, stall_velocity_threshold_; // Stall related parameters
  double default_max_effort_;                       // Max allowed effort
  double goal_tolerance_;

};

} // namespace

#include <bender_gazebo/bender_gripper_action_controller_impl.h>

#endif // header guard
