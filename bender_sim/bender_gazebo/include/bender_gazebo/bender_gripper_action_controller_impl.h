// \author Rodrigo Munoz

#ifndef BENDER_GAZEBO_BENDER_GRIPPER_ACTION_CONTROLLER_IMPL_H
#define BENDER_GAZEBO_BENDER_GRIPPER_ACTION_CONTROLLER_IMPL_H

namespace bender_gazebo
{

namespace util
{
std::string getLeafNamespace(const ros::NodeHandle& nh)
{
  const std::string complete_ns = nh.getNamespace();
  std::size_t id   = complete_ns.find_last_of("/");
  return complete_ns.substr(id + 1);
}  

std::shared_ptr<urdf::Model> getUrdf(const ros::NodeHandle& nh, const std::string& param_name)
{
  std::shared_ptr<urdf::Model> urdf(new urdf::Model);

  std::string urdf_str;
  // Check for robot_description in proper namespace
  if (nh.getParam(param_name, urdf_str))
  {
    if (!urdf->initString(urdf_str))
    {
      ROS_ERROR_STREAM("Failed to parse URDF contained in '" << param_name << "' parameter (namespace: " <<
        nh.getNamespace() << ").");
      return std::shared_ptr<urdf::Model>();
    }
  }
  // Check for robot_description in root
  else if (!urdf->initParam("robot_description"))
  {
    ROS_ERROR_STREAM("Failed to parse URDF contained in '" << param_name << "' parameter");
    return std::shared_ptr<urdf::Model>();
  }
  return urdf;
}

typedef std::shared_ptr<const urdf::Joint> UrdfJointConstPtr;
std::vector<UrdfJointConstPtr> getUrdfJoints(const urdf::Model& urdf, const std::vector<std::string>& joint_names)
{
  std::vector<UrdfJointConstPtr> out;
  for (unsigned int i = 0; i < joint_names.size(); ++i)
  {
    UrdfJointConstPtr urdf_joint = urdf.getJoint(joint_names[i]);
    if (urdf_joint)
    {
      out.push_back(urdf_joint);
    }
    else
    {
      ROS_ERROR_STREAM("Could not find joint '" << joint_names[i] << "' in URDF model.");
      return std::vector<UrdfJointConstPtr>();
    }
  }
  return out;
}

} // util namespace

// Implementation

inline void MultiGripperActionController::
starting(const ros::Time& time)
{
  ROS_DEBUG_NAMED(name_, "Controller starting.");
  for(unsigned int i = 0; i < num_joints_; i++){
    // Joint position
    command_struct_rt_[i].position_ = joints_[i].getPosition();
    command_struct_rt_[i].max_effort_ = default_max_effort_;
  }
  // Init RT
  commands_.initRT(command_struct_rt_);
  // Hardware interface adapter
  hw_iface_adapter_.starting(ros::Time(0.0));
  last_movement_time_ = time;
}

inline void MultiGripperActionController::
stopping(const ros::Time& time)
{
  preemptActiveGoal();
}

inline void MultiGripperActionController::
preemptActiveGoal()
{
  RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);
   
  // Cancels the currently active goal
  if (current_active_goal)
  {
    // Marks the current goal as canceled
    rt_active_goal_.reset();
    if(current_active_goal->gh_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      current_active_goal->gh_.setCanceled();
  }
}

// Constructor
MultiGripperActionController::
MultiGripperActionController()
: verbose_(false) // Set to true during debugging
{}

bool MultiGripperActionController::init(hardware_interface::EffortJointInterface* hw,
                  ros::NodeHandle&   root_nh,
                  ros::NodeHandle&   controller_nh)
{  
  using namespace util;
  
  // Cache controller node handle
  controller_nh_ = controller_nh;
  
  // Controller name
  name_ = getLeafNamespace(controller_nh_);
  ROS_DEBUG_STREAM_NAMED(name_, "Init MultiGripperActionController with '" <<
  this->getHardwareInterfaceType() << "'.");
  
  // Action status checking update rate
  double action_monitor_rate = 20.0;
  controller_nh_.getParam("action_monitor_rate", action_monitor_rate);
  action_monitor_period_ = ros::Duration(1.0 / action_monitor_rate);
  ROS_DEBUG_STREAM_NAMED(name_, "Action status changes will be monitored at " << action_monitor_rate << "Hz.");
  
  // Get joints names
  if (!controller_nh_.getParam("joints", joint_names_)){
    ROS_ERROR_NAMED(name_, "Could not find joint names on param server");
    return false;
  }
  ROS_DEBUG_NAMED(name_, "Joint names: %s.", boost::algorithm::join(joint_names_, ", ").c_str());
  // Get joint num
  num_joints_ = joint_names_.size();

  // Get URDF
  std::shared_ptr<urdf::Model> urdf = getUrdf(root_nh, "robot_description");
  if (!urdf) 
  {
    ROS_ERROR_NAMED(name_, "Error getting URDF");
    return false;
  }
  
  // Get URDF joints
  std::vector<UrdfJointConstPtr> urdf_joints = getUrdfJoints(*urdf, joint_names_);
  if (urdf_joints.empty()) 
  {
    ROS_ERROR_NAMED(name_, "Error getting URDF joints");
    return false;
  }
  
  // Initialize members
  // Joint handle
  joints_.resize(num_joints_);
  for(unsigned int i = 0; i < num_joints_; i++) {
    try {
      ROS_DEBUG_NAMED(name_, "Getting %s HW", joint_names_[i].c_str());
      joints_[i] = hw->getHandle(joint_names_[i]);
    }
    catch (...) {
      ROS_ERROR_STREAM_NAMED(name_, "Could not find joint '" << joint_names_[i] << "' in '" <<
           this->getHardwareInterfaceType() << "'.");
      return false;
    }
  }

  // Default tolerances
  controller_nh_.param<double>("goal_tolerance", goal_tolerance_, 0.05);
  goal_tolerance_ = std::abs(goal_tolerance_);
  ROS_DEBUG_NAMED(name_, "Goal tolerance setting at %.3f", goal_tolerance_);
  // Max allowable effort 
  controller_nh_.param<double>("max_effort", default_max_effort_, 0.0);
  default_max_effort_ = std::abs(default_max_effort_);
  ROS_DEBUG_NAMED(name_, "Max allowable effort setting at %.3f", default_max_effort_);
  // Stall - stall velocity threshold, stall timeout
  controller_nh_.param<double>("stall_velocity_threshold", stall_velocity_threshold_, 0.05);
  controller_nh_.param<double>("stall_timeout", stall_timeout_, 0.5);
  ROS_DEBUG_NAMED(name_, "Stall velocity threshold %.3f, stall timeout %.03f",
    stall_velocity_threshold_, stall_timeout_);
  
  // Hardware interface adapter
  ROS_DEBUG_NAMED(name_, "Init Hardware Interface Adapter.");
  if(!hw_iface_adapter_.init(joints_, controller_nh_))
    ROS_ERROR_NAMED(name_, "Error init Hardware Interface Adapter.");

  // Init RT buffers
  ROS_DEBUG_NAMED(name_, "Init RT commands and buffers.");
  // RT Buffer with zero elements
  commands_.writeFromNonRT(std::vector<Commands>(num_joints_, Commands()));
  // Commands with zero elements
  command_struct_ = std::vector<Commands>(num_joints_, Commands());
  command_struct_rt_ = std::vector<Commands>(num_joints_, Commands());

  // Command - non RT version
  ROS_DEBUG_NAMED(name_, "Init non RT commands.");
  for(unsigned int i = 0; i < num_joints_; i++){
    command_struct_[i].position_ = joints_[i].getPosition();
    command_struct_[i].max_effort_ = default_max_effort_;
  }

  // Result
  pre_alloc_result_.reset(new control_msgs::GripperCommandResult());
  pre_alloc_result_->position = command_struct_[0].position_;
  pre_alloc_result_->reached_goal = false;
  pre_alloc_result_->stalled = false;

  // ROS API: Action interface
  ROS_DEBUG_NAMED(name_, "Init ActionServer.");
  action_server_.reset(new ActionServer(controller_nh_, "gripper_action",
          boost::bind(&MultiGripperActionController::goalCB,   this, _1),
          boost::bind(&MultiGripperActionController::cancelCB, this, _1),
          false));
  action_server_->start();    
  return true;
}

void MultiGripperActionController::
update(const ros::Time& time, const ros::Duration& period)
{
  // Read buffer
  command_struct_rt_ = *(commands_.readFromRT());

  double max_effort_ = command_struct_rt_[0].max_effort_*3.1; // TODO Fix max effort for gripper skill

  std::vector<double> error_position(num_joints_), error_velocity(num_joints_),
    current_position(num_joints_), current_velocity(num_joints_), desired_position(num_joints_);
  for (unsigned int i = 0; i < num_joints_  ; ++i)
  {
    desired_position[i] = command_struct_rt_[i].position_;
    current_position[i] = joints_[i].getPosition();
    current_velocity[i] = joints_[i].getVelocity();
    error_position[i] =  desired_position[i] - current_position[i];
    error_velocity[i] = - current_velocity[i]; // Zero reference speed
  }
  // Generate and send commands to hardware interface adapter
  computed_command_ = hw_iface_adapter_.updateCommand(period, error_position, error_velocity, max_effort_);
  
  /* Condition checking, this implementation is based on PR2 Gripper controller.
   * See: https://github.com/PR2/pr2_controllers/blob/pr2_controllers-1.9.3/pr2_gripper_action/src/pr2_gripper_action.cpp#L199
   */
  // Null active goal
  if(!rt_active_goal_)
  {
    return;
  }
  if(rt_active_goal_->gh_.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE)
    return;

  // Check goal tolerance for joints
  bool reach_goal_tolerance = true;
  for(unsigned int i = 0; i < num_joints_ ; ++i)
  {
    if (std::abs(error_position[i]) > goal_tolerance_)
    {
      reach_goal_tolerance = false;
      break;
    }
  }
  // Mean position of gripper joints
  double mean_position = 0;
  for(unsigned int i = 0; i < num_joints_ ; ++i)
  {
    mean_position += current_position[i];
  }
  mean_position /= num_joints_;
  // Set result
  if(reach_goal_tolerance)
  {
    pre_alloc_result_->effort = computed_command_;
    pre_alloc_result_->position = mean_position;
    pre_alloc_result_->reached_goal = true;
    pre_alloc_result_->stalled = false;
    rt_active_goal_->setSucceeded(pre_alloc_result_);
  }
  else
  {
    // Check stall velocity for joint
    bool reach_stall_velocity = false;
    for(unsigned int i = 0; i < num_joints_ ; ++i)
    {
      if (std::abs(current_velocity[i]) < stall_velocity_threshold_ && computed_command_ > 0.95*max_effort_)
      {
        reach_stall_velocity = true;
        break;
      }
    }
    // Update movement time time
    if (!reach_stall_velocity)
    {
      last_movement_time_ = time;
    }
    // Check stall timeout
    else if( (time - last_movement_time_).toSec() > stall_timeout_ && max_effort_ != 0.0)
    {
      pre_alloc_result_->effort = computed_command_;
      pre_alloc_result_->position = mean_position;
      pre_alloc_result_->reached_goal = false;
      pre_alloc_result_->stalled = true;
      rt_active_goal_->setAborted(pre_alloc_result_);
    }
  }
}


void MultiGripperActionController::
goalCB(GoalHandle gh)
{
  ROS_DEBUG_NAMED(name_, "[%s] Recieved new action goal (effort=%.2f)", name_.c_str(), gh.getGoal()->command.max_effort);
  
  // Precondition: Running controller
  if (!this->isRunning())
  {
    ROS_ERROR_NAMED(name_, "Can't accept new action goals. Controller is not running.");
    control_msgs::GripperCommandResult result;
    gh.setRejected(result);
    return;
  }

  // Try to update goal
  RealtimeGoalHandlePtr rt_goal(new RealtimeGoalHandle(gh));

  // Accept new goal
  preemptActiveGoal();
  gh.setAccepted();

  // This is the non-realtime command_struct
  for (unsigned int i = 0; i < num_joints_; ++i)
  {
    command_struct_[i].position_ = gh.getGoal()->command.position;
    command_struct_[i].max_effort_ = gh.getGoal()->command.max_effort;
  }
  // Save from non RT commands
  commands_.writeFromNonRT(command_struct_);

  pre_alloc_result_->reached_goal = false;
  pre_alloc_result_->stalled = false;

  last_movement_time_ = ros::Time::now();
    
  // Setup goal status checking timer
  goal_handle_timer_ = controller_nh_.createTimer(action_monitor_period_,
              &RealtimeGoalHandle::runNonRealtime,
              rt_goal);
  goal_handle_timer_.start();
  rt_active_goal_ = rt_goal;
}

void MultiGripperActionController::
cancelCB(GoalHandle gh)
{
  RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);
  
  // Check that cancel request refers to currently active goal (if any)
  if (current_active_goal && current_active_goal->gh_ == gh)
  {
    // Reset current goal
    rt_active_goal_.reset();
    
    // Enter hold current position mode
    setHoldPosition(ros::Time(0.0));
    ROS_INFO_NAMED(name_, "Canceling active action goal because cancel callback recieved from actionlib.");
    
    // Mark the current goal as canceled
    current_active_goal->gh_.setCanceled();
  }
}

void MultiGripperActionController::
setHoldPosition(const ros::Time& time)
{
  for (unsigned int i = 0; i < num_joints_; ++i)
  {
    command_struct_[i].position_ = joints_[i].getPosition();
    command_struct_[i].max_effort_ = default_max_effort_;
  }
  commands_.writeFromNonRT(command_struct_);
}

} // namespace

#endif // header guard
