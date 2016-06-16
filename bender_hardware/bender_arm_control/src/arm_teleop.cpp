/* 
* Permite controlar el brazo usando un joystick
* Autor: Rodrigo Munoz
*/

// Boost Thread
#include <boost/thread/mutex.hpp>
// ROS
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
// Moveit
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
// Msgs
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>

/* --------------------------------------------------------------------------------
* Clase para el manejo de cinematica inversa usando interfaz de MoveIt!
*/

class ArmIKWrapper {

public:
  ArmIKWrapper(const std::string& arm_name, robot_model::RobotModelPtr robot_model, double timeout = 0.01);
  bool getIK(const geometry_msgs::Pose& goal_pose, std::vector<double>& joint_values);
  void sendGoal(const geometry_msgs::Pose& goal_pose);

private:
  std::string arm_name_;
  // Modelo de robot
  robot_model::RobotModelPtr robot_model_;
  robot_state::RobotStatePtr kinematic_state_;

  const robot_state::JointModelGroup* arm_jmg_;

  // IK timeout
  double timeout_;
};

ArmIKWrapper::ArmIKWrapper(const std::string& arm_name, robot_model::RobotModelPtr robot_model, double timeout):
arm_name_(arm_name), robot_model_(robot_model),
kinematic_state_(new robot_state::RobotState(robot_model_)),
arm_jmg_(robot_model_->getJointModelGroup(arm_name_)),
timeout_(timeout)
{
  kinematic_state_->setToDefaultValues();
  // Check puntero nulo
  if (!arm_jmg_)
  {
    std::string error_msg = "Joint model group '" + arm_name_ + "' not found.";
    ROS_ERROR("%s", error_msg.c_str());
    throw ros::InvalidNameException(error_msg);
  }
}

bool ArmIKWrapper::getIK(const geometry_msgs::Pose& goal_pose, std::vector<double>& joint_values)
{
  bool found_ik = kinematic_state_->setFromIK(arm_jmg_, goal_pose, 5, timeout_);
  if (found_ik) {
      ROS_INFO("IK!");
      // Actualizar joint_values
      //kinematic_state_->copyJointGroupPositions(arm_jmg_, joint_values);
      return true;
  }
  else
  {
    return false;
  }
}

void ArmIKWrapper::sendGoal(const geometry_msgs::Pose& goal_pose)
{
  ;
}

typedef boost::shared_ptr<ArmIKWrapper> ArmIKWrapperPtr;

/* --------------------------------------------------------------------------------
* Clase que envia commandos a cada brazo
*/

static const std::string JOY_TOPIC = "joy"; // Topico joystick
static const std::string ROBOT_MODEL = "robot_description"; // Parametro

// Ejes
static const std::size_t X = 1, Y = 0, Z_P = 5, Z_N = 2;
// Botones
static const std::size_t PITCH_P = 14, PITCH_N = 13, YAW_P = 11, YAW_N = 12;

class ArmTeleop {

public:
  ArmTeleop(const ros::NodeHandle& nh);
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void goalPoseCallback(const ros::TimerEvent& event);

private:
  // Map {"nombre_brazo": brazo}
  std::vector<ArmIKWrapperPtr> arms_;
  std::vector<std::string> arm_names_;
  std::vector<double> joint_values_;
  
  ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;
  ros::Publisher goal_pose_pub_;

  // Goal pose
  geometry_msgs::PoseStamped goal_pose_;
  double goal_pitch_, goal_yaw_;
  std::string ref_frame_;

  // Timer y msg de ultimo comando enviado
  ros::Timer gp_timer_;
  sensor_msgs::Joy current_command_;
  boost::mutex gp_lock_;

  // Brazo seleccionado
  std::size_t selected_arm_, arm_num_;
};

ArmTeleop::ArmTeleop(const ros::NodeHandle& nh):
nh_(nh)
{
  // Nodehande privado para parametros
  ros::NodeHandle np("~");
  ROS_INFO("Init ArmTeleop with namespace '%s'", np.getNamespace().c_str());

  // Obtener Brazos
  if (!np.hasParam("arm_name"))
  {
    ROS_WARN("Using default param arm_name=['l_arm','r_arm']");
    arm_names_.push_back("l_arm");
    arm_names_.push_back("r_arm");
  }
  else
  {
    np.getParam("arm_name", arm_names_);  
  }
  // Brazo seleccionado
  selected_arm_ = 0;
  arm_num_ = arm_names_.size();

  // Parametro rate
  int rate;
  np.param("rate", rate, 10);
  ROS_INFO("Goal pose at: %d Hz", rate);

  // Suscriber para joystick
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &ArmTeleop::joyCallback, this);
  // Mensaje vacio
  current_command_.axes.resize(6,0.0);
  current_command_.axes[Z_N]=1.0;
  current_command_.axes[Z_P]=1.0;
  current_command_.buttons.resize(15,0);

  // Publisher para goal pose
  goal_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("goal_pose", 1);

  // Cargar modelo de robot
  robot_model_loader::RobotModelLoader robot_model_loader(ROBOT_MODEL);
  robot_model_loader.loadKinematicsSolvers();
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  if (!robot_model)
  {
    std::string error_msg = "Robot model parameter '" + ROBOT_MODEL + "' not found.";
    ROS_ERROR("%s", error_msg.c_str());
    throw ros::InvalidNameException(error_msg);
  }
  // Frame base por defecto
  ref_frame_ = robot_model->getModelFrame();
  ROS_INFO("Reference frame: %s", ref_frame_.c_str());
  goal_pose_.header.frame_id = ref_frame_;
  goal_pitch_ = 0; goal_yaw_ = 0;

  // Crear map de brazos
  for(std::size_t i = 0; i < arm_num_; ++i)
  {
    // Anadir elementos
    ArmIKWrapperPtr arm_ptr;
    try
    {
      arm_ptr.reset(new ArmIKWrapper(arm_names_[i], robot_model));
      arms_.push_back(arm_ptr);
    }
    catch (ros::InvalidNameException& e)
    {
      ROS_WARN_STREAM("Error getting '" << arm_names_[i] << "': " << e.what());
    }
  }

 
  // Goal pose timer
  gp_timer_ = nh.createTimer(ros::Duration(1.0/rate), &ArmTeleop::goalPoseCallback, this);
}

void ArmTeleop::goalPoseCallback(const ros::TimerEvent& event)
{
  double dt = (event.current_real - event.last_real).toSec();
  static double scale = 0.2;
  {
    boost::mutex::scoped_lock lock(gp_lock_);
    // Update goal pose
    goal_pose_.pose.position.x += scale*dt*current_command_.axes[X];
    goal_pose_.pose.position.y += scale*dt*current_command_.axes[Y];
    goal_pose_.pose.position.z = goal_pose_.pose.position.z 
      + scale*dt*(current_command_.axes[Z_N]-1.0) - scale*dt*(current_command_.axes[Z_P]-1.0);
    // Angulo
    goal_pitch_ = goal_pitch_ + 0.1*dt*current_command_.buttons[PITCH_P]
      - 0.1*dt*current_command_.buttons[PITCH_N];
    goal_yaw_ = goal_yaw_ + 0.1*dt*current_command_.buttons[YAW_P]
      - 0.1*dt*current_command_.buttons[YAW_N];
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, goal_pitch_, goal_yaw_),
      goal_pose_.pose.orientation);
 
    //ROS_ERROR("%.2f | %.2f",current_command_.axes[Z_N],current_command_.axes[Z_P]);
    if (current_command_.buttons[0]==1)
    {
      selected_arm_ = (selected_arm_ + 1) % arm_num_;
      ROS_INFO("Selected arm: %s", arm_names_[selected_arm_].c_str());
    }
  }
  goal_pose_pub_.publish(goal_pose_);

  if(!arms_[selected_arm_])
  {
    ROS_ERROR("Interface for '%s' not initializated", arm_names_[selected_arm_].c_str());
  }
  else if(arms_[selected_arm_]->getIK(goal_pose_.pose, joint_values_))
  {
    ROS_ERROR("IK! REACHED");
    for (std::size_t i = 0; i < joint_values_.size(); ++i)
    {
      ;
    }
  }

}

void ArmTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(gp_lock_);
  current_command_ = *msg; // Copiar msg

  // Hardcoded para evitar movimientos al inicio @TODO
  if (current_command_.axes[Z_N] == 0.0 || current_command_.axes[Z_P] == 0.0)
  {
    current_command_.axes[Z_N]=1.0;
    current_command_.axes[Z_P]=1.0;
  }
}


int main(int argc, char **argv)
{
  // Iniciar nodo IK
  ros::init (argc, argv, "arm_teleop");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ArmTeleop arm_teleop(nh);

  ros::waitForShutdown();
  return 0;
}
