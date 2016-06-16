
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/common_planning_interface_objects/common_objects.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/trajectory_tools.h> // for plan_execution

// Grasp
#include <moveit_simple_grasps/simple_grasps.h>
#include <moveit_simple_grasps/grasp_data.h>
#include <moveit_simple_grasps/grasp_filter.h>

// Visual
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/MoveGroupGoal.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_msgs/RobotState.h>

#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <ros/ros.h>

#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PoseStamped.h>
// Servicios
#include <std_srvs/Empty.h>
#include <bender_arm_control/Position.h>
#include <bender_arm_control/PositionNamed.h>
#include <bender_arm_control/PositionServoing.h>
#include <bender_arm_control/EndEffectorPose.h>
#include <bender_arm_control/GripperOrientation.h>
#include <bender_arm_control/SetPlannerID.h>
#include <bender_arm_control/Grasp.h>
#include <bender_arm_control/AttachObject.h>
#include <bender_arm_control/JointTarget.h>


// Action Server
#include <bender_arm_control/GraspGeneratorAction.h>
#include <actionlib/server/simple_action_server.h>


namespace bender_arm_control {

using namespace moveit::planning_interface;

static const std::string JOINT_STATE_TOPIC = "/bender/joint_states";
static const std::string ROBOT_DESCRIPTION = "robot_description";
static const std::string MONITORED_PLANNING_SCENE_TOPIC = "/move_group/monitored_planning_scene";
static const std::string ATTACHED_COLLISION_OBJECT_TOPIC = "/attached_collision_object";
static const std::string COLLISION_OBJECT_TOPIC = "/collision_object";
static const std::string PLANNING_SCENE_TOPIC = "/planning_scene";
static const std::string PLANNING_SCENE_WORLD_TOPIC = "/planning_scene_world";

inline void poseEigenToMsgStamped(const Eigen::Affine3d &e, geometry_msgs::PoseStamped &m, const std::string &frame_id)
{
  tf::poseEigenToMsg(e,m.pose);
  m.header.frame_id = frame_id;
}

/**
 * @brief Transforma un geometry_msgs/PoseStamped al frame desired_frame usando TF
 * 
 * @param tf Transformer
 * @param desired_frame Frame de destino deseado
 * @param target Posicion que se desea transformar
 */
inline void transformPose(const tf::Transformer& tf, const std::string &desired_frame, geometry_msgs::PoseStamped &target)
{
  if (desired_frame != target.header.frame_id)
  {
    tf::Stamped<tf::Pose> stamped_target, stamped_target_out;
    tf::poseStampedMsgToTF(target, stamped_target);    
    tf.transformPose(desired_frame, stamped_target, stamped_target_out);
    tf::poseStampedTFToMsg(stamped_target_out, target);
  }
}

inline double distVector(const std::vector<double>& v1, const std::vector<double>& v2)
{
  if (v1.size() != v2.size())
  {
    ROS_ERROR("Vectors dont have the same size!");
    return 0.0;
  }
  double val = 0.0;
  for (std::size_t i=0; i< v1.size(); ++i)
  {
    val += std::abs((v1[i]-v2[i]));
  }
  return val;
}

inline void getTransformation(double x, double y, double z, double roll, double pitch, double yaw, Eigen::Affine3d &t)
{
  double A = cos (yaw),  B = sin (yaw),  C  = cos (pitch), D  = sin (pitch),
         E = cos (roll), F = sin (roll), DE = D*E,         DF = D*F;
  // Matriz de rotacion
  t (0, 0) = A*C;  t (0, 1) = A*DF - B*E;  t (0, 2) = B*F + A*DE;  t (0, 3) = x;
  t (1, 0) = B*C;  t (1, 1) = A*E + B*DF;  t (1, 2) = B*DE - A*F;  t (1, 3) = y;
  t (2, 0) = -D;   t (2, 1) = C*F;         t (2, 2) = C*E;         t (2, 3) = z;
  t (3, 0) = 0;    t (3, 1) = 0;           t (3, 2) = 0;           t (3, 3) = 1;
}
/*
Eigen::Quaternion<float> q;

Eigen::AngleAxis<float> aaZ(pcl::deg2rad(rz), Eigen::Vector3f::UnitZ());

Eigen::AngleAxis<float> aaY(pcl::deg2rad(ry), Eigen::Vector3f::UnitY());

Eigen::AngleAxis<float> aaX(pcl::deg2rad(rx), Eigen::Vector3f::UnitX());

q = aaZ * aaY * aaX;

*/

typedef struct range{
  double min;
  double max;
  int resolution;
  double delta;
  double val;

  range(double mn = -1.0, double mx = 1.0, int rs = 10):
    min(mn), max(mx), resolution(rs), delta((mx-mn)/rs), val(mn+delta/2) {}
  
  range(double wd = 1.0, int rs = 10):
    min(-wd), max(wd), resolution(rs), delta((2*wd)/rs), val(-wd+delta/2) {}
  
  double operator++(int)
  {
    double tmp(val);
    val += delta;
    return tmp;
  }
  inline void reset()
  {
    val=min+delta/2;
  }
} Range;

struct OrderGraspQuality
{
  OrderGraspQuality(const std::vector<moveit_msgs::Grasp> &grasps) : grasps_(grasps)
  {
  }

  bool operator()(const std::size_t a, const std::size_t b) const
  {
    return grasps_[a].grasp_quality > grasps_[b].grasp_quality;
  }

  const std::vector<moveit_msgs::Grasp> &grasps_;
};


class BenderMoveGroupInterface {
  private:
    // Interfaces para move_group
    boost::scoped_ptr<MoveGroup> arm_;

    // Track planning scene
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

    // class for publishing stuff to rviz
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

    // Grasp filter
    moveit_simple_grasps::GraspFilterPtr grasp_filter_;

    // TF Listener
    boost::shared_ptr<tf::Transformer> tf_;

    // Plan
    MoveGroup::Plan plan_;

    // Robot model
    robot_model::RobotModelConstPtr robot_model_;

    // Solver
    kinematics::KinematicsBaseConstPtr kin_solver_;
    double ik_timeout_;
    // Transformada entre ik_frame y ref_frame
    Eigen::Affine3d ik_transform_;

    // Display de trayectorias
    ros::Publisher display_trajectory_pub_;
    moveit_msgs::DisplayTrajectory display_trajectory_;

    // ROS Nodehandle
    ros::NodeHandle nh_;

    // Logger
    const std::string name_;

    // Grasp generator
    moveit_simple_grasps::SimpleGraspsPtr simple_grasps_;
    // robot-specific data for generating grasps
    moveit_simple_grasps::GraspData grasp_data_;
    
  public:
    // Frames de referencia
    std::string ref_frame_;
    // Frames de efectores
    std::string ee_frame_;
    // Nombre brazo y gripper
    const std::string arm_name_;
    const std::string gripper_name_;
    // Nombres de joints
    std::vector<std::string> joint_names_;
  
  public:
    BenderMoveGroupInterface(const std::string& arm, const ros::NodeHandle& node_handle,const planning_scene_monitor::PlanningSceneMonitorPtr& psm):
      nh_(node_handle),
      arm_name_(arm),
      gripper_name_(arm.substr(0,1)+"_gripper"),
      name_("move_group_" + arm),
      planning_scene_monitor_(psm)
    {
      // TF
      tf_ = getSharedTF();      

      // Obtener modelo de robot
      robot_model_ = planning_scene_monitor_->getRobotModel();
      // Opciones de move_group, URDF y grupo
      MoveGroup::Options opts(arm, ROBOT_DESCRIPTION);
      opts.robot_model_ = robot_model_;
      // Cargar interfaz move_group
      arm_.reset(new move_group_interface::MoveGroup(opts));

      // Obtener frames de referencia
      ref_frame_ = arm_->getPlanningFrame();
      ROS_INFO_NAMED(name_, "Planning reference frame for %s: %s", arm_name_.c_str(), ref_frame_.c_str());
      // Obtener frames de referencia para efectores
      ee_frame_ = arm_->getEndEffectorLink();
      ROS_INFO_NAMED(name_, "End effector reference frame for %s: %s", arm_name_.c_str(), ee_frame_.c_str());

      // Cargar grasp data
      ros::NodeHandle nh_p("~");
      ROS_INFO_NAMED(name_, "Loading grasp data for %s in namespace %s", gripper_name_.c_str(), nh_p.getNamespace().c_str());
      bool load_grasp = grasp_data_.loadRobotGraspData(nh_p, gripper_name_);
      if (!load_grasp)
      {
        ROS_ERROR("Grasp data not loaded for %s", gripper_name_.c_str());
        ROS_BREAK();
      }
      
      // Cargar visual tools
      ROS_INFO_NAMED(name_, "MoveIt! markers on topic %smarkers", nh_.getNamespace().c_str());
      visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(ref_frame_, nh_.getNamespace()+"markers", planning_scene_monitor_));
      // ERROR API AL COMPILAR
      //visual_tools_->setMuted(false);
      ROS_INFO_STREAM("EE "<< grasp_data_.ee_group_ << " ARM NAME " << arm_name_);

      // Load end effector marker
      visual_tools_->loadEEMarker(robot_model_->getJointModelGroup(grasp_data_.ee_group_));

      // Generador de grasp
      simple_grasps_.reset( new moveit_simple_grasps::SimpleGrasps(visual_tools_) );
      robot_state::RobotState robot_state = planning_scene_monitor_->getPlanningScene()->getCurrentState();
      grasp_filter_.reset(new moveit_simple_grasps::GraspFilter(robot_state, visual_tools_) );

      // Solver de IK
      const robot_model::JointModelGroup* jmg = robot_model_->getJointModelGroup(arm_name_);
      kin_solver_ = jmg->getSolverInstance();
      const std::string &ik_frame = kin_solver_->getBaseFrame();
      ROS_INFO_NAMED(name_, "IK base frame: %s", ik_frame.c_str());
      joint_names_ = kin_solver_->getJointNames();
      // Transformada para IK solver
      if (!moveit::core::Transforms::sameFrame(ik_frame, robot_model_->getModelFrame()))
      {
        const robot_model::LinkModel *lm = robot_state.getLinkModel((!ik_frame.empty() && ik_frame[0] == '/') ? ik_frame.substr(1) : ik_frame);
        ROS_ASSERT_MSG(lm, "No transform for %s", ik_frame.c_str());
        //pose = getGlobalLinkTransform(lm).inverse() * pose;
        ik_transform_ = robot_state.getGlobalLinkTransform(lm).inverse();
        ROS_DEBUG_NAMED(name_,"IK Frame transform [%.2f, %.2f, %.2f]",ik_transform_(0,3), ik_transform_(1,3), ik_transform_(2,3));
      }

      // Solver IK timeout 
      ik_timeout_ = 0.05;
      ROS_INFO_NAMED(name_,"IK timeout %.2f", ik_timeout_);

      // Publisher para trayectorias
      display_trajectory_pub_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    }

    moveit_visual_tools::MoveItVisualToolsPtr getVisualTools()
    {
      return visual_tools_; 
    }

    const boost::shared_ptr<tf::Transformer>& getTF()
    {
      return tf_;
    }

    bool setCollisionObject(const moveit_msgs::AttachedCollisionObject& object)
    {
      bool res = false;
      {
        planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor_);
        ps->getCurrentStateNonConst().update();
        res = ps->processAttachedCollisionObjectMsg(object);
      }
      planning_scene_monitor_->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
      return res;
    }


    

    bool graspGenerator(const moveit_msgs::CollisionObject& object_target, std::vector<moveit_msgs::Grasp>& possible_grasps,
      std::vector<trajectory_msgs::JointTrajectoryPoint>& ik_solutions, std::vector<unsigned int>& grasp_order,
      unsigned int axial_resolution = 5, unsigned int angle_resolution = 8, bool verbose = false)
    {
      /* --------------------------------------------------------------------------------
      * Obtener geometrias
      * Obtener geometria cilindrica para generar grasps, se escoje la primera en el
      * arreglo object_target.primitives
      * @TODO: Manejo de distintas geometrias y combinacion de ellas
      *
      */

      if (object_target.primitives.empty())
      {
        ROS_ERROR_NAMED(name_, "No primitive elements for grasp generator with object %s", object_target.id.c_str());
        return false;
      }
      // Obtener geometrias cilindricas
      shape_msgs::SolidPrimitive target;
      geometry_msgs::Pose target_pose;
      double target_height, target_radius;
      std::string target_name;
      bool find_grasp = false;
      for (std::size_t i = 0; i < object_target.primitives.size(); ++i)
      {
        if (object_target.primitives[i].type == shape_msgs::SolidPrimitive::CYLINDER)
        {
          // Completar campos
          target = object_target.primitives[i];
          target_pose = object_target.primitive_poses[i];
          target_height = object_target.primitives[i].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT];
          target_radius = object_target.primitives[i].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
          target_name = object_target.id;
          find_grasp = true;
          break;
        }
      }
      if (!find_grasp)
      {
        ROS_ERROR_NAMED(name_, "Can not find cylinder component in object %s", object_target.id.c_str());
        return false;
      }
      // Publicar markers de objeto
      visual_tools_->setLifetime(200.0);      
      if (verbose)
      {
        visual_tools_->publishAxis(target_pose);
        visual_tools_->publishCylinder(target_pose, rviz_visual_tools::RED, target_height, target_radius);
      }

      /* --------------------------------------------------------------------------------
      * Parametros de grasping
      * @TODO: Usar dynamic reconfigure + YAML
      *
      */
      unsigned int height_resolution = axial_resolution; // Particiones lo largo del eje del cilindro
      grasp_data_.angle_resolution_ = angle_resolution; // Particiones de eje axial
      grasp_data_.approach_retreat_desired_dist_ = std::max(target_radius*2 + 0.08, 0.1); // Distancia de approach y retreat
      ROS_ERROR_STREAM("Distancia approach " << grasp_data_.approach_retreat_desired_dist_);
      grasp_data_.approach_retreat_min_dist_ = 0.1;

      // Generar grasps axiales
      double delta_height = target_height/height_resolution;
      target_pose.position.z -= target_height/2 + delta_height; // Base

      /* --------------------------------------------------------------------------------
      * Generar grasps
      * Genera distintas posiciones de grasp y pregrasp para objetos cilindricos
      * @TODO: Manejo de distintas orientaciones
      *
      */
      for (unsigned int j = 0; j < height_resolution; ++j)
      {
        target_pose.position.z += delta_height;
        // Generar grasps en eje z desde -M_PI/2 a M_PI/2
        simple_grasps_->generateAxisGrasps(target_pose, moveit_simple_grasps::Z_AXIS, moveit_simple_grasps::UP,
          moveit_simple_grasps::HALF, 0, grasp_data_, possible_grasps);
      }
      if (possible_grasps.empty())
      {
        ROS_ERROR_NAMED(name_, "No grasp generated!");
        return false;
      }
      ROS_INFO_STREAM_NAMED(name_, "Generated " << possible_grasps.size() << " grasps." );
      // Mostrar grasps
      if (false)
      {
        // ERROR API: comentado para compilar. revisar luego
        //visual_tools_->publishAnimatedGrasps(possible_grasps, grasp_data_.ee_parent_link_, 0.01);
      }
      
      /* --------------------------------------------------------------------------------
      * Filtrar usando IK
      * Cada posicion de grasp y pregrasp es filtrada comprobando su factibilidad usando 
      * cinematica inversa (IK). 
      * @TODO:
      * - Realizar chequeo de colisiones de estados de grasp y pregrasp
      * - Anadir bitset para saber cuales cumplen con IK en pregrasp
      *
      */
      bool filter_pregrasps = true;
      grasp_filter_->filterGrasps(possible_grasps, ik_solutions, filter_pregrasps, grasp_data_.ee_parent_link_, arm_name_);
      if (ik_solutions.empty())
      {
        ROS_ERROR_NAMED(name_, "No grasp pass the IK filter, grasp not reachable.");
        return false;
      }
      ROS_INFO_STREAM_NAMED(name_, "Found " << possible_grasps.size() << "possible grasp." );
      // Mostrar soluciones factibles
      if (verbose)
      {
        // ERROR API: comentado para compilar. revisar luego
        //visual_tools_->publishIKSolutions(ik_solutions, arm_name_, 0.05);
      }
      
      /* --------------------------------------------------------------------------------
      * Calidad de grasp
      * Se asigna la calidad de grasp de acuerdo los criterios:
      * @TODO:
      * - Alineacion
      * - Reducir calidad a soluciones con angulos de joint raros
      */
      /*
      // Obtener posicion actual
      robot_state::RobotState state = planning_scene_monitor_->getPlanningScene()->getCurrentState();
      state.updateLinkTransforms();
      // Cartesiana
      Eigen::Affine3d current_eff_pose = state.getGlobalLinkTransform(ee_frame_);
      Eigen::Vector3d current_eff_pose_vector = current_eff_pose.translation();
      // Joints
      std::vector<double> current_joint;
      const robot_model::JointModelGroup* jmg = robot_model_->getJointModelGroup(arm_name_);
      state.copyJointGroupPositions(jmg, current_joint);

      for (std::size_t i = 0; i < possible_grasps.size(); ++i)
      {
        // Distancia en espacio de joint wrt posicion actual
        double dist_joint = distVector(current_joint, ik_solutions[i].positions);
        // Actualizar quality
        possible_grasps[i].grasp_quality += dist_joint;
        ROS_DEBUG_STREAM_NAMED(name_,"Grasp " << i << " quality: " << possible_grasps[i].grasp_quality);
      }
      */
      /* --------------------------------------------------------------------------------
      * Ordenar por calidad de grasp
      */
      grasp_order.clear();
      grasp_order.resize(possible_grasps.size());
      for (std::size_t i = 0 ; i < possible_grasps.size(); ++i)
        grasp_order[i] = i;
      OrderGraspQuality oq(possible_grasps);
      std::sort(grasp_order.begin(), grasp_order.end(), oq);


      /* --------------------------------------------------------------------------------
      * Objetos de collision
      * Se anade como factible la colision entre los links del gripper y el objeto a tomar
      * @TODO: Anadir al gripper un objeto para forzar al planificador pasar mas lejos
      * 
      */
      std::vector<std::string> allowed_touch_objects;
      allowed_touch_objects.push_back(target_name);
      // Anadir a todos los grasp
      for (std::size_t i = 0; i < possible_grasps.size(); ++i)
      {
        possible_grasps[i].allowed_touch_objects = allowed_touch_objects;
      }
      return true;
    }



    void approx_ik(geometry_msgs::Pose& target)
    {
      ;
    }

    void setPlanner(const std::string &planner_id)
    {
      /* --------------------------------------------------------------------------------
      * Seleccionar planificador
      * @TODO:
      * - Parametros del planificador
      * - Eleccion basada en brenchmark
      * 
      */
      //arm_->setPlannerId("LBKPIECEkConfigDefault");
      arm_->setPlannerId(planner_id);
    }

    const std::string& getPoseReferenceFrame() const
    {
      return arm_->getPoseReferenceFrame();
    }

    bool loadPlanningSceneMonitor()
    {
      ROS_DEBUG_NAMED(name_,"Loading planning scene monitor");

      

      if (!planning_scene_monitor_->getPlanningScene())
      {
        ROS_ERROR_STREAM_NAMED(name_,"Planning scene not configured");
        return false;
      }

      return true;
    }

    std::string getArmName()
    {
      return arm_name_;
    }

    planning_scene_monitor::PlanningSceneMonitorPtr getPlanningSceneMonitor()
    {
      return planning_scene_monitor_;
    }

    /**
     * @brief Obtener la posición actual del efector.
     * 
     * @return Retorna la posición actual del efector (ee_frame_)(wrist_pitch_link) robot con respecto 
     * al frame de planificación (base_link).
     */
    geometry_msgs::PoseStamped getCurrentPose(){
      // Obtener estado del robot
      robot_state::RobotState state = planning_scene_monitor_->getPlanningScene()->getCurrentState();
      ROS_DEBUG_NAMED(name_,"Planning frame %s", robot_model_->getModelFrame().c_str());
      state.updateLinkTransforms();
      geometry_msgs::PoseStamped pos;
      // Usando base_link base del robot
      poseEigenToMsgStamped(state.getGlobalLinkTransform(ee_frame_), pos, robot_model_->getModelFrame());
      return pos;
    }

    /**
     * @brief Planificación en coordenadas cartesianas sin restriccion orientación con respecto al frame de planificación.
     *  
     * @param x Posición x
     * @param y Posición y
     * @param z Posición z
     * @return Retorna el código de error del planificador.
     */
    MoveItErrorCode plan(double x, double y, double z)
    {
      ROS_DEBUG_NAMED(name_, "Sending to position: [%.2f, %.2f, %.2f]", x, y, z);
      // Set current position
      arm_->setStartStateToCurrentState();
      arm_->setPlanningTime(10);
      // Set goal
      arm_->setPositionTarget( x, y, z);
      return arm_->plan(plan_);
    }

    /**
     * @brief Planificación en coordenadas cartesianas con restricción de orientación con frame arbitrario, el frame debe estar
     * en el arbol del TF.
     * 
     * @param target Posición objetivo.
     * @return Retorna el código de error del planificador
     */
    MoveItErrorCode plan(geometry_msgs::PoseStamped &target, bool approx_ik = false)
    {
      ROS_DEBUG_NAMED(name_, "Sending to position: [%.2f, %.2f, %.2f] frame_id: %s", target.pose.position.x, target.pose.position.y,
        target.pose.position.z, target.header.frame_id.c_str());
      // Set current position
      arm_->setStartStateToCurrentState();
      arm_->setPlanningTime(10);
      // Set goal
      if (!approx_ik)
      {
        arm_->setPoseTarget(target);  
      }
      else
      {
        ROS_INFO_NAMED(name_,"Using approx. IK.");
        if (arm_->setApproximateJointValueTarget(target))
        {
          return MoveItErrorCode(moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION);
        }
      }
      return arm_->plan(plan_);
    }

    /**
     * @brief Planificación en coordenadas cartesianas con restricción de orientación con respecto al frame de planificación.
     * 
     * @param target Posición objetivo con respecto al frame de planificación.
     * @return Retorna el código de error del planificador.
     */
    MoveItErrorCode plan(geometry_msgs::Pose &target)
    {
      geometry_msgs::PoseStamped target_stamped;
      target_stamped.pose = target;
      target_stamped.header.frame_id = robot_model_->getModelFrame();
      
      return plan(target_stamped);
    }

    /**
     * @brief Planificación en espacio de joints
     * 
     * @param target Posiciones de joints
     * @return Retorna el código de error del planificador.
     */
    MoveItErrorCode plan(const std::vector<double>& joint_values)
    {
      if(!arm_->setJointValueTarget(joint_values))
      {
        ROS_ERROR_NAMED(name_, "Error with with joint target");
        return MoveItErrorCode(moveit_msgs::MoveItErrorCodes::FAILURE);
      }
      return arm_->plan(plan_);
    }

    /**
     * @brief Planificación para posiciones establecidas en archivo de configuración SRDF.
     *  
     * @param name Nombre del estado.
     * @return Retorna el código de error del planificador.
     */
    MoveItErrorCode plan(const std::string &name)
    {
      ROS_DEBUG_NAMED(name_, "Sending to position: %s", name.c_str());
      if (!arm_->setNamedTarget(name))
      {
        ROS_ERROR_NAMED(name_, "Error with position: %s", name.c_str());
        return MoveItErrorCode(moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
      }
      return arm_->plan(plan_);
    }


    bool publishTrajectory()
    {
      if (plan_.trajectory_.joint_trajectory.points.size())
      {
        display_trajectory_.trajectory_start = plan_.start_state_;
        display_trajectory_.trajectory.push_back(plan_.trajectory_);
        display_trajectory_pub_.publish(display_trajectory_);
        ROS_INFO_NAMED(name_, "Publishing planed trajectory");
        ros::Duration(0.5).sleep();
        // Clear trajectory msg
        display_trajectory_ = moveit_msgs::DisplayTrajectory();
        return true;
      }
      return false;
    }

    MoveItErrorCode move()
    {
      return arm_->move();
    }

    static double planScore(const MoveGroup::Plan& plan)
    {
      // Costo inicial basado en tiempo de calculo 
      double time_factor = 10, aux = 0;
      double score = plan.planning_time_ * time_factor;
      // Suma de desplazamientos de puntos de trayectoria
      const std::vector<trajectory_msgs::JointTrajectoryPoint>& trajectory = plan.trajectory_.joint_trajectory.points;
      for(std::vector<int>::size_type i = 0; i < trajectory.size()-1; i++) {
        aux = 0;
        for(std::vector<int>::size_type j = 0; j < trajectory[i].positions.size(); j++){
          aux += fabs(trajectory[i+1].positions[j] - trajectory[i].positions[j]);
        }
        score += aux;
      }
      return score;
    }

}; // Clase BenderMoveGroupInterface

typedef boost::shared_ptr<BenderMoveGroupInterface> BenderMoveGroupInterfacePtr;



class BenderPlanningServer
{
  private:
    // BenderPlanning
    BenderMoveGroupInterfacePtr arm_;

    // Planning scene
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_; // Markers


    const std::string arm_name_;
    const std::string name_;
    // ROS Nodehandle
    ros::NodeHandle nh_;

    // Servicios
    ros::ServiceServer
      position_server_,
      joint_server_,
      position_named_server_,
      position_servoing_server_,
      eef_pose_server_,
      orientation_server_,
      planner_server_,
      //grasp_server_,
      attach_object_,
      clear_object_;

    // Action server para generar grasps
    actionlib::SimpleActionServer<GraspGeneratorAction> grasp_server_;
    GraspGeneratorResult grasp_result_;

  public:
    BenderPlanningServer(BenderMoveGroupInterfacePtr arm):
      arm_(arm),
      arm_name_(arm->arm_name_),
      name_(arm->arm_name_ + "_planning"),
      nh_(name_),
      grasp_server_(nh_, "grasp_generator", boost::bind(&BenderPlanningServer::graspGeneratorCB, this, _1), false)
    {
      // Obtener visual tools
      visual_tools_ = arm->getVisualTools();

      planning_scene_monitor_ = arm_->getPlanningSceneMonitor();

      // position server
      position_server_ = nh_.advertiseService("set_position", &BenderPlanningServer::setPosition, this);

      // Joint target server
      joint_server_ = nh_.advertiseService("set_joint", &BenderPlanningServer::setJoint, this);
      // position named server
      position_named_server_ = nh_.advertiseService("set_position_named", &BenderPlanningServer::setPositionNamed, this);
      // position servoing
      position_servoing_server_ = nh_.advertiseService("servoing", &BenderPlanningServer::setPositionServoing, this);
      // get end effector pose
      eef_pose_server_ = nh_.advertiseService("get_eef", &BenderPlanningServer::getEndEffectorPose, this);
      // set gripper orientation
      orientation_server_ = nh_.advertiseService("set_orientation", &BenderPlanningServer::setGripperOrientation, this);

      planner_server_ = nh_.advertiseService("set_planner", &BenderPlanningServer::setPlanner, this);
    
      attach_object_ = nh_.advertiseService("attach_object", &BenderPlanningServer::attachObject, this);

      clear_object_ = nh_.advertiseService("clear_collision", &BenderPlanningServer::removeCollisionObjects, this);

      grasp_server_.start();

    }

    bool setJoint(JointTarget::Request& req, JointTarget::Response& res)
    {
      // Planicar
      res.error_code.val = (arm_->plan(req.joints)).val;
      if (res.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
      {
        ROS_ERROR_NAMED(name_, "Error planning joint target");
        return true;
      }
      // Ejecutar
      res.error_code.val = (arm_->move()).val;
      if (res.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
      {
        ROS_ERROR_NAMED(name_, "Error executing joint target");
      }
      return true;
    }


    bool removeCollisionObjects(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
      return visual_tools_->removeAllCollisionObjects();
    }

    bool attachObject(AttachObject::Request &req, AttachObject::Response &res)
    {    
      res.succeed = visual_tools_->processCollisionObjectMsg( req.object, rviz_visual_tools::RAND);
      return true;
    }

    bool setPlanner(SetPlannerID::Request &req, SetPlannerID::Response &res)
    { 
      ROS_INFO_NAMED(name_, "Setting planner id to: %s", req.planner.c_str());
      arm_->setPlanner(req.planner);
      return true;
    }

    void graspGeneratorCB(const GraspGeneratorGoalConstPtr &goal)
    {
      // Resetar result
      grasp_result_ = GraspGeneratorResult();

      // Generar grasps
      ROS_INFO_NAMED(name_, "Generating grasps for: %s", goal->target.id.c_str());
      grasp_result_.succeed = arm_->graspGenerator(goal->target, grasp_result_.grasps, grasp_result_.ik_solutions, 
        grasp_result_.order, goal->axial_resolution, goal->angle_resolution, true);

      grasp_server_.setSucceeded(grasp_result_);
    }

    // Set position server
    bool setPosition(Position::Request &req, Position::Response &res)
    {
      ROS_INFO_NAMED(name_, "Position request with position: [%.2f, %.2f, %.2f] frame_id: %s",
        req.target.pose.position.x, req.target.pose.position.y, req.target.pose.position.z,
        req.target.header.frame_id.c_str());
      // Obtener transformada con respecto a frame de referencia
      try
      {
        transformPose(*arm_->getTF(), arm_->ref_frame_.c_str(), req.target);
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR("Can't transform request to %s. Error: %s", arm_->ref_frame_.c_str(), ex.what());
        res.reach = false;
        return true;
      }

      // Objeto con respecto a frame de referencia   
      Eigen::Affine3d object_global_transform;
      tf::poseMsgToEigen(req.target.pose, object_global_transform);

      // Offset de efector
      Eigen::Affine3d eff_pose;
      eff_pose = Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitY());
      eff_pose.translation() = Eigen::Vector3d( -0.18, 0.02 , 0.0);

      // Aplicar transformada
      geometry_msgs::Pose eff_pose_msg;
      tf::poseEigenToMsg(object_global_transform * eff_pose, eff_pose_msg);
     
      if (req.approx_ik)
      {
        if(!arm_->plan(eff_pose_msg.position.x, eff_pose_msg.position.y, eff_pose_msg.position.z))
        {
          ROS_ERROR_NAMED(name_, "Error in request with position: [%.2f, %.2f, %.2f] frame_id: %s. Using approx. IK.",
            eff_pose_msg.position.x, eff_pose_msg.position.y, eff_pose_msg.position.z,
            req.target.header.frame_id.c_str());
            res.reach = false;
            return true;
        }
        res.reach = arm_->move();
        return true;
      }
      else if (!arm_->plan(eff_pose_msg))
      {
        ROS_INFO_NAMED(name_, "Error in request with position: [%.2f, %.2f, %.2f] frame_id: %s",
          eff_pose_msg.position.x, eff_pose_msg.position.y, eff_pose_msg.position.z,
          req.target.header.frame_id.c_str());
        res.reach = false;
        return true;
      }
      // Publicar marker
      visual_tools_->setLifetime(1.0);
      visual_tools_->publishAxis(req.target.pose);
      res.reach = arm_->move();
      return true;
    }

    // Set position named
    bool setPositionNamed(PositionNamed::Request &req, PositionNamed::Response &res)
    {
      std::string arm_target = arm_name_ + "_" + req.target;
      ROS_INFO_NAMED(name_, "Position named request: %s", arm_target.c_str());
      
      if (!arm_->plan(arm_target))
      {
        ROS_ERROR_NAMED(name_, "Error setting position named %s", req.target.c_str());
        res.reach = false;
        return true;
      }
      res.reach = arm_->move();
      return true;
    }

/*    bool ik_exploration(){
            geometry_msgs::Pose object_pose;
      object_pose.position.x = 0.4;
      object_pose.position.y = -0.25;
      object_pose.position.z = 0.8;

      tf::Quaternion q;

      double roll = 0, pitch = 0, yaw = 0;
      q.setRPY(roll, pitch, yaw);
      tf::quaternionTFToMsg(q, object_pose.orientation);

      // Rangos
      Range x_r(0.03, 3);
      Range y_r(0.03, 3);
      Range z_r(0.03, 3);
      Range roll_r(M_PI/10, 3);
      Range pitch_r(M_PI/6, 5);
      Range yaw_r(M_PI/6, 3);
      
      std::vector<geometry_msgs::Pose> explore_pose;
      // Variar X
      for(int i=0; i<x_r.resolution; ++i)
      {
        for(int j=0; j<y_r.resolution; ++j)
        {
          for(int k=0; k<z_r.resolution; ++k)
          {
            for(int l=0; l<roll_r.resolution; ++l)
            {
              for(int m=0; m<pitch_r.resolution; ++m)
              {
                for(int n=0; n<yaw_r.resolution; ++n)
                {
                  
                  q.setRPY(roll += roll_r++, pitch += pitch_r++, yaw += yaw_r++);
                  tf::quaternionTFToMsg(q, object_pose.orientation);
                  
                  explore_pose.push_back(object_pose);
                  
                }
              } 
            }
            object_pose.position.z += z_r++;
            ROS_INFO_NAMED(name_, "Position servoing request with position: [%.2f, %.2f, %.2f]",
              object_pose.position.x, object_pose.position.y, object_pose.position.z);
            ROS_WARN_NAMED(name_, "Position servoing request with position: [%.2f, %.2f, %.2f]",
              x_r.val, y_r.val, z_r.val);
          }
          object_pose.position.y += y_r++;
        }
        object_pose.position.x += x_r++;
      }
      ROS_INFO_STREAM("Exploration points " << explore_pose.size());
      return true;
    }*/


    // Servoing
    bool setPositionServoing(PositionServoing::Request &req, PositionServoing::Response &res)
    {
      ROS_INFO_NAMED(name_, "Position servoing request with position: [%.2f, %.2f, %.2f]",
        req.delta.position.x, req.delta.position.y, req.delta.position.z);
      // Obtener posicion actual
      geometry_msgs::PoseStamped current_pose = arm_->getCurrentPose();
      // Position
      current_pose.pose.position.x += req.delta.position.x;
      current_pose.pose.position.y += req.delta.position.y;
      current_pose.pose.position.z += req.delta.position.z;
      // Orientacion
      current_pose.pose.orientation.x += req.delta.orientation.x;
      current_pose.pose.orientation.y += req.delta.orientation.y;
      current_pose.pose.orientation.z += req.delta.orientation.z;
      current_pose.pose.orientation.w += req.delta.orientation.w;

      visual_tools_->publishAxis(current_pose.pose);
      visual_tools_->publishText(current_pose.pose, "target");

      // Planificar
      if (!arm_->plan(current_pose))
      {
        ROS_ERROR_NAMED(name_, "Servoing not reachable.");
        res.reach = false;
        return true;
      };
      res.reach = arm_->move();
      return true;
    }

    bool getEndEffectorPose(EndEffectorPose::Request &req, EndEffectorPose::Response &res)
    {
      ROS_INFO_NAMED(name_, "End effector pose request.");
      // Obtener posicion actual
      res.pose = arm_->getCurrentPose();
      return true;
    }

    bool setGripperOrientation(GripperOrientation::Request &req, GripperOrientation::Response &res)
    {
      geometry_msgs::PoseStamped arm_pose = arm_->getCurrentPose();
      tf::Quaternion q;
      if (req.wrt_base)
      {
        ROS_INFO_NAMED(name_, "Gripper orientation request wrt /bender/base_link.");
        req.pitch = -M_PI/2;
      }
      // Get current RPY
      tf::Pose tf_pose;
      tf::poseMsgToTF(arm_pose.pose, tf_pose);
      double roll, pitch, yaw;
      tf_pose.getBasis().getRPY(roll, pitch, yaw);
      // Actualizar pitch
      q.setRPY(roll, req.pitch, yaw);
      tf::quaternionTFToMsg(q, arm_pose.pose.orientation);
      ROS_INFO_NAMED(name_, "[%.2f, %.2f, %.2f, %.2f]", arm_pose.pose.orientation.x, arm_pose.pose.orientation.y,
        arm_pose.pose.orientation.z, arm_pose.pose.orientation.w);

      if (!arm_->plan(arm_pose))
      {
        ROS_ERROR_NAMED(name_, "Orientation not reachable.");
        res.reach = false;
        return true;
      };
      res.reach = arm_->move();
      return true;
    }

}; // Clase BenderPlanningServer

//class 

} // bender_arm_control namespace

int main(int argc, char **argv)
{
  using namespace bender_arm_control;

  ros::init(argc, argv, "bender_planning");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(3); // move_group y queues locales
  spinner.start();

  // Obtener argumentos
  std::string planning_group = "l_arm";
  if (argc > 1)
  {
    for (std::size_t i = 0; i < argc; ++i)
    {
      if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0)
      {
        std::cout << "Uso: rosrun bender_arm_control bender_planning --planning_group [l_arm,r_arm]" << std::endl;
        return 0;
      }
      if (strcmp(argv[i], "--planning_group") == 0)
      {
        ++i;
        planning_group = argv[i];
        ROS_INFO_STREAM_NAMED("main","Using planning_group " << planning_group);
      }
    }
  }
  // Cargar planning scene monitor
  planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION));
  ros::spinOnce();
  ros::Duration(0.5).sleep();
  ros::spinOnce();
  // Modificar planning scene monitor para obtener el estado del robot
  psm->startWorldGeometryMonitor(COLLISION_OBJECT_TOPIC, PLANNING_SCENE_WORLD_TOPIC, true);
  psm->startSceneMonitor(MONITORED_PLANNING_SCENE_TOPIC);
  psm->startStateMonitor(JOINT_STATE_TOPIC, ATTACHED_COLLISION_OBJECT_TOPIC);
  psm->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE, 
                                                      PLANNING_SCENE_TOPIC);
  ROS_INFO_STREAM("Publishing planning scene on " << PLANNING_SCENE_TOPIC);

  // Interface para move_group l_arm y r_arm
  bender_arm_control::BenderMoveGroupInterfacePtr l_arm(new bender_arm_control::BenderMoveGroupInterface("l_arm", node_handle, psm));
  bender_arm_control::BenderMoveGroupInterfacePtr r_arm(new bender_arm_control::BenderMoveGroupInterface("r_arm", node_handle, psm));

  // Capa de servicios
  bender_arm_control::BenderPlanningServer l_arm_server(l_arm);
  bender_arm_control::BenderPlanningServer r_arm_server(r_arm);

  ros::waitForShutdown();
  return 0;
}
