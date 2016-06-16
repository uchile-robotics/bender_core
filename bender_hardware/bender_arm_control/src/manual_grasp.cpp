bool grasp(bender_msgs::CylindricalObject& target, std::vector<GraspPoint>& grasp_points)
    {
      // Asumimos posicion wrt base_link
      double height = target.height;
      double radius = target.radius;

      target.pose.orientation.x = 0.0;
      target.pose.orientation.y = 0.0;
      target.pose.orientation.z = 0.0;
      target.pose.orientation.w = 1.0;


      visual_tools_->setLifetime(200.0);
      visual_tools_->publishAxis(target.pose);
      visual_tools_->publishCylinder(target.pose, rviz_visual_tools::RED , height, radius);
      
      // Pose wrt base_link
      Eigen::Affine3d object_global_transform;
      tf::poseMsgToEigen(target.pose, object_global_transform);

      // Offset de efector
      Eigen::Affine3d eff_offset;
      eff_offset = Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitY());
      eff_offset.translation() = Eigen::Vector3d( -0.18, 0.02 , 0.0);

      // Distintos Pose del objeto wrt al ik_frame
      std::vector<geometry_msgs::Pose> possible_grasps;
      
      Eigen::Affine3d grasp_pose;
      geometry_msgs::Pose eff_target_msg;

      Range z_r(height/2, 5); // Rango Z
      Range yaw_r(1.0, 15); // Rango yaw

      // Grasp central optimo
      Eigen::Affine3d optimal_grasp = object_global_transform * eff_offset;
      Eigen::Vector3d optimal_grasp_vector = optimal_grasp.translation();
      
      for(int i=0; i<yaw_r.resolution; ++i)
      {
        grasp_pose = Eigen::AngleAxisd(yaw_r++, Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
        
        for (int j = 0; j < z_r.resolution; ++j)
        {
          grasp_pose.translation() = Eigen::Vector3d( 0.0, 0.0, z_r++);
          grasp_pose = object_global_transform * grasp_pose;
          Eigen::Affine3d eff_target = grasp_pose * eff_offset;

          // Anadir pose wrt ik_frame
          tf::poseEigenToMsg(ik_transform_ * eff_target, eff_target_msg);
          possible_grasps.push_back(eff_target_msg);

          //visual_tools_->publishAxis(eff_target_msg);
          //ros::Duration(0.2).sleep();
        }
        z_r.reset();
      }
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
      for(std::size_t i = 0; i < joint_names_.size(); ++i)
      {
        ROS_INFO("Joint %s: %f", joint_names_[i].c_str(), current_joint[i]);
      }

      //IK
      std::vector<double> solution;
      moveit_msgs::MoveItErrorCodes error_code;
      // Trayectoria
      std::vector<trajectory_msgs::JointTrajectoryPoint> ik_solutions;
      // Aprox IK
      kinematics::KinematicsQueryOptions opt;
      opt.return_approximate_solution = false;

      // Puntos
      Eigen::Affine3d test_grasp_pose, ik_transform_inv = ik_transform_.inverse();

      // Check IK
      for (std::vector<geometry_msgs::Pose>::iterator grasp = possible_grasps.begin(); grasp != possible_grasps.end(); ++grasp)
      {
        kin_solver_->searchPositionIK(*grasp, current_joint, ik_timeout_, solution, error_code, opt);
        if( error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS )
        {
          trajectory_msgs::JointTrajectoryPoint point;
          point.positions = solution;
          ik_solutions.push_back(point);

          
          tf::poseMsgToEigen(*grasp, test_grasp_pose);
          test_grasp_pose =  ik_transform_inv * test_grasp_pose;
          Eigen::Vector3d test_vector = test_grasp_pose.translation();

          // FUNCION DE COSTO @TODO: Con funcion externa
          // Distancia al grasp de centro
          double dist_opt = (test_vector - optimal_grasp_vector).squaredNorm();
          // Distancia cartesiana del grasp wrt posicion actual
          double dist_cart = (test_vector - current_eff_pose_vector).squaredNorm();
          // Distancia en espacio de joint wrt posicion actual
          double dist_joint = distVector(current_joint, solution);

          double cost =  3.0*dist_joint + 5.0*dist_cart + 1.0*dist_opt;


          grasp_points.push_back(GraspPoint(point, test_grasp_pose, cost));

          ROS_INFO_NAMED(name_,"Solucion para grasp [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] costo %.3f", solution[0],
            solution[1], solution[2], solution[3], solution[4], solution[5], cost);
        }
      }
      // Ver trayectoria
      visual_tools_->publishIKSolutions(ik_solutions, arm_name_, 0.4);
      ros::Duration(2.0).sleep();

      if (grasp_points.size() == 0)
      {
        ROS_ERROR_NAMED(name_, "No grasp found!");
        return true;
      }
      // Ordenar en funcion de costo
      std::sort(grasp_points.begin(), grasp_points.end());
      ROS_INFO_NAMED(name_,"Costo optimo: %.3f", grasp_points[0].cost);

      return true;
    }