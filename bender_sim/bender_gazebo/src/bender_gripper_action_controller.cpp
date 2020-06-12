// Pluginlib
#include <pluginlib/class_list_macros.hpp>

// Controller
#include <bender_gazebo/bender_gripper_action_controller.h>

namespace bender_gazebo
{
  namespace effort_controllers
  {
    /**
     * \brief Gripper action controller that sends
     * commands to a \b effort interface.
     */
    typedef bender_gazebo::MultiGripperActionController MultiGripperActionController;
  }
}

PLUGINLIB_EXPORT_CLASS(bender_gazebo::effort_controllers::MultiGripperActionController, controller_interface::ControllerBase)
