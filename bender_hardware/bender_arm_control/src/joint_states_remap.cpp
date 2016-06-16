#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// Republisher para move_group, remap no funciona al obtener posiciones desde move_group

ros::Publisher joint_states_pub;

void jointStatesCallback(const sensor_msgs::JointState& msg) {
  joint_states_pub.publish(msg);
}

int main(int argc, char **argv)
{
  // Iniciar nodo
  ros::init (argc, argv, "joint_states_remap");
  ros::NodeHandle n;
  
  // Publisher y subscriber para joint states
  joint_states_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
  ros::Subscriber joint_states_sub = n.subscribe("/bender/joint_states", 1, jointStatesCallback);

  ros::spin();
  return 0;
}