#include <ros/ros.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/PlanningSceneWorld.h>
#include <octomap_msgs/Octomap.h>

ros::Publisher octomapPub;
octomap_msgs::Octomap octomap_msg;

// Recibe PlanningScene y extrae el msg octomap para publicar de forma individual
void octomapCallback(const moveit_msgs::PlanningScene::ConstPtr& msg) {
  octomap_msg = msg->world.octomap.octomap; // Copiar datos
  octomap_msg.header.frame_id = msg->world.octomap.header.frame_id; // Obtener frame_id
  octomap_msg.header.seq++;
  octomapPub.publish(octomap_msg);
}

int main(int argc, char **argv)
{
  // Iniciar nodo
  ros::init (argc, argv, "subtopic_moveit");
  ros::NodeHandle n;
  
  // Publisher para octomap
  octomapPub = n.advertise<octomap_msgs::Octomap>("/octomap_world", 10);
  // Subscriber para PlanningScene
  ros::Subscriber sceneSub = n.subscribe("/move_group/monitored_planning_scene", 10, octomapCallback);

  ros::spin();
  return 0;
}