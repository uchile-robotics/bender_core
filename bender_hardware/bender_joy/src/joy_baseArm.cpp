#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "bender_srvs/setGoalBaseArm.h"
#include "bender_srvs/setAnglesBaseArm.h"
#include "bender_srvs/manipBaseArm.h"


class Joystick {

public:
  Joystick();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  // ros stuff
  ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;
  ros::ServiceClient setGoalClient;
  ros::ServiceClient moveArmClient;
  ros::ServiceClient gripClient;

  // button state
  bool is_paused_;
  
};

float efectorGoal[3]={60.0,0,-15};

Joystick::Joystick(){

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 1, &Joystick::joyCallback, this);


  // services
  setGoalClient = nh_.serviceClient<bender_srvs::setGoalBaseArm>("setGoalBaseArm");  
  moveArmClient = nh_.serviceClient<bender_srvs::setAnglesBaseArm>("setAnglesBaseArm");
  gripClient = nh_.serviceClient<bender_srvs::manipBaseArm>("manipBaseArm");

  is_paused_ = false;

}

/**
* joyCallback features:
* - up, down, left, right
* - diferent velocities (depends on angle of stick)
* - pause/resume
*/
void Joystick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {

  bender_srvs::setGoalBaseArm armGoal;
  bender_srvs::manipBaseArm gripAction;
  
  // check buttons
  if ( joy->buttons[8] == 1 ) {


  }else if (joy->buttons[0]==1 ) {

    gripAction.request.action=1;

    if(gripClient.call(gripAction)){
      ROS_INFO("\n Grip %d",gripAction.response.received);
    }else{
      ROS_ERROR("Grip service Failed");
      return;
    }

  }else if (joy->buttons[1]==1 ) {

    gripAction.request.action=0;

    if(gripClient.call(gripAction)){
      ROS_INFO("\n Grip %d",gripAction.response.received);
    }else{
      ROS_ERROR("Grip service Failed");
      return;
    }

  }else if (joy->buttons[2]==1 ) {

    armGoal.request.x = 30.0;
    armGoal.request.y = 0.0;
    armGoal.request.z = 40.0;

    efectorGoal[0]=30.0;
    efectorGoal[1]=0.0;
    efectorGoal[2]=40.0;

    if(setGoalClient.call(armGoal)){
      ROS_INFO("\n Goal %d",armGoal.response.received);
    }else{
      ROS_ERROR("armGoal service Failed");
      return;
    }


  }else if (joy->buttons[3]==1 ) {
    bender_srvs::setAnglesBaseArm restAngles;

    restAngles.request.alpha = 0.0;
    restAngles.request.beta = -1.5;
    restAngles.request.gamma = 1.65;

    efectorGoal[0]=60.0;
    efectorGoal[1]=0.0;
    efectorGoal[2]=-14.0;

    if(moveArmClient.call(restAngles)){
      ROS_INFO("\n Angles %d",restAngles.response.received);
    }else{
      ROS_ERROR("restAngles service Failed");
      return;
    }
  }
  else if(joy->buttons[4]==1){
    if ( joy->axes[0]==1) {

      efectorGoal[1]=efectorGoal[1]-0.5;

      armGoal.request.x = efectorGoal[0];
      armGoal.request.y = efectorGoal[1];
      armGoal.request.z = efectorGoal[2];

      if(setGoalClient.call(armGoal)){
        ROS_INFO("\n Goal %d",armGoal.response.received);
      }else{
        ROS_ERROR("armGoal service Failed");
        return;
      }

    }
    else if ( joy->axes[0]==-1) {

      efectorGoal[1]=efectorGoal[1]+0.5;

      armGoal.request.x = efectorGoal[0];
      armGoal.request.y = efectorGoal[1];
      armGoal.request.z = efectorGoal[2];

      if(setGoalClient.call(armGoal)){
        ROS_INFO("\n Goal %d",armGoal.response.received);
      }else{
        ROS_ERROR("armGoal service Failed");
        return;
      }

    }
    else if ( joy->axes[1]==1) {

      efectorGoal[0]=efectorGoal[0]+0.5;

      armGoal.request.x = efectorGoal[0];
      armGoal.request.y = efectorGoal[1];
      armGoal.request.z = efectorGoal[2];

      if(setGoalClient.call(armGoal)){
        ROS_INFO("\n Goal %d",armGoal.response.received);
      }else{
        ROS_ERROR("armGoal service Failed");
        return;
      }



    }
    else if ( joy->axes[1]==-1) {

      efectorGoal[0]=efectorGoal[0]-0.5;

      armGoal.request.x = efectorGoal[0];
      armGoal.request.y = efectorGoal[1];
      armGoal.request.z = efectorGoal[2];

      if(setGoalClient.call(armGoal)){
        ROS_INFO("\n Goal %d",armGoal.response.received);
      }else{
        ROS_ERROR("armGoal service Failed");
        return;
      }


    }
    else if ( joy->axes[4]==1) {

      efectorGoal[2]=efectorGoal[2]+0.5;

      armGoal.request.x = efectorGoal[0];
      armGoal.request.y = efectorGoal[1];
      armGoal.request.z = efectorGoal[2];

      if(setGoalClient.call(armGoal)){
        ROS_INFO("\n Goal %d",armGoal.response.received);
      }else{
        ROS_ERROR("armGoal service Failed");
        return;
      }



    }
    else if ( joy->axes[4]==-1) {

      efectorGoal[2]=efectorGoal[2]-0.5;

      armGoal.request.x = efectorGoal[0];
      armGoal.request.y = efectorGoal[1];
      armGoal.request.z = efectorGoal[2];

      if(setGoalClient.call(armGoal)){
        ROS_INFO("\n Goal %d",armGoal.response.received);
      }else{
        ROS_ERROR("armGoal service Failed");
        return;
      }
    }

  }
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "joy_baseArm");
  Joystick joystick;



  ros::spin();
}
