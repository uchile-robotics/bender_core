void RosAriaNode::spin()
{
  ros::Rate r(10);
  while(ros::ok()) {

    if (!robot->areMotorsEnabled() && !robot->isEStopPressed()){
      std_srvs::Empty srv = std_srvs::Empty();
      enable_motors_cb(srv.request,srv.response);
      ROS_WARN("Stop button has been released. Enabling motors :)");
    }
    r.sleep();
    ros::spinOnce();
  }
}