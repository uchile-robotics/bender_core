/*
 * LaserProcessing.h
 *
 *      Author: LuzMartinez
 */

#ifndef LASERPROCESSING_H_
#define LASERPROCESSING_H_

#include <ros/ros.h> 
 #include <ros/console.h>

// C, C++
#include <cstdio> // for EOF
#include <string>
#include <sstream>
#include <vector>

// ROS
#include <ros/ros.h>
#include <bender_srvs/Onoff.h>



namespace bender_utils {

class LaserProcessing {

public:

  ros::NodeHandle priv;
	//bool display;
	bool ready_laser;

	std::string _laser_topic;

  sensor_msgs::LaserScan laser_in;

private:

  ros::Subscriber _subs_laser;

public:
	LaserProcessing(std::string name = "~");
	virtual ~LaserProcessing();
  bool _is_on;

protected:
	/** - - - - FUNCTIONS. - - - - **/


	/** - - - - CALLBACKS - - - - **/
  void _process_laser(const sensor_msgs::LaserScan _laser_in);


public:
	bool _active_service_laser(bender_srvs::Onoff::Request  &req, bender_srvs::Onoff::Response &res);
  void set_topic (std::string topic);

};



 LaserProcessing::LaserProcessing(std::string name) {

	priv = ros::NodeHandle(name);
	// display = true;
	_is_on = false;
	ready_laser = false;
}

 LaserProcessing::~LaserProcessing() {

}

void LaserProcessing::set_topic(std::string topic) {   
  _laser_topic = topic;
}



 bool LaserProcessing::_active_service_laser(bender_srvs::Onoff::Request  &req, bender_srvs::Onoff::Response &res) {

    if(req.select == true) {
        if (!_is_on) {
            _subs_laser = priv.subscribe(_laser_topic, 1, &LaserProcessing::_process_laser, this); 
            _is_on = true;

            ROS_INFO_STREAM("Turning on "+_laser_topic+". . . OK");
        } else ROS_DEBUG_STREAM("Already turned on");
    }
    else{
        if (_is_on) {
              _subs_laser.shutdown();
              _is_on = false;
              ready_laser = false;
              ROS_INFO_STREAM(" Turning off . . . OK");
        } else  ROS_DEBUG_STREAM("Already turned off"); 
    }
    return true;
}

void LaserProcessing::_process_laser(const sensor_msgs::LaserScan _laser_in){

    if(!_is_on) return;
    laser_in = _laser_in;
    ready_laser = true;
}



} /* namespace bender_utils */
#endif /* LaserProcessing_H_ */
