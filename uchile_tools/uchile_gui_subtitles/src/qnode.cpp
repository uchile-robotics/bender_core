/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

// Added this macro to support incompatible Boost 1.56.0 with QT 4.7.4
#ifndef Q_MOC_RUN

// NODE
#include "../include/uchile_gui_subtitles/qnode.hpp"

// C++
#include <string>
#include <sstream>
#include <iostream>
#include <time.h>
// ROS
#include <ros/ros.h>
#include <ros/network.h>

#endif

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace uchile_gui_subtitles {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) 
: init_argc(argc), init_argv(argv) 
{
    text_label = new QLabel();
}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
    wait();
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"gui_subtitles");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle priv("~");

    // Add your ros communications here.
    // callbacks
    _text_sub    = priv.subscribe("text", 1, &QNode::text_callback, this);
    _text_server = priv.advertiseService("set_text", &QNode::text_server, this);

    start();

    ROS_INFO("Ready to Work");
    return true;
}

void QNode::run() {

    ros::Rate rate(20);
    while ( ros::ok() ) {

        ros::spinOnce();
        rate.sleep();
    }

    // signal the gui for a shutdown (useful to roslaunch)
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); 
}

void QNode::text_callback(const std_msgs::String &msg) {

    ROS_INFO_STREAM("Received text message: " << msg.data);
    std::string text = msg.data;

    // cortar texto si tamaño es mayor a max_chars caracteres
    int max_chars = 40;
    if (text.length() > max_chars) {
        std::size_t found = text.find(" ");  
        std::string sentence="", complete="";
        int lineas=0;
        while (found != std::string:: npos) {
        
            std::string word = text.substr (0,found); 
            text = text.substr(found+1);
            
            if (sentence.length() + word.length() < max_chars) {
                sentence += " " + word;

            } else {

                if (lineas == 2) {
                    
                    complete += sentence;
                    displayText(complete);
                    complete="";
                    sentence = word;
                    lineas=0;
                    usleep(9000000);

                } else {

                    complete+=sentence+" \n";
                    sentence = word;
                    lineas=lineas+1;
                }
            }
            found = text.find(" ");
        }

        if (sentence.length()+text.length() < max_chars) {
            text = complete + " " + sentence + " " + text;

        } else {
            text = complete + " " +sentence + "\n " + text;
        }
    }
    ROS_INFO_STREAM("displaying: " << text);
    displayText(text);
}

bool QNode::text_server(uchile_srvs::String::Request &req,uchile_srvs::String::Response &res) {

    ROS_INFO_STREAM("Received text request: " << req.data);
    displayText(req.data);

    return true;
}

void QNode::displayText(std::string &text) {
  
    // prevent weird characters!
    QString qtext = QString::fromUtf8(text.c_str());
    text_label->setText(qtext);
    Q_EMIT textUpdated();
}

}  // namespace uchile_gui_subtitles
