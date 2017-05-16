#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include "uchile_srvs/synthesize.h"
#include <uchile_srvs/String.h>
#include <uchile_msgs/Emotion.h>
#include <uchile_srvs/play_sound.h>
#include <cstdlib>
#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <string.h>
#include "std_msgs/String.h"

using namespace std;
using namespace cv;


ros::Publisher face_pub_;
ros::ServiceClient talk_srv;
ros::ServiceClient play_srv;
ros::ServiceClient _text_pub;
ros::ServiceClient _text_pubP;

ros::ServiceServer server;
ros::ServiceServer server2;
string window_name = "Video Fun";
string synthesizer_status;

bool play;
string filename;


void changeface(string face,string type="changeFace",int x=0){
    uchile_msgs::Emotion emotion;
    emotion.Order = type;
    emotion.Action = face;
    emotion.X = x;
    face_pub_.publish(emotion);
    
    ros::Duration(1.5).sleep();
  
}


void talk(string msg,string port)
{
    uchile_srvs::String en;
    en.request.data= msg;
    uchile_srvs::String po ;
    po.request.data=port;
    
    _text_pub.call(en);
    _text_pubP.call(po);
}

bool playvideoOff(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res){ 
    play=false;
}

bool playvideoOn(uchile_srvs::play_sound::Request  &req, uchile_srvs::play_sound::Response &res){ 


    std::string path = ros::package::getPath("uchile_fun");
    filename=path+"/database/video/"+req.sound;

     if (req.play == true){  
        play=true;
        res.success = true; 
    }
    else res.success = false;

    return true;

}


int main(int argc, char** argv)
{
  
    ros::init(argc, argv, "video");
    ros::NodeHandle n;

    face_pub_ = n.advertise<uchile_msgs::Emotion>("/bender/head/cmd", 1);
    server = n.advertiseService("playvideoOn", playvideoOn);
    server2 = n.advertiseService("playvideoOff", playvideoOff);

    _text_pubP = n.serviceClient<uchile_srvs::String>("/bender/speech/synthesizer/text_p",1);
    _text_pub = n.serviceClient<uchile_srvs::String>("/bender/speech/synthesizer/text",1);


    play=false;


    while (ros::ok()) {

        if(play){
                      
            VideoCapture capture(filename);
        
            Mat frame;    
            
            if( !capture.isOpened() )
        
            changeface("happy2");

        
            cvStartWindowThread();
            namedWindow( window_name, WINDOW_NORMAL );
            setWindowProperty(window_name, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

        
    
            for( ; play ; )
            {
                capture >> frame;
                if(frame.empty())   break;
                imshow(window_name, frame);
                waitKey(20); 
                ros::spinOnce();
            }

            frame.release();
            destroyWindow(window_name);
            ros::Duration(1).sleep();
            play=false;
        }

        ros::spinOnce();
    }
	exit(1);

    

    return 1;
}

