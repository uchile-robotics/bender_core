#include <ros/ros.h>
#include <ros/package.h>

#include "uchile_srvs/ImageService.h"
#include "uchile_srvs/MaskInfo.h"
#include "uchile_srvs/synthesize.h"
#include <uchile_srvs/String.h>
#include <uchile_msgs/Emotion.h>
#include <sensor_msgs/image_encodings.h>

#include <cstdlib>
#include <stdio.h>
#include <iostream>
#include <dirent.h> 
#include <string.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>





using namespace std;
using namespace cv;

#define MAX_FACE_ANGLE 20//30
#define MIN_FACE_ANGLE -40//-50
#define CENTER_FACE_ANGLE -10


struct BD {
  string name_mask;
  int id;
};


struct mask {
  Mat image;
  Rect pos;
  string name;
};


vector<Rect> detectFaces(Mat & frame);
bool loadmasc(string name) ;
bool loadpicture();


ros::ServiceClient client;
ros::ServiceServer server;
ros::ServiceServer server2;

uchile_srvs::ImageService srv;
cv_bridge::CvImagePtr cv_ptr;

ros::Publisher face_pub_;
ros::ServiceClient _text_pub;
ros::ServiceClient _text_pubP;

string window_name = "RoboCup 2014";
String face_cascade_name = 	"/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml";
CascadeClassifier face_cascade;

vector < vector < mask > > masks;
vector < vector < mask > > bender;
vector <BD> listmask;

int maskcont=0;

void changeface(string face,string type="changeFace",int x=0){
    uchile_msgs::Emotion emotion;
    emotion.Order = type;
    emotion.Action = face;
    emotion.X = x;
    face_pub_.publish(emotion);
    ros::Duration(1).sleep();
  
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

int search_id (string name_conj){

	for(int i=0; i < listmask.size(); i++)
		if(listmask[i].name_mask==name_conj)
			return i;
	return -1;
}

bool wait_face(vector<Rect>& faces,int& width){

	bool facedetect=false; 
    while(!facedetect){	   
		if (client.call(srv)){
			cv_ptr = cv_bridge::toCvCopy((srv.response.im), sensor_msgs::image_encodings::BGR8);
		
			faces = detectFaces(cv_ptr->image);
			if(faces.size()>0){ 
			    changeface("happy2");
			    facedetect=true;
			    width=cv_ptr->image.cols/2;
			    return true;
			}	
		    ros::Duration(1).sleep();
		}
    }
    return false;
}

void prepare_face(vector<Rect>& faces){
	talk(" 1...2...3...say cheese","1...2...3...dizer cheese");

	cvStartWindowThread();
	namedWindow( window_name, WINDOW_NORMAL );
	resizeWindow(window_name, 1366,768);
	moveWindow(window_name, 0, 0);

	for (unsigned int k = 0; k < 10 || (k > 10 && faces.size() == 0);k++){
	    if (client.call(srv)) {
		cv_ptr = cv_bridge::toCvCopy((srv.response.im), sensor_msgs::image_encodings::BGR8);
		Mat img=cv_ptr->image.clone();
		faces = detectFaces(cv_ptr->image);
		for (unsigned int i = 0; i < faces.size();	i++){
		    cv::rectangle(img, cv::Point(faces[i].x, faces[i].y),
		    cv::Point(faces[i].x + faces[i].width,faces[i].y + faces[i].height),
		    cv::Scalar(0, 255, 255));
		}
		imshow(window_name, img);
		waitKey(10);
	    }
	}
	talk("","");
}

void copy_mask(vector<Rect>& faces,int tipo){
	for (unsigned int i = 0; i < faces.size();	i++){
	    int val=maskcont% masks[tipo].size();
	    maskcont++;

	    float factor=(float)faces[i].width/(masks[tipo][val].image.cols-masks[tipo][val].pos.x*2);
	    Mat obj2;

	    if(masks[tipo][val].image.cols*factor>0 && masks[tipo][val].image.rows*factor>0)
	    resize(masks[tipo][val].image,obj2,Size(masks[tipo][val].image.cols*factor,masks[tipo][val].image.rows*factor), 0, 0, INTER_CUBIC);

	    for (unsigned int a = 0; a <obj2.cols;a++)
	    for (unsigned int b =0; b < obj2.rows;b++)
			if(obj2.at<cv::Vec3b>(b,a)[0]>150 && obj2.at<cv::Vec3b>(b,a)[1]<30 && obj2.at<cv::Vec3b>(b,a)[2]>150){}
			else{
			    int xl=faces[i].x+a-masks[tipo][val].pos.x*factor, yl=faces[i].y+b-masks[tipo][val].pos.y*factor ;
			    if(xl>0 && yl>0 && xl<cv_ptr->image.cols && yl<cv_ptr->image.rows){
				cv_ptr->image.at<cv::Vec3b>(yl,xl)[0]= obj2.at<cv::Vec3b>(b,a)[0];
				cv_ptr->image.at<cv::Vec3b>(yl,xl)[1]= obj2.at<cv::Vec3b>(b,a)[1];
				cv_ptr->image.at<cv::Vec3b>(yl,xl)[2]= obj2.at<cv::Vec3b>(b,a)[2];
			    }
			}		  
	}
}

void copy_bender(vector<Rect>& faces,int tipo){

		int id=0;
		int val=rand() % bender[id].size(),i=0;

		float factor=(float)faces[i].height/(bender[id][val].image.rows-2*bender[id][val].pos.y - bender[id][val].pos.height);
		Mat obj2;

		if(bender[id][val].image.cols*factor>0 && bender[id][val].image.rows*factor>0)
		    resize(bender[id][val].image,obj2,Size(bender[id][val].image.cols*factor,bender[id][val].image.rows*factor), 0, 0, INTER_CUBIC);

		for (unsigned int a = 0; a <obj2.cols;a++)
		  for (unsigned int b =0; b < obj2.rows;b++)
		    if(obj2.at<cv::Vec3b>(b,a)[0]>120 && obj2.at<cv::Vec3b>(b,a)[1]<30 && obj2.at<cv::Vec3b>(b,a)[2]>120){}
		    else{
			int xl=faces[i].x+faces[i].width+a-bender[id][val].pos.x*factor, yl=faces[i].y+b-bender[id][val].pos.y*factor ;
			if(xl>0 && yl>0 && xl<cv_ptr->image.cols && yl<cv_ptr->image.rows){
			    cv_ptr->image.at<cv::Vec3b>(yl,xl)[0]= obj2.at<cv::Vec3b>(b,a)[0];
			    cv_ptr->image.at<cv::Vec3b>(yl,xl)[1]= obj2.at<cv::Vec3b>(b,a)[1];
			    cv_ptr->image.at<cv::Vec3b>(yl,xl)[2]= obj2.at<cv::Vec3b>(b,a)[2];
			}
		    }	

}

bool draw_mask(uchile_srvs::MaskInfo::Request &req, uchile_srvs::MaskInfo::Response &res) {


    if (client.call(srv)) {
		try {
			int id = search_id(req.mask);
		    
		    int tipo=-1,width;
		    vector<Rect> faces;
		    
		  	wait_face(faces,width);

		    res.n_faces = faces.size();
		    if(faces.size()>0){

			    int angle=( CENTER_FACE_ANGLE - MAX_FACE_ANGLE )*(faces[0].x+(faces[0].width/2)-width)/(-width);
			    changeface(" ","MoveX",angle);
		    
				tipo = id;
				if(tipo < 0){
					ROS_ERROR("Failed name of mask");
				 	return false;
				}

				prepare_face(faces);

				res.n_faces = faces.size();
				copy_mask(faces,tipo);

				imshow(window_name, cv_ptr->image);
				waitKey(30);

				ros::Duration(2).sleep();

			    
			    destroyWindow(window_name);
			}
	    } catch (cv_bridge::Exception& e) {
	      	ROS_ERROR("cv_bridge exception: %s", e.what());
	    }
    } else {
 	   ROS_ERROR("Failed to call service camera_service");
    }

    talk("",""); 
    changeface(" ","MoveX",CENTER_FACE_ANGLE);
    return true;
    
}

bool draw_bender(uchile_srvs::MaskInfo::Request &req, uchile_srvs::MaskInfo::Response &res) {


    if (client.call(srv)) {
		try {
			int id = 0;
		    
		    int tipo=-1,width;
		    vector<Rect> faces;
		    
		  	wait_face(faces,width);

		    res.n_faces = faces.size();
		    if(faces.size()>0){

			    int angle=( CENTER_FACE_ANGLE - MAX_FACE_ANGLE )*(faces[0].x+(faces[0].width/2)-width)/(-width);
			    changeface(" ","MoveX",angle);
		    
				tipo = id;
				if(tipo < 0){
					ROS_ERROR("Failed name of mask");
				 	return false;
				}

				prepare_face(faces);

				res.n_faces = faces.size();
				copy_bender(faces,tipo);

				imshow(window_name, cv_ptr->image);
				waitKey(30);

				ros::Duration(2).sleep();

			    
			    destroyWindow(window_name);
			}
	    } catch (cv_bridge::Exception& e) {
	      	ROS_ERROR("cv_bridge exception: %s", e.what());
	    }
    } else {
 	   ROS_ERROR("Failed to call service camera_service");
    }

    talk("",""); 
    changeface(" ","MoveX",CENTER_FACE_ANGLE);
    return true;
    
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "face_mask");

    if (!face_cascade.load(face_cascade_name)) {
		ROS_INFO("(!)Error loading face file\n");
		return -1;
    };

    loadpicture() ;
    loadmasc("scare");
    loadmasc("superheroe");
    loadmasc("movies");
    loadmasc("cute");
    
    
    ros::NodeHandle n("~"); 

    std::string cam_name = "camera_right_eye";
    if(!n.getParam("cam_name",cam_name)) n.setParam("cam_name",cam_name);

    client = n.serviceClient<uchile_srvs::ImageService>("/bender/sensors/" + cam_name + "/get_image");
    face_pub_ = n.advertise<uchile_msgs::Emotion>("/bender/head/cmd", 1);
    server = n.advertiseService("draw_mask", draw_mask);
 	server2 = n.advertiseService("draw_bender", draw_bender);
    _text_pubP = n.serviceClient<uchile_srvs::String>("/bender/speech/synthesizer/text_p",1);
    _text_pub = n.serviceClient<uchile_srvs::String>("/bender/speech/synthesizer/text",1);

    // publishers
	while ( ros::ok() && !client.waitForExistence(ros::Duration(3.0)) );

    if (!ros::ok()) 
	    exit(1);
        
    ros::spin();

    return 0;
}



std::vector<Rect> detectFaces(Mat & frame) {
    std::vector<Rect> faces;
    Mat frame_gray;
    cvtColor(frame, frame_gray, CV_BGR2GRAY);
    equalizeHist(frame_gray, frame_gray);
    double min_face_size=30;
    double max_face_size=400;

    face_cascade.detectMultiScale( frame_gray, faces, 1.1, 10, 
				    0|CV_HAAR_SCALE_IMAGE, Size(min_face_size, min_face_size) );

    return faces;
}


bool loadmasc(string namemask) 
{ 
    DIR *dp; 
    struct dirent *entry; 
  
    std::string path = ros::package::getPath("uchile_fun");
    string namedir=path+"/database/masks/"+namemask;
    if((dp = opendir(namedir.c_str())) == NULL) { 
        cout<<"cannot open directory: Bender"<<endl; 
        return false; 
    } 

   // bendermask.name = "bender";
    std::vector< mask> v;
    int i=0;
    while((entry = readdir(dp)) != NULL) { 
		if(strlen(entry->d_name) > 3) {
			mask newmask;
		    string name=entry->d_name;

		    newmask.name = name;
		    newmask.image = imread(namedir+"/"+name);

		    size_t found1 = name.find("_");
		    size_t found2 = name.find("_",found1+1);

		    newmask.pos.x=atoi(name.substr (found1+1,found2-found1-1).c_str());
		    newmask.pos.y=atoi(name.substr (found2+1,strlen(entry->d_name)-found2-1).c_str());

		    newmask.pos.width=0;
		   	newmask.pos.height=0;
		    v.push_back(newmask);
		}
 
    } 
    
    masks.push_back(v);
    
    BD newconj;
    newconj.name_mask = namemask;
    newconj.id = listmask.size();
    listmask.push_back(newconj);

    closedir(dp); 
    return true;
}



bool loadpicture() 
{ 
    DIR *dp; 
    struct dirent *entry; 
    std::string path = ros::package::getPath("uchile_fun");
    string namedir=path+"/database/bender";
    if((dp = opendir(namedir.c_str())) == NULL) { 
        cout<<"cannot open directory: Bender"<<endl; 
        return false; 
    } 

 //   bendermask.name = "bender";
    std::vector< mask> v;
    int i=0;
    while((entry = readdir(dp)) != NULL) { 
		if(strlen(entry->d_name) > 3) {
			mask newmask;
		    string name=entry->d_name;

		    newmask.name = name;
		    newmask.image = imread(namedir+"/"+name);

		    size_t found1 = name.find("_");
		    size_t found2 = name.find("_",found1+1);
		    size_t found3 = name.find("_",found2+1);
		    size_t found4 = name.find("_",found3+1);

		    newmask.pos.x=atoi(name.substr (found1+1,found2-found1-1).c_str());
		    newmask.pos.width=atoi(name.substr (found2+1,found3-found2-1).c_str()) -  newmask.pos.x;
		    newmask.pos.y=atoi(name.substr (found3+1,found4-found3-1).c_str());
		   	newmask.pos.height=atoi(name.substr (found4+1,strlen(entry->d_name)-found3-1).c_str()) -  newmask.pos.y;
		    v.push_back(newmask);
		}
 
    } 
    
    bender.push_back(v);
    
    closedir(dp); 
    return true;
}
