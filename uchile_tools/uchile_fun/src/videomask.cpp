#include <ros/ros.h>
#include <ros/package.h>

#include "uchile_srvs/ImageService.h"
#include "uchile_srvs/MaskInfo.h"
#include "uchile_srvs/synthesize.h"
#include <uchile_srvs/String.h>
#include <uchile_srvs/Onoff.h>
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

string window_name = "RoboCup 2015";
String face_cascade_name = 	"/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml";
CascadeClassifier face_cascade;

vector < vector < mask > > masks;
vector <BD> listmask;

vector<Rect> prev_faces;
vector<int> prev_mask;

int tim=0;
int maskcont=0;
bool activo = false;
void changeface(string face,string type="changeFace",int x=0){
    uchile_msgs::Emotion emotion;
    emotion.Order = type;
    emotion.Action = face;
    emotion.X = x;
    // face_pub_.publish(emotion);
    //ros::Duration(1).sleep();
  
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


bool wait_face(vector<Rect>& faces,int& width){

    bool facedetect=false; 
    if (client.call(srv)){
	    cv_ptr = cv_bridge::toCvCopy((srv.response.im), sensor_msgs::image_encodings::BGR8);
    
	    faces = detectFaces(cv_ptr->image);
	    if(faces.size()>0){ 
		//changeface("happy2");
		facedetect=true;
		width=cv_ptr->image.cols/2;
		return true;
	    }	
	// ros::Duration(1).sleep();
    }
    
    return false;
}


bool compRect(Rect A, Rect B){
    float calc = 0,xcA,ycA,xcB,ycB,calc2=0;
   /* calc = (A.x-B.x)*(A.x-B.x) + (A.y-B.y)*(A.y-B.y); 
    calc+= (A.x+A.width - B.x+B.width)*(A.x+A.width - B.x+B.width);
    calc+= (A.y+A.height - B.y+B.height)*(A.y+A.height - B.y+B.height);
    calc = sqrt(calc);*/
    
    xcA = A.x+A.width/2;
    ycA = A.y+A.height/2;
    xcB = B.x+B.width/2;
    ycB = B.y+B.height/2;
    calc2 = (xcB-xcA)*(xcB-xcA) + (ycB-ycA)*(ycB-ycA);
    calc2 = sqrt(calc2);
    
  //  cout<<calc2<<endl;;
    
    
    if(calc2< 100)return true;
    return false;
}


vector<int> searchid(vector<Rect>& faces){
    vector<int> id_mask;
    for (unsigned int j = 0; j < faces.size();	j++){
	int id = -1;
	for (unsigned int i = 0; i < prev_faces.size();	i++){
	    if(compRect(faces[j],prev_faces[i])) id = prev_mask[i];
	}
	id_mask.push_back(id);
    }
  
   return id_mask;
}

vector<int> copy_mask(vector<Rect>& faces,int tipo){
    vector<int> id_mask;
    id_mask = searchid(faces);
    
    for (unsigned int i = 0; i < faces.size();	i++){
	
	int val;
	if(id_mask[i] != -1)val = id_mask[i];
	else{
	  val=maskcont% masks[tipo].size();
	  id_mask[i] = val;
	  maskcont++;
	}
	
	

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
		    id_mask.push_back(val);
    }
    return id_mask;
}



bool draw_mask() {

    try {

	int id = 0;
	int tipo=-1,width;
	vector<Rect> faces;

	wait_face(faces,width);
	
	if(faces.size()>0){

	    int angle=0;//( CENTER_FACE_ANGLE - MAX_FACE_ANGLE )*(faces[0].x+(faces[0].width/2)-width)/(-width);
	///   changeface(" ","MoveX",angle);

	    tipo = id;

	    vector<int> id_mask = copy_mask(faces,tipo);

	    if (tim>0){
	      prev_faces = faces;
	      prev_mask = id_mask;
	    }
	}
	tim++;
	
	// imshow(window_name, cv_ptr->image);
    IplImage imagen = cv_ptr->image;
    cvShowImage("RoboCup 2015", &imagen);
	waitKey(3);
      
    } catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  
    

    return true;
    
}



bool MaskActive(uchile_srvs::Onoff::Request  &req, uchile_srvs::Onoff::Response &res)
{
    if(req.select == true){
	tim=0;
	activo = true; 
	
	cvStartWindowThread();
	// namedWindow( window_name, WINDOW_NORMAL );
    cvNamedWindow( "RoboCup 2015", CV_WINDOW_NORMAL);
    cvSetWindowProperty("RoboCup 2015", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	// resizeWindow(window_name, 1366,768);
	// moveWindow(window_name, 0, 0);
	
    }else{
	cvDestroyWindow("RoboCup 2015");
	prev_faces.clear();
	activo = false;	
    }
    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "video_mask");

    if (!face_cascade.load(face_cascade_name)) {
		ROS_INFO("(!)Error loading face file\n");
		return -1;
    };

    loadmasc("all");   
    loadmasc("scare");
    loadmasc("superheroe");
    loadmasc("movies");
    loadmasc("cute");
 
    
    ros::NodeHandle n("~"); 

    std::string cam_name = "camera_right_eye";
    if(!n.getParam("cam_name",cam_name)) n.setParam("cam_name",cam_name);
    client = n.serviceClient<uchile_srvs::ImageService>("/bender/sensors/" + cam_name + "/get_image");
    
    ros::ServiceServer service1 = n.advertiseService("MaskActive", MaskActive);
	
    face_pub_ = n.advertise<uchile_msgs::Emotion>("/bender/head/cmd", 1);
    _text_pubP = n.serviceClient<uchile_srvs::String>("/bender/speech/synthesizer/text_p",1);
    _text_pub = n.serviceClient<uchile_srvs::String>("/bender/speech/synthesizer/text",1);

    while ( ros::ok() && !client.waitForExistence(ros::Duration(3.0)) );

    	cv::startWindowThread();
    while ( ros::ok()){
      if (activo && client.call(srv)) draw_mask();
      ros::spinOnce();
    }

    if (!ros::ok()) 
	    exit(1);
        


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

