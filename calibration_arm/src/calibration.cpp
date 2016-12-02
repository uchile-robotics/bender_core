
#include <calibration.h>

namespace bender_perception {

Calibration::Calibration(std::string name) {

	priv = ros::NodeHandle(name);
	_name = name;
	display = true;
	_is_on = false;
	ready_rgb = false;
	ready_depth = false;


	// - - - - - - - p a r a m e t e r s - - - - - - - - - - -
	bender_utils::ParameterServerWrapper psw;
    psw.getParameter("camera_topic",_RGB_topic, "/bender/sensors/rgbd_head/rgb/image_raw");
    psw.getParameter("depth_topic",_Depth_topic, "/bender/sensors/rgbd_head/depth/image_raw");
    psw.getParameter("display",display, true);


    set_topics(_RGB_topic,_Depth_topic);


    _active_server = priv.advertiseService("active", &Calibration::_active_service_RGBD, (ImageProcessing*)this);
    _display_server = priv.advertiseService("set_display", &Calibration::set_display, (ImageProcessing*)this);

    win_rgb = "Calibration";   win_name.push_back(win_rgb);
    win_depth = "Calibration Depth";   win_name.push_back(win_depth);

    if(display){
         cv::startWindowThread();
         for (int i = 0; i < win_name.size(); ++i)
            namedWindow(win_name[i], WINDOW_AUTOSIZE );
        
    }
}

Calibration::~Calibration() {}

ros::Subscriber joint_states_sub = n.subscribe("/bender/sensors/rgbd_head/rgb/image_raw", 1, getCorners);

void Calibration::run(){


    cv::Mat ImageIn = get_mat (image_in, sensor_msgs::image_encodings::BGR8), DepthIn;

    if (_is_on_depth && ready_depth){ 
        DepthIn = get_mat (depth_in, sensor_msgs::image_encodings::TYPE_16UC1);
        geometry_msgs::Point p = get_point(DepthIn,Point(10,10));
        std::cout<<p.x<<" "<<p.y<<std::endl;
    }
    if(display){
        Display(win_rgb, ImageIn);
        if (_is_on_depth && ready_depth) {
            Display(win_depth, DepthIn);
        }
    }

}


} /* namespace bender_perception*/

int main(int argc, char **argv) {

	ros::init(argc, argv, "Calibration");
	ros::NodeHandle priv("~");

	bender_perception::Calibration node(ros::this_node::getName());

    int _fps = 25;
    if(!priv.getParam("fps",_fps)) priv.setParam("fps",_fps);

    ros::Rate r(_fps);

    while(ros::ok()){

        if (node._is_on && node.ready_rgb)
          node.run();
          
        r.sleep();
        ros::spinOnce();
    }

	ROS_INFO_STREAM("[" << ros::this_node::getName() << "] Quitting ... \n");

	return 0;
}



