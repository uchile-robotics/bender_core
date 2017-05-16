#include <uchile_report_generator/report_generator.h>

namespace uchile_report_generator {


ReportGenerator::ReportGenerator(std::string name) {

	this->_name = name;

	ros::NodeHandle priv("~");

	_pkg_path = ros::package::getPath("uchile_report_generator");
    loaded_images = 0;

    // - - - - - - - P A R A M E T E R   S E R V E R - - - - - - - - -
    uchile_util::ParameterServerWrapper psw;

    // compilation command
    psw.getParameter("compile_cmd", _compile_cmd, "pdflatex -synctex=1 -interaction=nonstopmode");

    // Gaps
    psw.getParameter("gap_img_path", _gap_img_path, "%{IMG-PATH}");
    psw.getParameter("gap_img_caption", _gap_img_caption, "%{IMG-CAPTION}");
    psw.getParameter("gap_img_section", _gap_img_section, "%{IMG-SECTION}");
    psw.getParameter("gap_map_section", _gap_map_section, "%{MAP-SECTION}");
    psw.getParameter("gap_test_name", _gap_test_name, "%{TEST_NAME}");
    psw.getParameter("gap_try_number", _gap_try_number, "%{TRY}");


    // templates
    psw.getParameter("template_main", _main_template, "main.tex");
    psw.getParameter("template_map", _map_template, "map.tex");
    psw.getParameter("template_image", _img_template, "img.tex");
    _main_template = _pkg_path + "/tex/templates/" + _main_template;
    _map_template = _pkg_path + "/tex/templates/" + _map_template;
    _img_template = _pkg_path + "/tex/templates/" + _img_template;


    psw.getParameter("test_name", _test_name, "Manipulation");

    // service
    _image_srv =  priv.advertiseService("add_image", &ReportGenerator::add_image,this);
    _map_srv =  priv.advertiseService("add_map", &ReportGenerator::add_map,this);
    _create_srv =  priv.advertiseService("generate", &ReportGenerator::generate,this);

    _map_filename = saveMapImage();

   // getMapParameters();

    //Load Report File
    loadParametersFile();

	ROS_WARN_STREAM("[" << name << "]: Ready to work");
}

ReportGenerator::~ReportGenerator() {

}

void ReportGenerator::loadParametersFile() {
    std::string _report_file;
    //TIMENOW : ros::Time::now
    //NTRIES  :  number of tries
    uchile_util::ParameterServerWrapper psw;
    // psw.getParameter("report_file", _report_file, "Homebreakers_TIMENOW");
    psw.getParameter("report_file", _report_file, "Homebreakers_NTRIES");


    //Replace in report file name  : TIMENOW 
    std::stringstream timenow, tried;
    timenow <<ros::Time::now();
    std::size_t postime = _report_file.find("TIMENOW");
    if (postime!=std::string::npos)
        _report_file.replace(postime,7,timenow.str());

    //Replace in report file name  : NTRIES 
    tried <<1; //TODO buscar el numero
    std::size_t pos_try = _report_file.find("NTRIES");
    if (pos_try!=std::string::npos)
        _report_file.replace(pos_try,6,tried.str());

    //Define path
    report_file <<_pkg_path + "/tex/" << _report_file;
    ROS_INFO_STREAM("[" << this->_name << "]: FILE : "<<report_file.str());

    //Load main template
    readFile(_main_template, tex_code);

    //Replace report file parameters
    replaceGap(tex_code, _gap_test_name, _test_name);
    replaceGap(tex_code, _gap_try_number, tried.str());
}

void ReportGenerator::getMapParameters() {

	ros::NodeHandle priv("~");

	// map variables
	nav_msgs::OccupancyGrid map;
	nav_msgs::GetMap map_srv;
	ros::ServiceClient getMap_client = priv.serviceClient<nav_msgs::GetMap>("/static_map");

	// - - - - - - Get Map & MapInfo - - - - - -
	ROS_WARN("Waiting map server");
	while ( ros::ok() && !getMap_client.waitForExistence(ros::Duration(3.0)) );
	getMap_client.call(map_srv);

	_res = map_srv.response.map.info.resolution;
	_map_x0 = - map_srv.response.map.info.origin.position.x;
	_map_y0 = - map_srv.response.map.info.origin.position.y;
	_img_width = map_srv.response.map.info.width;
	_img_height = map_srv.response.map.info.height;

	ROS_WARN_STREAM("[" << _name << "]: Map resolution: " << _res);
	ROS_WARN_STREAM("[" << _name << "]: Map origin: (x,y)=(" << _map_x0 << "," << _map_y0 << ")");
	ROS_WARN_STREAM("[" << _name << "]: Map size: (w,h)=(" << _img_width << "," << _img_height << ")");
}

std::string ReportGenerator::saveMapImage() {

	std::string pgm_map_path = ros::package::getPath("uchile_maps") + "/maps/map.pgm";
	std::string out_map_path = _pkg_path + "/tex/map/map.png";

	cv::Mat pgm_map = cv::imread(pgm_map_path);
	cv::imwrite(out_map_path, pgm_map);

	return out_map_path;
}

void ReportGenerator::compile(std::string output_filename) {

	std::string cmd = _compile_cmd + "  -output-directory " + _pkg_path + "/tex"
		+ " " + output_filename;

	// Redirect output to null
	cmd += " > /dev/null 2>&1";

	// compile
	int i = system(cmd.c_str());
	ROS_INFO("pdflatex compilation command returned %d", i);
}

bool ReportGenerator::writeFile(std::string filename, std::string text) {

	std::ofstream out_stream;
	out_stream.open(filename.c_str());

	if(!out_stream.is_open()) {
		ROS_ERROR_STREAM("Could not open output file: " << filename);
		return false;
	}

	out_stream << text;
	out_stream.close();

	return true;
}

bool ReportGenerator::readFile(std::string filename, std::string& result) {

	std::ifstream file;
	std::stringstream buffer;

	// open file
	file.open(filename.c_str());
	if(!file.is_open()) {
		ROS_ERROR_STREAM("Could not open file: " << filename);
		return false;
	}

	// read file
	buffer << file.rdbuf();
	result = buffer.str();

	// close file
	file.close();

	return true;
}

void ReportGenerator::replaceGap(std::string& subject, const std::string& key, const std::string& value) {

	size_t pos = 0;
	while ((pos = subject.find(key, pos)) != std::string::npos) {
		subject.replace(pos, key.length(), value);
		pos += value.length();
	}
}

bool ReportGenerator::add_image(uchile_srvs::ReportGenerator::Request &req, uchile_srvs::ReportGenerator::Response &res) {


    // - - - - - - - - - - - - - Images stuff - - - - - - - - - - - - - - - - -
    std::string total_img_code;

    cv_bridge::CvImagePtr cv_ptr;
    std::string tmp_img_code;
    std::ostringstream tmp_path_ss;

    // load img template
    std::string img_code_template;
    readFile(_img_template, img_code_template);

    // base filename
    std::string base_path = _pkg_path + "/tex/img/img_";

    for(int k=0; k<req.imgs.size() ; k++) {

        tmp_path_ss.str("");              // clear path
        tmp_img_code = img_code_template; // clean template

        // current image fullfilename
        tmp_path_ss << base_path << loaded_images+k << ".png";

        // fill gaps
        replaceGap(tmp_img_code, _gap_img_path, tmp_path_ss.str());
        replaceGap(tmp_img_code, _gap_img_caption, req.img_captions[k]);

        // add source code  to  total template
        total_img_code += tmp_img_code;

        // save images
        cv_ptr = cv_bridge::toCvCopy(req.imgs[k],req.imgs[k].encoding);
        cv::imwrite(tmp_path_ss.str(), cv_ptr->image);

    }

    loaded_images = req.imgs.size();
    replaceGap(tex_code, _gap_img_section, total_img_code);
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    return true;
}



bool ReportGenerator::add_map(uchile_srvs::ReportGenerator::Request &req, uchile_srvs::ReportGenerator::Response &res) {


    // - - - - - - - - - Map stuff - - - - - - - - - - - - - -
    // Get map template
    std::string map_code;
    readFile(_map_template, map_code);

    // Put map image on template
    replaceGap(map_code, _gap_img_path, _map_filename);

    char buff1[100];
    char buff2[100];

    sprintf(buff1, "%.3f", _res);
    replaceGap(map_code, "%{IMG-RES}", buff1);

    sprintf(buff1, "%d", _img_width);
    replaceGap(map_code, "%{IMG-WIDTH}", buff1);

    sprintf(buff1, "%d", _img_height);
    replaceGap(map_code, "%{IMG-HEIGHT}", buff1);

    sprintf(buff1, "%.3f", _map_x0);
    replaceGap(map_code, "%{MAP-X0}", buff1);

    sprintf(buff1, "%.3f", _map_y0);
    replaceGap(map_code, "%{MAP-Y0}", buff1);

    sprintf(buff1, "%.2f", req.person.pose.position.x);
    replaceGap(map_code, "%{PERSON-X}", buff1);

    sprintf(buff1, "%.2f", req.person.pose.position.y);
    replaceGap(map_code, "%{PERSON-Y}", buff1);

    sprintf(buff1, "%d", (int)truncf(0 - _map_x0));
    sprintf(buff2, "%d", (int)truncf(_img_width*_res - _map_x0));
    replaceGap(map_code, "%{GRID-X}", std::string(buff1) + ",...," + std::string(buff2));

    sprintf(buff1, "%d", (int)truncf(0 - _map_y0));
    sprintf(buff2, "%d", (int)truncf(_img_width*_res - _map_y0));
    replaceGap(map_code, "%{GRID-Y}", std::string(buff1) + ",...," + std::string(buff2));
    replaceGap(map_code, "%{IMG-GUY-PATH}", _pkg_path + "/tex/img/guy.png");

    // put map template on main template
    replaceGap(tex_code, _gap_map_section, map_code);


	return true;
}

bool ReportGenerator::generate(uchile_srvs::ReportGenerator::Request &req, uchile_srvs::ReportGenerator::Response &res) {

    // create & compile tex file
    std::cout<<report_file<<std::endl;
    writeFile(report_file.str() + ".tex", tex_code);
    compile(report_file.str() + ".tex");

    res.report_path = report_file.str() + ".pdf";
    return true;
}


}  // namespace uchile_report_generator


int main(int argc, char **argv) {

	ros::init(argc, argv, "report_generator");

	boost::scoped_ptr<uchile_report_generator::ReportGenerator> node(
			new uchile_report_generator::ReportGenerator(ros::this_node::getName())
	);

	ros::spin();

	printf("\nQuitting... \n\n");

	return 0;
}
