#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <string>

static struct termios olds, news;

/* Initialize new terminal i/o settings */
void initTermios(int echo) 
{
  tcgetattr(0, &olds); /* grab old terminal i/o settings */
  news = olds; /* make new settings same as old settings */
  news.c_lflag &= ~ICANON; /* disable buffered i/o */
  news.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
  tcsetattr(0, TCSANOW, &news); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
void resetTermios(void) 
{
  tcsetattr(0, TCSANOW, &olds);
}

/* Read 1 character - echo defines echo mode */
char getch_(int echo) 
{
  char ch;
  initTermios(echo);
  ch = getchar();
  resetTermios();
  return ch;
}

/* Read 1 character without echo */
char getch(void) 
{
  return getch_(0);
}

/* Read 1 character with echo */
char getche(void) 
{
  return getch_(1);
}

bool kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;
 
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
 
  ch = getchar();
 
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
 
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return true;
  }
 
  return false;
}

class EmergencyStopNode : public rclcpp::Node{
	// attributes
	rclcpp::Rate *rate;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motors_st_sub;
	rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clEnable;
	rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clDisable;
	std::string ns;
private:
	bool state;
public:
	EmergencyStopNode(rclcpp::Rate *);
	void run();
	void stateChanged(const std_msgs::msg::Bool &ptref);
};

EmergencyStopNode::EmergencyStopNode(rclcpp::Rate *r) : Node("emergency_stop"){
	rate = r;
	this->declare_parameter<std::string>("namespace", "");
	this->get_parameter("namespace", ns);
	RCLCPP_INFO_STREAM(this->get_logger(), "Namespace = \"" << ns << '"');
	// motors_st_sub = this->create_subscription<std_msgs::msg::Bool>(
	// 	( ns.empty()? "motors_state" : (ns + '/' + "motors_state") ),
	// 	1,
	// 	std::bind(&EmergencyStopNode::stateChanged, this, std::placeholders::_1)
	// );
	clEnable = this->create_client<std_srvs::srv::Empty>(
		( ns.empty()? "enable_motors" : (ns + '/' + "enable_motors") )
	);
	clDisable = this->create_client<std_srvs::srv::Empty>(
		( ns.empty()? "disable_motors" : (ns + '/' + "disable_motors") )
	);
	state = true; // current robot's motors state (default is true)
	RCLCPP_INFO_STREAM(this->get_logger(), "emergency_stop node has been started");
}

void EmergencyStopNode::stateChanged(const std_msgs::msg::Bool &ptref){ 
	state = ptref.data;
	if (state) RCLCPP_INFO_STREAM(this->get_logger(), "robot's motors have been ENABLED");
	else RCLCPP_INFO_STREAM(this->get_logger(), "robot's motors have been DISABLED");
}

const unsigned short int timeout_ms = 500; 

void EmergencyStopNode::run(void){
	char c;
	bool r;
	//std::shared_ptr<std_srvs::srv::Empty_Request_<std::allocator<void> > > request =
	//	std::make_shared<std_srvs::srv::Empty::Request>();
	auto request = std::make_shared<std_srvs::srv::Empty::Request>();

	do{
		if (kbhit()) {
			c = toupper( getch() );
			RCLCPP_INFO_STREAM(this->get_logger(), "The key '" << c << "' has been pressed");
			if (state) {
				RCLCPP_INFO_STREAM(this->get_logger(), "Call service disable_motors");	
				r = clDisable->wait_for_service( std::chrono::milliseconds(timeout_ms) );
				if (!r) {
					RCLCPP_ERROR_STREAM(this->get_logger(), "Service disable_motors is not available");
					continue;
				}
				auto result = clDisable->async_send_request(request);
				if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
							rclcpp::FutureReturnCode::SUCCESS) {
					state = false;
  				RCLCPP_INFO_STREAM(this->get_logger(), "Robot's motors have been DISABLED");
  			}
  			else RCLCPP_ERROR_STREAM(this->get_logger(), "Service call NOT successful");
			}
			else if (c == 'G') {
				RCLCPP_INFO_STREAM(this->get_logger(), "Call service enable_motors");
				r = clEnable->wait_for_service( std::chrono::milliseconds(timeout_ms) );
				if (!r) {
					RCLCPP_ERROR_STREAM(this->get_logger(), "Service enable_motors is not available");
					continue;
				}
				auto result = clEnable->async_send_request(request);
				if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
						rclcpp::FutureReturnCode::SUCCESS) {
					state = true;
  				RCLCPP_INFO_STREAM(this->get_logger(), "Robot's motors have been ENABLED");
  			}
  			else RCLCPP_ERROR_STREAM(this->get_logger(), "Service call NOT successful");
			}

			if (c == 'Q') RCLCPP_INFO_STREAM(this->get_logger(), "Exit key ('Q') has been pressed! Terminate node.");
		}

		rate->sleep();

	}while ( rclcpp::ok() && c != 'Q');
}

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	rclcpp::Rate loop_rate(50.0);
	std::shared_ptr<EmergencyStopNode> node = std::make_shared<EmergencyStopNode>(&loop_rate);
	node->run();
	rclcpp::shutdown();
	return 0;
}