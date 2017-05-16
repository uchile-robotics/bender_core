/**
 * @file /include/uchile_gui_subtitles/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef uchile_gui_subtitles_QNODE_HPP_
#define uchile_gui_subtitles_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// Added this macro to support incompatible Boost 1.56.0 with QT 4.7.4
#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QLabel>
#include <QString>
//#include <qtextcodec.h>

// ROS messages, services
#include <std_msgs/String.h>
#include <uchile_srvs/String.h>

#endif

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace uchile_gui_subtitles {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();

	void setTextLabel(QLabel* label) {
		if (text_label != label) {
			delete text_label;
			text_label = label;
		}
	}


Q_SIGNALS:
	void textUpdated();	// signals new text
    void rosShutdown(); // signals ros::ok() == false 

private:
	int init_argc;
	char** init_argv;
    QLabel *text_label;

    // ros communication
    ros::Subscriber _text_sub;
	ros::ServiceServer _text_server;

    // ros callbacks
    void text_callback(const std_msgs::String &msg);
	bool text_server(uchile_srvs::String::Request &req, uchile_srvs::String::Response &res);

	// methods
	void displayText(std::string &text);

};

}  // namespace uchile_gui_subtitles


#endif /* uchile_gui_subtitles_QNODE_HPP_ */
