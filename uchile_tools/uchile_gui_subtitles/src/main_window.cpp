/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/uchile_gui_subtitles/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace uchile_gui_subtitles {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	// connects all ui's triggers to on_...() callbacks in this class.
	ui.setupUi(this);
    
    // setup GUI
	QDesktopWidget screen;
	int screen_width = screen.availableGeometry(screen.primaryScreen()).width();

	this->setGeometry(0, 0, screen_width, 100);
	this->setWindowIcon(QIcon(":/images/icon.png"));
	this->setWindowFlags(Qt::WindowStaysOnTopHint);

	// setup label
	ui.label->setVisible(true);
	ui.label->setAutoFillBackground(true);
	ui.label->setStyleSheet("QLabel { color : black; }"); //{ background-color : rgb(255,255,255); color : black; }
	
	// -- ROS connections --
	// connect ros_shutdown() trigger
	QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	// connect text label to node
	qnode.setTextLabel(ui.label);
	QObject::connect(&qnode, SIGNAL(textUpdated()), this, SLOT(updateTextView()));

    // init ros communications
	qnode.init();
}

MainWindow::~MainWindow() {}

// - - -  [Slots] - - - //
// This function is signalled by the underlying model.
void MainWindow::updateTextView() {
    //ui.label->scrollToBottom();
    std::string sentence = ui.label->text().toStdString();
    QPoint loc =  this->pos();
    int len = sentence.length();
    QDesktopWidget screen;
    int screen_width = screen.availableGeometry(screen.primaryScreen()).width();
    this->setGeometry(loc.x(), loc.y() + 38, screen_width, 50*(1+len/40));
    this->adjustSize();
}

}  // namespace uchile_gui_subtitles
