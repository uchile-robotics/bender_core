/**
 * @file /include/uchile_gui_subtitles/main_window.hpp
 *
 * @brief Qt based gui for uchile_gui_subtitles.
 *
 * @date November 2010
 **/
#ifndef uchile_gui_subtitles_MAIN_WINDOW_H
#define uchile_gui_subtitles_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"


/*****************************************************************************
** Namespace
*****************************************************************************/

namespace uchile_gui_subtitles {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

public Q_SLOTS:   
    void updateTextView();

private:
	Ui::MainWindow ui;
	QNode qnode;
};

}  // namespace uchile_gui_subtitles

#endif // uchile_gui_subtitles_MAIN_WINDOW_H
