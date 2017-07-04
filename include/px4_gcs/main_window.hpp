/**
 * @file /include/px4_gcs/main_window.hpp
 *
 * @brief Qt based gui for px4_gcs.
 *
 * @date November 2010
 **/
#ifndef px4_gcs_MAIN_WINDOW_H
#define px4_gcs_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_icsl_gcs.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace px4_gcs {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class ICSL_GCS : public QMainWindow {
Q_OBJECT

public:
	ICSL_GCS(int argc, char** argv, QWidget *parent = 0);
	~ICSL_GCS();

public Q_SLOTS:
	void on_pushButton_connect_ros_clicked();
	void on_pushButton_connect_px4_clicked();
	void updateLoggingView();

private:
	Ui::ICSL_GCS ui;
	QNode qnode;
};

}  // namespace px4_gcs

#endif // px4_gcs_MAIN_WINDOW_H
