/**
 * @file /include/px4_gcs/main_window.hpp
 *
 * @brief Qt based gui for px4_gcs.
 *
 * @date November 2010
 **/
#ifndef px4_gcs_MAIN_WINDOW_H
#define px4_gcs_MAIN_WINDOW_H

#define QCUSTOMPLOT_USE_LIBRARY
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_icsl_gcs.h"
#include "qnode.hpp"
#include "qcustomplot.h"
#include "ros/ros.h"

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
	void set_pushButton_connect_ros_color(bool);
	void set_pushButton_connect_px4_color(bool);
	void set_initialization();

	void set_position_data(double,double,double);
	void set_velocity_data(double,double,double);
	void set_attitude_data(double,double,double);
	void set_angular_velocity_data(double,double,double);

	void on_pushButton_set_gain_clicked();
	void on_pushButton_get_gain_clicked();

private:
	Ui::ICSL_GCS ui;
	QNode qnode;
	bool initializationFlag;
	ros::Time t_init;
	double position_lim[6], velocity_lim[6], attitude_lim[6], angular_velocity_lim[6]; 
	double position_margin, velocity_margin, attitude_margin, angular_velocity_margin;
	double position_height, velocity_height, attitude_height, angular_velocity_height;

	std::vector<std::string> gainNames;
	void loadGains();
	void showGains(std::vector<float>);
};

}  // namespace px4_gcs

#endif // px4_gcs_MAIN_WINDOW_H
