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
#include "../include/px4_gcs/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace px4_gcs {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

ICSL_GCS::ICSL_GCS(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
	, ROSConnectionFlag(false), PX4ConnectionFlag(false)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
	setWindowIcon(QIcon(":/images/icon.jpeg"));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
	ui.view_logging->setModel(qnode.loggingModel());
	QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView())); 
}

ICSL_GCS::~ICSL_GCS() {}

void ICSL_GCS::updateLoggingView()
{
	ui.view_logging->scrollToBottom();
}

void ICSL_GCS::on_pushButton_connect_ros_clicked()
{
	if(qnode.init())
	{
		std::cout << "Connected to ROS master" << std::endl;
		ROSConnectionFlag = true;
	}
	else
	{
		std::cout << "Failed to connect ROS master" << std::endl;
		ROSConnectionFlag = false;
	}
}

void ICSL_GCS::on_pushButton_connect_px4_clicked()
{
	if(qnode.connect_px4())
	{
		std::cout << "Connected to PX4" << std::endl;
		PX4ConnectionFlag = true;
	}
	else
	{
		std::cout << "Failed to connect PX4" << std::endl;
		PX4ConnectionFlag = false;
	}
}
}  // namespace px4_gcs

