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
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
	setWindowIcon(QIcon(":/images/icon.jpeg"));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
	ui.view_logging->setModel(qnode.loggingModel());
	QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView())); 
	QObject::connect(&qnode, SIGNAL(pushButton_connect_ros_color(bool)), this, SLOT(set_pushButton_connect_ros_color(bool))); 
	QObject::connect(&qnode, SIGNAL(pushButton_connect_px4_color(bool)), this, SLOT(set_pushButton_connect_px4_color(bool))); 
}

ICSL_GCS::~ICSL_GCS() {}

void ICSL_GCS::updateLoggingView()
{
	ui.view_logging->scrollToBottom();
}

void ICSL_GCS::on_pushButton_connect_ros_clicked()
{
	qnode.init();
}

void ICSL_GCS::on_pushButton_connect_px4_clicked()
{
	qnode.connect_px4();
}

void ICSL_GCS::set_pushButton_connect_ros_color(bool flag)
{
	if(flag)	ui.pushButton_connect_ros->setStyleSheet("background-color: rgba(0,255,0,128);");
	else		ui.pushButton_connect_ros->setStyleSheet("background-color: rgba(255,0,0,128);");
}

void ICSL_GCS::set_pushButton_connect_px4_color(bool flag)
{
}

}  // namespace px4_gcs

