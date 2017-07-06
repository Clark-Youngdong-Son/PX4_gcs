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
	: QMainWindow(parent),
	qnode(argc,argv),
	initializationFlag(false)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
	setWindowIcon(QIcon(":/images/icon.jpeg"));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
	ui.view_logging->setModel(qnode.loggingModel());
	QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView())); 
	QObject::connect(&qnode, SIGNAL(pushButton_connect_ros_color(bool)), this, SLOT(set_pushButton_connect_ros_color(bool))); 
	QObject::connect(&qnode, SIGNAL(pushButton_connect_px4_color(bool)), this, SLOT(set_pushButton_connect_px4_color(bool))); 
	QObject::connect(&qnode, SIGNAL(emit_initialization()), this, SLOT(set_initialization())); 
	QObject::connect(&qnode, SIGNAL(emit_position_data(double,double,double)), this, SLOT(set_position_data(double,double,double))); 
	QObject::connect(&qnode, SIGNAL(emit_velocity_data(double,double,double)), this, SLOT(set_velocity_data(double,double,double))); 
	QObject::connect(&qnode, SIGNAL(emit_attitude_data(double,double,double)), this, SLOT(set_attitude_data(double,double,double))); 
	QObject::connect(&qnode, SIGNAL(emit_angular_velocity_data(double,double,double)), this, SLOT(set_angular_velocity_data(double,double,double))); 


	position_lim[0] = -4.0;
	position_lim[1] = 4.0;
	position_lim[2] = -4.0;
	position_lim[3] = 4.0;
	position_lim[4] = -1.0;
	position_lim[5] = 7.0;

	position_margin = 1.0;
	position_height = 8.0;

	velocity_lim[0] = -3.0;
	velocity_lim[1] = 3.0;
	velocity_lim[2] = -3.0;
	velocity_lim[3] = 3.0;
	velocity_lim[4] = -3.0;
	velocity_lim[5] = 3.0;

	velocity_margin = 0.5;
	velocity_height = 6.0;

	attitude_lim[0] = -30.0;
	attitude_lim[1] = 30.0;
	attitude_lim[2] = -30.0;
	attitude_lim[3] = 30.0;
	attitude_lim[4] = -30.0;
	attitude_lim[5] = 30.0;

	attitude_margin = 10.0;
	attitude_height = 60.0;

	angular_velocity_lim[0] = -60.0;
	angular_velocity_lim[1] = 60.0;
	angular_velocity_lim[2] = -60.0;
	angular_velocity_lim[3] = 60.0;
	angular_velocity_lim[4] = -60.0;
	angular_velocity_lim[5] = 60.0;

	angular_velocity_margin = 10.0;
	angular_velocity_height = 120.0;

	ui.widget_p_x->addGraph();
	ui.widget_p_x->graph(0)->setPen(QPen(QColor(40,110,255)));
	ui.widget_p_x->addGraph();
	ui.widget_p_x->graph(1)->setPen(QPen(QColor(255,110,40)));
	ui.widget_p_x->yAxis->setRange(position_lim[0],position_lim[1]);
	ui.widget_p_x->yAxis->setSubTicks(false);

	ui.widget_p_y->addGraph();
	ui.widget_p_y->graph(0)->setPen(QPen(QColor(40,110,255)));
	ui.widget_p_y->addGraph();
	ui.widget_p_y->graph(1)->setPen(QPen(QColor(255,110,40)));
	ui.widget_p_y->yAxis->setRange(position_lim[2],position_lim[3]);
	ui.widget_p_y->yAxis->setSubTicks(false);

	ui.widget_p_z->addGraph();
	ui.widget_p_z->graph(0)->setPen(QPen(QColor(40,110,255)));
	ui.widget_p_z->addGraph();
	ui.widget_p_z->graph(1)->setPen(QPen(QColor(255,110,40)));
	ui.widget_p_z->yAxis->setRange(position_lim[4],position_lim[5]);
	ui.widget_p_z->yAxis->setSubTicks(false);

	ui.widget_v_x->addGraph();
	ui.widget_v_x->graph(0)->setPen(QPen(QColor(40,110,255)));
	ui.widget_v_x->addGraph();
	ui.widget_v_x->graph(1)->setPen(QPen(QColor(255,110,40)));
	ui.widget_v_x->yAxis->setRange(velocity_lim[0],velocity_lim[1]);
	ui.widget_v_x->yAxis->setSubTicks(false);

	ui.widget_v_y->addGraph();
	ui.widget_v_y->graph(0)->setPen(QPen(QColor(40,110,255)));
	ui.widget_v_y->addGraph();
	ui.widget_v_y->graph(1)->setPen(QPen(QColor(255,110,40)));
	ui.widget_v_y->yAxis->setRange(velocity_lim[2],velocity_lim[3]);
	ui.widget_v_y->yAxis->setSubTicks(false);

	ui.widget_v_z->addGraph();
	ui.widget_v_z->graph(0)->setPen(QPen(QColor(40,110,255)));
	ui.widget_v_z->addGraph();
	ui.widget_v_z->graph(1)->setPen(QPen(QColor(255,110,40)));
	ui.widget_v_z->yAxis->setRange(velocity_lim[4],velocity_lim[5]);
	ui.widget_v_z->yAxis->setSubTicks(false);

	ui.widget_roll->addGraph();
	ui.widget_roll->graph(0)->setPen(QPen(QColor(40,110,255)));
	ui.widget_roll->addGraph();
	ui.widget_roll->graph(1)->setPen(QPen(QColor(255,110,40)));
	ui.widget_roll->yAxis->setRange(attitude_lim[0],attitude_lim[1]);
	ui.widget_roll->yAxis->setSubTicks(false);

	ui.widget_pitch->addGraph();
	ui.widget_pitch->graph(0)->setPen(QPen(QColor(40,110,255)));
	ui.widget_pitch->addGraph();
	ui.widget_pitch->graph(1)->setPen(QPen(QColor(255,110,40)));
	ui.widget_pitch->yAxis->setRange(attitude_lim[2],attitude_lim[3]);
	ui.widget_pitch->yAxis->setSubTicks(false);

	ui.widget_yaw->addGraph();
	ui.widget_yaw->graph(0)->setPen(QPen(QColor(40,110,255)));
	ui.widget_yaw->addGraph();
	ui.widget_yaw->graph(1)->setPen(QPen(QColor(255,110,40)));
	ui.widget_yaw->yAxis->setRange(attitude_lim[4],attitude_lim[5]);
	ui.widget_yaw->yAxis->setSubTicks(false);

	ui.widget_p->addGraph();
	ui.widget_p->graph(0)->setPen(QPen(QColor(40,110,255)));
	ui.widget_p->addGraph();
	ui.widget_p->graph(1)->setPen(QPen(QColor(255,110,40)));
	ui.widget_p->yAxis->setRange(angular_velocity_lim[0],angular_velocity_lim[1]);
	ui.widget_p->yAxis->setSubTicks(false);

	ui.widget_q->addGraph();
	ui.widget_q->graph(0)->setPen(QPen(QColor(40,110,255)));
	ui.widget_q->addGraph();
	ui.widget_q->graph(1)->setPen(QPen(QColor(255,110,40)));
	ui.widget_q->yAxis->setRange(angular_velocity_lim[2],angular_velocity_lim[3]);
	ui.widget_q->yAxis->setSubTicks(false);

	ui.widget_r->addGraph();
	ui.widget_r->graph(0)->setPen(QPen(QColor(40,110,255)));
	ui.widget_r->addGraph();
	ui.widget_r->graph(1)->setPen(QPen(QColor(255,110,40)));
	ui.widget_r->yAxis->setRange(angular_velocity_lim[4],angular_velocity_lim[5]);
	ui.widget_r->yAxis->setSubTicks(false);

	gainNames.resize(16);
	gainNames[0] = "MC_ROLLRATE_I";	
	gainNames[1] = "MC_PITCHRATE_I";	
	gainNames[2] = "MC_YAWRATE_I";	
	gainNames[3] = "MPC_XY_VEL_I";	
	gainNames[4] = "MPC_Z_VEL_I";	
	gainNames[5] = "MPC_XY_FF";	
	gainNames[6] = "MPC_XY_P";	
	gainNames[7] = "MPC_XY_VEL_P";	
	gainNames[8] = "MPC_Z_P";	
	gainNames[9] = "MPC_Z_VEL_P";	
	gainNames[10] = "MC_ROLL_P";	
	gainNames[11] = "MC_PITCH_P";	
	gainNames[12] = "MC_YAW_P";	
	gainNames[13] = "MC_ROLLRATE_P";	
	gainNames[14] = "MC_PITCHRATE_P";	
	gainNames[15] = "MC_YAWRATE_P";	
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
	if(flag)	ui.pushButton_connect_px4->setStyleSheet("background-color: rgba(0,255,0,128);");
	else		ui.pushButton_connect_px4->setStyleSheet("background-color: rgba(255,0,0,128);");
}

void ICSL_GCS::set_initialization()
{
	t_init = ros::Time::now();
	initializationFlag = true;
	loadGains();
}

void ICSL_GCS::set_position_data(double x, double y, double z)
{
	double t_now = (ros::Time::now()-t_init).toSec();
	ui.widget_p_x->graph(0)->addData(t_now,x);
	ui.widget_p_x->xAxis->setRange(t_now,4, Qt::AlignRight);
	if(x<position_lim[0]+position_margin)
	{
		position_lim[0] = x-position_margin;
		position_lim[1] = position_lim[0]+position_height;
		ui.widget_p_x->yAxis->setRange(x-position_margin,x+(position_lim[1]-position_lim[0])-position_margin);
	}
	else if(x>position_lim[1]-position_margin)
	{
		position_lim[1] = x+position_margin;
		position_lim[0] = position_lim[1]-position_height;
		ui.widget_p_x->yAxis->setRange(x-(position_lim[1]-position_lim[0])+position_margin,x+position_margin);
	}
	ui.widget_p_x->replot();

	ui.widget_p_y->graph(0)->addData(t_now,y);
	ui.widget_p_y->xAxis->setRange(t_now,4, Qt::AlignRight);
	if(y<position_lim[2]+position_margin)
	{
		position_lim[2] = y-position_margin;
		position_lim[3] = position_lim[2]+position_height;
		ui.widget_p_y->yAxis->setRange(y-position_margin,y+(position_lim[3]-position_lim[2])-position_margin);
	}
	else if(y>position_lim[3]-position_margin)
	{
		position_lim[3] = z+position_margin;
		position_lim[2] = position_lim[3]-position_height;
		ui.widget_p_y->yAxis->setRange(y-(position_lim[3]-position_lim[2])+position_margin,y+position_margin);
	}
	ui.widget_p_y->replot();

	ui.widget_p_z->graph(0)->addData(t_now,z);
	ui.widget_p_z->xAxis->setRange(t_now,4, Qt::AlignRight);
	if(z<position_lim[4]+position_margin)
	{
		position_lim[4] = z-position_margin;
		position_lim[5] = position_lim[4]+position_height;
		ui.widget_p_z->yAxis->setRange(z-position_margin,z+(position_lim[5]-position_lim[4])-position_margin);
	}
	else if(z>position_lim[5]-position_margin)
	{
		position_lim[5] = z+position_margin;
		position_lim[4] = position_lim[5]-position_height;
		ui.widget_p_z->yAxis->setRange(z-(position_lim[5]-position_lim[4])+position_margin,z+position_margin);
	}
	ui.widget_p_z->replot();
}

void ICSL_GCS::set_velocity_data(double vx, double vy, double vz)
{
	double t_now = (ros::Time::now()-t_init).toSec();
	ui.widget_v_x->graph(0)->addData(t_now,vx);
	ui.widget_v_x->xAxis->setRange(t_now,4, Qt::AlignRight);
	if(vx<velocity_lim[0]+velocity_margin)
	{
		velocity_lim[0] = vx-velocity_margin;
		velocity_lim[1] = velocity_lim[0]+velocity_height;
		ui.widget_v_x->yAxis->setRange(vx-velocity_margin,vx+(velocity_lim[1]-velocity_lim[0])-velocity_margin);
	}
	else if(vx>velocity_lim[1]-velocity_margin)
	{
		velocity_lim[1] = vx+velocity_margin;
		velocity_lim[0] = velocity_lim[1]-velocity_height;
		ui.widget_v_x->yAxis->setRange(vx-(velocity_lim[1]-velocity_lim[0])+velocity_margin,vx+velocity_margin);
	}
	ui.widget_v_x->replot();

	ui.widget_v_y->graph(0)->addData(t_now,vy);
	ui.widget_v_y->xAxis->setRange(t_now,4, Qt::AlignRight);
	if(vy<velocity_lim[2]+velocity_margin)
	{
		velocity_lim[2] = vy-velocity_margin;
		velocity_lim[3] = velocity_lim[2]+velocity_height;
		ui.widget_v_y->yAxis->setRange(vy-velocity_margin,vy+(velocity_lim[3]-velocity_lim[2])-velocity_margin);
	}
	else if(vy>velocity_lim[3]-velocity_margin)
	{
		velocity_lim[3] = vy+velocity_margin;
		velocity_lim[2] = velocity_lim[3]-velocity_height;
		ui.widget_v_y->yAxis->setRange(vy-(velocity_lim[3]-velocity_lim[2]+velocity_margin),vy+velocity_margin);
	}
	ui.widget_v_y->replot();

	ui.widget_v_z->graph(0)->addData(t_now,vz);
	ui.widget_v_z->xAxis->setRange(t_now,4, Qt::AlignRight);
	if(vz<velocity_lim[4]+velocity_margin)
	{
		velocity_lim[4] = vz-velocity_margin;
		velocity_lim[5] = velocity_lim[4]+velocity_height;
		ui.widget_v_z->yAxis->setRange(vz-velocity_margin,vz+(velocity_lim[5]-velocity_lim[4])-velocity_margin);
	}
	else if(vz>velocity_lim[5]-velocity_margin)
	{
		velocity_lim[5] = vz+velocity_margin;
		velocity_lim[4] = velocity_lim[5]-velocity_height;
		ui.widget_v_z->yAxis->setRange(vz-(velocity_lim[5]-velocity_lim[4])+velocity_margin,vz+velocity_margin);
	}
	ui.widget_v_z->replot();
}
void ICSL_GCS::set_attitude_data(double roll, double pitch, double yaw)
{
	double t_now = (ros::Time::now()-t_init).toSec();
	ui.widget_roll->graph(0)->addData(t_now,roll);
	ui.widget_roll->xAxis->setRange(t_now,4, Qt::AlignRight);
	if(roll<attitude_lim[0]+attitude_margin)
	{
		attitude_lim[0] = roll-attitude_margin;
		attitude_lim[1] = attitude_lim[0]+attitude_height;
		ui.widget_roll->yAxis->setRange(roll-attitude_margin,roll+(attitude_lim[1]-attitude_lim[0])-attitude_margin);
	}
	else if(roll>attitude_lim[1]-attitude_margin)
	{
		attitude_lim[1] = roll+attitude_margin;
		attitude_lim[0] = attitude_lim[1]-attitude_height;
		ui.widget_roll->yAxis->setRange(roll-(attitude_lim[1]-attitude_lim[0])+attitude_margin,roll+attitude_margin);
	}
	ui.widget_roll->replot();

	ui.widget_pitch->graph(0)->addData(t_now,pitch);
	ui.widget_pitch->xAxis->setRange(t_now,4, Qt::AlignRight);
	if(pitch<attitude_lim[2]+attitude_margin)
	{
		attitude_lim[2] = pitch-attitude_margin;
		attitude_lim[3] = attitude_lim[2]+attitude_height;
		ui.widget_pitch->yAxis->setRange(pitch-attitude_margin,pitch+(attitude_lim[3]-attitude_lim[2])-attitude_margin);
	}
	else if(pitch>attitude_lim[3]-attitude_margin)
	{
		attitude_lim[3] = pitch+attitude_margin;
		attitude_lim[2] = attitude_lim[3]-attitude_height;
		ui.widget_pitch->yAxis->setRange(pitch-(attitude_lim[3]-attitude_lim[2])+attitude_margin,pitch+attitude_margin);
	}
	ui.widget_pitch->replot();

	ui.widget_yaw->graph(0)->addData(t_now,yaw);
	ui.widget_yaw->xAxis->setRange(t_now,4, Qt::AlignRight);
	if(yaw<attitude_lim[4]+attitude_margin)
	{
		attitude_lim[4] = yaw-attitude_margin;
		attitude_lim[5] = attitude_lim[4]+attitude_height;
		ui.widget_yaw->yAxis->setRange(yaw-attitude_margin,yaw+(attitude_lim[5]-attitude_lim[4])-attitude_margin);
	}
	else if(yaw>attitude_lim[5]-attitude_margin)
	{
		attitude_lim[5] = yaw+attitude_margin;
		attitude_lim[4] = attitude_lim[5]-attitude_height;
		ui.widget_yaw->yAxis->setRange(yaw-(attitude_lim[5]-attitude_lim[4])+attitude_margin,yaw+attitude_margin);
	}
	ui.widget_yaw->replot();
}
void ICSL_GCS::set_angular_velocity_data(double p, double q, double r)
{
	double t_now = (ros::Time::now()-t_init).toSec();
	ui.widget_p->graph(0)->addData(t_now,p);
	ui.widget_p->xAxis->setRange(t_now,4, Qt::AlignRight);
	if(p<angular_velocity_lim[0]+angular_velocity_margin)
	{
		angular_velocity_lim[0] = p-angular_velocity_margin;
		angular_velocity_lim[1] = angular_velocity_lim[0]+angular_velocity_height;
		ui.widget_p->yAxis->setRange(p-angular_velocity_margin,p+(angular_velocity_lim[1]-angular_velocity_lim[0])-angular_velocity_margin);
	}
	else if(p>angular_velocity_lim[1]-angular_velocity_margin)
	{
		angular_velocity_lim[1] = p+angular_velocity_margin;
		angular_velocity_lim[0] = angular_velocity_lim[1]-angular_velocity_height;
		ui.widget_p->yAxis->setRange(p-(angular_velocity_lim[1]-angular_velocity_lim[0])+angular_velocity_margin,p+angular_velocity_margin);
	}
	ui.widget_p->replot();

	ui.widget_q->graph(0)->addData(t_now,q);
	ui.widget_q->xAxis->setRange(t_now,4, Qt::AlignRight);
	if(q<angular_velocity_lim[2]+angular_velocity_margin)
	{
		angular_velocity_lim[2] = q-angular_velocity_margin;
		angular_velocity_lim[3] = angular_velocity_lim[2]+angular_velocity_height;
		ui.widget_q->yAxis->setRange(q-angular_velocity_margin,q+(angular_velocity_lim[3]-angular_velocity_lim[2])-angular_velocity_margin);
	}
	else if(q>angular_velocity_lim[3]-angular_velocity_margin)
	{
		angular_velocity_lim[3] = q+angular_velocity_margin;
		angular_velocity_lim[2] = angular_velocity_lim[3]-angular_velocity_height;
		ui.widget_q->yAxis->setRange(q-(angular_velocity_lim[3]-angular_velocity_lim[2])+angular_velocity_margin,q+angular_velocity_margin);
	}
	ui.widget_q->replot();

	ui.widget_r->graph(0)->addData(t_now,r);
	ui.widget_r->xAxis->setRange(t_now,4, Qt::AlignRight);
	if(r<angular_velocity_lim[4]+angular_velocity_margin)
	{
		angular_velocity_lim[4] = r-angular_velocity_margin;
		angular_velocity_lim[5] = angular_velocity_lim[4]+angular_velocity_height;
		ui.widget_r->yAxis->setRange(r-angular_velocity_margin,r+(angular_velocity_lim[5]-angular_velocity_lim[4])-angular_velocity_margin);
	}
	else if(r>angular_velocity_lim[5]-angular_velocity_margin)
	{
		angular_velocity_lim[5] = r+angular_velocity_margin;
		angular_velocity_lim[4] = angular_velocity_lim[5]-angular_velocity_height;
		ui.widget_r->yAxis->setRange(r-(angular_velocity_lim[5]-angular_velocity_lim[4])+angular_velocity_margin,r+angular_velocity_margin);
	}
	ui.widget_r->replot();
}

void ICSL_GCS::loadGains()
{
	bool successFlag = false;
	std::vector<float> gainValues = qnode.subscribeGains(gainNames, successFlag);
	if(successFlag)
	{
		ui.pushButton_get_gain->setStyleSheet("background-color: rgba(0,255,0,128);");
		showGains(gainValues);	
	}
	else	    	
	{
		ui.pushButton_get_gain->setStyleSheet("background-color: rgba(255,0,0,128);");
	}
}

void ICSL_GCS::showGains(std::vector<float> gainValues)
{
	ui.gain_1->setValue(gainValues[0]);
	ui.gain_2->setValue(gainValues[1]);
	ui.gain_3->setValue(gainValues[2]);
	ui.gain_4->setValue(gainValues[3]);
	ui.gain_5->setValue(gainValues[4]);
	ui.gain_6->setValue(gainValues[5]);
	ui.gain_7->setValue(gainValues[6]);
	ui.gain_8->setValue(gainValues[7]);
	ui.gain_9->setValue(gainValues[8]);
	ui.gain_10->setValue(gainValues[9]);
	ui.gain_11->setValue(gainValues[10]);
	ui.gain_12->setValue(gainValues[11]);
	ui.gain_13->setValue(gainValues[12]);
	ui.gain_14->setValue(gainValues[13]);
	ui.gain_15->setValue(gainValues[14]);
	ui.gain_16->setValue(gainValues[15]);
}

void ICSL_GCS::on_pushButton_set_gain_clicked()
{
	std::vector<float> gainValues;
	gainValues.resize(16);
	gainValues[0] = ui.gain_1->value();
	gainValues[1] = ui.gain_2->value();
	gainValues[2] = ui.gain_3->value();
	gainValues[3] = ui.gain_4->value();
	gainValues[4] = ui.gain_5->value();
	gainValues[5] = ui.gain_6->value();
	gainValues[6] = ui.gain_7->value();
	gainValues[7] = ui.gain_8->value();
	gainValues[8] = ui.gain_9->value();
	gainValues[9] = ui.gain_10->value();
	gainValues[10] = ui.gain_11->value();
	gainValues[11] = ui.gain_12->value();
	gainValues[12] = ui.gain_13->value();
	gainValues[13] = ui.gain_14->value();
	gainValues[14] = ui.gain_15->value();
	gainValues[15] = ui.gain_16->value();
	bool successFlag = qnode.sendGains(gainNames,gainValues);
	if(successFlag)	ui.pushButton_set_gain->setStyleSheet("background-color: rgba(0,255,0,128);");
	else	    	ui.pushButton_set_gain->setStyleSheet("background-color: rgba(255,0,0,128);");
}

void ICSL_GCS::on_pushButton_get_gain_clicked()
{
	loadGains();
}

}  // namespace px4_gcs

