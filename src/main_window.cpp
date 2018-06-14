#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "main_window.hpp"

namespace px4_gcs {

using namespace Qt;

ICSL_GCS::ICSL_GCS(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent),
	qnode(argc,argv)
{
	ui.setupUi(this);

	setWindowIcon(QIcon(":/images/ICSL.png"));
    
	QObject::connect(&qnode, SIGNAL(ros_shutdown()), this, SLOT(close()));
	QObject::connect(&qnode, SIGNAL(emit_pushButton_connect_ros_color(bool)), 
						  this, SLOT(set_pushButton_connect_ros_color(bool))); 
	QObject::connect(&qnode, SIGNAL(emit_pushButton_connect_px4_color(bool)), 
						  this, SLOT(set_pushButton_connect_px4_color(bool))); 
	QObject::connect(&qnode, SIGNAL(emit_arming_state(bool)), 
						  this, SLOT(set_arming_state(bool)));
	QObject::connect(&qnode, SIGNAL(emit_flight_mode(const char*)), 
						  this, SLOT(set_flight_mode(const char*)));
	QObject::connect(&qnode, SIGNAL(emit_navigation_state(double*)), 
						  this, SLOT(set_navigation_state(double*))); 
	QObject::connect(&qnode, SIGNAL(emit_imu_state(double*)), 
						  this, SLOT(set_imu_state(double*))); 
	QObject::connect(&qnode, SIGNAL(emit_position_setpoint(double*, int, bool)), 
						  this, SLOT(set_position_setpoint(double*, int, bool))); 
	QObject::connect(&qnode, SIGNAL(emit_attitude_setpoint(double*, bool)), 
						  this, SLOT(set_attitude_setpoint(double*, bool))); 
	QObject::connect(&qnode, SIGNAL(emit_thrust_setpoint(double, bool)), 
						  this, SLOT(set_thrust_setpoint(double, bool))); 
	QObject::connect(&qnode, SIGNAL(emit_kill_switch_enabled(bool)), 
						  this, SLOT(set_kill_switch_enabled(bool))); 

	key_I = new QShortcut(Qt::Key_I, ui.centralwidget);
	key_P = new QShortcut(Qt::Key_P, ui.centralwidget);
	key_O = new QShortcut(Qt::Key_O, ui.centralwidget);
	key_A = new QShortcut(Qt::Key_A, ui.centralwidget);
	key_D = new QShortcut(Qt::Key_D, ui.centralwidget);
	key_W = new QShortcut(Qt::Key_W, ui.centralwidget);
	key_S = new QShortcut(Qt::Key_S, ui.centralwidget);
	key_Z = new QShortcut(Qt::Key_Z, ui.centralwidget);
	key_X = new QShortcut(Qt::Key_X, ui.centralwidget);
	key_C = new QShortcut(Qt::Key_C, ui.centralwidget);
	key_V = new QShortcut(Qt::Key_V, ui.centralwidget);
	key_Space = new QShortcut(Qt::Key_Space, ui.centralwidget);
	key_M = new QShortcut(Qt::Key_M, ui.centralwidget);
	key_COMMA = new QShortcut(Qt::Key_Comma, ui.centralwidget);
	key_N = new QShortcut(Qt::Key_N, ui.centralwidget);
	key_B = new QShortcut(Qt::Key_B, ui.centralwidget);
	
	QObject::connect( key_I, SIGNAL(activated()), this, SLOT(on_btn_I_pressed()) );
	QObject::connect( key_P, SIGNAL(activated()), this, SLOT(on_btn_P_pressed()) );
	QObject::connect( key_O, SIGNAL(activated()), this, SLOT(on_btn_O_pressed()) );
	QObject::connect( key_A, SIGNAL(activated()), this, SLOT(on_btn_A_pressed()) );
	QObject::connect( key_D, SIGNAL(activated()), this, SLOT(on_btn_D_pressed()) );
	QObject::connect( key_W, SIGNAL(activated()), this, SLOT(on_btn_W_pressed()) );
	QObject::connect( key_S, SIGNAL(activated()), this, SLOT(on_btn_S_pressed()) );
	QObject::connect( key_Z, SIGNAL(activated()), this, SLOT(on_btn_Z_pressed()) );
	QObject::connect( key_X, SIGNAL(activated()), this, SLOT(on_btn_X_pressed()) );
	QObject::connect( key_C, SIGNAL(activated()), this, SLOT(on_btn_C_pressed()) );
	QObject::connect( key_V, SIGNAL(activated()), this, SLOT(on_btn_V_pressed()) );
	QObject::connect( key_Space, SIGNAL(activated()), this, SLOT(on_btn_Space_pressed()) );
	QObject::connect( key_M, SIGNAL(activated()), this, SLOT(on_btn_M_pressed()) );
	QObject::connect( key_COMMA, SIGNAL(activated()), this, SLOT(on_btn_COMMA_pressed()) );
	QObject::connect( key_N, SIGNAL(activated()), this, SLOT(on_btn_N_pressed()) );
	QObject::connect( key_B, SIGNAL(activated()), this, SLOT(on_btn_B_pressed()) );

	setupGraph();
}

ICSL_GCS::~ICSL_GCS()
{
	delete key_I;
	delete key_P;
	delete key_O;
	delete key_A;
	delete key_D;
	delete key_W;
	delete key_S;
	delete key_Z;
	delete key_X;
	delete key_C;
	delete key_V;
	delete key_N;
	delete key_B;
}

/** pushButton **/
void ICSL_GCS::on_pushButton_arming_clicked()
{
	if( !strcmp( ui.pushButton_arming->text().toStdString().c_str(), "Armed" ) )
	{
		qnode.set_disarm();
	}
	else
	{
		qnode.set_arm();
	}
}
void ICSL_GCS::on_pushButton_flight_mode_clicked()
{
	if( !strcmp( ui.pushButton_flight_mode->text().toStdString().c_str(), "Offboard" ) )
	{	
		qnode.set_manual();
	}
	else
	{
		qnode.set_offboard();
	}
}

//void ICSL_GCS::on_btn_Space_pressed()
//{ 
//	qnode.emergency_stop(); 
//	ui.pushButton_emergency->setStyleSheet("background-color: rgba(255,0,0,128);");
//}

/** slots **/
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

void ICSL_GCS::set_arming_state(bool armed)
{
	if(armed)
	{
		ui.pushButton_arming->setStyleSheet("background-color: rgba(255,0,0,128);");
		ui.pushButton_arming->setText( QString("Armed") );
	}
	else
	{
		ui.pushButton_arming->setStyleSheet("background-color: rgba(0,255,0,128);");
		ui.pushButton_arming->setText( QString("Disarmed") );
	}
}
void ICSL_GCS::set_flight_mode(const char* mode)
{
	if( !strcmp(mode, "OFFBOARD") )
	{	// offboard
		ui.pushButton_flight_mode->setText( QString("Offboard") );
		ui.pushButton_flight_mode->setStyleSheet("background-color: rgba(255,0,0,128);");	
	}
	else
	{	// else
		ui.pushButton_flight_mode->setText( QString("Manual") );
		ui.pushButton_flight_mode->setStyleSheet("background-color: rgba(0,255,0,128);");
	}
}
void ICSL_GCS::set_navigation_state(double* buf)
{
	graph[0] ->draw(buf[0], buf[1],  0);
	graph[1] ->draw(buf[0], buf[2],  0);
	graph[2] ->draw(buf[0], buf[3],  0);
	graph[3] ->draw(buf[0], buf[4],  0);
	graph[4] ->draw(buf[0], buf[5],  0);
	graph[5] ->draw(buf[0], buf[6],  0);
	graph[6] ->draw(buf[0], buf[7],  0);
	graph[7] ->draw(buf[0], buf[8],  0);
	graph[8] ->draw(buf[0], buf[9],  0);
}
void ICSL_GCS::set_position_setpoint(double* buf, int mode, bool throttle)
{
	if( mode & HOLD )
	{
		graph[2]->draw(buf[0], buf[3], 1); // z
		graph[3]->draw(buf[0], buf[4], 1); // vx
		graph[4]->draw(buf[0], buf[5], 1); // vy
		if( throttle )
		{
			ui.pushButton_mode_hold->setStyleSheet("background-color: rgba(0,255,0,128);");
			ui.pushButton_mode_pos->setStyleSheet("background-color: rgba(255,255,255,128);");
			ui.pushButton_mode_vel->setStyleSheet("background-color: rgba(255,255,255,128);");
			ui.pushButton_mode_traj->setStyleSheet("background-color: rgba(255,255,255,128);");
		}
	}
	else if( mode & POSITION )
	{
		graph[0]->draw(buf[0], buf[1], 1); // x
		graph[1]->draw(buf[0], buf[2], 1); // y
		graph[2]->draw(buf[0], buf[3], 1); // z
		if( throttle )
		{
			ui.pushButton_mode_hold->setStyleSheet("background-color: rgba(255,255,255,128);");
			ui.pushButton_mode_pos->setStyleSheet("background-color: rgba(0,255,0,128);");
			ui.pushButton_mode_vel->setStyleSheet("background-color: rgba(255,255,255,128);");
			ui.pushButton_mode_traj->setStyleSheet("background-color: rgba(255,255,255,128);");
		}
	}
	else if( mode & VELOCITY )
	{
		graph[3]->draw(buf[0], buf[4], 1); // vx
		graph[4]->draw(buf[0], buf[5], 1); // vy
		graph[5]->draw(buf[0], buf[6], 1); // vz
		if( throttle )
		{
			ui.pushButton_mode_hold->setStyleSheet("background-color: rgba(255,255,255,128);");
			ui.pushButton_mode_pos->setStyleSheet("background-color: rgba(255,255,255,128);");
			ui.pushButton_mode_vel->setStyleSheet("background-color: rgba(0,255,0,128);");
			ui.pushButton_mode_traj->setStyleSheet("background-color: rgba(255,255,255,128);");
		}	
	}
	else if( mode & TRAJECTORY )
	{
		graph[0]->draw(buf[0], buf[1], 1); // x
		graph[1]->draw(buf[0], buf[2], 1); // y
		graph[2]->draw(buf[0], buf[3], 1); // z
		graph[3]->draw(buf[0], buf[4], 1); // vx
		graph[4]->draw(buf[0], buf[5], 1); // vy
		graph[5]->draw(buf[0], buf[6], 1); // vz
		if( throttle )
		{
			ui.pushButton_mode_hold->setStyleSheet("background-color: rgba(255,255,255,128);");
			ui.pushButton_mode_pos->setStyleSheet("background-color: rgba(255,255,255,128);");
			ui.pushButton_mode_vel->setStyleSheet("background-color: rgba(255,255,255,128);");
			ui.pushButton_mode_traj->setStyleSheet("background-color: rgba(0,255,0,128);");
		}
	}
	else
	{
		std::cout << "unrecognized mode 1" << std::endl;
	}

	if( mode & YAW )
	{
		graph[8]->draw(buf[0], buf[7], 1); // yaw
		if( throttle )
		{
			ui.pushButton_mode_yaw->setStyleSheet("background-color: rgba(0,255,0,128);");
			ui.pushButton_mode_yaw_rate->setStyleSheet("background-color: rgba(255,255,255,128);");
		}
	}
	else if( mode & YAWRATE )
	{
		std::cout << "this mode is currently not supported" << std::endl;
		if( throttle )
		{
			ui.pushButton_mode_yaw->setStyleSheet("background-color: rgba(255,255,255,128);");
			ui.pushButton_mode_yaw_rate->setStyleSheet("background-color: rgba(0,255,0,128);");
		}
	}
	else
	{
		std::cout << "unrecognized mode 2" << std::endl;
	}
}

void ICSL_GCS::set_attitude_setpoint(double* buf, bool throttle)
{
	if( throttle )
	{
		graph[6] ->draw(buf[0], buf[1], 1); // roll
		graph[7] ->draw(buf[0], buf[2], 1); // pitch
		graph[9] ->draw(buf[0], buf[3], 1); // wx
		graph[10]->draw(buf[0], buf[4], 1); // wy
		graph[11]->draw(buf[0], buf[5], 1); // wz
	}
}

void ICSL_GCS::set_thrust_setpoint(double buf, bool throttle)
{
	if( throttle )
	{
		ui.pushButton_thrust->setText(QString( std::to_string( (int)(buf*100.0) ).c_str() ));
	}
}

void ICSL_GCS::set_imu_state(double* buf)
{
	graph[6] ->draw(buf[0], buf[1], 2); // roll
	graph[7] ->draw(buf[0], buf[2], 2); // pitch
//	graph[8] ->draw(buf[0], buf[3], 2); // yaw
	graph[9] ->draw(buf[0], buf[4], 2); // wx
	graph[10]->draw(buf[0], buf[5], 2); // wy
	graph[11]->draw(buf[0], buf[6], 2); // wz
}

void ICSL_GCS::set_kill_switch_enabled(bool tf)
{
	if( tf )
	{
		ui.pushButton_kill_switch->setText( QString("Enabled") );
		ui.pushButton_kill_switch->setStyleSheet("background-color: rgba(0,255,0,128);");
	}
	else
	{
		ui.pushButton_kill_switch->setText( QString("Disabled") );
		ui.pushButton_kill_switch->setStyleSheet("background-color: rgba(255,0,0,128);");	
	}
}

/** etcs **/
void ICSL_GCS::setupGraph()
{
	graph[0] = new DrawingModule( ui.widget_p_x );
	graph[0]->setYLims(-0.5, 0.5);

	graph[1] = new DrawingModule( ui.widget_p_y );
	graph[1]->setYLims(-0.5, 0.5);

	graph[2] = new DrawingModule( ui.widget_p_z );
	graph[2]->setYLims(-0.5, 0.5);

	graph[3] = new DrawingModule( ui.widget_v_x );
	graph[3]->setYLims(-0.5, 0.5);
	
	graph[4] = new DrawingModule( ui.widget_v_y );
	graph[4]->setYLims(-0.5, 0.5);
	
	graph[5] = new DrawingModule( ui.widget_v_z );
	graph[5]->setYLims(-0.5, 0.5);
	
	graph[6] = new DrawingModule( ui.widget_roll );
	graph[6]->setYLims(-10.0, 10.0);
	
	graph[7] = new DrawingModule( ui.widget_pitch );
	graph[7]->setYLims(-10.0, 10.0);
	
	graph[8] = new DrawingModule( ui.widget_yaw );
	graph[8]->setYLims(-10.0, 10.0);
	
	graph[9] = new DrawingModule( ui.widget_p );
	graph[9]->setYLims(-30.0, 30.0);
	
	graph[10] = new DrawingModule( ui.widget_q );
	graph[10]->setYLims(-30.0, 30.0);
	
	graph[11] = new DrawingModule( ui.widget_r );
	graph[11]->setYLims(-30.0, 30.0);

	//graph[12] = new DrawingModule( ui.widget_f_x );
	//graph[12]->setYLims(-0.5, 0.5);
	//
	//graph[13] = new DrawingModule( ui.widget_f_y );
	//graph[13]->setYLims(-0.5, 0.5);
	//
	//graph[14] = new DrawingModule( ui.widget_f_z );
	//graph[14]->setYLims(-0.5, 0.5);
	//
	//graph[15] = new DrawingModule( ui.widget_z_x );
	//graph[15]->setYLims(-0.5, 0.5);
	//
	//graph[16] = new DrawingModule( ui.widget_z_y );
	//graph[16]->setYLims(-0.5, 0.5);
	//
	//graph[17] = new DrawingModule( ui.widget_z_z );
	//graph[17]->setYLims(-0.5, 0.5);
	//
	//graph[18] = new DrawingModule( ui.widget_z_roll );
	//graph[18]->setYLims(-10.0, 10.0);
	//
	//graph[19] = new DrawingModule( ui.widget_z_pitch );
	//graph[19]->setYLims(-10.0, 10.0);
	//
	//graph[20] = new DrawingModule( ui.widget_z_yaw );
	//graph[20]->setYLims(-10.0, 10.0);
	//
	//graph[21] = new DrawingModule( ui.widget_gps_x );
	//graph[21]->setYLims(-0.5, 0.5);
	//
	//graph[22] = new DrawingModule( ui.widget_gps_y );
	//graph[22]->setYLims(-0.5, 0.5);
	//
	//graph[23] = new DrawingModule( ui.widget_lidar );
	//graph[23]->setYLims(0.0, 0.5);
}

}  // namespace px4_gcs
