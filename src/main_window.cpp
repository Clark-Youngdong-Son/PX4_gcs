#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "main_window.hpp"


namespace px4_gcs {

using namespace Qt;

ICSL_GCS::ICSL_GCS(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent),
	qnode(argc,argv),
	initializationFlag(false)
{
	ui.setupUi(this);

	setWindowIcon(QIcon(":/images/ICSL.png"));
    
	QObject::connect(&qnode, SIGNAL(rosShutdown()), 
					this, SLOT(close()));
	QObject::connect(&qnode, SIGNAL(emit_log_message(const std::string&)), 
					this, SLOT(set_log_message(const std::string&)));
	QObject::connect(&qnode, SIGNAL(emit_pushButton_connect_ros_color(bool)), 
					this, SLOT(set_pushButton_connect_ros_color(bool))); 
	QObject::connect(&qnode, SIGNAL(emit_pushButton_connect_px4_color(bool)), 
					this, SLOT(set_pushButton_connect_px4_color(bool))); 
	QObject::connect(&qnode, SIGNAL(emit_initialization()), 
					this, SLOT(set_initialization())); 
	QObject::connect(&qnode, SIGNAL(emit_lpe_position_data(double,double,double,double)), 
					this, SLOT(set_lpe_position_data(double,double,double,double))); 
	QObject::connect(&qnode, SIGNAL(emit_lpe_linear_velocity_data(double,double,double,double)), 
					this, SLOT(set_lpe_linear_velocity_data(double,double,double,double))); 
	QObject::connect(&qnode, SIGNAL(emit_lpe_attitude_data(double,double,double,double)), 
					this, SLOT(set_lpe_attitude_data(double,double,double,double))); 
	QObject::connect(&qnode, SIGNAL(emit_lpe_angular_velocity_data(double,double,double,double)), 
					this, SLOT(set_lpe_angular_velocity_data(double,double,double,double))); 
	QObject::connect(&qnode, SIGNAL(emit_mocap_position_data(double,double,double,double)), 
					this, SLOT(set_mocap_position_data(double,double,double,double))); 
	QObject::connect(&qnode, SIGNAL(emit_mocap_linear_velocity_data(double,double,double,double)), 
					this, SLOT(set_mocap_linear_velocity_data(double,double,double,double))); 
	QObject::connect(&qnode, SIGNAL(emit_mocap_attitude_data(double,double,double,double)), 
					this, SLOT(set_mocap_attitude_data(double,double,double,double))); 
	QObject::connect(&qnode, SIGNAL(emit_mocap_angular_velocity_data(double,double,double,double)), 				   this, SLOT(set_mocap_angular_velocity_data(double,double,double,double))); 
	QObject::connect(&qnode, SIGNAL(emit_sp_position_data(double,double,double,double)), 
					this, SLOT(set_sp_position_data(double,double,double,double))); 
	QObject::connect(&qnode, SIGNAL(emit_sp_velocity_data(double,double,double,double)), 
					this, SLOT(set_sp_velocity_data(double,double,double,double))); 
	QObject::connect(&qnode, SIGNAL(emit_rp_target_data(double,double,double)), 
					this, SLOT(set_rp_target_data(double,double,double))); 

	QObject::connect(&qnode, SIGNAL(emit_arming_state(bool)), 
					this, SLOT(set_arming_state(bool)));
	QObject::connect(&qnode, SIGNAL(emit_flight_mode(const char*)), 
					this, SLOT(set_flight_mode(const char*)));
	QObject::connect(&qnode, SIGNAL(emit_gps_local(double,double,double,double,double,double,double)), this, SLOT(set_gps_local(double,double,double,double,double,double,double)));
	QObject::connect(&qnode, SIGNAL(emit_gps_global(double,double,double,int,int,double)), 
			this, SLOT(set_gps_global(double,double,double,int,int,double)));
	QObject::connect(&qnode, SIGNAL(emit_gps_comp_hdg(double,double)), 
			this, SLOT(set_gps_comp_hdg(double,double)));
	QObject::connect(&qnode, SIGNAL(emit_gps_rel_alt(double,double)), 
			this, SLOT(set_gps_rel_alt(double,double)));
	QObject::connect(&qnode, SIGNAL(emit_gps_raw_vel(double,double,double,double)), 
			this, SLOT(set_gps_raw_vel(double,double,double,double)));

	logger = new LoggingModule( ui.view_logging );
	
	param = new ParameterModule();
	param->add( logger );
	param->add_widget( ui.tbl_1 );
	param->add_widget( ui.tbl_2 );
	param->add_widget( ui.tbl_3 );

	keyboard = new KeyboardModule( ui.centralwidget, this );
	keyboard->connect();

	setupGraph();
}

ICSL_GCS::~ICSL_GCS()
{
	delete logger;
	delete param;
	delete keyboard;
}

void ICSL_GCS::set_log_message(const std::string &msg)
{
	log( msg );
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
	initializationFlag = true;
}

void ICSL_GCS::set_lpe_position_data(double x, double y, double z, double t)
{
	graph[0]->draw(t, x, 0);
	graph[1]->draw(t, y, 0);
	graph[2]->draw(t, z, 0);
}

void ICSL_GCS::set_lpe_linear_velocity_data(double vx, double vy, double vz, double t)
{
	graph[3]->draw(t, vx, 0);
	graph[4]->draw(t, vy, 0);
	graph[5]->draw(t, vz, 0);
}
void ICSL_GCS::set_lpe_attitude_data(double roll, double pitch, double yaw, double t)
{
	graph[6]->draw(t, roll, 0);
	graph[7]->draw(t, pitch, 0);
	graph[8]->draw(t, yaw, 0);
}
void ICSL_GCS::set_lpe_angular_velocity_data(double p, double q, double r, double t)
{
	graph[9]->draw(t, p, 0);
	graph[10]->draw(t, q, 0);
	graph[11]->draw(t, r, 0);
}

void ICSL_GCS::set_mocap_position_data(double x, double y, double z, double t)
{
	graph[0]->draw(t, x, 2);
	graph[1]->draw(t, y, 2);
	graph[2]->draw(t, z, 2);
}
void ICSL_GCS::set_mocap_linear_velocity_data(double vx, double vy, double vz, double t)
{
	graph[3]->draw(t, vx, 2);
	graph[4]->draw(t, vy, 2);
	graph[5]->draw(t, vz, 2);
}
void ICSL_GCS::set_mocap_attitude_data(double roll, double pitch, double yaw, double t)
{
	graph[6]->draw(t, roll, 2);
	graph[7]->draw(t, pitch, 2);
	graph[8]->draw(t, yaw, 2);
}
void ICSL_GCS::set_mocap_angular_velocity_data(double p, double q, double r, double t)
{
	graph[9]->draw(t, p, 2);
	graph[10]->draw(t, q, 2);
	graph[11]->draw(t, r, 2);
}

void ICSL_GCS::set_sp_position_data(double x, double y, double z, double t)
{
	graph[0]->draw(t, x, 1);
	graph[1]->draw(t, y, 1);
	graph[2]->draw(t, z, 1);
}

void ICSL_GCS::set_sp_velocity_data(double x, double y, double z, double t)
{
//	graph[3]->draw(t, x, 1);
//	graph[4]->draw(t, y, 1);
//	graph[5]->draw(t, z, 1);
}

void ICSL_GCS::set_rp_target_data(double r_t, double p_t, double t)
{
	graph[6]->draw(t, r_t, 1);
	graph[7]->draw(t, p_t, 1);
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

void ICSL_GCS::set_gps_local(double x,double y,double z,double vx,double vy,double vz,double t)
{
	graph[12]->draw(t,x,0);
	graph[13]->draw(t,y,0);
	graph[14]->draw(t,z,0);
	graph[15]->draw(t,vx,0);
	graph[16]->draw(t,vy,0);
	graph[17]->draw(t,vz,0);
}

void ICSL_GCS::set_gps_global(double lat, double lon, double alt, int fix, int service, double t)
{
	graph[18]->draw(t,lat,0);
	graph[19]->draw(t,lon,0);
	graph[20]->draw(t,alt,0);
	graph[23]->draw(t,(double)fix,0);
	graph[24]->draw(t,(double)service,0);
}

void ICSL_GCS::set_gps_comp_hdg(double val, double t)
{
	graph[22]->draw(t,val,0);
}

void ICSL_GCS::set_gps_rel_alt(double val, double t)
{
	graph[21]->draw(t,val,0);
}

void ICSL_GCS::set_gps_raw_vel(double vx, double vy, double vz, double t)
{
	graph[15]->draw(t,vx,1);
	graph[16]->draw(t,vy,1);
	graph[17]->draw(t,vz,1);
}

void ICSL_GCS::on_pushButton_connect_ros_clicked()
{
	qnode.init();
}

void ICSL_GCS::on_pushButton_connect_px4_clicked()
{
	qnode.connect_px4();
}

void ICSL_GCS::on_pushButton_set_gain_clicked()
{
	std::vector<std::string> gainNames;
	std::vector<std::string> gainTypes;
	std::vector<double> gcsValues;
	param->query_list( gainNames, gainTypes, gcsValues );
	
	bool successFlag = qnode.publishGains(gainNames, gainTypes, gcsValues);
	if(successFlag)
	{
		// again, we have to subscribe fcu values in order to confirm.
		std::vector<double> fcuValues;
		bool assurance = qnode.subscribeGains(gainNames, gainTypes, fcuValues);

		if( assurance )
			param->update_fcu_values( gainNames, fcuValues );
	}

//	if(successFlag)	ui.pushButton_set_gain->setStyleSheet("background-color: rgba(0,255,0,128);");
//	else	    	ui.pushButton_set_gain->setStyleSheet("background-color: rgba(255,0,0,128);");
}

void ICSL_GCS::on_pushButton_get_gain_clicked()
{
	std::vector<std::string> gainNames;
	std::vector<std::string> gainTypes;
	std::vector<double> tmp;

	param->query_list( gainNames, gainTypes, tmp );

	std::vector<double> gainValues;	
	bool successFlag = qnode.subscribeGains( gainNames, gainTypes, gainValues );

	if( successFlag )
		param->update_fcu_values( gainNames, gainValues );

//	if(successFlag)
//		ui.pushButton_get_gain->setStyleSheet("background-color: rgba(0,255,0,128);");
//	else	    	
//		ui.pushButton_get_gain->setStyleSheet("background-color: rgba(255,0,0,128);");
}

void ICSL_GCS::on_pushButton_load_gain_clicked()
{
	std::string pwd = std::string(getenv("PX4_GCS_PATH"));
	QFileDialog fileDialog(this, "Load file", (pwd+"/params").c_str(), "Parameter files (*.xml)");
	fileDialog.setAcceptMode(QFileDialog::AcceptOpen);

	std::string filename;
	if( fileDialog.exec() )
	{
		filename = fileDialog.selectedFiles().front().toStdString();
		param->load( filename );
	}
	else
		return;
}

void ICSL_GCS::on_pushButton_save_gain_clicked()
{
	std::string pwd = std::string(getenv("PX4_GCS_PATH"));
	QFileDialog fileDialog(this, "Save file", (pwd+"/params").c_str(), "Parameter files (*.xml)");
	fileDialog.setAcceptMode(QFileDialog::AcceptSave);

	std::string filename;
	if( fileDialog.exec() )
	{
		filename = fileDialog.selectedFiles().front().toStdString();
		param->save( filename );
	}
	else
		return;
}

void ICSL_GCS::on_pushButton_arming_clicked()
{
	if( !strcmp( ui.pushButton_arming->text().toStdString().c_str(), "Armed" ) )
	{	// it is armed, so disarm
		qnode.setDisarm();
	}
	else
	{
		qnode.setArm();
	}
}

void ICSL_GCS::on_pushButton_flight_mode_clicked()
{
	if( !strcmp( ui.pushButton_flight_mode->text().toStdString().c_str(), "Offboard" ) )
	{	// it is offboard, so change to manual
		qnode.setManual();
	}
	else
	{
		qnode.setOffboard();
	}
//	log( ui.pushButton_flight_mode->text().toStdString() );
}

//void ICSL_GCS::on_ignore_pxy_stateChanged(int state)
//{
////	std::cout << state << std::endl;
//	if( state == 2 )
//		qnode.setSpMasPxy(true);
//	else
//		qnode.setSpMaskPxy(false);
//}

void ICSL_GCS::setupGraph()
{
	// graph settings, it would be more desirable if we can set these params at runtime
	graph[0] = new DrawingModule( ui.widget_p_x );
	graph[0]->setMargin(0.1);
	graph[0]->setYLims(-0.5, 0.5);

	graph[1] = new DrawingModule( ui.widget_p_y );
	graph[1]->setMargin(0.1);
	graph[1]->setYLims(-0.5, 0.5);

	graph[2] = new DrawingModule( ui.widget_p_z );
	graph[2]->setMargin(0.1);
	graph[2]->setYLims(-0.5, 0.5);

	graph[3] = new DrawingModule( ui.widget_v_x );
	graph[3]->setMargin(0.1);
	graph[3]->setYLims(-0.5, 0.5);
	
	graph[4] = new DrawingModule( ui.widget_v_y );
	graph[4]->setMargin(0.1);
	graph[4]->setYLims(-0.5, 0.5);
	
	graph[5] = new DrawingModule( ui.widget_v_z );
	graph[5]->setMargin(0.1);
	graph[5]->setYLims(-0.5, 0.5);
	
	graph[6] = new DrawingModule( ui.widget_roll );
	graph[6]->setMargin(2.0);
	graph[6]->setYLims(-10.0, 10.0);
	
	graph[7] = new DrawingModule( ui.widget_pitch );
	graph[7]->setMargin(2.0);
	graph[7]->setYLims(-10.0, 10.0);
	
	graph[8] = new DrawingModule( ui.widget_yaw );
	graph[8]->setMargin(2.0);
	graph[8]->setYLims(-10.0, 10.0);
	
	graph[9] = new DrawingModule( ui.widget_p );
	graph[9]->setMargin(6.0);
	graph[9]->setYLims(-30.0, 30.0);
	
	graph[10] = new DrawingModule( ui.widget_q );
	graph[10]->setMargin(6.0);
	graph[10]->setYLims(-30.0, 30.0);
	
	graph[11] = new DrawingModule( ui.widget_r );
	graph[11]->setMargin(6.0);
	graph[11]->setYLims(-30.0, 30.0);

	graph[12] = new DrawingModule( ui.widget_gps_local_x );
	graph[12]->setMargin(0.1);
	graph[12]->setYLims(-0.5, 0.5);
	
	graph[13] = new DrawingModule( ui.widget_gps_local_y );
	graph[13]->setMargin(0.1);
	graph[13]->setYLims(-0.5, 0.5);
	
	graph[14] = new DrawingModule( ui.widget_gps_local_z );
	graph[14]->setMargin(0.1);
	graph[14]->setYLims(-0.5, 0.5);
	
	graph[15] = new DrawingModule( ui.widget_gps_local_vx );
	graph[15]->setMargin(0.1);
	graph[15]->setYLims(-0.5, 0.5);
	
	graph[16] = new DrawingModule( ui.widget_gps_local_vy );
	graph[16]->setMargin(0.1);
	graph[16]->setYLims(-0.5, 0.5);
	
	graph[17] = new DrawingModule( ui.widget_gps_local_vz );
	graph[17]->setMargin(0.1);
	graph[17]->setYLims(-0.5, 0.5);
	
	graph[18] = new DrawingModule( ui.widget_gps_lat );
	graph[18]->setMargin(0.004);
	graph[18]->setYLims(-0.02, 0.02);
	
	graph[19] = new DrawingModule( ui.widget_gps_lon );
	graph[19]->setMargin(0.004);
	graph[19]->setYLims(-0.02, 0.02);
	
	graph[20] = new DrawingModule( ui.widget_gps_alt );
	graph[20]->setMargin(0.02);
	graph[20]->setYLims(-0.1, 0.1);
	
	graph[21] = new DrawingModule( ui.widget_gps_rel_alt );
	graph[21]->setMargin(0.02);
	graph[21]->setYLims(-0.1, 0.1);
	
	graph[22] = new DrawingModule( ui.widget_gps_comp_hdg );
	graph[22]->setMargin(2.0);
	graph[22]->setYLims(-10.0, 10.0);
	
	graph[23] = new DrawingModule( ui.widget_gps_fix );
	graph[23]->setMargin(0.2);
	graph[23]->setYLims(-1.0, 1.0);
	
	graph[24] = new DrawingModule( ui.widget_gps_service );
	graph[24]->setMargin(0.2);
	graph[24]->setYLims(-1.0, 1.0);
}


}  // namespace px4_gcs

