#ifndef px4_gcs_MAIN_WINDOW_H
#define px4_gcs_MAIN_WINDOW_H

#define QCUSTOMPLOT_USE_LIBRARY

#include <QtGui/QMainWindow>
#include "ui_icsl_gcs4.h"
#include "qnode.hpp"

#include "modules/drawing_module.h"
#include "modules/logging_module.h"
#include "modules/parameter_module.h"
#include "modules/keyboard_module.h"

namespace px4_gcs 
{

class ICSL_GCS : public QMainWindow 
{
	Q_OBJECT
	
	public:
		ICSL_GCS(int argc, char** argv, QWidget *parent = 0);
		~ICSL_GCS();
	
	public Q_SLOTS:
		void set_pushButton_connect_ros_color(bool);
		void set_pushButton_connect_px4_color(bool);
		void set_initialization();
		void set_log_message(const std::string&);
		void set_lpe_position_data(double,double,double,double);
		void set_lpe_linear_velocity_data(double,double,double,double);
		void set_lpe_attitude_data(double,double,double,double);
		void set_lpe_angular_velocity_data(double,double,double,double);
		void set_mocap_position_data(double,double,double,double);
		void set_mocap_linear_velocity_data(double,double,double,double);
		void set_mocap_attitude_data(double,double,double,double);
		void set_mocap_angular_velocity_data(double,double,double,double);
		void set_sp_position_data(double,double,double,double);
		void set_sp_velocity_data(double,double,double,double);
		void set_rp_target_data(double,double,double);
		void set_arming_state(bool);
		void set_flight_mode(const char*);
		void set_gps_local(double,double,double,double,double,double,double);
		void set_gps_global(double,double,double,int,int,double);
		void set_gps_comp_hdg(double,double);
		void set_gps_rel_alt(double,double);
		void set_gps_raw_vel(double,double,double,double);

		void on_pushButton_connect_ros_clicked();
		void on_pushButton_connect_px4_clicked();
		void on_pushButton_set_gain_clicked();
		void on_pushButton_get_gain_clicked();
		void on_pushButton_load_gain_clicked();
		void on_pushButton_save_gain_clicked();
		void on_pushButton_arming_clicked();
		void on_pushButton_flight_mode_clicked();

//		void on_ignore_pxy_stateChanged(int);

		// keyboard interaction
		void on_btn_O_pressed(){ qnode.decreaseHeight(); }
		void on_btn_P_pressed(){ qnode.increaseHeight(); }
		//void on_btn_O_pressed(){ qnode.decreaseHeightVel(); }
		//void on_btn_P_pressed(){ qnode.increaseHeightVel(); }
		void on_btn_A_pressed(){ qnode.moveLeft(); }
		void on_btn_D_pressed(){ qnode.moveRight(); }
		void on_btn_W_pressed(){ qnode.moveForward(); }
		void on_btn_S_pressed(){ qnode.moveBackward(); }
		//void on_btn_Z_pressed(){ qnode.setArm(); }
		//void on_btn_Space_pressed(){ qnode.setDisarm(); }
		//void on_btn_X_pressed(){ qnode.setOffboard(); }

	private:
		Ui::ICSL_GCS ui;
		QNode qnode;
		bool initializationFlag;
	
		DrawingModule* graph[25];
		LoggingModule* logger;
		ParameterModule* param;
		KeyboardModule* keyboard;

		void setupGraph();
		void log( const std::string msg ){ logger->log(msg); }
};

}  // namespace px4_gcs

#endif // px4_gcs_MAIN_WINDOW_H
