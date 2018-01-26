#ifndef px4_gcs_MAIN_WINDOW_H
#define px4_gcs_MAIN_WINDOW_H

#define QCUSTOMPLOT_USE_LIBRARY

#include <QtGui/QMainWindow>
#include <QtGui/QWidget>
#include <QObject>
#include <QShortcut>
#include "ui_simple_gcs.h"
#include "qnode.hpp"

#include "modules/drawing_module.h"
#include "control_modes.h"
#include "keyboard_handler.h"

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
		void set_arming_state(bool);
		void set_flight_mode(const char*);
		void set_navigation_state(double*);
		void set_imu_state(double*);
		void set_position_setpoint(double*, int, bool);
		void set_attitude_setpoint(double*, bool);
		void set_thrust_setpoint(double, bool);
		void set_kill_switch_enabled(bool);
		void set_keyinput(int);
		void set_window_title(const char*);

		//void on_btn_I_pressed(){ qnode.initialize_pos_setpoint(); }
		void on_btn_P_pressed(){ qnode.move_setpoint( 2, true ); }
		void on_btn_O_pressed(){ qnode.move_setpoint( 2, false ); }
		void on_btn_A_pressed(){ qnode.move_setpoint( 1, true ); }
		void on_btn_D_pressed(){ qnode.move_setpoint( 1, false ); }
		void on_btn_W_pressed(){ qnode.move_setpoint( 0, true ); }
		void on_btn_S_pressed(){ qnode.move_setpoint( 0, false ); }
		void on_btn_Z_pressed(){ qnode.move_setpoint( 3, true ); }
		void on_btn_X_pressed(){ qnode.move_setpoint( 3, false ); }
		void on_btn_C_pressed(){ qnode.start_control_service(); }
		void on_btn_V_pressed(){ qnode.stop_control_service(); }
		void on_btn_I_pressed(){ qnode.move_to_initial_traj(); }
		void on_btn_U_pressed(){ qnode.move_traj(); }
		void on_btn_Space_pressed();
		
		void on_pushButton_connect_ros_clicked(){ qnode.init(); }
		void on_pushButton_connect_px4_clicked(){ qnode.connect_px4(); }
		void on_pushButton_arming_clicked();
		void on_pushButton_flight_mode_clicked();
		void on_pushButton_mode_hold_clicked(){ qnode.set_ctrl_mode( HOLD ); }
		void on_pushButton_mode_pos_clicked(){ qnode.set_ctrl_mode( POSITION ); };
		void on_pushButton_mode_vel_clicked(){ qnode.set_ctrl_mode( VELOCITY ); };
		void on_pushButton_mode_traj_clicked(){ qnode.set_ctrl_mode( TRAJECTORY ); };
		void on_pushButton_mode_yaw_clicked(){ qnode.set_ctrl_mode( YAW ); };
		void on_pushButton_mode_yaw_rate_clicked(){ qnode.set_ctrl_mode( YAWRATE ); };

	private:
		Ui::ICSL_GCS ui;
		QNode qnode;
	
		DrawingModule* graph[25];
		
		QShortcut* key_I;
		QShortcut* key_U;
		QShortcut* key_P;
		QShortcut* key_O;
		QShortcut* key_A;
		QShortcut* key_D;
		QShortcut* key_W;
		QShortcut* key_S;
		QShortcut* key_Z;
		QShortcut* key_X;
		QShortcut* key_C;
		QShortcut* key_V;
		QShortcut* key_Space;

		void setupGraph();
};

}  // namespace px4_gcs

#endif // px4_gcs_MAIN_WINDOW_H
