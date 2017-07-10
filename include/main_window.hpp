#ifndef px4_gcs_MAIN_WINDOW_H
#define px4_gcs_MAIN_WINDOW_H

#define QCUSTOMPLOT_USE_LIBRARY

#include <QtGui/QMainWindow>
#include "ui_icsl_gcs3.h"
#include "qnode.hpp"

#include "modules/drawing_module.h"
#include "modules/logging_module.h"
#include "modules/parameter_module.h"

namespace px4_gcs 
{

class ICSL_GCS : public QMainWindow 
{
	Q_OBJECT
	
	public:
		ICSL_GCS(int argc, char** argv, QWidget *parent = 0);
		~ICSL_GCS();
	
	public Q_SLOTS:
		void on_pushButton_connect_ros_clicked();
		void on_pushButton_connect_px4_clicked();
		void set_pushButton_connect_ros_color(bool);
		void set_pushButton_connect_px4_color(bool);
		void set_initialization();
	
		void set_log_message(const std::string&);
		void set_lpe_position_data(double,double,double,double);
		void set_lpe_linear_velocity_data(double,double,double,double);
		void set_lpe_attitude_data(double,double,double,double);
		void set_lpe_angular_velocity_data(double,double,double,double);
		void set_sp_position_data(double,double,double,double);
		void set_rp_target_data(double,double,double);

		void on_pushButton_set_gain_clicked();
		void on_pushButton_get_gain_clicked();
		void on_pushButton_load_gain_clicked();
		void on_pushButton_save_gain_clicked();

	private:
		Ui::ICSL_GCS ui;
		QNode qnode;
		bool initializationFlag;
	
		DrawingModule* graph[12];
		LoggingModule* logger;
		ParameterModule* param;

		void setupGraph();
};

}  // namespace px4_gcs

#endif // px4_gcs_MAIN_WINDOW_H
