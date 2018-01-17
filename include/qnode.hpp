#ifndef px4_gcs_QNODE_HPP_
#define px4_gcs_QNODE_HPP_

#include <ros/ros.h>
#include <ros/network.h>
#include <keyboard/Key.h>

#include <string>
#include <QThread>
#include <Eigen/Dense>

#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>

// for dji
#include <dji_sdk/DroneArmControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>

// for rate_controller
#include "control_modes.h"

#include <thread>

#define PX4_LOSS_TIME 2.0

enum PlatformCode
{
	PIXHAWK = 0,
	DJI = 1
};

enum NavigationCode
{
	VICON = 0,
	MSF = 1
};

namespace px4_gcs 
{

class QNode : public QThread 
{

Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();

	//MAVROS
	bool connect_px4();
	void set_arm(); 
	void set_disarm();
	void set_offboard();
	void set_manual();
	void set_ctrl_mode( ControlModes mode );
	
	// Keyboard interaction
	void move_setpoint(int, bool);
	void emergency_stop();

	void initialize_pos_setpoint();
	void start_control_service();
	void stop_control_service();

Q_SIGNALS:
    // quit
	void ros_shutdown();
	
	// mavros
	void emit_pushButton_connect_ros_color(bool);
	void emit_pushButton_connect_px4_color(bool);
	void emit_arming_state(bool);
	void emit_flight_mode( const char* );
	void emit_navigation_state(double*);
	void emit_imu_state(double*);
	void emit_position_setpoint(double*, int, bool);
	void emit_attitude_setpoint(double*, bool);
	void emit_thrust_setpoint(double, bool);
	void emit_kill_switch_enabled(bool);
	void emit_keyinput(int);

private:
	int init_argc;
	char** init_argv;
	double now(){ return (ros::Time::now() - t_init_).toSec(); }

	ros::Time t_init_;

	/** subscriber and callbacks **/
	ros::Subscriber sub_[10];
	void px4_state_cb(const mavros_msgs::State::ConstPtr &);
	void rc_in_cb(const mavros_msgs::RCIn::ConstPtr &);
	void att_sp_cb(const mavros_msgs::AttitudeTarget::ConstPtr &);
	void odom_cb(const nav_msgs::Odometry::ConstPtr &);
	void imu_cb(const sensor_msgs::Imu::ConstPtr &);
	void key_cb(const keyboard::Key::ConstPtr &);

	/** current states **/
	mavros_msgs::State px4_state_;
	nav_msgs::Odometry odom_;
	
	/** publisher **/
	ros::Publisher pub_[2];
	mavros_msgs::PositionTarget pos_sp_;
	
	/** service client **/
	ros::ServiceClient srv_client_[4];
	// 0 : arming, 1 : flight mode change (get authority),
	// 2 : start control, 3 : stop control

	/** flags **/
	bool init_flag_ = false;
	bool ros_flag_ = false;
	bool px4_flag_ = false; // true if temporal treatment for DJI platforms
	bool px4_signal_loss_ = false;
	bool init_pos_sp_ = false;
	double px4_timer_ = 0.0;
	bool emergency_stop_ = false;
	bool control_flag_ = false;

	void override_kill_switch();

	PlatformCode platform_code_;
	bool is_pixhawk(){ return platform_code_ == PIXHAWK; }
	bool is_dji(){ return platform_code_ == DJI; }

	NavigationCode navigation_code_;
	bool is_vicon(){ return navigation_code_ == VICON; }
	bool is_msf(){ return navigation_code_ == MSF; }
};

// utility functions
void q2e(const double, const double, const double, const double, double&, double&, double&);
void q2e(const geometry_msgs::Quaternion, double&, double&, double&);

}  // namespace px4_gcs

#endif /* px4_gcs_QNODE_HPP_ */
