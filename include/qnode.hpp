#ifndef px4_gcs_QNODE_HPP_
#define px4_gcs_QNODE_HPP_

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <QThread>

#include <mavros_msgs/State.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/RollPitchTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#define PX4_LOSS_TIME 2.0

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
	bool subscribeGains(const std::vector<std::string>, const std::vector<std::string>, 
						std::vector<double>& );
	bool publishGains(const std::vector<std::string>, const std::vector<std::string>,
						const std::vector<double>);

	// Keyboard interaction
	void setArm(); 
	void setDisarm();
	void setOffboard();
	void setManual();
	void increaseHeight(){ spInitializedFlag ? sp.position.z += 0.2 : sp.position.z += 0.0; }
	void decreaseHeight(){ spInitializedFlag ? sp.position.z -= 0.2 : sp.position.z -= 0.0; }
	void moveLeft(){ spInitializedFlag ? sp.position.y += 0.2 : sp.position.y += 0.0; }
	void moveRight(){ spInitializedFlag ? sp.position.y -= 0.2 : sp.position.y -= 0.0; }
	void moveForward(){ spInitializedFlag ? sp.position.x += 0.2 : sp.position.x += 0.0; }
	void moveBackward(){ spInitializedFlag ? sp.position.x -= 0.2 : sp.position.x -= 0.0; }
	void increaseHeightVel(){ spInitializedFlag ? sp.velocity.z += 0.1 : sp.velocity.z += 0.0; }
	void decreaseHeightVel(){ spInitializedFlag ? sp.velocity.z -= 0.1 : sp.velocity.z -= 0.0; }

Q_SIGNALS:
    void rosShutdown();
	void emit_pushButton_connect_ros_color(bool);
	void emit_pushButton_connect_px4_color(bool);
	void emit_initialization();
	void emit_log_message(const std::string&);
	void emit_lpe_position_data(double,double,double,double);
	void emit_lpe_linear_velocity_data(double,double,double,double);
	void emit_lpe_attitude_data(double,double,double,double);
	void emit_lpe_angular_velocity_data(double,double,double,double);
	void emit_sp_position_data(double,double,double,double);
	void emit_sp_velocity_data(double,double,double,double);
	void emit_rp_target_data(double,double,double);
	void emit_mocap_position_data(double,double,double,double);
	void emit_mocap_linear_velocity_data(double,double,double,double);
	void emit_mocap_attitude_data(double,double,double,double);
	void emit_mocap_angular_velocity_data(double,double,double,double);
	void emit_arming_state(bool);
	void emit_flight_mode( const char* );
	void emit_gps_local(double,double,double,double,double,double,double);
	void emit_gps_global(double,double,double,int,int,double);
	void emit_gps_comp_hdg(double,double);
	void emit_gps_rel_alt(double,double);
	void emit_gps_raw_vel(double,double,double,double);

private:
	int init_argc;
	char** init_argv;
	void log(const std::string &msg){ Q_EMIT emit_log_message( msg ); }
	double now(){ return (ros::Time::now() - t_init).toSec(); }

	//MAVROS
	ros::Time t_init;
	
	/** subscriber and callbacks **/
	ros::Subscriber state_subscriber; 			// connection status
	ros::Subscriber	lpe_pose_subscriber; 		// LPE state (6dof pose)
	ros::Subscriber	lpe_twist_subscriber; 		// LPE state (6dof twist)
	ros::Subscriber rp_subscriber;				// desired roll/pitch angles
	ros::Subscriber mocap_pos_subscriber;		// vicon position
	ros::Subscriber mocap_vel_subscriber;		// vicon velocity
	ros::Subscriber gps_local_subscriber;		// gps in local coordinate
	ros::Subscriber gps_global_subscriber;		// gps global (lat, lon, alt)
	ros::Subscriber gps_comp_hdg_subscriber;	// compass heading
	ros::Subscriber gps_rel_alt_subscriber; 	// relative altitude
	ros::Subscriber gps_raw_vel_subscriber; 	// gps raw velocity
	void state_cb(const mavros_msgs::State::ConstPtr &);
	void lpe_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &);
	void lpe_twist_cb(const geometry_msgs::TwistStamped::ConstPtr &);
	void rp_cb(const mavros_msgs::RollPitchTarget::ConstPtr &);
	void mocap_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &);
	void mocap_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &);
	void gps_local_cb(const nav_msgs::Odometry::ConstPtr &);
	void gps_global_cb(const sensor_msgs::NavSatFix::ConstPtr &);
	void gps_comp_hdg_cb(const std_msgs::Float64::ConstPtr &);
	void gps_rel_alt_cb(const std_msgs::Float64::ConstPtr &);
	void gps_raw_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &);
	mavros_msgs::State current_state;
	geometry_msgs::PoseStamped lpe_pose;
	geometry_msgs::TwistStamped lpe_twist;
	mavros_msgs::RollPitchTarget rp;
	geometry_msgs::PoseStamped mocap_pos;
	geometry_msgs::TwistStamped mocap_vel;
	nav_msgs::Odometry gps_local;
	sensor_msgs::NavSatFix gps_global;
	std_msgs::Float64 gps_comp_hdg;
	std_msgs::Float64 gps_rel_alt;
	geometry_msgs::TwistStamped gps_raw_vel;
	bool lpePoseUpdateFlag;
	bool lpeTwistUpdateFlag;
	bool rpUpdateFlag;
	bool mocapPosUpdateFlag;
	bool mocapVelUpdateFlag;
	bool gpsLocalUpdateFlag;
	bool gpsGlobalUpdateFlag;
	bool gpsCompHdgUpdateFlag;
	bool gpsRelAltUpdateFlag;
	bool gpsRawVelUpdateFlag;

	/** publisher **/
	ros::Publisher sp_publisher;			// setpoint raw
	mavros_msgs::PositionTarget sp;
	bool spInitializedFlag;
	
	/** service client **/
	ros::ServiceClient get_gain_client;		// get gain of FCU
	ros::ServiceClient set_gain_client;		// set gain with GCS value
	ros::ServiceClient arming_client;		// arm/disarm
	ros::ServiceClient set_mode_client;
	mavros_msgs::ParamSet paramset_srv;
	mavros_msgs::CommandBool arm_cmd;

	bool initializationFlag;
	bool ROSConnectionFlag, ROSDisconnectionFlag;
	bool PX4ConnectionFlag, PX4DisconnectionFlag;
	double PX4StateTimer;

	void initializeSetpoint();
	bool getGain(std::string, double&);
	bool getGain(std::string, int&);
	bool setGain(std::string, double);
	bool setGain(std::string, int);
	void setSpMaskPxy(bool);
};

void q2e(const double, const double, const double, const double, double&, double&, double&);
void q2e(const geometry_msgs::Quaternion, double&, double&, double&);

}  // namespace px4_gcs

#endif /* px4_gcs_QNODE_HPP_ */
