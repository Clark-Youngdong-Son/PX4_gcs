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
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

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
	void increaseHeight(){ spInitializedFlag ? sp.position.z += 0.2 : sp.position.z += 0.0; }
	void decreaseHeight(){ spInitializedFlag ? sp.position.z -= 0.2 : sp.position.z -= 0.0; }
	void moveLeft(){ spInitializedFlag ? sp.position.y += 0.2 : sp.position.y += 0.0; }
	void moveRight(){ spInitializedFlag ? sp.position.y -= 0.2 : sp.position.y -= 0.0; }
	void moveForward(){ spInitializedFlag ? sp.position.x += 0.2 : sp.position.x += 0.0; }
	void moveBackward(){ spInitializedFlag ? sp.position.x -= 0.2 : sp.position.x -= 0.0; }

Q_SIGNALS:
    void rosShutdown();
	void pushButton_connect_ros_color(bool);
	void pushButton_connect_px4_color(bool);
	void emit_initialization();
	void emit_log_message(const std::string&);
	void emit_lpe_position_data(double,double,double,double);
	void emit_lpe_linear_velocity_data(double,double,double,double);
	void emit_lpe_attitude_data(double,double,double,double);
	void emit_lpe_angular_velocity_data(double,double,double,double);
	void emit_sp_position_data(double,double,double,double);
	void emit_rp_target_data(double,double,double);

private:
	int init_argc;
	char** init_argv;
	void log(const std::string &msg){ Q_EMIT emit_log_message( msg ); }

	//MAVROS
	ros::Time t_init;
	
	/** subscriber and callbacks **/
	ros::Subscriber state_subscriber; 		// connection status
	ros::Subscriber	lpe_pose_subscriber; 	// LPE state (6dof pose)
	ros::Subscriber	lpe_twist_subscriber; 	// LPE state (6dof twist)
	ros::Subscriber rp_subscriber;			// desired roll/pitch angles
	void state_cb(const mavros_msgs::State::ConstPtr &);
	void lpe_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &);
	void lpe_twist_cb(const geometry_msgs::TwistStamped::ConstPtr &);
	void rp_cb(const mavros_msgs::RollPitchTarget::ConstPtr &);
	mavros_msgs::State current_state;
	geometry_msgs::PoseStamped lpe_pose;
	geometry_msgs::TwistStamped lpe_twist;
	mavros_msgs::RollPitchTarget rp;

	/** publisher **/
	ros::Publisher sp_publisher;			// setpoint raw
	mavros_msgs::PositionTarget sp;
	
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

	bool lpePoseUpdateFlag, lpeTwistUpdateFlag, rpUpdateFlag;
	bool spInitializedFlag;

	void initializeSetpoint();
	bool getGain(std::string, double&);
	bool getGain(std::string, int&);
	bool setGain(std::string, double);
	bool setGain(std::string, int);
};

void q2e(const double, const double, const double, const double, double&, double&, double&);
void q2e(const geometry_msgs::Quaternion, double&, double&, double&);

}  // namespace px4_gcs

#endif /* px4_gcs_QNODE_HPP_ */
