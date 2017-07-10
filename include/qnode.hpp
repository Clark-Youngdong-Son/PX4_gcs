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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>

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
	std::vector<double> subscribeGains(std::vector<std::string>, bool &);
	bool publishGains(std::vector<std::string>, std::vector<double>);

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
	void log(const std::string &msg);

	//MAVROS
	ros::Time t_init;
	ros::Publisher chatter_publisher;
	ros::Subscriber state_subscriber;
	ros::Subscriber	lpe_pose_subscriber; 
	ros::Subscriber	lpe_twist_subscriber;
	ros::Subscriber sp_subscriber;
	ros::Subscriber rp_subscriber;
	ros::ServiceClient get_gain_client;
	ros::ServiceClient set_gain_client;

	void state_cb(const mavros_msgs::State::ConstPtr &);
	void lpe_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &);
	void lpe_twist_cb(const geometry_msgs::TwistStamped::ConstPtr &);
	void sp_cb(const mavros_msgs::PositionTarget::ConstPtr &);
	void rp_cb(const mavros_msgs::RollPitchTarget::ConstPtr &);

	geometry_msgs::PoseStamped lpe_pose;
	geometry_msgs::TwistStamped lpe_twist;
	mavros_msgs::PositionTarget sp;
	mavros_msgs::RollPitchTarget rp;

	bool initializationFlag;
	bool ROSConnectionFlag, ROSDisconnectionFlag;
	bool PX4ConnectionFlag, PX4DisconnectionFlag;
	double PX4StateTimer;

	bool lpePoseUpdateFlag, lpeTwistUpdateFlag, spUpdateFlag, rpUpdateFlag;

	mavros_msgs::ParamGet paramget_srv;
	mavros_msgs::ParamSet paramset_srv;
	double getGain(std::string);
	bool setGain(std::string, double);
};

void q2e(const double, const double, const double, const double, double&, double&, double&);
void q2e(const geometry_msgs::Quaternion, double&, double&, double&);

}  // namespace px4_gcs

#endif /* px4_gcs_QNODE_HPP_ */
