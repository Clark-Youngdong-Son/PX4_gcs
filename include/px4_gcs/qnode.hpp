/**
 * @file /include/px4_gcs/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef px4_gcs_QNODE_HPP_
#define px4_gcs_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#define PX4_LOSS_TIME 2.0

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace px4_gcs {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();

	QStringListModel* loggingModel() { return &logging_model; }
	void log(const std::string &msg);

	//MAVROS
	bool connect_px4();
	std::vector<float> subscribeGains(std::vector<std::string>);
Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
	void pushButton_connect_ros_color(bool);
	void pushButton_connect_px4_color(bool);
	void emit_initialization();
	void emit_position_data(double,double,double);
	void emit_velocity_data(double,double,double);
	void emit_attitude_data(double,double,double);
	void emit_angular_velocity_data(double,double,double);

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    QStringListModel logging_model;

	//MAVROS
	ros::Subscriber state_subscriber, pose_subscriber, twist_subscriber;
	void state_cb(const mavros_msgs::State::ConstPtr &);
	bool initializationFlag;
	bool ROSConnectionFlag, ROSDisconnectionFlag;
	bool PX4ConnectionFlag, PX4DisconnectionFlag;
	double PX4StateTimer;

	void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &);
	void twist_cb(const geometry_msgs::TwistStamped::ConstPtr &);
	bool poseUpdateFlag, twistUpdateFlag;
	double x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r;

	float getGain(std::string);
	//void setGain(std::string, double);
};

}  // namespace px4_gcs

#endif /* px4_gcs_QNODE_HPP_ */
