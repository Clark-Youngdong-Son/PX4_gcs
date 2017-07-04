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

#define PX4_LOSS_TIME 3

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
Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
	void pushButton_connect_ros_color(bool);
	void pushButton_connect_px4_color(bool);

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    QStringListModel logging_model;

	//MAVROS
	ros::Subscriber state_subscriber;
	void state_cb(const mavros_msgs::State::ConstPtr &msg);
	bool initializationFlag;
	bool ROSConnectionFlag, ROSDisconnectionFlag;
	bool PX4ConnectionFlag, PX4DisconnectionFlag;
	double PX4StateTimer;
};

}  // namespace px4_gcs

#endif /* px4_gcs_QNODE_HPP_ */
