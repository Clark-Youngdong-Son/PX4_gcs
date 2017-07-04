/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/px4_gcs/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace px4_gcs {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv),
	
	PX4ConnectionFlag(false), ROSConnectionFlag(false),
	PX4DisconnectionFlag(false), ROSDisconnectionFlag(false),
	initializationFlag(false),
	PX4StateTimer(0.0)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init()
{
	if(!ROSConnectionFlag)
	{
		ros::init(init_argc,init_argv,"px4_gcs");
		if (!ros::master::check())
		{
			log("Failed to connect ROS master");
			return false;
		}
		ROSConnectionFlag = true;
		log("Connected to ROS master");

		ros::start(); // explicitly needed since our nodehandle is going out of scope.
		ros::NodeHandle n;

		chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
		state_subscriber = n.subscribe<mavros_msgs::State>("mavros/state", 1, &QNode::state_cb, this);
		start();
		return true;
	}
	else
	{
		log("Already Connected to ROS master");
	}
}

bool QNode::connect_px4()
{
	if(!ROSConnectionFlag)
	{
		log("Connect ROS master before connecting PX4");
		return false;
	}
	else
	{
		ros::spinOnce();
		if(PX4ConnectionFlag)
		{
			log("Connected to PX4");
			initializationFlag = true;
			return true;
		}
		else
		{
			log("Failed to connect PX4");
			return false;
		}
	}
}

void QNode::run() {
	double rate = 30.0;
	ros::Rate loop_rate(rate);

	while ( ros::ok() )
	{
		ros::spinOnce();

		PX4StateTimer += 1.0/rate;
		if(PX4StateTimer>PX4_LOSS_TIME && !PX4DisconnectionFlag)
		{
			log("[Error] PX4 signal loss!");
			PX4DisconnectionFlag = true;
		}
		if(PX4DisconnectionFlag && PX4StateTimer<PX4_LOSS_TIME)
		{
			log("[Info] PX4 signal regained");
			PX4DisconnectionFlag = false;
		}
		loop_rate.sleep();
	}
	log("Ros shutdown, proceeding to close the gui");
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log(const std::string &msg)
{
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	logging_model_msg<<msg;
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::state_cb(const mavros_msgs::State::ConstPtr &msg)
{
	PX4StateTimer = 0.0;
	PX4ConnectionFlag = msg->connected;
}

}  // namespace px4_gcs
