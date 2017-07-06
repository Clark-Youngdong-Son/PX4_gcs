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
	PX4StateTimer(0.0),
	poseUpdateFlag(false),twistUpdateFlag(false),
	x(0.0),y(0.0),z(0.0),vx(0.0),vy(0.0),vz(0.0),roll(0.0),pitch(0.0),yaw(0.0),p(0.0),q(0.0),r(0.0)
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
			Q_EMIT pushButton_connect_ros_color(false);
			return false;
		}
		ROSConnectionFlag = true;
		Q_EMIT pushButton_connect_ros_color(true);
		log("Connected to ROS master");

		ros::start(); // explicitly needed since our nodehandle is going out of scope.
		ros::NodeHandle n;

		chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
		state_subscriber = n.subscribe<mavros_msgs::State>("mavros/state", 1, &QNode::state_cb, this);
		pose_subscriber = n.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, &QNode::pose_cb, this);
		twist_subscriber = n.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity", 1, &QNode::twist_cb, this);
		get_gain_client = n.serviceClient<mavros_msgs::ParamGet>("mavros/param/get");
		set_gain_client = n.serviceClient<mavros_msgs::ParamSet>("mavros/param/set");
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
		Q_EMIT pushButton_connect_ros_color(false);
		return false;
	}
	else
	{
		if(initializationFlag) log("Already connected to PX4");
		else
		{
			ros::spinOnce();
			if(PX4ConnectionFlag)
			{
				log("Connected to PX4");
				initializationFlag = true;
				Q_EMIT pushButton_connect_px4_color(true);
				Q_EMIT emit_initialization();
				return true;
			}
			else
			{
				log("Failed to connect PX4");
				Q_EMIT pushButton_connect_px4_color(false);
				return false;
			}
		}
	}
}

void QNode::run() {
	double rate = 30.0;
	ros::Rate loop_rate(rate);

	while ( ros::ok() )
	{
		ros::spinOnce();
		if(initializationFlag)
		{
			PX4StateTimer += 1.0/rate;
			if(PX4StateTimer>PX4_LOSS_TIME && !PX4DisconnectionFlag)
			{
				log("[Error] PX4 signal loss!");
				Q_EMIT pushButton_connect_px4_color(false);
				PX4DisconnectionFlag = true;
				PX4ConnectionFlag = false;
			}
			if(PX4DisconnectionFlag && PX4StateTimer<PX4_LOSS_TIME)
			{
				log("[Info] PX4 signal regained");
				Q_EMIT pushButton_connect_px4_color(true);
				PX4DisconnectionFlag = false;
			}

			if(poseUpdateFlag)
			{
				Q_EMIT emit_position_data(x,y,z);
				Q_EMIT emit_attitude_data(roll,pitch,yaw);
				poseUpdateFlag = false;
			}
			if(twistUpdateFlag)
			{
				Q_EMIT emit_velocity_data(vx,vy,vz);
				Q_EMIT emit_angular_velocity_data(p,q,r);
				poseUpdateFlag = false;
			}
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
	PX4ConnectionFlag = msg->connected;
	if(PX4ConnectionFlag) PX4StateTimer = 0.0;
}

void QNode::pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	x = msg->pose.position.x;
	y = msg->pose.position.y;
	z = msg->pose.position.z;
	double q1 = msg->pose.orientation.x;
	double q2 = msg->pose.orientation.y;
	double q3 = msg->pose.orientation.z;
	double q0 = msg->pose.orientation.w;

	roll = 180.0/3.14*atan2(2*(q2*q3+q0*q1),q0*q0-q1*q1-q2*q2+q3*q3);
	pitch = 180.0/3.14*atan2(-2*(q1*q3-q0*q2),sqrt((2*(q2*q3+q0*q1))*(2*(q2*q3+q0*q1))+(q0*q0-q1*q1-q2*q2+q3*q3)*(q0*q0-q1*q1-q2*q2+q3*q3)));
	yaw = 180.0/3.14*atan2(2*(q1*q2+q0*q3),q0*q0+q1*q1-q2*q2-q3*q3);
	poseUpdateFlag = true;
}

void QNode::twist_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
	vx = msg->twist.linear.x;
	vy = msg->twist.linear.y;
	vz = msg->twist.linear.z;
	p = 180.0/3.14*msg->twist.angular.x;
	q = 180.0/3.14*msg->twist.angular.y;
	r = 180.0/3.14*msg->twist.angular.z;
	twistUpdateFlag = true;
}

std::vector<float> QNode::subscribeGains(std::vector<std::string> gainNames, bool &successFlag)
{
	if(initializationFlag)
	{
		std::vector<float> gainValues;
		gainValues.resize(gainNames.size());
		successFlag = true;
		for(int i=0; i<gainNames.size(); i++)
		{
			float gain = getGain(gainNames[i]);
			if(gain==99.0f) successFlag = false;
			gainValues[i] = gain;
		}
		if(successFlag) log("Gain get finished");
		else log("Gain get failed");
		return gainValues;
	}
	else
	{
		log("Connect both ROS and PX4 first");
		successFlag = false;
		std::vector<float> empty;
		empty.resize(gainNames.size());
		return empty;
	}
}

float QNode::getGain(std::string gainName)
{
	paramget_srv.request.param_id = gainName;
	get_gain_client.call(paramget_srv);
	if(paramget_srv.response.success) return paramget_srv.response.value.real;
	else return 99.0f;
} 

bool QNode::sendGains(std::vector<std::string> gainNames, std::vector<float> gainValues)
{
	if(initializationFlag)
	{
		int data_num = gainNames.size();
		bool successFlag = true;
		for(int i=0; i<data_num; i++)
		{
			successFlag &= setGain(gainNames[i],gainValues[i]);
		}
		if(successFlag)
		{
			log("Gain set finished");
		}
		else
		{
			log("Gain set failed");
		}
		return successFlag;
	}
	else
	{
		log("Connect both ROS and PX4 first");
		return false;
	}
}

bool QNode::setGain(std::string gainName, float gainValue)
{
	paramset_srv.request.param_id = gainName;
	paramset_srv.request.value.real     = gainValue;
	set_gain_client.call(paramset_srv);
	if(paramset_srv.response.success) return true;
	else return false;
}

}  // namespace px4_gcs
