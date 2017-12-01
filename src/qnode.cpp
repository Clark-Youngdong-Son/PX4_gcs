#include "qnode.hpp"

namespace px4_gcs 
{

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv),	
	PX4ConnectionFlag(false), ROSConnectionFlag(false),
	PX4DisconnectionFlag(false), ROSDisconnectionFlag(false),
	initializationFlag(false),
	PX4StateTimer(0.0),
	lpePoseUpdateFlag(false),
	lpeTwistUpdateFlag(false),
	rpUpdateFlag(false),
	mocapPosUpdateFlag(false),
	mocapVelUpdateFlag(false),
	gpsLocalUpdateFlag(false),
	gpsGlobalUpdateFlag(false),
	gpsCompHdgUpdateFlag(false),
	gpsRelAltUpdateFlag(false),
	gpsRawVelUpdateFlag(false),
	spInitializedFlag(false),
	mpcStartFlag(false),
	mpcGoFlag(false)
	{}

QNode::~QNode() 
{
    if(ros::isStarted()) 
	{
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
			Q_EMIT emit_pushButton_connect_ros_color(false);
			return false;
		}
		ROSConnectionFlag = true;
		Q_EMIT emit_pushButton_connect_ros_color(true);
		log("Connected to ROS master");

		ros::start(); // explicitly needed since our nodehandle is going out of scope.
		ros::NodeHandle n;

		t_init = ros::Time::now();

		// Subscription
		state_subscriber = n.subscribe<mavros_msgs::State>
			("mavros/state", 1, &QNode::state_cb, this);
		lpe_pose_subscriber = n.subscribe<geometry_msgs::PoseStamped>
			("mavros/local_position/pose", 1, &QNode::lpe_pose_cb, this);
		lpe_twist_subscriber = n.subscribe<geometry_msgs::TwistStamped>
			("mavros/local_position/velocity", 1, &QNode::lpe_twist_cb, this);
		rp_subscriber = n.subscribe<mavros_msgs::RollPitchTarget>
			("mavros/rp_target", 1, &QNode::rp_cb, this);
		mocap_pos_subscriber = n.subscribe<geometry_msgs::PoseStamped>
			("mavros/mocap/pose", 1, &QNode::mocap_pos_cb, this);
		mocap_vel_subscriber = n.subscribe<geometry_msgs::TwistStamped>
			("mavros/mocap/twist", 1, &QNode::mocap_vel_cb, this);	
		gps_local_subscriber = n.subscribe<nav_msgs::Odometry>
			("mavros/global_position/local", 1, &QNode::gps_local_cb, this);
		gps_global_subscriber = n.subscribe<sensor_msgs::NavSatFix>
			("mavros/global_position/global", 1, &QNode::gps_global_cb, this);
		gps_comp_hdg_subscriber = n.subscribe<std_msgs::Float64>
			("mavros/global_position/compass_hdg", 1, &QNode::gps_comp_hdg_cb, this);
		gps_rel_alt_subscriber = n.subscribe<std_msgs::Float64>
			("mavros/global_position/rel_alt", 1, &QNode::gps_rel_alt_cb, this);
		gps_raw_vel_subscriber = n.subscribe<geometry_msgs::TwistStamped>
			("mavros/global_position/raw/gps_vel", 1, &QNode::gps_raw_vel_cb, this);
		mpc_sp_subscriber = n.subscribe<mavros_msgs::PositionTarget>
			("mavros/setpoint_raw/local", 1, &QNode::mpc_sp_cb, this);
		mpc_sp_subscriber2 = n.subscribe<mavros_msgs::AttitudeTarget>
			("mavros/setpoint_raw/attitude", 1, &QNode::mpc_sp_cb2, this);

		// Publication
		sp_publisher = n.advertise<mavros_msgs::PositionTarget>
			//("mavros/setpoint_raw/local", 1);
			("gcs/position_setpoint", 1);
		mpc_publisher = n.advertise<keyboard::Key>
			("keyboard/keydown", 1);

		// Services
		get_gain_client = n.serviceClient<mavros_msgs::ParamGet>("mavros/param/get");
		set_gain_client = n.serviceClient<mavros_msgs::ParamSet>("mavros/param/set");
		arming_client = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
		set_mode_client = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

		start();
		return true;
	}
	else
	{
		log("Already Connected to ROS master");
	}
}

void QNode::setArm()
{
	bool success = false;

	if(PX4ConnectionFlag)
	{
		if(current_state.armed) // px4 is already armed.
		{
			log( "PX4 is already armed" );
			success = true;
		}
		else // px4 is disarmed, so try to arm.
		{
			arm_cmd.request.value = true;
			if( arming_client.call(arm_cmd) && arm_cmd.response.success )
			{
				log("Armed");
				initializeSetpoint();
				success = true;
			}
			else
			{
				log("Arming failed");
				success = false;
			}
		}
	}
	else
	{
		log("Connect PX4 first");
		success = false;
	}
}

void QNode::setDisarm()
{
	bool success = false;

	if(PX4ConnectionFlag)
	{
		if(!current_state.armed) // px4 is already armed.
		{
			log( "PX4 is already disarmed" );
			success = true;
		}
		else // px4 is disarmed, so try to arm.
		{
			arm_cmd.request.value = false;
			if( arming_client.call(arm_cmd) && arm_cmd.response.success )
			{
				log("Disarmed");
				success = true;
			}
			else
			{
				log("Disarming failed");
				success = false;
			}
		}
	}
	else
	{
		log("Connect PX4 first");
		success = false;
	}
}

void QNode::setOffboard()
{
	mavros_msgs::SetMode set_mode;
	set_mode.request.custom_mode = "OFFBOARD";

	if(set_mode_client.call(set_mode) && set_mode.response.success)
		log("Offboard mode enabled");
	else
		log("Offboard mode transition falied");
}

void QNode::setManual()
{
	mavros_msgs::SetMode set_mode;
	set_mode.request.custom_mode = "MANUAL";

	if(set_mode_client.call(set_mode) && set_mode.response.success)
		log("Manual mode enabled");
	else
		log("Manual mode transition falied");
}

void QNode::mpcSetting(bool flag)
{
	keyboard::Key key_msg;
	if(flag)
	{
		mpcStartFlag = true;
		key_msg.code = key_msg.KEY_m;
		mpc_publisher.publish(key_msg);
	}
	else
	{
		initializeSetpoint();
		mpcStartFlag = false;
		key_msg.code = key_msg.KEY_COMMA;
		mpc_publisher.publish(key_msg);
	}
}

void QNode::mpcInitialize()
{
	keyboard::Key key_msg;
	key_msg.code = key_msg.KEY_r;
	mpc_publisher.publish(key_msg);
}
void QNode::offsetSetting(bool flag)
{
	keyboard::Key key_msg;
	if(flag)
	{
		key_msg.code = key_msg.KEY_n;
		mpc_publisher.publish(key_msg);
	}
}

void QNode::offsetChange(int command)
{
	keyboard::Key key_msg;
	if(command==1)
	{
		key_msg.code = key_msg.KEY_i;
		mpc_publisher.publish(key_msg);
	}
	else if(command==2)
	{
		key_msg.code = key_msg.KEY_j;
		mpc_publisher.publish(key_msg);
	}
	else if(command==3)
	{
		key_msg.code = key_msg.KEY_k;
		mpc_publisher.publish(key_msg);
	}
	else if(command==4)
	{
		key_msg.code = key_msg.KEY_l;
		mpc_publisher.publish(key_msg);
	}
}

void QNode::changeFinal()
{
	keyboard::Key key_msg;
	key_msg.code = key_msg.KEY_t;
	mpc_publisher.publish(key_msg);
}

void QNode::mpcGo()
{
	mpcGoFlag = true;
	keyboard::Key key_msg;
	key_msg.code = key_msg.KEY_g;
	mpc_publisher.publish(key_msg);
}

bool QNode::connect_px4()
{
	if(!ROSConnectionFlag)
	{
		log("Connect ROS master before connecting PX4");
		Q_EMIT emit_pushButton_connect_ros_color(false);
		return false;
	}
	else
	{
		if(initializationFlag) 
			log("Already connected to PX4");
		else
		{
			ros::spinOnce();
			if(PX4ConnectionFlag)
			{
				log("Connected to PX4");
				initializationFlag = true;

				Q_EMIT emit_pushButton_connect_px4_color(true);
				Q_EMIT emit_initialization();
				return true;
			}
			else
			{
				log("Failed to connect PX4");
				Q_EMIT emit_pushButton_connect_px4_color(false);
				return false;
			}
		}
	}
}

void QNode::run() 
{
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
				Q_EMIT emit_pushButton_connect_px4_color(false);
				PX4DisconnectionFlag = true;
				PX4ConnectionFlag = false;
			}
			if(PX4DisconnectionFlag && PX4StateTimer<PX4_LOSS_TIME)
			{
				log("[Info] PX4 signal regained");
				Q_EMIT emit_pushButton_connect_px4_color(true);
				PX4DisconnectionFlag = false;
			}

			if(lpePoseUpdateFlag)
			{
				double x = lpe_pose.pose.position.x;
				double y = lpe_pose.pose.position.y;
				double z = lpe_pose.pose.position.z;
				double roll, pitch, yaw;
				q2e( lpe_pose.pose.orientation, roll, pitch, yaw);

				double t = (ros::Time::now() - t_init).toSec();

				Q_EMIT emit_lpe_position_data(x,y,z,t);
				Q_EMIT emit_lpe_attitude_data(roll,pitch,yaw,t);
				lpePoseUpdateFlag = false;
			}
			if(lpeTwistUpdateFlag)
			{
				double vx = lpe_twist.twist.linear.x;
				double vy = lpe_twist.twist.linear.y;
				double vz = lpe_twist.twist.linear.z;
				double p = 180.0/3.14*lpe_twist.twist.angular.x;
				double q = 180.0/3.14*lpe_twist.twist.angular.y;
				double r = 180.0/3.14*lpe_twist.twist.angular.z;
				
				double t = (ros::Time::now() - t_init).toSec();

				Q_EMIT emit_lpe_linear_velocity_data(vx,vy,vz,t);
				Q_EMIT emit_lpe_angular_velocity_data(p,q,r,t);
				lpeTwistUpdateFlag = false;
			}
			if(rpUpdateFlag)
			{
				//double r_target = (180.0/3.14)*rp.roll_target;
				//double p_target = -(180.0/3.14)*rp.pitch_target; //// coordination problem..

				//double t = (ros::Time::now() - t_init).toSec();

				//Q_EMIT emit_rp_target_data(r_target, p_target, t);
			}
			if(mocapPosUpdateFlag)
			{
				double x = mocap_pos.pose.position.x;
				double y = mocap_pos.pose.position.y;
				double z = mocap_pos.pose.position.z;
				double roll, pitch, yaw;
				q2e( mocap_pos.pose.orientation, roll, pitch, yaw);

				double t = (ros::Time::now() - t_init).toSec();

				Q_EMIT emit_mocap_position_data(x,y,z,t);
				//Q_EMIT emit_mocap_attitude_data(roll,pitch,yaw,t);
				mocapPosUpdateFlag = false;
			}
			if(mocapVelUpdateFlag)
			{
				double vx = mocap_vel.twist.linear.x;
				double vy = mocap_vel.twist.linear.y;
				double vz = mocap_vel.twist.linear.z;
				double p = 180.0/3.14*mocap_vel.twist.angular.x;
				double q = 180.0/3.14*mocap_vel.twist.angular.y;
				double r = 180.0/3.14*mocap_vel.twist.angular.z;	
				double t = (ros::Time::now() - t_init).toSec();

				Q_EMIT emit_mocap_linear_velocity_data(vx,vy,vz,t);
				//Q_EMIT emit_mocap_angular_velocity_data(p,q,r,t);
				mocapVelUpdateFlag = false;
			}
			if(gpsLocalUpdateFlag)
			{
				double x = gps_local.pose.pose.position.x;	
				double y = gps_local.pose.pose.position.y;	
				double z = gps_local.pose.pose.position.x;	
				double vx = gps_local.twist.twist.linear.x;	
				double vy = gps_local.twist.twist.linear.y;	
				double vz = gps_local.twist.twist.linear.z;	
				double t = (ros::Time::now() - t_init).toSec();
				Q_EMIT emit_gps_local(x,y,z,vx,vy,vz,t);
				gpsLocalUpdateFlag = false;
			}
			if(gpsGlobalUpdateFlag)
			{
				double lat = gps_global.latitude;
				double lon = gps_global.longitude;
				double alt = gps_global.altitude;
				int fix = gps_global.status.status;
				int service = gps_global.status.service;
				double t = now();
				Q_EMIT emit_gps_global(lat,lon,alt,fix,service,t);
				gpsGlobalUpdateFlag = false;
			}
			if(gpsCompHdgUpdateFlag)
			{
				double hdg = gps_comp_hdg.data;
				double t = now();
				Q_EMIT emit_gps_comp_hdg(hdg,t);
				gpsCompHdgUpdateFlag = false;
			}
			if(gpsRelAltUpdateFlag)
			{
				double rel_alt = gps_rel_alt.data;
				double t = now();
				Q_EMIT emit_gps_rel_alt(rel_alt,t);
				gpsRelAltUpdateFlag = false;
			}
			if(gpsRawVelUpdateFlag)
			{
				double vx = gps_raw_vel.twist.linear.x;
				double vy = gps_raw_vel.twist.linear.y;
				double vz = gps_raw_vel.twist.linear.z;
				double t = now();
				Q_EMIT emit_gps_raw_vel(vx,vy,vz,t);
				gpsRawVelUpdateFlag = false;
			}

			if(spInitializedFlag)
			{
				sp.header.stamp = ros::Time::now();
//				if(mpcGoFlag && mpcStartFlag)	sp.type_mask = sp.IGNORE_PX | sp.IGNORE_PY | sp.IGNORE_PZ;
				//else                sp.type_mask = 0;
				sp.type_mask = 0;
				if(!mpcStartFlag) sp_publisher.publish( sp );	

				double x = sp.position.x;
				double y = sp.position.y;
				double z = sp.position.z;
				double vx = sp.velocity.x;
				double vy = sp.velocity.y;
				double vz = sp.velocity.z;
				double t = (sp.header.stamp - t_init).toSec();

				Q_EMIT emit_sp_position_data(x,y,z,t);
				//Q_EMIT emit_sp_velocity_data(vx,vy,vz,t);
			}
		}
		loop_rate.sleep();
	}
	log("Ros shutdown, proceeding to close the gui");
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

/** subscription callbacks **/
void QNode::state_cb(const mavros_msgs::State::ConstPtr &msg)
{
	current_state = *msg;

	PX4ConnectionFlag = msg->connected;
	
	if(PX4ConnectionFlag)
	{
		PX4StateTimer = 0.0;
	}
	
	if(initializationFlag)
	{
		Q_EMIT emit_arming_state( current_state.armed );
		Q_EMIT emit_flight_mode( current_state.mode.c_str() );
	}
}

void QNode::lpe_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	lpe_pose = *msg;
	lpePoseUpdateFlag = true;
}

void QNode::lpe_twist_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
	lpe_twist = *msg;
	lpeTwistUpdateFlag = true;
}

void QNode::rp_cb(const mavros_msgs::RollPitchTarget::ConstPtr &msg)
{
	rp = *msg;
	rpUpdateFlag = true;
}

void QNode::mocap_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	mocap_pos = *msg;
	mocapPosUpdateFlag = true;
}

void QNode::mocap_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
	mocap_vel = *msg;
	mocapVelUpdateFlag = true;
}

void QNode::gps_local_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
	gps_local = *msg;
	gpsLocalUpdateFlag = true;
}

void QNode::gps_global_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
	gps_global = *msg;
	gpsGlobalUpdateFlag = true;
}

void QNode::gps_comp_hdg_cb(const std_msgs::Float64::ConstPtr &msg)
{
	gps_comp_hdg = *msg;
	gpsCompHdgUpdateFlag = true;
}

void QNode::gps_rel_alt_cb(const std_msgs::Float64::ConstPtr &msg)
{
	gps_rel_alt = *msg;
	gpsRelAltUpdateFlag = true;
}

void QNode::gps_raw_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
	gps_raw_vel = *msg;
	gpsRawVelUpdateFlag = true;
}

void QNode::mpc_sp_cb(const mavros_msgs::PositionTarget::ConstPtr &msg)
{
	if(mpcStartFlag)
	{
		double x = msg->position.x;
		double y = msg->position.y;
		double z = msg->position.z;
		double vx = msg->velocity.x;
		double vy = msg->velocity.y;
		double vz = msg->velocity.z;
		double t = (ros::Time::now() - t_init).toSec();
		Q_EMIT emit_sp_position_data(x,y,z,t);
		Q_EMIT emit_sp_velocity_data(vx,vy,vz,t);
	}
}

void QNode::mpc_sp_cb2(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
{
//	if(mpcStartFlag)
//	{
		//Attitude quarternion
		double q0 = msg->orientation.w;
		double q1 = msg->orientation.x;
		double q2 = msg->orientation.y;
		double q3 = msg->orientation.z;

		double roll = 180.0/3.14*atan2(2*(q0*q1 + q2*q3),1-2*(q1*q1+q2*q2));
		double pitch = 180.0/3.14*asin(2*(q0*q2-q3*q1));
		double yaw = 180.0/3.14*atan2(2*(q0*q3 + q1*q2),1-2*(q2*q2+q3*q3));

		//Attitude rates
		double p = 180.0/3.14*msg->body_rate.x;
		double q = 180.0/3.14*msg->body_rate.y;
		double r = 180.0/3.14*msg->body_rate.z;
		double t = (ros::Time::now() - t_init).toSec();
		Q_EMIT emit_pqr_target_data(p,q,r,t);
		//Q_EMIT emit_att_target_data(roll, pitch, yaw, t);
//	}
}

void QNode::initializeSetpoint()
{
	sp.coordinate_frame = sp.FRAME_LOCAL_NED;
	sp.header.frame_id = "fcu";
	sp.type_mask = sp.IGNORE_AFX | sp.IGNORE_AFY | sp.IGNORE_AFZ;
	//sp.type_mask = sp.IGNORE_PZ | sp.IGNORE_AFX | sp.IGNORE_AFY | sp.IGNORE_AFZ;

	// position sp mode
	sp.position = lpe_pose.pose.position;
	sp.velocity.x = 0.0;
	sp.velocity.y = 0.0;
	sp.velocity.z = 0.0;
	sp.acceleration_or_force.x = 0.0;
	sp.acceleration_or_force.y = 0.0;
	sp.acceleration_or_force.z = 0.0;
	sp.yaw = 90.0*(3.14/180.0);
	//sp.yaw = 0.0;

	spInitializedFlag = true;
}

bool QNode::subscribeGains(const std::vector<std::string> gainNames, 
						   const std::vector<std::string> gainTypes,
						   std::vector<double>& gainValues)
{
	gainValues.clear();

	if(initializationFlag)
	{
		for(unsigned int i=0; i<gainNames.size(); i++)
		{
			if( strcmp( gainTypes[i].c_str(), "double" ) ) // integer type
			{
				int _value;
				if( getGain(gainNames[i], _value) ) 
				{
					gainValues.push_back( (double)_value );
				}
			}
			else // double type
			{
				double _value;
				if( getGain(gainNames[i], _value) ) 
				{
					gainValues.push_back( _value );
				}

			}
		}

		if( gainValues.size() == gainNames.size() ) 
		{
			log("Gain get finished");
			return true;
		}
		else
		{
			log("Gain get failed");
			return false;
		}
	}
	else
	{
		log("Connect both ROS and PX4 first");
		return false;
	}
}

bool QNode::getGain(std::string gainName, double& value)
{
	mavros_msgs::ParamGet param; 
	param.request.param_id = gainName;
	get_gain_client.call( param );
	if(param.response.success)
	{
		value = (double)param.response.value.real;
		return true;
	}
	else
		return false;
} 

bool QNode::getGain(std::string gainName, int& value)
{
	mavros_msgs::ParamGet param; 
	param.request.param_id = gainName;
	get_gain_client.call( param );
	if(param.response.success)
	{
		value = (int)param.response.value.integer;
		return true;
	}
	else
		return false;
} 

bool QNode::publishGains(const std::vector<std::string> gainNames, 
						const std::vector<std::string> gainTypes,
						const std::vector<double> gainValues)
{
	if(initializationFlag)
	{
		bool successFlag = true;
		
		for(unsigned int i=0; i<gainNames.size(); i++)
		{
			if( strcmp(gainTypes[i].c_str(), "double") ) // integer
				successFlag &= setGain(gainNames[i], (int)gainValues[i]);
			else
				successFlag &= setGain(gainNames[i], gainValues[i]);
		}

		if(successFlag)
			log("Gain set finished");
		else
			log("Gain set failed");
		
		return successFlag;
	}
	else
	{
		log("Connect both ROS and PX4 first");
		return false;
	}
}

bool QNode::setGain(std::string gainName, double gainValue)
{
	mavros_msgs::ParamSet param;
	param.request.param_id 		= gainName;
	param.request.value.real 	= (float)gainValue;
	set_gain_client.call( param );
	if(param.response.success) 
		return true;
	else 
		return false;
}

bool QNode::setGain(std::string gainName, int gainValue)
{
	mavros_msgs::ParamSet param;
	param.request.param_id 		= gainName;
	param.request.value.integer = gainValue;
	set_gain_client.call( param );
	if(param.response.success) 
		return true;
	else 
		return false;
}

void setSpMaskPxy(bool tf)
{
	if(tf)
	{
	}
	else
	{
	}
}

void q2e(const double q0, const double q1, const double q2, const double q3, 
		double& roll, double& pitch, double& yaw)
{
	roll = 180.0/3.14*atan2(2*(q2*q3+q0*q1),q0*q0-q1*q1-q2*q2+q3*q3);
	pitch = 180.0/3.14*atan2(-2*(q1*q3-q0*q2),sqrt((2*(q2*q3+q0*q1))*(2*(q2*q3+q0*q1))+(q0*q0-q1*q1-q2*q2+q3*q3)*(q0*q0-q1*q1-q2*q2+q3*q3)));
	yaw = 180.0/3.14*atan2(2*(q1*q2+q0*q3),q0*q0+q1*q1-q2*q2-q3*q3);
}
void q2e(const geometry_msgs::Quaternion q, double& roll, double& pitch, double& yaw)
{
	double ysqr = q.y * q.y;

	double t0 = 2.0 * (q.w * q.x + q.y * q.z);
	double t1 = 1.0 - 2.0 * (q.x * q.x + ysqr);
	roll = (180.0/3.14)*std::atan2(t0, t1);

	double t2 = 2.0 * (q.w * q.y - q.z * q.x);
	t2 = ((t2 > 1.0) ? 1.0 : t2);
	t2 = ((t2 < -1.0) ? -1.0 : t2);
	pitch = (180.0/3.14)*std::asin(t2);

	double t3 = 2.0 * (q.w * q.z + q.x * q.y);
	double t4 = 1.0 - 2.0 * (ysqr + q.z * q.z);
	yaw = (180.0/3.14)*std::atan2(t3, t4);
}

}  // namespace px4_gcs
