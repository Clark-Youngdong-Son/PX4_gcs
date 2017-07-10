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
	lpePoseUpdateFlag(false),lpeTwistUpdateFlag(false),rpUpdateFlag(false),
	spInitializedFlag(false)
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
			Q_EMIT pushButton_connect_ros_color(false);
			return false;
		}
		ROSConnectionFlag = true;
		Q_EMIT pushButton_connect_ros_color(true);
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

		// Publication
		sp_publisher = n.advertise<mavros_msgs::PositionTarget>
			("mavros/setpoint_raw/local", 1);

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

			if(lpePoseUpdateFlag)
			{
				double x = lpe_pose.pose.position.x;
				double y = lpe_pose.pose.position.y;
				double z = lpe_pose.pose.position.z;
				double roll, pitch, yaw;
				q2e( lpe_pose.pose.orientation, roll, pitch, yaw);

				double t = (lpe_pose.header.stamp - t_init).toSec();

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
				
				double t = (lpe_twist.header.stamp - t_init).toSec();

				Q_EMIT emit_lpe_linear_velocity_data(vx,vy,vz,t);
				Q_EMIT emit_lpe_angular_velocity_data(p,q,r,t);
				lpeTwistUpdateFlag = false;
			}
			if(rpUpdateFlag)
			{
				double r_target = (180.0/3.14)*rp.roll_target;
				double p_target = -(180.0/3.14)*rp.pitch_target; //// coordination problem..

				double t = (rp.header.stamp - t_init).toSec();

				Q_EMIT emit_rp_target_data(r_target, p_target, t);
			}
			if(spInitializedFlag)
			{
				sp.header.stamp = ros::Time::now();
				sp_publisher.publish( sp );	

				double x = sp.position.x;
				double y = sp.position.y;
				double z = sp.position.z;
				double t = (sp.header.stamp - t_init).toSec();

				Q_EMIT emit_sp_position_data(x,y,z,t);
			}
		}
		loop_rate.sleep();
	}
	log("Ros shutdown, proceeding to close the gui");
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::state_cb(const mavros_msgs::State::ConstPtr &msg)
{
	current_state = *msg;

	PX4ConnectionFlag = msg->connected;
	if(PX4ConnectionFlag) PX4StateTimer = 0.0;
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

void QNode::initializeSetpoint()
{
	sp.coordinate_frame = sp.FRAME_LOCAL_NED;
	sp.type_mask = sp.IGNORE_AFX | sp.IGNORE_AFY | sp.IGNORE_AFZ;

	// position sp mode
	sp.position = lpe_pose.pose.position;
	sp.velocity.x = 0.0;
	sp.velocity.y = 0.0;
	sp.velocity.z = 0.0;
	sp.acceleration_or_force.x = 0.0;
	sp.acceleration_or_force.y = 0.0;
	sp.acceleration_or_force.z = 0.0;
	sp.yaw = -90.0*(3.14/180.0);

	spInitializedFlag = true;
}

std::vector<double> QNode::subscribeGains(std::vector<std::string> gainNames, bool &successFlag)
{
	if(initializationFlag)
	{
		successFlag = true;
		
		std::vector<double> _values;

		for(unsigned int i=0; i<gainNames.size(); i++)
		{
			double gain = getGain(gainNames[i]);
			if(gain==99.0f) successFlag = false;
				_values.push_back( gain );
		}

		if(successFlag) log("Gain get finished");
		else log("Gain get failed");
		return _values;
	}
	else
	{
		log("Connect both ROS and PX4 first");
		successFlag = false;
		std::vector<double> empty;
		return empty;
	}
}

double QNode::getGain(std::string gainName)
{
	paramget_srv.request.param_id = gainName;
	get_gain_client.call(paramget_srv);
	if(paramget_srv.response.success) return (double)paramget_srv.response.value.real;
	else return 99.0;
} 

bool QNode::publishGains(std::vector<std::string> gainNames, std::vector<double> gainValues)
{
	if(initializationFlag)
	{
		bool successFlag = true;
		
		for(unsigned int i=0; i<gainNames.size(); i++)
		{
			successFlag &= setGain(gainNames[i],gainValues[i]);
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
	paramset_srv.request.param_id 		= gainName;
	paramset_srv.request.value.real 	= (float)gainValue;
	set_gain_client.call(paramset_srv);
	if(paramset_srv.response.success) return true;
	else return false;
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
