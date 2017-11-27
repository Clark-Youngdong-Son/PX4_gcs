#include "qnode.hpp"

namespace px4_gcs 
{

QNode::QNode(int argc, char** argv ) : init_argc(argc), init_argv(argv)
{
}

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
	if(!ros_flag_)
	{
		ros::init(init_argc,init_argv,"px4_gcs");
		if (!ros::master::check())
		{
			Q_EMIT emit_pushButton_connect_ros_color(false);
			return false;
		}
		ros_flag_ = true;

		ros::start(); // explicitly needed since our nodehandle is going out of scope.
		ros::NodeHandle n;
		
		// parse platform info
		if( n.getParam( (ros::this_node::getName()+"/platform").c_str(), platform_ ) )
		{
			if( platform_ == "pixhawk" )
			{
				#define GCS_PIXHAWK
				ROS_WARN("GCS for pixhawk");
			}
			else if( platform_ == "dji")
			{
				#define GCS_DJI
				ROS_WARN("GCS for dji");
			}
			else
			{
				ROS_FATAL("Platform must be either [pixhawk] or [dji], please re-launch");
				return false;
			}
		}
		else
		{
			ROS_FATAL("Platform must be defined, please re-launch");
			return false;
		}

		t_init_ = ros::Time::now();
		
		// Subscription
		sub_[1] = n.subscribe<AttSp>
			("mavros/setpoint_raw/attitude",10,&QNode::att_sp_cb,this);
		sub_[2] = n.subscribe<MSFState>
			("msf_core/odometry",10,&QNode::msf_state_cb,this);
		sub_[3] = n.subscribe<geometry_msgs::PoseStamped>
			("vicon/pose",10,&QNode::vicon_pos_cb,this);
		sub_[4] = n.subscribe<geometry_msgs::TwistStamped>
			("vicon/velocity",10,&QNode::vicon_vel_cb,this);
		
		//if(platform_ == "pixhawk")
		//{
#if defined(GCS_PIXHAWK)
			sub_[5] = n.subscribe<PX4State>
				("mavros/state",10,&QNode::px4_state_cb,this);
			sub_[6] = n.subscribe<RCIn>
				("mavros/rc/in",10,&QNode::rc_in_cb,this);
#elif defined(GCS_DJI)
		//}
		//else if(platform_ == "dji")
		//{
			sub_[5] = n.subscribe<sensor_msgs::Imu>
				("dji_sdk/imu", 10, &QNode::dji_att_cb, this);
#endif
		//}
		// Publication
		pub_[0] = n.advertise<PosSp>("gcs/setpoint_raw/position", 10);
#ifdef GCS_PIXHAWK
		pub_[1] = n.advertise<RCOverride>("mavros/rc/override", 10);
#endif

		// Services
#ifdef GCS_PIXHAWK
		//// pixhawk
		srv_client_[0] = n.serviceClient<mavros_msgs::CommandBool>
			("mavros/cmd/arming");
		srv_client_[1] = n.serviceClient<mavros_msgs::SetMode>
			("mavros/set_mode");
#endif
#ifdef GCS_DJI
		///// dji
		srv_client_[0] = n.serviceClient<dji_sdk::DroneArmControl>
			("dji_sdk/drone_arm_control");
		srv_client_[1] = n.serviceClient<dji_sdk::SDKControlAuthority>
			("dji_sdk/sdk_control_authority");
#endif
		//// rate controller
		srv_client_[2] = n.serviceClient<std_srvs::SetBool>
			("rate_controller/start");
		srv_client_[3] = n.serviceClient<std_srvs::SetBool>
			("rate_controller/stop");

		start();

		Q_EMIT emit_pushButton_connect_ros_color(true);
		return true;
	}
}

bool QNode::connect_px4()
{
#ifdef GCS_DJI
	px4_flag_ = true; // dji does not require px4 connection status
	ROS_INFO("SSIBAL");
#endif

	if(!ros_flag_)
	{
		Q_EMIT emit_pushButton_connect_ros_color(false);
		return false;
	}
	else
	{
		if(init_flag_)
		{
			if(px4_flag_)
			{
				Q_EMIT emit_pushButton_connect_px4_color(true);
				return true;
			}
			else
			{
				Q_EMIT emit_pushButton_connect_px4_color(false);
				return false;
			}
		}
		else
		{
			if(px4_flag_)
			{
				init_flag_ = true;
				Q_EMIT emit_pushButton_connect_px4_color(true);
				return true;
			}
			else
			{
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

	while( ros::ok() )
	{
		ros::spinOnce();
		if(init_flag_)
		{
#ifdef GCS_PIXHAWK
			px4_timer_ += 1.0/rate;
			if( (px4_timer_ > PX4_LOSS_TIME) && !px4_signal_loss_ )
			{
				Q_EMIT emit_pushButton_connect_px4_color(false);
				px4_flag_ = false;
				px4_signal_loss_ = true;
			}
			if( (px4_timer_ < PX4_LOSS_TIME) && px4_signal_loss_ )
			{
				Q_EMIT emit_pushButton_connect_px4_color(true);
				px4_signal_loss_ = false;
			}
#endif

			if( init_pos_sp_ )
			{	// publish setpoint
				pos_sp_.header.stamp = ros::Time::now();
				pub_[0].publish( pos_sp_ );	
				
				double* buf = (double*)malloc(9*sizeof(double));	
				buf[1] = pos_sp_.position.x;
				buf[2] = pos_sp_.position.y;
				buf[3] = pos_sp_.position.z;
				buf[4] = pos_sp_.velocity.x;
				buf[5] = pos_sp_.velocity.y;
				buf[6] = pos_sp_.velocity.z;
				buf[7] = pos_sp_.yaw*(180.0/3.14);
				buf[8] = pos_sp_.yaw_rate*(180.0/3.14);
				buf[0] = now();
				
				Q_EMIT emit_position_setpoint( buf, pos_sp_.type_mask, 
						(pos_sp_.header.seq % 30) == 0 );

				pos_sp_.header.seq = pos_sp_.header.seq + 1;
			}
		}
		loop_rate.sleep();
	}
	Q_EMIT ros_shutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::set_arm()
{
#ifdef GCS_PIXHAWK
	if(px4_flag_)
	{
		if(!px4_state_.armed) // px4 is disarmed, so try to arm.
		{
			mavros_msgs::CommandBool arm_cmd;
			arm_cmd.request.value = true;
			if( srv_client_[0].call(arm_cmd) && arm_cmd.response.success )
			{
				initialize_pos_setpoint();
				Q_EMIT emit_arming_state( true );
			}
		}
	}
#elif defined(GCS_DJI)
	dji_sdk::DroneArmControl arm_cmd;
	arm_cmd.request.arm = true;
	srv_client_[0].call( arm_cmd );

	if( arm_cmd.response.result )
	{
		initialize_pos_setpoint();
		Q_EMIT emit_arming_state( true );
	}
	else
		ROS_WARN("Receive control authority first");
#endif
}

void QNode::set_disarm()
{
#ifdef GCS_PIXHAWK
	if(px4_flag_)
	{
		if(px4_state_.armed) // px4 is armed, so try to disarm.
		{
			mavros_msgs::CommandBool arm_cmd;
			arm_cmd.request.value = false;
			if( srv_client_[0].call(arm_cmd) && arm_cmd.response.success )
			{
				Q_EMIT emit_arming_state( false );
				// if disarmed, then stop control
				std_srvs::SetBool stop;
				srv_client_[3].call( stop );
				init_pos_sp_ = false;
			}
		}
	}
#elif defined( GCS_DJI )
	dji_sdk::DroneArmControl arm_cmd;
	arm_cmd.request.arm = false;
	srv_client_[0].call( arm_cmd );

	if( arm_cmd.response.result )
	{
		Q_EMIT emit_arming_state( false );
		// if disarmed, then stop control
		std_srvs::SetBool stop;
		srv_client_[3].call( stop );
		init_pos_sp_ = false;
	}
#endif
}

void QNode::set_offboard()
{
#ifdef GCS_PIXHAWK
	mavros_msgs::SetMode set_mode;
	set_mode.request.custom_mode = "OFFBOARD";
	srv_client_[1].call( set_mode );
#elif defined( GCS_DJI )
	dji_sdk::SDKControlAuthority set_mode;
	set_mode.request.control_enable = true; // ack_data wan to be 2
	srv_client_[1].call( set_mode );

	if( set_mode.response.ack_data == 2 )
	{
		Q_EMIT emit_flight_mode( "OFFBOARD" );
	}
	else if( set_mode.response.ack_data == 0 )
	{
		ROS_WARN("Turn on RC transmitter first");
	}
#endif
}

void QNode::set_manual()
{
#ifdef GCS_PIXHAWK
	mavros_msgs::SetMode set_mode;
	set_mode.request.custom_mode = "MANUAL";
	srv_client_[1].call( set_mode );
#elif defined( GCS_DJI )
	dji_sdk::SDKControlAuthority set_mode;
	set_mode.request.control_enable = false; // ack_data want to be 1
	srv_client_[1].call( set_mode );

	if( set_mode.response.ack_data == 1 )
	{
		Q_EMIT emit_flight_mode( "MANUAL" );
	}
#endif
}

void QNode::set_ctrl_mode( ControlModes mode )
{
	if(init_pos_sp_)
	{
		if (mode >= YAW && mode <= YAWRATE)
		{	// change heading mode only
			// reset setpoint yaw as current yaw
			// reset setpoint yawrate as 0
			double roll, pitch, yaw;
		
			if( msf_flag_ )
				q2e( msf_state_.pose.pose.orientation, roll, pitch, yaw);
			else if( vicon_flag_ )
				q2e( vicon_pos_.pose.orientation, roll, pitch, yaw);

			pos_sp_.yaw = yaw;
			pos_sp_.yaw_rate = 0.0;
			
			// preserve current translation mode
			if( pos_sp_.type_mask & HOLD )
			{
				pos_sp_.type_mask = ( HOLD | mode );
			}
			else if( pos_sp_.type_mask & POSITION )
			{
				pos_sp_.type_mask = ( POSITION | mode );
			}
			else if( pos_sp_.type_mask & VELOCITY )
			{
				pos_sp_.type_mask = ( VELOCITY | mode );
			}
			else
			{
				pos_sp_.type_mask = ( TRAJECTORY | mode );
			}
		}
		else if(mode >= HOLD && mode <= TRAJECTORY )
		{	// change translation mode only
			// reset setpoint position as current position
			// reset setpoint velocity as 0
			if( msf_flag_ )
				pos_sp_.position = msf_state_.pose.pose.position;
			else if( vicon_flag_ )
				pos_sp_.position = vicon_pos_.pose.position;

			pos_sp_.velocity.x = 0.0;
			pos_sp_.velocity.y = 0.0;
			pos_sp_.velocity.z = 0.0;
			
			// preserve current heading mode
			(pos_sp_.type_mask & YAW) ? pos_sp_.type_mask = (mode | YAW)
									  : pos_sp_.type_mask = (mode | YAWRATE);
		}
		else
		{
			ROS_ERROR("unrecognized control mode");
		}
	}
}

void QNode::move_setpoint(int idx, bool increase)
{	// idx : x, y, z, yaw
	// tf : true -> increase / false -> decrease

	int mode = pos_sp_.type_mask;

	if( init_pos_sp_ )
	{
		switch( idx )
		{
			case 0: // x
				if( (mode & HOLD) || (mode & VELOCITY) )
					increase ? pos_sp_.velocity.x += 0.1 : pos_sp_.velocity.x -= 0.1;
				else if( (mode & POSITION) || (mode & TRAJECTORY) )
					increase ? pos_sp_.position.x += 0.2 : pos_sp_.position.x -= 0.2;
				else
					ROS_ERROR("unrecognized control mode");
			break;

			case 1: // y
				if( (mode & HOLD) || (mode & VELOCITY) )
					increase ? pos_sp_.velocity.y += 0.1 : pos_sp_.velocity.y -= 0.1;
				else if( (mode & POSITION) || (mode & TRAJECTORY) )
					increase ? pos_sp_.position.y += 0.2 : pos_sp_.position.y -= 0.2;
				else
					ROS_ERROR("unrecognized control mode");
			break;
			
			case 2: // z
				if(mode & VELOCITY)
					increase ? pos_sp_.velocity.z += 0.1 : pos_sp_.velocity.z -= 0.1;
				else if( (mode & HOLD) || (mode & POSITION) || (mode & TRAJECTORY) )
					increase ? pos_sp_.position.z += 0.2 : pos_sp_.position.z -= 0.2;
				else
					ROS_ERROR("unrecognized control mode");

				if( pos_sp_.position.z < 0 )
					pos_sp_.position.z = 0.0;
			break;
			
			case 3: // yaw
				if(mode & YAW)
					increase ? pos_sp_.yaw += (3.14/180.0)*(5.0) 
							 : pos_sp_.yaw -= (3.14/180.0)*(5.0);
				else if(mode & YAWRATE)
					increase ? pos_sp_.yaw_rate += (3.14/180.0)*(2.0) 
							 : pos_sp_.yaw_rate -= (3.14/180.0)*(2.0);
				else
					ROS_ERROR("unrecognized control mode");
			break;
		}
	}
}

void QNode::emergency_stop()
{
#ifdef GCS_PIXHAWK
	std::thread	stop_thread( &QNode::override_kill_switch, this );
	stop_thread.detach();
#endif
}

void QNode::start_control_service()
{
	if( init_pos_sp_ )
	{
		std_srvs::SetBool start;
		bool is_send = srv_client_[2].call( start );
		if( !is_send )
			ROS_WARN("Start rate controller first");
	}
	else
		ROS_WARN("Initialize position setpoint first");
}

void QNode::stop_control_service()
{
	std_srvs::SetBool start;
	srv_client_[3].call( start );
	init_pos_sp_ = false;
}

/** subscription callbacks **/
void QNode::px4_state_cb(const PX4State::ConstPtr &msg)
{
	px4_state_ = *msg;

	px4_flag_ = msg->connected;
	
	if(px4_flag_)
	{
		px4_timer_ = 0.0;
	}
	
	if(init_flag_)
	{
		Q_EMIT emit_arming_state( px4_state_.armed );
		Q_EMIT emit_flight_mode( px4_state_.mode.c_str() );
	}
}

void QNode::rc_in_cb(const RCIn::ConstPtr &msg)
{
	int signal = msg->channels[4]; // signal of kill switch for my case..	
	if(init_flag_)
	{
		if( signal > 1500 )
		{	// kill switch enabled
			Q_EMIT emit_kill_switch_enabled( true );
		}
		else
		{	// kill switch disabled
			Q_EMIT emit_kill_switch_enabled( false );
		}
	}
}

void QNode::att_sp_cb(const AttSp::ConstPtr &msg)
{
	if(init_flag_)
	{
		double roll, pitch, yaw;
		q2e( msg->orientation, roll, pitch, yaw);
		
		double* buf = (double*)malloc(6*sizeof(double));
		buf[1] = (180.0/3.14)*roll;
		buf[2] = (180.0/3.14)*pitch;
		buf[3] = (180.0/3.14)*msg->body_rate.x;	// we use these values as "rotational velocity",
		buf[4] = (180.0/3.14)*msg->body_rate.y; // not "euler rates"
		buf[5] = (180.0/3.14)*msg->body_rate.z;
		buf[0] = now();
		
		Q_EMIT emit_attitude_setpoint( buf, (msg->header.seq % 40) == 0 );
	}
}

void QNode::msf_state_cb(const MSFState::ConstPtr &msg)
{
	msf_state_ = *msg;

	if(init_flag_)
	{
		if( (msg->header.seq % 10) == 0 )
		{
			double roll, pitch, yaw;
			q2e( msg->pose.pose.orientation, roll, pitch, yaw);
			
			Eigen::Quaternion<double> q;
			q.w() = msg->pose.pose.orientation.w;
			q.x() = msg->pose.pose.orientation.x;
			q.y() = msg->pose.pose.orientation.y;
			q.z() = msg->pose.pose.orientation.z;
			Eigen::Matrix<double,3,3> R = q.toRotationMatrix();

			double* buf = (double*)malloc(13*sizeof(double));	
			buf[1] = msg->pose.pose.position.x;
			buf[2] = msg->pose.pose.position.y;
			buf[3] = msg->pose.pose.position.z;
		
			// this is now inertial velocity
			Eigen::Matrix<double,3,1> v_b;
			v_b(0,0) = msg->twist.twist.linear.x;
			v_b(1,0) = msg->twist.twist.linear.y;
			v_b(2,0) = msg->twist.twist.linear.z;
			Eigen::Matrix<double,3,1> v = R*v_b;
			buf[4] = v(0,0); 
			buf[5] = v(1,0);
			buf[6] = v(2,0);

			buf[7] = roll*(180.0/3.14);
			buf[8] = pitch*(180.0/3.14);
			buf[9] = yaw*(180.0/3.14);
			buf[10] = (180.0/3.14)*msg->twist.twist.angular.x;
			buf[11] = (180.0/3.14)*msg->twist.twist.angular.y;
			buf[12] = (180.0/3.14)*msg->twist.twist.angular.z;
			buf[0] = now();

			Q_EMIT emit_msf_state( buf );
		}
	}
	msf_flag_ = true;
}

void QNode::dji_att_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
	if(init_flag_)
	{
		if( msg->header.seq % 40 == 0)
		{
			double* buf = (double*)malloc(7*sizeof(double));
			buf[0] = now();
	
			double roll, pitch, yaw;
			q2e( msg->orientation, roll, pitch, yaw);
	
			buf[1] = (180.0/3.14)*roll;
			buf[2] = (180.0/3.14)*pitch;
			buf[3] = (180.0/3.14)*yaw;
			buf[4] = (180.0/3.14)*msg->angular_velocity.x;
			buf[5] = (180.0/3.14)*msg->angular_velocity.y;
			buf[6] = (180.0/3.14)*msg->angular_velocity.z;
	
			Q_EMIT emit_dji_att( buf );

			ROS_WARN("SSIBAL");
		}
	}
}

void QNode::vicon_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	vicon_pos_ = *msg;
	
	if(init_flag_)
	{
		if( (msg->header.seq % 10) == 0 )
		{
			double roll, pitch, yaw;
			q2e( msg->pose.orientation, roll, pitch, yaw);

			double* buf = (double*)malloc(7*sizeof(double));	
			buf[1] = msg->pose.position.x;
			buf[2] = msg->pose.position.y;
			buf[3] = msg->pose.position.z;

			buf[4] = roll*(180.0/3.14);
			buf[5] = pitch*(180.0/3.14);
			buf[6] = yaw*(180.0/3.14);
			buf[0] = now();

			Q_EMIT emit_vicon_pos( buf );
		}
	}
	vicon_flag_ = true;
}

void QNode::vicon_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
	if(init_flag_)
	{
		if( (msg->header.seq % 10) == 0 )
		{
			double* buf = (double*)malloc(7*sizeof(double));	
			buf[1] = msg->twist.linear.x;
			buf[2] = msg->twist.linear.y;
			buf[3] = msg->twist.linear.z;
			buf[4] = (180.0/3.14)*msg->twist.angular.x;
			buf[5] = (180.0/3.14)*msg->twist.angular.y;
			buf[6] = (180.0/3.14)*msg->twist.angular.z;
			buf[0] = now();

			Q_EMIT emit_vicon_vel( buf );
		}
	}
}

void QNode::initialize_pos_setpoint()
{
	if( !init_pos_sp_ )
	{
		if( msf_flag_ )
		{
			pos_sp_.position = msf_state_.pose.pose.position;
			double roll, pitch, yaw;
			q2e( msf_state_.pose.pose.orientation, roll, pitch, yaw);
			pos_sp_.yaw = yaw;
			ROS_WARN("Initialize position setpoint with msf states");
		}
		else if( vicon_flag_ )
		{
			pos_sp_.position = vicon_pos_.pose.position;
			double roll, pitch, yaw;
			q2e( vicon_pos_.pose.orientation, roll, pitch, yaw);
			pos_sp_.yaw = yaw;
			ROS_WARN("Initialize position setpoint with vicon states");
		}
		else
		{
			ROS_ERROR("No state measurements");
			return;
		}
	
		pos_sp_.coordinate_frame = pos_sp_.FRAME_LOCAL_NED; 
		pos_sp_.header.frame_id = "fcu"; 
		pos_sp_.header.seq = 0;
	
		pos_sp_.type_mask = POSITION | YAW;
		pos_sp_.velocity.x = 0.0;
		pos_sp_.velocity.y = 0.0;
		pos_sp_.velocity.z = 0.0;
		pos_sp_.yaw_rate = 0.0;
	
		init_pos_sp_ = true;
	}
}

void QNode::override_kill_switch()
{
	double rate = 200.0;
	ros::Rate loop_rate(rate);
	
	RCOverride msg;
	msg.channels[0] = 1100;
	msg.channels[1] = 1500;
	msg.channels[2] = 1500;
	msg.channels[3] = 1500;
	msg.channels[4] = 1900;
	msg.channels[5] = 1100;
	msg.channels[6] = 1100;
	msg.channels[7] = 1100;

	if(init_flag_)
	{
		while( ros::ok() )
		{
			ros::spinOnce();
			pub_[1].publish( msg );		
			loop_rate.sleep();
		}
	}
}

void q2e(const double q0, const double q1, const double q2, const double q3, 
		double& roll, double& pitch, double& yaw)
{
	roll = atan2(2*(q2*q3+q0*q1),q0*q0-q1*q1-q2*q2+q3*q3);
	pitch = atan2(-2*(q1*q3-q0*q2),sqrt((2*(q2*q3+q0*q1))*(2*(q2*q3+q0*q1))+(q0*q0-q1*q1-q2*q2+q3*q3)*(q0*q0-q1*q1-q2*q2+q3*q3)));
	yaw = atan2(2*(q1*q2+q0*q3),q0*q0+q1*q1-q2*q2-q3*q3);
}
void q2e(const geometry_msgs::Quaternion q, double& roll, double& pitch, double& yaw)
{
	double ysqr = q.y * q.y;

	double t0 = 2.0 * (q.w * q.x + q.y * q.z);
	double t1 = 1.0 - 2.0 * (q.x * q.x + ysqr);
	roll = std::atan2(t0, t1);

	double t2 = 2.0 * (q.w * q.y - q.z * q.x);
	t2 = ((t2 > 1.0) ? 1.0 : t2);
	t2 = ((t2 < -1.0) ? -1.0 : t2);
	pitch = std::asin(t2);

	double t3 = 2.0 * (q.w * q.z + q.x * q.y);
	double t4 = 1.0 - 2.0 * (ysqr + q.z * q.z);
	yaw = std::atan2(t3, t4);
}

}  // namespace px4_gcs
