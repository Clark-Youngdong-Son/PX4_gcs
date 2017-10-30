#include "qnode.hpp"

namespace px4_gcs 
{

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv),	
	init_flag_(false),
	ros_flag_(false),
	px4_flag_(false),
	px4_signal_loss_(false),
	init_pos_sp_(false),
	px4_timer_(0.0),
	emergency_stop_(false)
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
	if(!ros_flag_)
	{
		ros::init(init_argc,init_argv,"px4_gcs");
		if (!ros::master::check())
		{
			Q_EMIT emit_pushButton_connect_ros_color(false);
			return false;
		}
		ros_flag_ = true;
		Q_EMIT emit_pushButton_connect_ros_color(true);

		ros::start(); // explicitly needed since our nodehandle is going out of scope.
		ros::NodeHandle n;

		t_init_ = ros::Time::now();

		// Subscription
		sub_[0] = n.subscribe<PX4State>("mavros/state",10,&QNode::px4_state_cb,this);
		sub_[1] = n.subscribe<RCIn>("mavros/rc/in",10,&QNode::rc_in_cb,this);
		sub_[2] = n.subscribe<AttSp>("mavros/setpoint_raw/attitude",10,&QNode::att_sp_cb,this);
		sub_[3] = n.subscribe<MSFState>("msf_core/odometry",10,&QNode::msf_state_cb,this);
		sub_[4] = n.subscribe<Flow>("optical_flow/measurements",10,&QNode::flow_cb,this);
		sub_[5] = n.subscribe<VO>("rovio/pose_with_covariance_stamped",10,&QNode::vo_cb,this);
		sub_[6] = n.subscribe<GPSPos>("ublox_rover/navrelposned",10,&QNode::gps_pos_cb,this);
		sub_[7] = n.subscribe<Lidar>("mavros/distance_sensor/lidarlite",10,&QNode::lidar_cb,this);

		// Publication
		pub_[0] = n.advertise<PosSp>("gcs/setpoint_raw/position", 10);
		pub_[1] = n.advertise<RCOverride>("mavros/rc/override", 10);

		// Services
		arming_client = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
		set_mode_client = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

		start();
		return true;
	}
}

bool QNode::connect_px4()
{
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
			//ros::spinOnce();
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
			// connection check
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

			if(init_pos_sp_)
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
	bool success = false;

	if(px4_flag_)
	{
		if(px4_state_.armed) // px4 is already armed.
		{
			success = true;
		}
		else // px4 is disarmed, so try to arm.
		{
			arm_cmd.request.value = true;
			if( arming_client.call(arm_cmd) && arm_cmd.response.success )
			{
				initialize_pos_setpoint();
				success = true;
			}
			else
			{
				success = false;
			}
		}
	}
	else
	{
		success = false;
	}
}

void QNode::set_disarm()
{
	bool success = false;

	if(px4_flag_)
	{
		if(!px4_state_.armed) // px4 is already armed.
		{
			success = true;
		}
		else // px4 is disarmed, so try to arm.
		{
			arm_cmd.request.value = false;
			if( arming_client.call(arm_cmd) && arm_cmd.response.success )
			{
				success = true;
			}
			else
			{
				success = false;
			}
		}
	}
	else
	{
		success = false;
	}
}

void QNode::set_offboard()
{
	mavros_msgs::SetMode set_mode;
	set_mode.request.custom_mode = "OFFBOARD";
	set_mode_client.call( set_mode );
}

void QNode::set_manual()
{
	mavros_msgs::SetMode set_mode;
	set_mode.request.custom_mode = "MANUAL";
	set_mode_client.call( set_mode );
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
			q2e( msf_state_.pose.pose.orientation, roll, pitch, yaw);
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
			pos_sp_.position = msf_state_.pose.pose.position;
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
	std::thread	stop_thread( &QNode::override_kill_switch, this );
	stop_thread.detach();
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
		double roll, pitch, yaw;
		q2e( msg->pose.pose.orientation, roll, pitch, yaw);
		
		double* buf = (double*)malloc(13*sizeof(double));	
		buf[1] = msg->pose.pose.position.x;
		buf[2] = msg->pose.pose.position.y;
		buf[3] = msg->pose.pose.position.z;
		buf[4] = msg->twist.twist.linear.x;
		buf[5] = msg->twist.twist.linear.y;
		buf[6] = msg->twist.twist.linear.z;
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

void QNode::flow_cb(const Flow::ConstPtr &msg)
{
	if(init_flag_)
	{
		double* buf = (double*)malloc(4*sizeof(double));
		buf[1] = msg->vel[0];
		buf[2] = msg->vel[1];
		buf[3] = msg->vel[2];
		buf[0] = now();
		
		Q_EMIT emit_flow_measurements( buf );
	}
}

void QNode::vo_cb(const VO::ConstPtr &msg)
{
	if(init_flag_)
	{
		double roll, pitch, yaw;
		q2e( msg->pose.pose.orientation, roll, pitch, yaw);
		
		double* buf = (double*)malloc(7*sizeof(double));
		buf[1] = msg->pose.pose.position.x;
		buf[2] = msg->pose.pose.position.y;
		buf[3] = msg->pose.pose.position.z;
		buf[4] = (180.0/3.14)*roll;
		buf[5] = (180.0/3.14)*pitch;
		buf[6] = (180.0/3.14)*yaw;
		buf[0] = now();
		
		Q_EMIT emit_vo_measurements( buf );
	}
}

void QNode::gps_pos_cb(const GPSPos::ConstPtr &msg)
{
	if(init_flag_)
	{
		double* buf = (double*)malloc(3*sizeof(double));
		buf[1] = (double)(msg->relPosN + msg->relPosHPN*1e-2)*1e-2;
		buf[2] = (double)(msg->relPosE + msg->relPosHPE*1e-2)*1e-2*(-1.0); 
		buf[0] = now();
		
		Q_EMIT emit_gps_pos_measurements( buf );
	}
}

void QNode::lidar_cb(const Lidar::ConstPtr &msg)
{
	if(init_flag_)
	{
		double* buf = (double*)malloc(2*sizeof(double));
		buf[1] = msg->point.z;
		buf[0] = now();
		
		Q_EMIT emit_lidar_measurements( buf );
	}
}

void QNode::initialize_pos_setpoint()
{
	pos_sp_.coordinate_frame = pos_sp_.FRAME_LOCAL_NED; 
	pos_sp_.header.frame_id = "fcu"; 
	pos_sp_.header.seq = 0;

	pos_sp_.type_mask = HOLD | YAW;
	pos_sp_.position = msf_state_.pose.pose.position;
	pos_sp_.velocity.x = 0.0;
	pos_sp_.velocity.y = 0.0;
	pos_sp_.velocity.z = 0.0;
	
	double roll, pitch, yaw;
	q2e( msf_state_.pose.pose.orientation, roll, pitch, yaw);

	pos_sp_.yaw = yaw;
	pos_sp_.yaw_rate = 0.0;

	init_pos_sp_ = true;
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
