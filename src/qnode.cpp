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
		
		t_init_ = ros::Time::now();
		
		// Subscription
		sub_[0] = n.subscribe<mavros_msgs::AttitudeTarget>
			("mavros/setpoint_raw/attitude",10,&QNode::att_sp_cb,this);
		sub_[1] = n.subscribe<nav_msgs::Odometry>
			//("mavros/local_position/odom",10,&QNode::odom_cb,this); //SITL
			("vicon/odometry",10,&QNode::odom_cb,this); //Experiment
		
		sub_[2] = n.subscribe<mavros_msgs::State>
			("mavros/state",10,&QNode::px4_state_cb,this);
		sub_[3] = n.subscribe<mavros_msgs::RCIn>
			("mavros/rc/in",10,&QNode::rc_in_cb,this);
		sub_[4] = n.subscribe<sensor_msgs::Imu>
			("mavros/imu/data", 10, &QNode::imu_cb, this);
		// Publication
		pub_[0] = n.advertise<mavros_msgs::PositionTarget>("gcs/setpoint_raw/position", 10);
		pub_[1] = n.advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override", 10);
		pub_[2] = n.advertise<std_msgs::Bool>("px4_gcs/mocapType", 10);
			//// pixhawk
		srv_client_[0] = n.serviceClient<mavros_msgs::CommandBool>
			("mavros/cmd/arming");
		srv_client_[1] = n.serviceClient<mavros_msgs::SetMode>
			("mavros/set_mode");
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

			if( init_pos_sp_ )
			{	// publish setpoint
				pos_sp_.header.stamp = ros::Time::now();
				if(flight_type_flag_)
				{
					pub_[0].publish( pos_sp_ );	
				}
				else
				{
					double t_now = (ros::WallTime::now() - t_circle_init_).toSec();
					double pi = 3.14;
					//pos_sp_.position.x = circle_center_(0) + radius*sinf(2*pi*frequency*t_now);
					//pos_sp_.position.y = circle_center_(1) - radius*cosf(2*pi*frequency*t_now);
					//pos_sp_.position.x = circle_center_(0) + 1.25*sinf(2*pi*frequency*t_now);
					//pos_sp_.position.y = circle_center_(1) - 1.25*cosf(2*pi*frequency*t_now);
					//pos_sp_.position.z = circle_center_(2);
					//pos_sp_.velocity.x =  2*pi*frequency*1.25*cosf(2*pi*frequency*t_now);
					//pos_sp_.velocity.y =  2*pi*frequency*1.25*sinf(2*pi*frequency*t_now);
					//pos_sp_.velocity.z = 0.0;
					pos_sp_.position.x = circle_center_(0) - 1.25*cosf(2*pi*frequency*t_now);
					pos_sp_.position.y = circle_center_(1);
					pos_sp_.position.z = circle_center_(2);
					pos_sp_.velocity.x =  2*pi*frequency*1.25*sinf(2*pi*frequency*t_now);
					pos_sp_.velocity.y =  0.0;
					pos_sp_.velocity.z = 0.0;
					pos_sp_.yaw = yaw_circle_;
					pos_sp_.yaw_rate = 0.0;
					pub_[0].publish( pos_sp_ );	
				}
				
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
}

void QNode::set_disarm()
{
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
				stop_control_service();
			}
		}
	}
}

void QNode::set_offboard()
{
	if( !control_flag_ )
	{
		ROS_WARN("Start rate controller (press C) first");
		return;
	}
	mavros_msgs::SetMode set_mode;
	set_mode.request.custom_mode = "OFFBOARD";
	srv_client_[1].call( set_mode );
}

void QNode::set_manual()
{
	mavros_msgs::SetMode set_mode;
	set_mode.request.custom_mode = "MANUAL";
	srv_client_[1].call( set_mode );
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
			q2e( odom_.pose.pose.orientation, roll, pitch, yaw);
		
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

			pos_sp_.position = odom_.pose.pose.position;
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

void QNode::set_mocap_type( MocapTypes type )
{
	std_msgs::Bool mocap_type;
	if(type==VICON)
	{
		mocap_type.data = true;
	}
	else if(type==VISION)
	{
		mocap_type.data = false;
	}
	pub_[2].publish(mocap_type);
	start_control_service();
}

void QNode::set_flight_type( FlightTypes type )
{
	if(type == KEYBOARD)
	{
		flight_type_flag_old_ = flight_type_flag_;
		flight_type_flag_ = true;
		double roll, pitch, yaw;
		q2e( odom_.pose.pose.orientation, roll, pitch, yaw);
		
		pos_sp_.position = odom_.pose.pose.position;
		pos_sp_.velocity.x = 0.0;
		pos_sp_.velocity.y = 0.0;
		pos_sp_.velocity.z = 0.0;
		pos_sp_.yaw = yaw;
		pos_sp_.yaw_rate = 0.0;
	}
	else if(type == CIRCLE)
	{
		flight_type_flag_old_ = flight_type_flag_;
		flight_type_flag_ = false;
		if(flight_type_flag_old_)
		{
			t_circle_init_ = ros::WallTime::now();
			//circle_center_(0) = odom_.pose.pose.position.x;
			//circle_center_(1) = odom_.pose.pose.position.y + radius;
			//circle_center_(1) = odom_.pose.pose.position.y + 1.25;
			//circle_center_(2) = odom_.pose.pose.position.z;
			circle_center_(0) = odom_.pose.pose.position.x + 1.25;
			circle_center_(1) = odom_.pose.pose.position.y;
			circle_center_(2) = odom_.pose.pose.position.z;

			double roll, pitch;
			q2e( odom_.pose.pose.orientation, roll, pitch, yaw_circle_);
		
			pos_sp_.yaw = yaw_circle_;
			pos_sp_.yaw_rate = 0.0;
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
	std::thread	stop_thread( &QNode::override_kill_switch, this );
	stop_thread.detach();
}

void QNode::start_control_service()
{
	if( init_pos_sp_ )
	{
		std_srvs::SetBool start;
		bool is_send = srv_client_[2].call( start );
		if( !is_send )
			ROS_WARN("Launch rate controller first");
		else
			control_flag_ = true;
	}
	else
		ROS_WARN("Initialize position setpoint first");
}

void QNode::stop_control_service()
{
	std_srvs::SetBool stop;
	srv_client_[3].call( stop );
	init_pos_sp_ = false;
	control_flag_ = false;
}

/** subscription callbacks **/
void QNode::px4_state_cb(const mavros_msgs::State::ConstPtr &msg)
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

void QNode::rc_in_cb(const mavros_msgs::RCIn::ConstPtr &msg)
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

void QNode::att_sp_cb(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
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
		Q_EMIT emit_thrust_setpoint( msg->thrust, (msg->header.seq % 40) == 0 );
	}
}

void QNode::odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
	odom_ = *msg;

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
		
			Eigen::Matrix<double,3,1> v_b;
			v_b(0,0) = msg->twist.twist.linear.x;
			v_b(1,0) = msg->twist.twist.linear.y;
			v_b(2,0) = msg->twist.twist.linear.z;
			Eigen::Matrix<double,3,1> v = R*v_b; // this is now pos_rate
			buf[4] = v(0,0); 
			buf[5] = v(1,0);
			buf[6] = v(2,0);

			buf[7] = roll*(180.0/3.14);
			buf[8] = pitch*(180.0/3.14);
			buf[9] = yaw*(180.0/3.14);

			buf[0] = now();

			Q_EMIT emit_navigation_state( buf );
		}
	}
}

void QNode::imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
	if(init_flag_)
	{
		if( msg->header.seq % 10 == 0)
		{
			double* buf = (double*)malloc(7*sizeof(double));
			buf[0] = now();
	
			double phi, theta, psi;
			q2e( msg->orientation, phi, theta, psi);
	
			buf[1] = (180.0/3.14)*phi;
			buf[2] = (180.0/3.14)*theta;
			buf[3] = (180.0/3.14)*psi;

			Eigen::Matrix<double,3,1> w_b;
			w_b(0,0) = msg->angular_velocity.x;
			w_b(1,0) = msg->angular_velocity.y;
			w_b(2,0) = msg->angular_velocity.z;
			Eigen::Matrix<double,3,3> T;
			T(0,0) = 1.0; T(0,1) = sin(phi)*tan(theta); T(0,2) = cos(phi)*tan(theta);
			T(1,0) = 0.0; T(1,1) = cos(phi); 			T(1,2) = -sin(phi);
			T(2,0) = 0.0; T(2,1) = sin(phi)/cos(theta); T(2,2) = cos(phi)/cos(theta);
			Eigen::Matrix<double,3,1> euler_rate = T * w_b;

			buf[4] = (180.0/3.14)*euler_rate(0,0);
			buf[5] = (180.0/3.14)*euler_rate(1,0);
			buf[6] = (180.0/3.14)*euler_rate(2,0);
	
			Q_EMIT emit_imu_state( buf );
		}
	}
}

void QNode::initialize_pos_setpoint()
{
	if( !init_pos_sp_ )
	{
		pos_sp_.position = odom_.pose.pose.position;
		double roll, pitch, yaw;
		q2e( odom_.pose.pose.orientation, roll, pitch, yaw);
		pos_sp_.yaw = yaw;

		ROS_WARN("Initialize position setpoint with vicon");
	
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
	
	mavros_msgs::OverrideRCIn msg;
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
		ROS_FATAL("Emergency mode enabled!!!");
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
