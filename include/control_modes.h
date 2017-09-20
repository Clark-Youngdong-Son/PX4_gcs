#ifndef CONTROL_MODES
#define CONTROL_MODES

// do not change this, rate_controller also use this enumeration
enum ControlModes 
{
	HOLD 		= 1 << 0,
	POSITION 	= 1 << 1,
	VELOCITY 	= 1 << 2,
	TRAJECTORY 	= 1 << 3,
	YAW 		= 1 << 4,
	YAWRATE 	= 1 << 5,
};


#endif
