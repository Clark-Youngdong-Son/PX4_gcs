#ifndef KEYBOARD_HANDLER
#define KEYBOARD_HANDLER

#include "ros/ros.h"
#include "keyboard/Key.h"
#include <sstream>
#include <string>

// inicrease joint position
#define KEY_Q 113 
#define KEY_W 119
#define KEY_E 101
#define KEY_R 114

// decrease joint position
#define KEY_A 97	
#define KEY_S 115	
#define KEY_D 100	
#define KEY_F 102

// quit 
#define KEY_SPACE 32

// mode change
#define KEY_M 109

// initialize
#define KEY_I 105	

// grip position
#define KEY_G 103

// moving upward
#define KEY_H 104

// moving upward
#define KEY_J 106

// pioneer motor start
#define KEY_Z 122

// pioneer linear velocity
#define KEY_UP 273
#define KEY_DOWN 274

// pioneer angular velocity
#define KEY_RIGHT 275
#define KEY_LEFT 276

#define KEY_O 111	
#define KEY_P 112	
#define KEY_C 99	
#define KEY_U 117	
#define KEY_V 118	
#define KEY_B 98	
#define KEY_N 110	
#define KEY_H 104
#define KEY_X 120
#define KEY_ENTER 13
//#define ARMED -22
//#define DISARMED -33

namespace syd 
{

class KeyboardHandler
{
	public:
		KeyboardHandler();
		~KeyboardHandler();

	private:
//		ros::NodeHandle nh;	

		ros::Subscriber keyboard_sub;
		ros::Publisher keyboard_pub[5];

		void keyboard_cb(const keyboard::Key::ConstPtr& msg);
		int cnt;	
};

} // syd

#endif
