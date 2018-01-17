#include "ros/ros.h"
#include "keyboard_handler.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "px4_gcs_keyboard");

	syd::KeyboardHandler *keyboard = new syd::KeyboardHandler();
	
	ros::spin();

	delete keyboard;
	return 0;
}
