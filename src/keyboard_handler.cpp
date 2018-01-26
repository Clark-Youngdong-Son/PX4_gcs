#include <keyboard_handler.h>
#include <string>

using namespace syd;

KeyboardHandler::KeyboardHandler()
{
	ros::NodeHandle nh;
	ros::NodeHandle pnh("px4_gcs_keyboard");

	keyboard_sub = nh.subscribe<keyboard::Key>("keyboard/keydown", 100, 
			&KeyboardHandler::keyboard_cb, this);

	cnt = 0;
	pnh.getParam("gcs_count", cnt);
	
	std::string name;
	for(int i=0; i<cnt; i++)
	{
		pnh.getParam(("name"+std::to_string(i+1)).c_str(), name);
		keyboard_pub[i] = nh.advertise<keyboard::Key>((name+"/keyinput").c_str(), 100);
	}
}

KeyboardHandler::~KeyboardHandler()
{
}

void KeyboardHandler::keyboard_cb(const keyboard::Key::ConstPtr& msg)
{
	keyboard::Key key_input;
	key_input.header = msg->header;
	key_input.code = msg->code;

	for(int i=0; i<cnt; i++)	
		keyboard_pub[i].publish( key_input );
}
