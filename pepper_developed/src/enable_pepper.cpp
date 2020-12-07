//
// This node allows the user to operate with Autonomous Life and Rest states.
// Use it with enable_pepper.launch with the argument action.
//
// Available values for action:
// - alstart: start the Autonomous Life
// - alstop: stop the Autonomous Life
// - rest: set the state to the Rest state
// - wakeup: wake up the robot
// - reset: set the Stand posture
//

// Naoqi
#include <qi/applicationsession.hpp>

// ROS
#include <ros/ros.h>

// PepperTools Class
#include <pepper_lib_developed/pepper_tools.hpp>

// Boost
#include <boost/shared_ptr.hpp>


int main(int argc, char** argv) {

	// Init ros
	ros::init(argc, argv, "enable_pepper");
	ros::NodeHandle nh;

	// Check command line argument
	std::string actions_available = "Valid actions:\n --alstart --alstop --rest --wakeup --reset";
	std::string action;
	if (argc < 2)
	{
		ROS_ERROR_STREAM("Action argument must be specified!\n" << actions_available);
		return -1;
	}
	else
	{
		action = argv[1];
	}

	// Retrieve connection parameters from parameter server
	std::string nao_ip;
	std::string roscore_ip;
	std::string network_interface;
	int nao_port;
	nh.getParam("nao_ip", nao_ip);
	nh.getParam("nao_port", nao_port);
	nh.getParam("roscore_ip", roscore_ip);
	nh.getParam("network_interface", network_interface);

	// Launch naoqi service
	std::stringstream strstr;
  strstr << "tcp://" << nao_ip << ":" << nao_port;
	ROS_INFO_STREAM("Connecting to: " << strstr.str());

	qi::ApplicationSession app(argc, argv, 0, strstr.str());
	app.startSession();

	// Create pepper tools object
	boost::shared_ptr <pepper_tools::PepperTools> pt = boost::shared_ptr<pepper_tools::PepperTools>(new pepper_tools::PepperTools(app.session()));

	// Select the correct action
	if (action == "--alstart")
	{
		pt->alStart();
	}
	else if (action == "--alstop")
	{
		pt->alStop();
	}
	else if (action == "--rest")
	{
		pt->rest();
	}
	else if (action == "--wakeup")
	{
		pt->wakeUp();
	}
	else if (action == "--reset")
	{
		pt->resetPosture();
	}
	else
	{
		ROS_ERROR_STREAM("Wrong action selected!\n" << actions_available);
	}

	return 0;
}
