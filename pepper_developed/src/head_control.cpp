//
//

// Naoqi
#include <qi/applicationsession.hpp>

// ROS
#include <ros/ros.h>

// PepperHead Class
#include <pepper_lib_developed/pepper_head.hpp>

// Boost
#include <boost/shared_ptr.hpp>



int main(int argc, char** argv) {
	ros::init(argc, argv, "pepper_head_control");
	ros::NodeHandle nh;

	// Check command line argument
	// std::string actions_available = "Valid actions:\n --alstart --alstop --rest --wakeup --reset";
	// std::string action;
	// if (argc < 2)
	// {
	// 	ROS_ERROR_STREAM("Action argument must be specified!\n" << actions_available);
	// 	return -1;
	// }
	// else
	// {
	// 	action = argv[1];
	// }

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
	boost::shared_ptr <pepper_tools::PepperHead> ph = boost::shared_ptr<pepper_tools::PepperHead>(new pepper_tools::PepperHead(app.session()));

	ph->setMaxStiffness();

	return 0;
}
