//
// This node stop the Autonomous Life and, then, wake up the robot.
// It's written to be included in pepper_bringup.launch.
// To operate with Autonomous Life and Rest, use enable_pepper instead.
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
	ros::init(argc, argv, "start_pepper");
	ros::NodeHandle nh;

	ROS_INFO_STREAM("Going to initialize...");

	// Launch naoqi service
	qi::ApplicationSession app(argc, argv);
	app.startSession();

	// Create pepper tools object
	boost::shared_ptr <pepper_tools::PepperTools> obj = boost::shared_ptr<pepper_tools::PepperTools>(new pepper_tools::PepperTools(app.session()));
	// pepper_tools::PepperTools obj = pepper_tools::PepperTools(app.session());

	// Stop autonomous life and wake up
	obj->alStop();
	obj->wakeUp();
	obj->resetPosture();

	return 0;
}
