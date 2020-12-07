// Pepper tools Header

#ifndef PEPPER_TOOLS_HPP
#define PEPPER_TOOLS_HPP


// Naoqi
#include <qi/session.hpp>
#include <qi/anyobject.hpp>
// #include <alproxies/alrobotpostureproxy.h>

// ROS
#include <ros/ros.h>


namespace pepper_tools
{

class PepperTools
{

public:

	PepperTools(const qi::SessionPtr& session);

//	~PepperTools();

	// Stop Autonomous Life
	void alStop();

	// Start Autonomous Life
	void alStart();

	// Rest
	void rest();

	//Wake Up
	void wakeUp();

	//Reset the posture setting the Init posture
	void resetPosture();

private:
	// qi::SessionPtr session_;
	qi::AnyObject life_proxy_;
	qi::AnyObject motion_proxy_;
	qi::AnyObject posture_proxy_;

};  // PepperTools class end
}  // pepper_tools ns end

#endif
