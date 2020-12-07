// Pepper Head  Header

#ifndef PEPPER_HEAD_HPP
#define PEPPER_HEAD_HPP


// Naoqi
#include <qi/session.hpp>
#include <qi/anyobject.hpp>

// ROS
#include <ros/ros.h>


namespace pepper_tools
{

class PepperHead
{

public:

	PepperHead(const qi::SessionPtr& session);

//	~PepperHead();

	// Set stiffness
	void setMaxStiffness();
	void setMinStiffness();
	void setStiffness(float value);


private:
	qi::AnyObject motion_proxy_;

};  // PepperTools class end
}  // pepper_tools ns end

#endif
