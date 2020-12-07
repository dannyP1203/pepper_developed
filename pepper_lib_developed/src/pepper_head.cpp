//
// Pepper Tools Class
//

#include <pepper_lib_developed/pepper_head.hpp>


namespace pepper_tools
{

	PepperHead::PepperHead(const qi::SessionPtr& session)
	{
		// Check if ros master is up and running
		if(!ros::master::check())
		{
			ROS_ERROR("Could not contact master!\nQuitting... ");
		}

		// Check if session is connected
		if (!(session->isConnected()))
		{
			ROS_ERROR("Cannot connect to session!");
			session->close();
		}

		// Create naoqi module proxies
		try
		{
			motion_proxy_ = session->service("ALMotion");
		}
		catch (const std::exception& e)
		{
			ROS_ERROR("Motion: Failed to connect to Motion Proxy!\n\tTrace: %s", e.what());
		}
	}



	void PepperHead::setMaxStiffness()
	{
		try
		{
			ROS_INFO_STREAM("Setting head stiffnesses to 1.0.");
			motion_proxy_.call<void>("setStiffnesses","Head", 1.0);
		}
		catch (const std::exception& e)
		{
			ROS_ERROR("Motion: Failed to set the stiffnesses!\n\tTrace: %s", e.what());
		}
	}

	void PepperHead::setMinStiffness()
	{
		try
		{
			ROS_INFO_STREAM("Setting head stiffnesses to 0.0.");
			motion_proxy_.call<void>("setStiffnesses","Head", 0.0);
		}
		catch (const std::exception& e)
		{
			ROS_ERROR("Motion: Failed to set the stiffnesses!\n\tTrace: %s", e.what());
		}
	}

	void PepperHead::setStiffness(float value)
	{
		try
		{
			ROS_INFO_STREAM("Setting head stiffnesses to " << value);
			motion_proxy_.call<void>("setStiffnesses","Head", value);
		}
		catch (const std::exception& e)
		{
			ROS_ERROR("Motion: Failed to set the stiffnesses!\n\tTrace: %s", e.what());
		}
	}

}  // pepper_tools ns end
