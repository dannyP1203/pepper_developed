//
// Pepper Tools Class
//

#include <pepper_lib_developed/pepper_tools.hpp>


namespace pepper_tools
{

	PepperTools::PepperTools(const qi::SessionPtr& session)
	// : session_(session)
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
			life_proxy_ = session->service("ALAutonomousLife");
		}
		catch (const std::exception& e)
		{
			ROS_ERROR("Autonomous Life: Failed to connect to ALAutonomousLife Proxy!\n\tTrace: %s", e.what());
		}
		try
		{
			motion_proxy_ = session->service("ALMotion");
		}
		catch (const std::exception& e)
		{
			ROS_ERROR("Motion: Failed to connect to Motion Proxy!\n\tTrace: %s", e.what());
		}
		try
		{
			posture_proxy_ = session->service("ALRobotPosture");
		}
		catch (const std::exception& e)
		{
			ROS_ERROR("Posture: Failed to connect to Posture Proxy!\n\tTrace: %s", e.what());
		}
	}


	// Stop Autonomous Life
	void PepperTools::alStop()
	{
		try
	  {
	    if (life_proxy_.call<std::string>("getState") != "disabled")
	    {
	      life_proxy_.call<void>("setState", "disabled");
	      ROS_INFO_STREAM("Shutting down Naoqi AutonomousLife ...");
	      ros::Duration(4.0).sleep();
	    }
			else
			{
				ROS_WARN("Autonomous Life is already turned off!");
			}
	  }
	  catch (const std::exception& e)
	  {
	    ROS_ERROR("Autonomous Life: Can not stop autonomous life!\n\tTrace: %s", e.what());
	  }
	}

	// Start Autonomous Life
	void PepperTools::alStart()
	{
		try
	  {
	    if (life_proxy_.call<std::string>("getState") == "disabled")
	    {
	      life_proxy_.call<void>("setState", "solitary");
	      ROS_INFO_STREAM("Turning up Naoqi AutonomousLife ...");
	      ros::Duration(4.0).sleep();
	    }
			else
			{
				ROS_WARN("Autonomous Life is already turned on with state: %s",life_proxy_.call<std::string>("getState").c_str());
			}
	  }
	  catch (const std::exception& e)
	  {
			ROS_ERROR("Autonomous Life: Can not start autonomous life!\n\tTrace: %s", e.what());
	  }
	}

	// Rest
	void PepperTools::rest()
	{
		try
	  {
	    ROS_INFO_STREAM("Going to rest ...");
	    motion_proxy_.call<void>("rest");
	    ros::Duration(4.0).sleep();
		}
	  catch (const std::exception& e)
	  {
	    ROS_ERROR("Motion: Failed to rest!\n\tTrace: %s", e.what());
	  }
	}

	void PepperTools::wakeUp()
	{
		try
		{
			ROS_INFO_STREAM("Going to wakeup ...");
			motion_proxy_.call<void>("wakeUp");
			ros::Duration(4.0).sleep();
		}
		catch (const std::exception& e)
		{
			ROS_ERROR("Motion: Failed to wake up!\n\tTrace: %s", e.what());
		}
	}

	void PepperTools::resetPosture()
	{
		try
		{
			ROS_INFO_STREAM("Going to the Init posture ...");
			posture_proxy_.call<bool>("goToPosture","Stand",0.5);
		}
		catch (const std::exception& e)
		{
			ROS_ERROR("Posture: Failed to reset the posture!\n\tTrace: %s", e.what());
		}
	}

}  // pepper_tools ns end
