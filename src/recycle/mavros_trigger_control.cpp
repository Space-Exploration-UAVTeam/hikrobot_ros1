#include "ros/ros.h"
#include "mavros_msgs/CommandTriggerControl.h"
#include <cstdlib>
#include <string>
#include <std_srvs/Trigger.h>

class TriggerReady
{
public:
	TriggerReady()
	{
		cam0_OK_ = false;
		cam1_OK_ = false;
		framerate_hz_ = 18; // default framerate TODO get this from the ueye node
		triggerClient_ = n_.serviceClient<mavros_msgs::CommandTriggerControl>("/mavros/cmd/trigger_control");
		//advertiseService();
	}
	void advertiseService()
	{
		serverCam0_ = n_.advertiseService("cam0/trigger_ready", &TriggerReady::servCam0, this);
		serverCam1_ = n_.advertiseService("cam1/trigger_ready", &TriggerReady::servCam1, this);
	}
	bool servCam0(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
	{
		cam0_OK_ = true;
		resp.success = true;
		ROS_INFO_STREAM("Camera 0 is primed for trigger");
		return true;
	}
	bool servCam1(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
	{
		cam1_OK_ = true;
		resp.success = true;
		ROS_INFO_STREAM("Camera 1 is primed for trigger");
		return true;
	}
	bool cam0_OK()
	{
		return cam0_OK_;
	}
	bool cam1_OK()
	{
		return cam1_OK_;
	}

	int enableTrigger()
	{
		// srv_.request.cycle_time = (1000 / framerate_hz_);
		srv_.request.trigger_enable = true;
		if (triggerClient_.call(srv_)) 
		{
			ROS_INFO("Successfully enabled camera trigger");
		} else {
			ROS_ERROR("Failed to call trigger_control service");
			return 1;
		}
		return 0;
	}
	int disableTrigger()
	{
		// srv_.request.cycle_time = 0;
		srv_.request.trigger_enable = false;
		if (triggerClient_.call(srv_)) 
		{
			ROS_INFO("Successfully disabled camera trigger");
		} else {
			ROS_ERROR("Failed to call trigger_control service");
			return 1;
		}
		return 0;
	}

private:
	bool cam0_OK_;
	bool cam1_OK_;
	int framerate_hz_;
	ros::NodeHandle n_;
	ros::ServiceClient triggerClient_;
	mavros_msgs::CommandTriggerControl srv_;
	ros::ServiceServer serverCam0_;
	ros::ServiceServer serverCam1_;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "StartTrigger");
	TriggerReady tr;
	ros::Rate r2(5); // Hz
	////wait if Failed to call trigger_control service = pass until trigger_control is called
	while (tr.disableTrigger() && ros::ok()) 
	{
		ROS_INFO_STREAM("Retrying reaching pixhawk");
		r2.sleep();
	}
	// ros::Rate r(100); // Hz
	// while (!(tr.cam0_OK() && tr.cam1_OK()) && ros::ok()) //如果相机和系统不都OK就等待
	// {
	// 	ros::spinOnce();
	// 	r.sleep();
	// }
	// Send start trigger command to Pixhawk
	while (tr.enableTrigger() && ros::ok()) 
	{
		ROS_INFO_STREAM("Retrying reaching pixhawk");
		r2.sleep();
	}
}

