#include "driverless/path_planning/path_planning.h"

#define __NAME__ "pure_tracking"

PathPlanning::PathPlanning():
	AutoDriveBase(__NAME__),
	controller_(nullptr)
{
	
}

PathPlanning::~PathPlanning()
{
	if(controller_!=nullptr)
	{
		delete path_planning_controler_;
		path_planning_controler_ = nullptr;
	}
}

bool PathPlanning::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	std::string which = nh_private.param<std::string>("which_path_planning_controller", "");
	if(which == "pure_tracking_controller")
		path_planning_controler_ = new PureTracking();
	else
	{
		ROS_ERROR("[%s] Unknown path_planning controller: %s!", __NAME__, which.c_str());
		return false;
	}
	is_initialed_ = path_planning_controler_->init(nh, nh_private);
	return is_initialed_;
}


bool PathPlanning::start()
{
	if(!is_initialed_)
	{
		ROS_ERROR("[%s] Please init extern controller before start!", __NAME__);
		return false;
	}
	return path_planning_controler_->start();
}

void PathPlanning::stop()
{
	if(is_initialed_)
		path_planning_controler_->stop();
}

bool PathPlanning::isRunning()
{
	return path_planning_controler_->isRunning();
}

controlCmd_t PathPlanning::getControlCmd() 
{
	return path_planning_controler_->getControlCmd();
}
