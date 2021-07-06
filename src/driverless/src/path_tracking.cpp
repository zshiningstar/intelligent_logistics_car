#include "driverless/path_tracking/path_tracking.h"

#define __NAME__ "pure_tracking"

PathTracking::PathTracking():
	AutoDriveBase(__NAME__),
	path_tracking_controller_(nullptr)
{
	
}

PathTracking::~PathTracking()
{
	if(path_tracking_controller_!=nullptr)
	{
		delete path_tracking_controller_;
		path_tracking_controller_ = nullptr;
	}
}

bool PathTracking::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	std::string which = nh_private.param<std::string>("which_path_tracking_controller", "");
	if(which == "pure_tracking_controller")
		path_tracking_controller_ = new PureTracking();
	else
	{
		ROS_ERROR("[%s] Unknown path_planning controller: %s!", __NAME__, which.c_str());
		return false;
	}
	std::cout << "初始化路径跟踪控制器开始!" << std::endl;
	is_initialed_ = path_tracking_controller_->init(nh, nh_private);
	return is_initialed_;
}

std::pair <float, float> PathTracking::getTrackingErr()
{
	return path_tracking_controller_->getTrackingErr();
}

bool PathTracking::setVehicleParams(VehicleParams temp_params_)
{
	return path_tracking_controller_->setVehicleParams(temp_params_);
}

float PathTracking::setExpectSpeed(float speed)
{
	return path_tracking_controller_->setExpectSpeed(speed);
}

bool PathTracking::setGlobalPath(Path &path_)
{
	return path_tracking_controller_->setGlobalPath(path_);
}

bool PathTracking::start()
{
	if(!is_initialed_)
	{
		ROS_ERROR("[%s] Please init extern controller before start!", __NAME__);
		return false;
	}
	return path_tracking_controller_->start();
}

void PathTracking::stop()
{
	if(is_initialed_)
		path_tracking_controller_->stop();
}

bool PathTracking::isRunning()
{
	return path_tracking_controller_->isRunning();
}

controlCmd_t PathTracking::getControlCmd() 
{
	return path_tracking_controller_->getControlCmd();
}
