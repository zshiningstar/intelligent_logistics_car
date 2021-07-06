#ifndef _PATH_PLANNING_H
#define _PATH_PLANNING_H


#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <vector>
#include <thread>
#include <mutex>
#include "driverless/structs.h"
#include "driverless/utils.hpp"
#include "driverless/auto_drive_base.h"
#include "pure_tracking/pure_tracking.hpp"

class PathTracking : public AutoDriveBase
{
public:
	PathTracking();
	~PathTracking();
	
	virtual bool init(ros::NodeHandle nh,ros::NodeHandle nh_private) override;
	virtual bool start() override;
	virtual void stop()  override;
	virtual bool isRunning() override;
	virtual controlCmd_t getControlCmd() override;
	float setExpectSpeed(float speed);
	bool setGlobalPath(Path &path_);
	bool setVehicleParams(VehicleParams temp_params_);
	std::pair <float, float> getTrackingErr();
	
private:
	ros::Timer update_timer_;
	ros::NodeHandle nh_, nh_private_;
	PathTrackingBase* path_tracking_controller_;
};


#endif
