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

class PathPlanning : public AutoDriveBase
{
public:
	ExternControl();
	~ExternControl();
	
	virtual bool init(ros::NodeHandle nh,ros::NodeHandle nh_private) override;
	virtual bool start() override;
	virtual void stop()  override;
	virtual bool isRunning() override;
	virtual controlCmd_t getControlCmd() override;
	
private:
	ros::Timer update_timer_;
	ros::NodeHandle nh_, nh_private_;
	ExternControlBase* controller_;
};


#endif
