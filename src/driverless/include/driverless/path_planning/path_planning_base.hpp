#ifndef _PATH_PLANNING_BASE_H_
#define _PATH_PLANNING_BASE_H_

#include "driverless/structs.h"

/*自动驾驶路径规划基类，子类继承该基类实现路径规划策略
 *为了方便多种路径规划方法实现
 */

class PathPlanningBase
{
public:
    PathPlanningBase() = delete;//禁用默认构造函数
	PathPlanningBase(const PathPlanningBase& ) = delete;//禁用复制构造函数

	explicit PathPlanningBase(const std::string& name)
	{
		is_initialed_ = false;
		is_running_ = false;
		name_ = name;
	}
	virtual ~PathPlanningBase()
	{

	}

	/*@brief 获取控制指令*/
	virtual controlCmd_t getControlCmd()
	{
		std::lock_guard<std::mutex> lock(cmd_mutex_);
		return cmd_;
	}

    virtual bool init(ros::NodeHandle nh,ros::NodeHandle nh_private) = 0;//纯虚函数
	virtual bool start() {is_running_ = true;}
	virtual void stop() {is_running_ = false;}
	virtual bool isRunning() {return is_running_;}


protected: //子类可以访问
	std::mutex cmd_mutex_;
	controlCmd_t cmd_;

	bool is_running_;
	bool is_initialed_;
	std::string name_;

};



#endif
