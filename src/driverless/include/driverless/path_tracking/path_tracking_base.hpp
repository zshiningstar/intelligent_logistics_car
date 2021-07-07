#ifndef _PATH_PLANNING_BASE_H_
#define _PATH_PLANNING_BASE_H_

#include <driverless_common/structs.h>

/*自动驾驶路径规划基类，子类继承该基类实现路径规划策略
 *为了方便多种路径规划方法实现
 */

class PathTrackingBase
{
public:
    PathTrackingBase() = delete;//禁用默认构造函数
	PathTrackingBase(const PathTrackingBase& ) = delete;//禁用复制构造函数

	explicit PathTrackingBase(const std::string& name):
		lateral_err_(0.0),
		yaw_err_(0.0)
	{
		is_initialed_ = false;
		is_running_ = false;
		name_ = name;
	}
	virtual ~PathTrackingBase()
	{

	}

	/*@brief 获取控制指令*/
	virtual controlCmd_t getControlCmd()
	{
		std::lock_guard<std::mutex> lock(cmd_mutex_);
		return cmd_;
	}
	
	virtual bool setPath(const Path &p)
	{
		path_ = p;
		return true;
	}
	
	virtual float setExpectSpeed(float speed)
	{
		return fabs(speed);
	}

	virtual bool setVehicleParams(const VehicleParams &temp_params_)
	{
		vehicle_params_ = temp_params_;
		std::cout << "车辆自身参数传入成功!" << std::endl;
//		std::cout << "validity: " << vehicle_params_.validity << std::endl;
		return true;
	}
	
	void updataVehicleState(const VehicleState& vs)
	{
		vehicle_state_ = vs;
	}
	
	virtual std::pair <float, float> getTrackingErr()
	{
//		trackingError.first = lateral_err_;
//		trackingError.second = g_yaw_err_;
		return std::make_pair<float,float>(lateral_err_, yaw_err_);
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
	
	Path path_;
	VehicleState vehicle_state_;
	VehicleParams vehicle_params_;
	
	std::atomic<float> lateral_err_;//横向偏差
	std::atomic<float> yaw_err_;    //航向偏差
	
    std::pair <float, float> trackingError;
};



#endif
