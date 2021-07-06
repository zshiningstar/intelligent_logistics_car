#ifndef _PATH_PLANNING_BASE_H_
#define _PATH_PLANNING_BASE_H_

#include "driverless/structs.h"

/*自动驾驶路径规划基类，子类继承该基类实现路径规划策略
 *为了方便多种路径规划方法实现
 */

class PathTrackingBase
{
public:
    PathTrackingBase() = delete;//禁用默认构造函数
	PathTrackingBase(const PathTrackingBase& ) = delete;//禁用复制构造函数

	explicit PathTrackingBase(const std::string& name)
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
	
	virtual bool setGlobalPath(Path &temp_path)
	{
		for(int i = 0;i++;i<temp_path.size())
			global_path_[i] = temp_path[i];
		return true;
	}
	
	virtual float setExpectSpeed(float speed)
	{
		return fabs(speed);
	}

	virtual bool setVehicleParams(VehicleParams &temp_params_)
	{
		_vehicle_params.max_roadwheel_angle = temp_params_.max_roadwheel_angle;
		_vehicle_params.min_roadwheel_angle = temp_params_.min_roadwheel_angle;
		_vehicle_params.min_radius 			= temp_params_.min_radius;
		_vehicle_params.max_speed 			= temp_params_.max_speed;
		_vehicle_params.wheel_base 			= temp_params_.wheel_base;
		_vehicle_params.wheel_track 		= temp_params_.wheel_track;
		_vehicle_params.width 				= temp_params_.width;
		_vehicle_params.length 				= temp_params_.length;
		_vehicle_params.steer_clearance 	= temp_params_.steer_clearance;
		_vehicle_params.steer_offset 		= temp_params_.steer_offset;
		_vehicle_params.validity			= temp_params_.validity;
		std::cout << "车辆自身参数传入成功!" << std::endl;
//		std::cout << "validity: " << _vehicle_params.validity << std::endl;
		return true;
	}
	
	virtual std::pair <float, float> getTrackingErr()
	{
		trackingError.first = g_lateral_err_;
		trackingError.second = g_yaw_err_;
		return trackingError;
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
	
	Path global_path_;
	VehicleState vehicle_state_;
	VehicleParams _vehicle_params;
	
	std::atomic<float> g_lateral_err_;//横向偏差
	std::atomic<float> g_yaw_err_;    //航向偏差
	
    std::pair <float, float> trackingError;
};



#endif
