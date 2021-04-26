#ifndef TRACK_H
#define TRACK_H

#include <thread>
#include <vector>
#include <climits>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h> 
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <logistics_msgs/RealState.h>
#include <logistics_msgs/GoalState.h>
#include <math_function/math_function.h>
#include "gps_msgs/Inspvax.h"


class PathTracking
{
public:
	PathTracking();
	~PathTracking();
	bool init_work(ros::NodeHandle nh,ros::NodeHandle nh_private);
	bool init_params(ros::NodeHandle nh,ros::NodeHandle nh_private);
	void run();
	void avoiding_flag_callback(const std_msgs::Float32::ConstPtr& msg);            // 订阅offset
	void pub_car_goal_callback(const ros::TimerEvent&);                             // 定时发布车辆目标状态信息（转角和速度）
	void gps_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);                // 订阅GPS发布过来的消息（包括车辆自身的航向角、车辆自身的utm_x/utm_y/yaw
	void gps_callback(const gps_msgs::Inspvax::ConstPtr& msg);
	void is_object_callback(const std_msgs::Float32::ConstPtr& msg);                // 判断前方是否出现障碍物，最近
	void car_state_callback(const logistics_msgs::RealState::ConstPtr& msg);
	bool is_gps_data_valid(gpsMsg_t& point);
	void rosSpinThread(){ros::spin();}
    bool extendPath(std::vector<gpsMsg_t>& path, float extendDis);
//	std::vector<gpsMsg_t> extendPath(std::vector<gpsMsg_t> path, float extendDis);
private:
	gpsMsg_t pointOffset(const gpsMsg_t& point,float offset);
private:
	ros::Subscriber sub_utm_odom;
	ros::Subscriber sub_gps;
	ros::Subscriber sub_car_state;
	ros::Timer timer_;
	ros::Subscriber sub_is_object; //判断前方是否有障碍物
	ros::Subscriber sub_is_offset; //订阅offset
	ros::Publisher pub_car_goal;
	boost::shared_ptr<boost::thread> rosSpin_thread_ptr_;
	std::string path_points_file_;
	std::vector<gpsMsg_t> path_points_;
	  std::vector<gpsMsg_t> path_extend_points;
	gpsMsg_t current_point_, target_point_;
	gps_msgs::Inspvax m_inspax;
	
	float min_foresight_distance_;
	float disThreshold_;
	float track_speed_;
	float object_data;
	bool vehicle_speed_status_;
	bool is_offset_;
	bool is_back_;
	
	float vehicle_speed_;
	float current_roadwheelAngle_;
	float safety_distance_front_;
	float max_roadwheelAngle_;
	float max_side_accel_;
	float lateral_err_;
	float sumlateral_err_;
	float yaw_err_;
	size_t target_point_index_;
	size_t nearest_point_index_;
	float foreSightDis_speedCoefficient_;
	float foreSightDis_latErrCoefficient_;
	// I控制
	float Ki_;
	float tolerate_laterror_;
	float safety_distance_;
    float steer_clearance_;
    float steer_offset_;
    float omega_;
    float theta_true_;
	float avoiding_offset_;
	double now;
	double timeout;
	int control_rate;
	logistics_msgs::GoalState car_goal;
	logistics_msgs::RealState car_state;
	bool isLocationValid;
};

#endif
