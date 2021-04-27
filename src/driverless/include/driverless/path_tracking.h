#include <ros/ros.h>
#include <fstream>
#include <vector>
#include <climits>
#include <thread>
#include <mutex>
#include <atomic>

#include <nav_msgs/Path.h>
#include "auto_drive_base.h"
#include <std_msgs/UInt32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include "utils.hpp"
#include <driverless/TrackingState.h>
#include <std_msgs/Float32.h>


class PathTracking : public AutoDriveBase
{
public:
	PathTracking();
	virtual ~PathTracking();
	virtual bool start() override;
	virtual bool init(ros::NodeHandle nh,ros::NodeHandle nh_private) override;	
	bool setExpectSpeed(float speed);

private:
	void  trackingThread();
	GpsPoint pointOffset(const GpsPoint& point,float offset);
	void  publishPathTrackingState();
	void  publishNearestIndex();
	float disToParkingPoint(const ParkingPoint& ParkingPoint);
	float limitSpeedByParkingPoint(const float& speed,const float& acc=5);
	float limitRoadwheelAngleBySpeed(const float& angle, const float& speed);
	void  is_object_callback(const std_msgs::Float32::ConstPtr& msg);
	void  gps_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
	
	float limitLateralErr(const float &laterr, float &sumlaterr);
	float generateRoadwheelAnglBypid(float &temp_roadWheelAngle, const float &errerr, const float &sumerr, const float &kp, const float &ki, const float &kd);
	
	float generateMaxTolarateSpeedByCurvature(const std::vector<GpsPoint>& path_points,
											const size_t& start_search_index,
											const size_t& end_search_index,
											float max_side_accel);
	float generateMaxTolarateSpeedByCurvature(const float& curvature, const float& max_accel);
	float limitSpeedByCurrentRoadwheelAngle(float speed,float angle);
	void publishLocalPath();
	
	inline float limitTheta(float &theta, const float &steerclearance)
	{
		if(theta > steerclearance)
        	theta = steerclearance;
    	else if(theta < -steerclearance)
    		theta = -steerclearance;
		return theta;
	}
	
	inline float limitThetaByOmega(float &theta, const float &omega)
	{
		int sign;
		if(theta > 0) sign = 1;
        else if(theta < 0) sign = -1;
        else sign = 0;
        return theta * sign;
	}
private:
	ros::Timer timer_;
	ros::Publisher pub_tracking_state_;
	ros::Publisher pub_nearest_index_;
	ros::Publisher pub_local_path_;
	
	ros::Subscriber sub_is_object;
	ros::Subscriber sub_utm_odom;

	driverless::TrackingState tracking_state_;
	
	//state
	float expect_speed_;
	float I_sumlateral_err_;
	float lat_err;
	float tolerate_laterror_;
	float currentPoint_err_;
	float lastPoint_err_;
	float P_errErr_;
	std::atomic<float> lat_err_;
	std::atomic<float> yaw_err_;
	size_t target_point_index_;
		
	float min_object_distence;
	double now;
	bool isLocationValid;
	//param
	float foreSightDis_speedCoefficient_;
	float foreSightDis_latErrCoefficient_;
	float min_foresight_distance_;
	float max_target_yaw_err_; //车辆沿圆弧到达预瞄点时的航向与预瞄点航向的偏差最大值
	float disThreshold_;
	float max_side_accel_;
	float control_rate_;
	float i_ki_;
	float d_omega_;
	float p_kp_;
	float steer_clearance_;
	float safety_distance_;
	float timeout_;
	
	GpsPoint current_point_;
};
