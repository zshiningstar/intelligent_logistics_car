#include "driverless/path_tracking.h"
#define __NAME__ "path_tracking"

PathTracking::PathTracking():
	AutoDriveBase(__NAME__),
	I_sumlateral_err_(0.0),
	lastPoint_err_(0.0),
	currentPoint_err_(0.0),
	P_errErr_(0.0),
	isLocationValid(false),
	expect_speed_(1.0) //defult expect speed
{
	 
}

PathTracking::~PathTracking()
{
}

bool PathTracking::setExpectSpeed(float speed)
{
	expect_speed_ = speed;
}

bool PathTracking::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	std::string tracking_info_topic = 
	nh_private.param<std::string>("tracking_info_topic","/tracking_state");
	nh_private.param<float>("foreSightDis_speedCoefficient", foreSightDis_speedCoefficient_,1.8);
	nh_private.param<float>("foreSightDis_latErrCoefficient", foreSightDis_latErrCoefficient_,-1.0);
	nh_private.param<float>("min_foresight_distance",min_foresight_distance_,5.0);
	nh_private.param<float>("max_side_accel",max_side_accel_,1.0);
	nh_private.param<float>("tolerate_laterror", tolerate_laterror_,0.3);
	nh_private.param<float>("control_rate",control_rate_,30);
	nh_private.param<float>("d_omega", d_omega_,5.0);
	nh_private.param<float>("i_ki", i_ki_,0.3);
	nh_private.param<float>("p_kp", p_kp_,5.0);
	nh_private.param<float>("steer_clearance", steer_clearance_,0.3);
	nh_private.param<float>("safety_distance",safety_distance_,4);
	nh_private.param<float>("timeout",timeout_,0.3);
	
    std::string avoid_object = nh_private.param<std::string>("is_object","/is_object");   // 是否有障碍物
    sub_is_object = nh.subscribe(avoid_object,10,&PathTracking::is_object_callback, this);
    
    std::string odom_topic = nh_private.param<std::string>("odom_topic","/odom");   
    sub_utm_odom  = nh.subscribe(odom_topic, 5,&PathTracking::gps_odom_callback,this);
    
	max_target_yaw_err_ = nh_private.param<float>("max_target_yaw_err",50.0)*M_PI/180.0;

	pub_tracking_state_ = nh.advertise<driverless::TrackingState>(tracking_info_topic,1);
	pub_nearest_index_  = nh.advertise<std_msgs::UInt32>("/driverless/nearest_index",1);
	pub_local_path_ = nh_private_.advertise<nav_msgs::Path>("/local_path",2);

	initDiagnosticPublisher(nh,__NAME__);

	is_ready_ = true;
		
	return true;
}

void PathTracking::is_object_callback(const std_msgs::Float32::ConstPtr& msg)
{       
    min_object_distence = msg->data;
    //std::cout << " 有障碍物体,距离为:" << min_object_distence << std::endl;
    now = ros::Time::now().toSec();
}

void PathTracking::gps_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	current_point_.x = msg->pose.pose.position.x;
	current_point_.y = msg->pose.pose.position.y;
	current_point_.yaw = msg->pose.covariance[0];
	
	isLocationValid = !msg->pose.covariance[4];
}
//启动跟踪线程
bool PathTracking::start()
{
	if(!is_ready_)
	{
		ROS_ERROR("[%s] System is not ready!",__NAME__);
		return false;
	}
	if(global_path_.size()==0)
	{
		ROS_ERROR("[%s] No global path!",__NAME__);
		return false;
	}
	if(global_path_.park_points.size()==0)
	{
		ROS_ERROR("[%s] No parking points!",__NAME__);
		return false;
	}

	if(vehicle_params_.validity == false)
	{
		ROS_ERROR("[%s] Vehicle parameters is invalid, please set them firstly.",__NAME__);
		return false;
	}

	is_running_ = true;
	std::thread t(&PathTracking::trackingThread,this);
	t.detach();
	return true;
}

//跟踪线程
void PathTracking::trackingThread()
{
	global_path_.pose_index = findNearestPoint(global_path_, vehicle_state_.getPose(LOCK)); 
	size_t nearest_index = global_path_.pose_index;

	if(nearest_index > global_path_.size() - 3)
	{
		ROS_ERROR("Remaind target path is too short! nearest_point_index:%lu", nearest_index);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR,"Remaind target path is too short!");
		is_running_ = false;
		return ;
	}

	ros::Rate loop_rate(30);
	float delta_t = 1.0/control_rate_;
	
	size_t cnt =0;
	while(ros::ok() && is_running_ && !global_path_.finish())
	{
		float goal_speed = expect_speed_;
		const Pose pose = vehicle_state_.getPose(LOCK);
		const float vehicle_speed = vehicle_state_.getSpeed(LOCK);
		lastPoint_err_ = currentPoint_err_;
		//横向偏差,左偏为负,右偏为正
		lat_err = calculateDis2path(pose.x, pose.y, global_path_, nearest_index, &nearest_index);
		/**************PID-Kp*****************/
		currentPoint_err_ = lat_err;
		P_errErr_ = currentPoint_err_ - lastPoint_err_;
		I_sumlateral_err_ = limitLateralErr(lat_err,I_sumlateral_err_);
		global_path_.pose_index = nearest_index; //更新到基类公用变量
		//航向偏差,左偏为正,右偏为负
		float yaw_err = global_path_[nearest_index].yaw - pose.yaw;
		if(yaw_err > M_PI)       yaw_err -= 2*M_PI;
		else if(yaw_err < -M_PI) yaw_err += 2*M_PI;

		yaw_err_ = yaw_err; //update the member var
		lat_err_ = lat_err; //update the member var
		
		disThreshold_ = foreSightDis_speedCoefficient_ * vehicle_speed + foreSightDis_latErrCoefficient_ * fabs(lat_err);
	
		if(disThreshold_ < min_foresight_distance_) 
			disThreshold_  = min_foresight_distance_;
		size_t target_index = nearest_index+1;//跟踪目标点索引
	
		GpsPoint target_point = global_path_[target_index];
		//获取当前点到目标点的距离和航向
		std::pair<float, float> dis_yaw = getDisAndYaw(target_point, pose);

		//循环查找满足disThreshold_的目标点
		while(dis_yaw.first < disThreshold_)
		{
			target_point = global_path_[++target_index];
			dis_yaw = getDisAndYaw(target_point, pose);
//			std::cout << target_point.x << "\t" << target_point.y << "\n"
//						<< pose.x << "\t" << pose.y << "\t" << dis_yaw.first << std::endl;
		}
		float theta = dis_yaw.second - pose.yaw;
		float sin_theta = sin(theta);
        if(sin_theta == 0)
            continue;
		
		float turning_radius = (0.5 * dis_yaw.first)/sin(theta);

		float t_roadWheelAngle = generateRoadwheelAngleByRadius(vehicle_params_.wheel_base, turning_radius);
		
		t_roadWheelAngle = limitRoadwheelAngleBySpeed(t_roadWheelAngle, vehicle_speed);
		t_roadWheelAngle = generateRoadwheelAnglBypid(t_roadWheelAngle, P_errErr_, I_sumlateral_err_, p_kp_, i_ki_, d_omega_);
		
		if(min_object_distence)
        {//判断是否有障碍物
            if(goal_speed > 0 && min_object_distence < safety_distance_)
            {   
                //float acceleration = (track_speed_ * track_speed_) / (2 * fabs(min_object_distence - safety_distance_));
                //car_goal.goal_speed = track_speed_ - acceleration;
                //track_speed_ = car_goal.goal_speed;
                goal_speed = 0;
            }
            double now_break = now - ros::Time::now().toSec(); 
            if(fabs(now_break) > timeout_)
            {
                min_object_distence = 0;
             	goal_speed = expect_speed_;
         	}
        }
        if(!isLocationValid)
        	goal_speed = 0.0;
		//float curvature_search_distance = disThreshold_ + 13; //曲率搜索距离
		float curvature_search_distance = vehicle_speed * vehicle_speed/(2 * 1);
		float max_curvature = maxCurvatureInRange(global_path_, nearest_index, curvature_search_distance);

		float max_speed_by_curve = generateMaxTolarateSpeedByCurvature(max_curvature, max_side_accel_);
		float max_speed_by_park =  limitSpeedByParkingPoint(max_speed_by_curve);
		float max_speed = max_speed_by_park;

		cmd_mutex_.lock();
		cmd_.validity = true;
		cmd_.speed = (goal_speed>max_speed) ? max_speed : expect_speed_;
		cmd_.roadWheelAngle = t_roadWheelAngle;
		cmd_mutex_.unlock();
		
		publishPathTrackingState();
		publishNearestIndex();
		
		
		if((++cnt)%50==1)
		{
			ROS_INFO("min_r:%.3f\t max_speed:%.1f",1.0/max_curvature, max_speed);
			ROS_INFO("max_v: expect:%.1f curve:%.1f  park:%.1f",expect_speed_, max_speed_by_curve, max_speed_by_park);
			ROS_INFO("set_v:%f\t true_v:%f",cmd_.speed ,vehicle_speed*3.6);
			ROS_INFO("yaw: %.2f\t targetYaw:%.2f", pose.yaw*180.0/M_PI , dis_yaw.second *180.0/M_PI);
			ROS_INFO("dis2target:%.2f\t yaw_err:%.2f\t lat_err:%.2f",dis_yaw.first,yaw_err_*180.0/M_PI,lat_err);
			ROS_INFO("disThreshold:%f\t expect roadwheel angle:%.2f",disThreshold_,t_roadWheelAngle);
			publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK,"Running");
			publishLocalPath();
			ROS_INFO("near_index:%lu\t goal_index:%lu\t final_index:%lu",
				nearest_index,target_index, global_path_.final_index);
			printf("now:(%.2f,%.2f)\tnear:(%.2f,%.2f)\tgoal:(%.2f,%.2f)",
				pose.x, pose.y, global_path_[nearest_index].x, global_path_[nearest_index].y, target_point.x, target_point.y);
		}
		loop_rate.sleep();
	}
	
	publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK,"Arrived at the destination.");
	ROS_INFO("[%s] path_tracking completed...", __NAME__);
	
	cmd_mutex_.lock();
	cmd_.speed = 0.0;
	cmd_.roadWheelAngle = 0.0;
	cmd_mutex_.unlock();
	
	is_running_ = false;
}

void PathTracking::publishPathTrackingState()
{
	if(pub_tracking_state_.getNumSubscribers())
	{
		const Pose pose = vehicle_state_.getPose(LOCK);
		const float speed = vehicle_state_.getSpeed(LOCK);
		tracking_state_.header.stamp = ros::Time::now();
		tracking_state_.position_x = pose.x;
		tracking_state_.position_y = pose.y;
		tracking_state_.yaw = pose.yaw;
		tracking_state_.vehicle_speed =  speed;
		tracking_state_.roadwheel_angle = vehicle_state_.getSteerAngle(LOCK);
		tracking_state_.lateral_error = lat_err_;
		tracking_state_.yaw_error = yaw_err_;
	
		pub_tracking_state_.publish(tracking_state_);
	}
}

void PathTracking::publishNearestIndex()
{
	static std_msgs::UInt32 msg;
	if(pub_nearest_index_.getNumSubscribers())
	{
		msg.data = global_path_.pose_index;
		pub_nearest_index_.publish(msg);
	}
}

GpsPoint PathTracking::pointOffset(const GpsPoint& point,float offset)
{
	GpsPoint result = point;
	result.x =  offset * cos(point.yaw) + point.x;
	result.y = -offset * sin(point.yaw) + point.y;
	return result;
}

/*@brief 当前位置到停车点的距离
*/
float PathTracking::disToParkingPoint(const ParkingPoint& parking_point)
{
	if(global_path_.pose_index >=  parking_point.index) //当前位置已经超过停车点
		return 0;
	
	float dis1 = global_path_.resolution * (parking_point.index-global_path_.pose_index); //估计路程
	float dis2 = disBetweenPoints(global_path_[parking_point.index], global_path_[global_path_.pose_index]);//直线距离
	float dis2end = dis1 > dis2 ? dis1 : dis2;
	return dis2end;
	
}

/*@brief 根据停车点限制车辆速度
*/
float PathTracking::limitSpeedByParkingPoint(const float& speed,const float& acc)
{
	ParkingPoints& parking_points = global_path_.park_points;
	//初始化时parking_points至少有一个点(终点)
	//每到达一个停车点并完成停车后,更新下一个停车点
	//程序初始化时,应将停车点由近及远排序,并将位于当前位置之后的停车点复位
	if(parking_points.size() == 0)
	{
		ROS_DEBUG("[%s] No Parking Points, Speed prohibition!",__NAME__);
		return 0.0;
	}
	
	//无可用停车点,已经到达终点
	if(!parking_points.available())
	{
		ROS_DEBUG("[%s] No Next Parking Point, destination reached!",__NAME__);
		return 0.0;
	}
	
	while(parking_points.next().index < global_path_.pose_index)
	{//更新停车点
		++ parking_points.next_index;
		if(!parking_points.available())
			return 0.0;
	}
		
	ParkingPoint& parking_point = parking_points.next();
	
	if(parking_point.isParking) //正在停车中
	{
		//停车周期为0,到达终点
		if(parking_point.parkingDuration == 0.0)
			return 0.0;
		//停车超时,移除当前停车点,下次利用下一停车点限速
		if(ros::Time::now().toSec()-parking_point.parkingTime >= parking_point.parkingDuration)
		{
			ROS_INFO("parking overtime. parking point :%lu",parking_point.index);
			return speed;
		}
		//正在停车,时间未到
		return 0.0;
	}
	
	float dis2ParkingPoint = disToParkingPoint(parking_point);
	float maxSpeed = sqrt(2*acc*dis2ParkingPoint);
	//ROS_ERROR("parking_point :%d" , parking_point.index);
	//ROS_INFO("dis2ParkingPoint:%.2f\tmaxSpeed:%.2f",dis2ParkingPoint,maxSpeed);
	if(dis2ParkingPoint < 0.5)//到达停车点附近,速度置0,,防止抖动
	{
		parking_point.parkingTime = ros::Time::now().toSec();
		parking_point.isParking = true;
		ROS_INFO("[%s] start parking. point:%lu",__NAME__, parking_point.index);
		return 0.0;
	}
	return speed > maxSpeed ? maxSpeed : speed;
}

/*@brief 由于物流车机械结构待优化,需要通过PID控制循迹
 *@param kp 
 *@param ki
 *@param kd
 *@return t_roadWheelAngle 前轮转角
 */
float PathTracking::generateRoadwheelAnglBypid(float &temp_roadWheelAngle, const float &errerr, const float &sumerr, const float &kp, const float &ki, const float &kd)
{
	float roadWheelAngle = temp_roadWheelAngle;
	float theta_byerrerr = kp * errerr;
	theta_byerrerr = limitTheta(theta_byerrerr, steer_clearance_);
	float theta_bysteerclearance = ki * sumerr;
	theta_bysteerclearance = limitTheta(theta_bysteerclearance, steer_clearance_);
	float theta_byomega = limitThetaByOmega(roadWheelAngle, kd);
	roadWheelAngle = theta_byerrerr + theta_bysteerclearance + theta_byomega;
	return roadWheelAngle;
}
/*@brief 计算并限制横向误差
 *@brief 横向偏差较小时不修正,因为稳态误差不能全部消除,否则会转向激进
 *@brief 横向偏差变号时,说明行驶靠右/左,总横向偏差需要置0
 *@param lateral_err_ 当前横向误差
 *@param sum_err_ 当前总横向误差
 *@return 当前总横向误差
 */
float PathTracking::limitLateralErr(const float &laterr, float &sumlaterr)
{
	float lateral_err_ = laterr;
	float sum_err_ = sumlaterr;
	if(fabs(lateral_err_) > tolerate_laterror_) 
		sum_err_ = sum_err_ + lateral_err_;
	if(sum_err_ * lateral_err_ < 0)
		sum_err_ = 0;
	return sum_err_;
}
/*@brief 根据当前车速限制车辆前轮转角
 *@brief 算法已经根据路径曲率对车速进行了限制，
 *@brief 但可能出现转弯半径小于路径转弯半径,而导致侧向加速度超限的情况
 *@param angle 期望转角
 *@param speed 车速
 *@return 限制后的转角
 */
float PathTracking::limitRoadwheelAngleBySpeed(const float& angle, const float& speed)
{
	float min_steering_radius = speed*speed/max_side_accel_;
	
	if(min_steering_radius < 1.0)
		return angle;
	
	float max_angle = fabs(generateRoadwheelAngleByRadius(vehicle_params_.wheel_base, min_steering_radius));
	if(max_angle > vehicle_params_.max_roadwheel_angle)
	   max_angle = vehicle_params_.max_roadwheel_angle;
	//ROS_INFO("max_angle:%f\t angle:%f",max_angle,angle);
	return saturationEqual(angle,max_angle);
}

/*@brief 利用路径曲率限制最大车速
 *@brief 保证车辆侧向加速度在一定范围
 *@param curvature 路径曲率
 *@param max_accel 最大允许侧向加速度
 *
 *@return 最大车速 km/h
 */
float PathTracking::generateMaxTolarateSpeedByCurvature(const float& curvature, const float& max_accel)
{
	return sqrt(1.0/fabs(curvature)*max_accel) *3.6;
}

/*@brief 利用路径曲率限制最大车速
 *@brief 保证车辆侧向加速度在一定范围
 *@param path_points        路径点
 *@param start_search_index 搜索起点
 *@param end_search_index   搜索终点
 *
 *@return 最大车速 km/h
 */
float PathTracking::generateMaxTolarateSpeedByCurvature(const std::vector<GpsPoint>& path_points,
											const size_t& start_search_index,
											const size_t& end_search_index,
											float max_side_accel)
{
	float max_cuvature = 0.0;
		
	for(size_t i=start_search_index; i < end_search_index; ++i)
	{
		if(fabs(path_points[i].curvature) > max_cuvature)
			max_cuvature = fabs(path_points[i].curvature);
	}
	return sqrt(1.0/max_cuvature*max_side_accel) *3.6;
}

/*@brief 利用当前前轮转角限制车速
 *@brief 保证车辆侧向加速度在一定范围
 *@param speed			期望车速
 *@param angle 			当前转角
 *
 *@return 最大车速 km/h
 */
float PathTracking::limitSpeedByCurrentRoadwheelAngle(float speed,float angle)
{
	float steering_radius = fabs(vehicle_params_.wheel_base/tan(angle*M_PI/180.0));
	float max_speed =  sqrt(steering_radius*max_side_accel_);
	
	return (speed>max_speed? max_speed: speed)*3.6;
}

void PathTracking::publishLocalPath()
{
    nav_msgs::Path path;
    path.header.stamp=ros::Time::now();
    path.header.frame_id="base_link";
	size_t startIndex = global_path_.pose_index;
	size_t endIndex   = global_path_.final_index;
	if(endIndex <= startIndex)
		return;

	Pose origin_point = vehicle_state_.getPose(LOCK);

	path.poses.reserve(endIndex-startIndex+1);
	
	for(size_t i=startIndex; i<endIndex; ++i)
	{
		const auto& global_point = global_path_[i];
		std::pair<float,float> local_point = 
			global2local(origin_point.x,origin_point.y,origin_point.yaw, global_point.x,global_point.y);
		
		geometry_msgs::PoseStamped poseStamped;
        poseStamped.pose.position.x = local_point.first;
        poseStamped.pose.position.y = local_point.second;

        poseStamped.header.frame_id="base_link";
        path.poses.push_back(poseStamped);
	}
	pub_local_path_.publish(path);
}

