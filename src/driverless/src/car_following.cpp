#include "driverless/car_following.h"

#define __NAME__ "car_following"

float offset = 0.0/180.0*M_PI;

CarFollowing::CarFollowing():
	AutoDriveBase(__NAME__)
{
	targetId_ = 0xff; //no target
	cmd_update_time_ = 0.0;
	safety_side_dis_ = 0.0+1.0+0.0;
}

bool CarFollowing::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	nh_ = nh;
	nh_private_ = nh_private;
	//objects_topic_     = nh_private.param<std::string>("mm_radar_objects_topic","/esr_objects");
	base_link_frame_   = nh_private.param<std::string>("base_link_frame", "base_link");
	target_repeat_threshold_ = nh_private.param<int>("target_repeat_threshold", 5);

	initDiagnosticPublisher(nh_,__NAME__);
	is_initialed_ = true;
	return true;
}


//启动跟踪线程
bool CarFollowing::start()
{
	if(global_path_.size() == 0)
	{
		ROS_ERROR("[%s]: global path is empty!", __NAME__);
		return false;
	}
	if(global_path_.park_points.size() == 0)
	{
		ROS_ERROR("[%s]: No parking points!", __NAME__);
		return false;
	}

	//确保停车点有序
	if(!global_path_.park_points.isSorted())
		global_path_.park_points.sort();

	/*@brief 获取终点索引,
	 * 用于限制障碍物搜索距离,超出终点的目标不予考虑,
	 * 以保证车辆驶入终点,而不被前方障碍干扰
	 * 从停车点文件获取停车时间为0(终点)的点
	 */
	for(const ParkingPoint& point : global_path_.park_points.points)
	{
		if(point.parkingDuration == 0.0) //查找第一个停车时间为0的停车点(终点)
			dst_index_ = point.index;
	}
		
	is_running_ = true;
	//sub_objects_  = nh_.subscribe(objects_topic_,10,&CarFollowing::object_callback,this);
	update_timer_ = nh_.createTimer(ros::Duration(0.10),&CarFollowing::updateTimer_callback,this);
	pub_local_path_=nh_.advertise<nav_msgs::Path>("/local_path",2);
	return true;
}

void CarFollowing::stop()
{
	sub_objects_.shutdown();
	update_timer_.stop();
	is_running_ = false;
	
	cmd_mutex_.lock();
	cmd_.validity = false;
	cmd_mutex_.unlock();
}

void CarFollowing::updateTimer_callback(const ros::TimerEvent&)
{
	//控制指令长时间未更新,有效位复位
	if(ros::Time::now().toSec() - cmd_update_time_ > 0.2)
	{
		cmd_mutex_.lock();
		cmd_.validity = false;
		cmd_mutex_.unlock();
	}
	publishLocalPath();
}

//1. 目标分类，障碍物/非障碍物(假定目标尺寸，计算与全局路径的距离)
//2. 障碍物由近及远排序，跟踪最近的障碍
//3. 判断最近目标ID是否与上次ID一致！
//void CarFollowing::object_callback(const esr_radar::ObjectArray::ConstPtr& objects)
//{
//	if(!is_ready_) return;
//	if(objects->objects.size()==0) return;
//	
//	//获取mm_radar与base_link(gps)间的坐标变换
//	static bool transform_ok = false;
//	if(!transform_ok)
//	{
//		if(!tf_listener.canTransform(base_link_frame_, objects->header.frame_id, ros::Time(0))) 
//		{
//			ROS_ERROR_STREAM("failed to find transform between " << base_link_frame_ << " and " << objects->header.frame_id);
//			return;
//		}
//		tf_listener.waitForTransform(base_link_frame_, objects->header.frame_id, ros::Time(0), ros::Duration(2.0));
//		tf_listener.lookupTransform(base_link_frame_, objects->header.frame_id, ros::Time(0), transform_);
//		transform_ok = true;
//		const auto& T = transform_.getOrigin();
//		const auto& R = transform_.getBasis();
//		radar_in_base_x_   = T[0];
//		radar_in_base_y_   = T[1];
//		double roll,yaw,pitch;
//		R.getRPY(roll,pitch,yaw);
//		radar_in_base_yaw_ = yaw;
//		ROS_INFO("mm_radar in base_link, x:%.2f  y:%.2f  yaw:%.2f",radar_in_base_x_,radar_in_base_y_,radar_in_base_yaw_*180.0/M_PI);
//	}

//	//创建车辆状态副本
//	const VehicleState vehicle = vehicle_state_;
//	const Pose& vehicle_pose = vehicle.pose;

//	//跟车距离： x = v*v/(2*a) + C常
//	follow_distance_ = vehicle.speed *vehicle.speed/(2*4.0)  + 8.0;
//	
//	//目标分类,障碍物和非障碍物
//	std::vector<esr_radar::Object> obstacles;
//	float targetDis2path = FLT_MAX;
//	for(size_t i=0; i< objects->objects.size(); ++i)
//	{
//		const esr_radar::Object& object = objects->objects[i];
//		if(object.distance > 50) //不考虑远处的目标
//			continue;
//		//目标雷达坐标转向GPS坐标
//		std::pair<float, float> base_pose = 
//			local2global(radar_in_base_x_,radar_in_base_y_,radar_in_base_yaw_, object.x,object.y);
//		
//		//目标局部坐标转换到大地全局坐标
//		std::pair<float, float> object_global_pos =  
//			local2global(vehicle_pose.x,vehicle_pose.y,-vehicle_pose.yaw, base_pose.first,base_pose.second);
//		
//		//计算目标到全局路径的距离
//		float dis2path = calculateDis2path(object_global_pos.first, object_global_pos.second, global_path_, global_path_.pose_index, dst_index_);
//		//std::cout << std::fixed << std::setprecision(2) <<
//		//	object.x <<"  "<<object.y <<"\t" << dis2path << std::endl;
//		if(fabs(dis2path) < safety_side_dis_)
//		{
//			targetDis2path = dis2path;
//			obstacles.push_back(object);
//		}
//	}
//	if(obstacles.size() == 0) return;

//	//std::sort(obstacles.begin(),obstacles.end(),[](const auto& obj1, const auto& obj2){return obj1.distance < obj2.distance});
//	//查找最近的障碍物
//	float minDis = FLT_MAX;
//	size_t nearestObstalIndex = 0;
//	for(size_t i=0; i<obstacles.size(); ++i)
//	{
//		if(obstacles[i].distance < minDis)
//		{
//			minDis = obstacles[i].distance;
//			nearestObstalIndex = i;
//		}
//	}
//	const esr_radar::Object& nearestObstal = obstacles[nearestObstalIndex];
//	
//	static uint8_t lastTargetId = 255; //上一时刻目标id
//	static int     targetRepeatTimes = 0; //目标重复出现次数
//	
//	//当前时刻目标id与上一时刻一致,及目标重复出现,计数器自加
//	//当前时刻目标id与上一时刻不同,则目标为误检目标或者跟踪目标丢失
//	//未处于跟踪状态时,目标重复出现N次则开始跟踪
//	//处于跟踪状态时,若当前id与上次id不同,即便是误检测,也归零计数器
//	//等待下次满足条件时继续跟踪
//	if(nearestObstal.id == lastTargetId)
//	{
//		//目标重复出现计数器自加,应防止溢出
//		if(++targetRepeatTimes > 0xffff)
//			targetRepeatTimes = 0xffff;
//	}
//	else
//	{
//		lastTargetId = nearestObstal.id;
//		targetRepeatTimes = 0;
//	}
//	
//	if(targetRepeatTimes < target_repeat_threshold_)
//		return;
//	
//	//distanceErr>0    acceleration
//	//distanceErr<0    deceleration
//	float distanceErr = nearestObstal.distance -follow_distance_;

//	//线控底盘加速度和减速度未知，只能模糊调整，后期设定底盘加速度为定值或区间定值
//	float t_speed;  //m/s
//	//加速度与减速度不同，单独计算
//	if(distanceErr >= 0)
//		t_speed = vehicle.speed + nearestObstal.speed + distanceErr *0.5; 
//	else
//		t_speed = vehicle.speed + nearestObstal.speed + distanceErr *0.3;
//			
//	ROS_INFO("target dis:%.2f  speed:%.2f  dis2path:%.2f  vehicle speed:%.2f  t_speed:%.2f  t_dis:%.2f   dis:%f",
//		minDis,vehicle.speed + nearestObstal.speed,targetDis2path, vehicle.speed,t_speed,follow_distance_,nearestObstal.distance);
//	ROS_INFO("target x:%.2f  y:%.2f  id:%2d",nearestObstal.x,nearestObstal.y,nearestObstal.id);
//	
//	//防止速度抖动
//	if(t_speed < 1.0)
//		t_speed = 0.0;
//	
//	cmd_update_time_ = ros::Time::now().toSec();
//	cmd_mutex_.lock();
//	cmd_.validity = true;
//	cmd_.speed = t_speed;
//	cmd_mutex_.unlock();
//}

/*@brief 发布局部路径以在RVIZ显示
 *
 */
void CarFollowing::publishLocalPath()
{
	nav_msgs::Path path;
    path.header.stamp=ros::Time::now();
    path.header.frame_id="base_link";
	size_t startIndex = global_path_.pose_index;
	size_t endIndex   = std::min(startIndex+200, global_path_.final_index);
	if(endIndex <= startIndex)
		return;

	const Pose origin_point = vehicle_state_.getPose(LOCK);

	path.poses.reserve(endIndex-startIndex+1);
	
	for(size_t i=startIndex; i<endIndex; ++i)
	{
		const auto& global_point = global_path_[i];
		std::pair<float,float> local_point = 
			global2local(origin_point.x,origin_point.y,-origin_point.yaw, global_point.x,global_point.y);
		
		geometry_msgs::PoseStamped poseStamped;
        poseStamped.pose.position.x = local_point.first;
        poseStamped.pose.position.y = local_point.second;

//        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(origin_point.yaw-global_point.yaw);
//        poseStamped.pose.orientation.x = goal_quat.x;
//        poseStamped.pose.orientation.y = goal_quat.y;
//        poseStamped.pose.orientation.z = goal_quat.z;
//        poseStamped.pose.orientation.w = goal_quat.w;

        //poseStamped.header.stamp=current_time;
        poseStamped.header.frame_id="base_link";
        path.poses.push_back(poseStamped);
	}
	pub_local_path_.publish(path);
}
