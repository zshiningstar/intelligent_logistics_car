#include <ros/ros.h>
#include <logistics_msgs/RealState.h>
#include <logistics_msgs/GoalState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt8.h>
#include <vector>
#include <nav_msgs/Odometry.h> 
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float32.h>
#include <climits>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <thread>
#include<ant_math/ant_math.h>

#define MAX_ANGLE  12.3

class PathTracking
{
public:
	PathTracking();
	~PathTracking();
	bool init(ros::NodeHandle nh,ros::NodeHandle nh_private);
	void run();
	void avoiding_flag_callback(const std_msgs::Float32::ConstPtr& msg);
	void pub_car_goal_callback(const ros::TimerEvent&);                       // 定时发布车辆目标状态信息（转角和速度）
	void gps_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);          // 订阅GPS发布过来的消息（包括车辆自身的航向角、车辆自身的utm_x/utm_y/yaw）
	
	void car_state_callback(const logistics_msgs::RealState::ConstPtr& msg);
	
	bool is_gps_data_valid(gpsMsg_t& point);
	void rosSpinThread(){ros::spin();}
	
private:
	gpsMsg_t pointOffset(const gpsMsg_t& point,float offset);
//	void publishPathTrackingState();
private:
	ros::Subscriber sub_utm_odom;
	ros::Subscriber sub_car_state;
	ros::Timer timer_;
	
	ros::Publisher pub_car_goal;
	
	boost::shared_ptr<boost::thread> rosSpin_thread_ptr_;
	
	std::string path_points_file_;
	std::vector<gpsMsg_t> path_points_;
	
	gpsMsg_t current_point_, target_point_;
	
	float min_foresight_distance_;
	float disThreshold_;

	float track_speed_;
	
	bool vehicle_speed_status_;
	
	float vehicle_speed_;
	float current_roadwheelAngle_;
	
	float safety_distance_front_;
	float danger_distance_front_;
	
	float max_roadwheelAngle_;
	float max_side_accel_;
	bool is_avoiding_;
	float lateral_err_;
	float yaw_err_;
	
	size_t target_point_index_;
	size_t nearest_point_index_;
	
	float foreSightDis_speedCoefficient_;
	float foreSightDis_latErrCoefficient_;
	
	float avoiding_offset_;
	
	logistics_msgs::GoalState car_goal;
	logistics_msgs::RealState car_state;
	
};
/*-----------------车辆目标状态初始化---------------*/

PathTracking::PathTracking():
	vehicle_speed_status_(true),
	target_point_index_(0),
	avoiding_offset_(0.0),
	nearest_point_index_(0),
	max_roadwheelAngle_(12.3)
{
	car_goal.goal_speed = 0;
	car_goal.goal_angle = 0;
	car_goal.goal_brake = 0;
	car_goal.goal_park  = 0;
	
	current_point_.x = 0;
	current_point_.y = 0;
}

PathTracking::~PathTracking()
{
}


/*------------------节点初始化--------------------*/

bool PathTracking::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	std::string odom_topic = nh_private.param<std::string>("odom_topic","/odom");   
	sub_utm_odom  = nh.subscribe(odom_topic, 5,&PathTracking::gps_odom_callback,this);     // 订阅来自GPS节点的消息，以获取车辆自身状态信息
	
	std::string car_state = nh_private.param<std::string>("car_state","/car_state");  
	sub_car_state = nh.subscribe(car_state,10,&PathTracking::car_state_callback, this);     // 订阅来自stm32向上位机的车辆反馈信息
	
	pub_car_goal  = nh.advertise<logistics_msgs::GoalState>(nh_private.param<std::string>("car_goal","/car_goal"),10);                     // 发布车辆目标状态信息
	timer_ = nh.createTimer(ros::Duration(0.01),&PathTracking::pub_car_goal_callback,this);    // 设置一个定时器每0.01秒发布一次
	
	
	nh_private.param<std::string>("path_points_file",path_points_file_,"");
	nh_private.param<float>("speed",track_speed_,5.0);

	nh_private.param<float>("foreSightDis_speedCoefficient", foreSightDis_speedCoefficient_,1.8);    // 参数设置
	nh_private.param<float>("foreSightDis_latErrCoefficient", foreSightDis_latErrCoefficient_,0.3);  // 系数
	
	nh_private.param<float>("min_foresight_distance",min_foresight_distance_,3.0);            // 最小前视距离为5米
	nh_private.param<float>("max_side_accel",max_side_accel_,1.5);
	
	if(path_points_file_.empty())
	{
		ROS_ERROR("no input path points file !!");
		return false;
	}
	
	//start the ros::spin() thread
	rosSpin_thread_ptr_ = boost::shared_ptr<boost::thread >(new boost::thread(boost::bind(&PathTracking::rosSpinThread, this)));
	
	if(!loadPathPoints(path_points_file_, path_points_))                                   // ant_math   cpp文件里面
		return false;
	
	ROS_INFO("pathPoints size:%d",path_points_.size());////////////////////11111111
	
	while(ros::ok() && !is_gps_data_valid(current_point_))                                 // 判断是不是有效的gps数据
	{
		ROS_INFO("gps data is invalid, please check the gps topic or waiting...");
		sleep(1);
	}
	
	target_point_index_ = findNearestPoint(path_points_,current_point_);                   // ant_math   
	std::cout << target_point_index_ << " / "  << path_points_.size() << std::endl;
	if(target_point_index_ > path_points_.size() - 10)
	{
		ROS_ERROR("target index:%d ?? file read over, No target point was found !!!",target_point_index_);
		return false;
	}
	
	while(!vehicle_speed_status_ && ros::ok())
	{
		ROS_INFO("waiting for vehicle speed data ok ...");
		usleep(200000);
	}
	
	target_point_ = path_points_[target_point_index_];
	return true;
}


gpsMsg_t PathTracking::pointOffset(const gpsMsg_t& point,float offset)
{
	gpsMsg_t result = point;
	result.x =  offset * cos(point.yaw) + point.x;
	result.y = -offset * sin(point.yaw) + point.y;
	return result;
}

/*--------------------判断是不是有效的GPS数据点----------------*/

bool PathTracking::is_gps_data_valid(gpsMsg_t& point)
{	
//	std::cout << 11111 << std::endl;
	if(point.x > 100 && point.y >100)
	{
//		std::cout << 22222 << std::endl;
		return true;
	}
	return false;
}


/*------------------获取车辆当前的GPS位置坐标信息-------------*/

void PathTracking::gps_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	current_point_.x = msg->pose.pose.position.x;
//	std::cout << current_point_.x << std::endl;
	current_point_.y = msg->pose.pose.position.y;
//	std::cout << current_point_.y << std::endl;
	current_point_.yaw = msg->pose.covariance[0];
}

/*--------------------------生成目标转角---------------------*/

void PathTracking::run()
{
//	std::cout << 555555 << std::endl;
	size_t i =0;
	
	ros::Rate loop_rate(30);
	std::cout << target_point_index_ << " / "  << path_points_.size() << std::endl;
	while(ros::ok() /*&& target_point_index_ < path_points_.size()-2*/)       //(减去2是啥意思)
	{
		try
		{
			lateral_err_ = calculateDis2path(current_point_.x,current_point_.y,path_points_,
											 target_point_index_,&nearest_point_index_) - avoiding_offset_;
		}
		catch(const char* str)
		{
			ROS_ERROR("what happen?");
			ROS_INFO("%s",str);
			break;
		}
		
		disThreshold_ = foreSightDis_speedCoefficient_ * vehicle_speed_ + foreSightDis_latErrCoefficient_ * fabs(lateral_err_);   // 前视距离计算公式
	
		if(disThreshold_ < min_foresight_distance_) 
			disThreshold_  = min_foresight_distance_;
			
//		 disThreshold_ = min_foresight_distance_ + 
//						 foreSightDis_speedCoefficient_ * car_state.real_speed + 
//						 foreSightDis_latErrCoefficient_ * fabs(lateral_err_);
	
		//ROS_INFO("disThreshold:%f\t lateral_err:%f",disThreshold_,lateral_err_);
									 
		std::pair<float, float> dis_yaw = get_dis_yaw(target_point_, current_point_);     //  pair的实现是一个结构体，主要的两个成员变量是first second 

		if( dis_yaw.first < disThreshold_)
		{
			target_point_ = path_points_[target_point_index_++];
			std::cout << target_point_index_ << " / "  << path_points_.size() << std::endl;
			std::cout << "dis_yaw.first: " << dis_yaw.first << "\r\n";
			target_point_.show();
			current_point_.show();
			if(target_point_index_ >= path_points_.size())
				break;
			continue;
		}
		
		yaw_err_ = dis_yaw.second - current_point_.yaw;       // 计算出车身姿态与目标点的夹角
		
		if(yaw_err_==0.0) continue;
		
		float turning_radius = (0.5 * dis_yaw.first)/sin(yaw_err_);                     // 转弯半径  l/2sin(a)

		float t_roadWheelAngle = generateRoadwheelAngleByRadius(turning_radius);         // 生成前轮转角
		
		//t_roadWheelAngle = limitRoadwheelAngleBySpeed(t_roadWheelAngle,vehicle_speed_);  // 受速度限制的前轮转角
//		if(t_roadWheelAngle < 0)
//		{	
//			if(-t_roadWheelAngle > MAX_ANGLE)
//			{	
//				t_roadWheelAngle = MAX_ANGLE;
//			}
//		}
//		if(t_roadWheelAngle > MAX_ANGLE)
//			t_roadWheelAngle = MAX_ANGLE;
//		
		//ROS_INFO("t_roadWheelAngle :%f\n",t_roadWheelAngle);
		
		//find the index of a path point x meters from the current point
		size_t index = findIndexForGivenDis(path_points_,nearest_point_index_,disThreshold_ + 3); 
		if(index ==0)
		{
			ROS_INFO("findIndexForGivenDis faild!");
			break;
		}
		float max_curvature = maxCurvatureInRange(path_points_, nearest_point_index_, index);
		float max_speed = generateMaxTolarateSpeedByCurvature(max_curvature, max_side_accel_);
		
//		std::cout << 11111111 << std::endl;
		
//		car_goal.goal_speed = track_speed_ > max_speed ? max_speed : track_speed_;
		car_goal.goal_speed = track_speed_;
		car_goal.goal_angle = t_roadWheelAngle;
		
//		this->publishPathTrackingState();
		if(i%20==0)
		{
			ROS_INFO("min_r:%.3f\t max_speed:%.1f",1.0/max_curvature, max_speed);
			ROS_INFO("set_speed:%f\t speed:%f",car_goal.goal_speed ,vehicle_speed_);
			ROS_INFO("dis2target:%.2f\t yaw_err:%.2f\t lat_err:%.2f",dis_yaw.first,yaw_err_*180.0/M_PI,lateral_err_);
			ROS_INFO("disThreshold:%f\t expect roadwheel angle:%.2f",disThreshold_,t_roadWheelAngle);
		}
		i++;		
		loop_rate.sleep();
	}
	
	ROS_INFO("driverless completed...");
	
	car_goal.goal_angle = 0.0;
	car_goal.goal_speed = 0.0;
	
	while(ros::ok())
	{
		sleep(1);
	}
}

void PathTracking::avoiding_flag_callback(const std_msgs::Float32::ConstPtr& msg)
{
//	//avoid to left(-) or right(+) the value presents the offset
//	avoiding_offset_ = msg->data;
}


void PathTracking::pub_car_goal_callback(const ros::TimerEvent&)
{
	pub_car_goal.publish(car_goal);
}  
/*--------------------从下位机获取的车辆自身状态信息-------------------*/

void PathTracking::car_state_callback(const logistics_msgs::RealState::ConstPtr& msg)
{
	car_state.real_speed = msg->real_speed;
	car_state.real_angle = msg->real_angle;
	car_state.real_brake = msg->real_brake;
	car_state.real_park  = msg->real_park;
	
	if(vehicle_speed_ >20.0)
		return;
	vehicle_speed_status_ = true;
	vehicle_speed_ = 0; //  km/h
}

/*----------------------------主函数---------------------------------*/

int main(int argc,char**argv)
{
	ros::init(argc,argv,"path_tracking");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	PathTracking path_tracking;
	if(!path_tracking.init(nh,nh_private))
		return 1;
	path_tracking.run();
	
	ROS_INFO("path tracking completed.");
	ros::shutdown();

	return 0;
}
