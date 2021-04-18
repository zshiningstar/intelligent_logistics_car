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
#include <math_function/math_function.h>   // 和头文件的名字保持一致;和define里面的不用保持一致
#include "gps_msgs/Inspvax.h"

// 目标转角向左为+,向右为-;
// 横向偏差向左为-,向右为+;

class PathTracking
{
public:
	PathTracking();
	~PathTracking();
	bool init(ros::NodeHandle nh,ros::NodeHandle nh_private);
	void run();
	void avoiding_flag_callback(const std_msgs::Float32::ConstPtr& msg);            // 订阅offset
	void pub_car_goal_callback(const ros::TimerEvent&);                             // 定时发布车辆目标状态信息（转角和速度）
	void gps_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);                // 订阅GPS发布过来的消息（包括车辆自身的航向角、车辆自身的utm_x/utm_y/yaw
	void gps_callback(const gps_msgs::Inspvax::ConstPtr& msg);
	void is_object_callback(const std_msgs::Float32::ConstPtr& msg);                // 判断5m内是否有障碍物闯入
	
	void car_state_callback(const logistics_msgs::RealState::ConstPtr& msg);
	
	bool is_gps_data_valid(gpsMsg_t& point);
	void rosSpinThread(){ros::spin();}
private:
	gpsMsg_t pointOffset(const gpsMsg_t& point,float offset);
private:
	ros::Subscriber sub_utm_odom;
	ros::Subscriber sub_gps;
	ros::Subscriber sub_car_state;
	ros::Timer timer_;
	ros::Subscriber sub_is_object; //是否有障碍物闯入5m之内
	ros::Subscriber sub_is_offset; //订阅offset
	ros::Publisher pub_car_goal;
	boost::shared_ptr<boost::thread> rosSpin_thread_ptr_;
	std::string path_points_file_;
	std::vector<gpsMsg_t> path_points_;
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
	float danger_distance_front_;
	float max_roadwheelAngle_;
	float max_side_accel_;
	bool is_avoiding_;
	float lateral_err_;
	// sumlateral_err_
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
	bool check_gps_;
	bool is_inroom_;
	
	bool isLocationValid;
};

/*
 *@fuc:     车辆目标状态初始化
 *param:    object_data:livox前方是否有障碍物体,范围可自己设定
 *param:    max_roadwheelAngle_:最大前轮转角
 */

PathTracking::PathTracking():
	vehicle_speed_status_(true),
	target_point_index_(0),
	avoiding_offset_(0.0),
	nearest_point_index_(0),
	max_roadwheelAngle_(12.3),
	object_data(0),
	lateral_err_(0),
	sumlateral_err_(0),
	vehicle_speed_(0),
	theta_true_(0),
	isLocationValid(false)
{
	car_goal.goal_speed = 0;
	car_goal.goal_angle = 0;
	car_goal.goal_brake = 0;
	car_goal.goal_park  = 0;
	
	//current_point_.x = 0;
	//current_point_.y = 0;
}

PathTracking::~PathTracking()
{
}


/*
 *@fuc:   节点初始化
 *@param  is_offset_ : 是否接收offset
 *@param  is_offset  : offset的值
 *
 */
 
bool PathTracking::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	std::string odom_topic = nh_private.param<std::string>("odom_topic","/odom");   
	sub_utm_odom  = nh.subscribe(odom_topic, 5,&PathTracking::gps_odom_callback,this);               
	
	std::string gps_topic = nh_private.param<std::string>("gps_topic","/gps");   
	sub_gps  = nh.subscribe(gps_topic, 5,&PathTracking::gps_callback,this);               
	
	std::string is_object = nh_private.param<std::string>("is_object","/is_object");  
	sub_is_object = nh.subscribe(is_object,10,&PathTracking::is_object_callback, this);
	
	std::string is_offset = nh_private.param<std::string>("is_offset","/is_offset");  
	sub_is_offset = nh.subscribe(is_object,10,&PathTracking::avoiding_flag_callback, this);
	
	std::string car_state = nh_private.param<std::string>("car_state","/car_state");  
//	sub_car_state = nh.subscribe(car_state,10,&PathTracking::car_state_callback, this);            
	pub_car_goal  = nh.advertise<logistics_msgs::GoalState>(nh_private.param<std::string>("car_goal","/car_goal"),10);           
	timer_ = nh.createTimer(ros::Duration(0.01),&PathTracking::pub_car_goal_callback,this);         // 设置一个定时器每0.01秒发布一次
	nh_private.param<std::string>("path_points_file",path_points_file_,"");
	nh_private.param<float>("speed",track_speed_,1.0);
	nh_private.param<float>("foreSightDis_speedCoefficient", foreSightDis_speedCoefficient_,1.8);    
	nh_private.param<float>("foreSightDis_latErrCoefficient", foreSightDis_latErrCoefficient_,0.3); 
	nh_private.param<float>("omega", omega_,5.0);  													// 转向角速度 PID-D
	nh_private.param<float>("Ki", Ki_,0.3);  														// 系数 PID-I
	nh_private.param<float>("steer_clearance", steer_clearance_,0.3);  								// 转向间隙补偿
	nh_private.param<float>("steer_offset", steer_offset_,0.3);  									// 转角补偿 | 车辆不会正常直线
	nh_private.param<float>("tolerate_laterror", tolerate_laterror_,0.3);  							// 容忍横向偏差
	nh_private.param<float>("min_foresight_distance",min_foresight_distance_,3.0);
	nh_private.param<float>("safety_distance",safety_distance_,4);
	nh_private.param<float>("max_side_accel",max_side_accel_,1.5);
	nh_private.param<bool>("is_offset_",is_offset_,false);        
	nh_private.param<int>("control_rate",control_rate,30);        
	nh_private.param<double>("timeout",timeout,0.3);     
	nh_private.param<bool>("is_back",is_back_,false);   
	nh_private.param<bool>("check_gps", check_gps_, true);
	nh_private.param<bool>("is_inroom", is_inroom_, true);
	if(path_points_file_.empty())
	{
		ROS_ERROR("no input path points file !!");
		return false;
	}
	
	//start the ros::spin() thread
	rosSpin_thread_ptr_ = boost::shared_ptr<boost::thread >(new boost::thread(boost::bind(&PathTracking::rosSpinThread, this)));
	ros::Duration(2.0).sleep();
	if(loadPathPoints(path_points_file_, path_points_))
	{	
		if(is_back_)
			reverse(path_points_.begin(), path_points_.end());                                            
	}
	ROS_INFO("pathPoints size:%d",path_points_.size());
	while(ros::ok() && !is_gps_data_valid(current_point_))                                          // 判断是不是有效的gps数据
	{
		ROS_INFO("gps data is invalid, please check the gps topic or waiting...");
		sleep(1);
	}
	
	//for(int i=0; i<path_points_.size(); ++i)
	//	std::cout << path_points_[i].x << "\t" << path_points_[i].y << std::endl;
	target_point_index_ = findNearestPoint(path_points_,current_point_);                      
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

/*
 *@fuc:  判断是否正确的GPS数据:不为0即为有效数据
 *
 */

bool PathTracking::is_gps_data_valid(gpsMsg_t& point)
{		
		std::cout << "x:" << point.x << "\t" << "y:" << point.y << std::endl;
		if(point.x > 100 && point.y > 100)
		{
			return true;
		}
		return false;
}

/*
 *@fuc: 订阅来自GPS节点的消息
 *
 */
void PathTracking::gps_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	current_point_.x = msg->pose.pose.position.x;
	current_point_.y = msg->pose.pose.position.y;
	current_point_.yaw = msg->pose.covariance[0];
	
	isLocationValid = !msg->pose.covariance[4];
}

/*
 *@fuc: 获取gps显示的车速
 *
 */
 
void PathTracking::gps_callback(const gps_msgs::Inspvax::ConstPtr& msg)
{   
    float north_velocity = msg->north_velocity;
    float east_velocity  = msg->east_velocity;
    float speed = sqrt(pow(north_velocity,2) + pow(east_velocity,2));
    vehicle_speed_ = speed;
}

/*
 *@fuc:  计算获取目标前轮转角和车速
 *@param:  t_roadWheelAngle:目标前轮转角
 *
 */
 
void PathTracking::run()
{
	size_t i =0;
	float dt = 1.0/control_rate;
	ros::Rate loop_rate(control_rate);  //30Hz
//	std::cout << target_point_index_ << " / "  << path_points_.size() << std::endl;
	while(ros::ok() /*&& target_point_index_ < path_points_.size()-2*/)       
	{
//		if( avoiding_offset_ != 0.0)
//		        target_point_ = pointOffset(path_points_[target_point_index_],avoiding_offset_);
        if(vehicle_speed_ = 0)
            ROS_INFO("gps speed is not correct!");
		float goal_speed = track_speed_;
		try
		{
			lateral_err_ = calculateDis2path(current_point_.x,current_point_.y,path_points_,
											 target_point_index_,&nearest_point_index_) - avoiding_offset_;
			if(fabs(lateral_err_) > tolerate_laterror_)  										//横向偏差较小时不修正,因为稳态误差不能全部消除,否则会转向激进
				sumlateral_err_ = sumlateral_err_ + lateral_err_;
			if(sumlateral_err_ * lateral_err_ < 0)       										//横向偏差变号时,说明行驶靠右/左,总横向偏差需要置0
				sumlateral_err_ = 0;
		}
		catch(const char* str)
		{
			ROS_ERROR("what happen?");
			ROS_INFO("%s",str);
			break;
		}
		 disThreshold_ = min_foresight_distance_ + 
						 foreSightDis_speedCoefficient_ * vehicle_speed_ + 
						 foreSightDis_latErrCoefficient_ * fabs(lateral_err_);
	
//		ROS_INFO("disThreshold:%f\t lateral_err:%f",disThreshold_,lateral_err_);
									 
		std::pair<float, float> dis_yaw = get_dis_yaw(target_point_, current_point_);     

		if( dis_yaw.first < disThreshold_)
		{
			target_point_ = path_points_[target_point_index_++];
			if(target_point_index_ >= path_points_.size())
				break;
			continue;
		}
		yaw_err_ = dis_yaw.second - current_point_.yaw;                                  		// 计算出车身姿态与目标点的夹角
		if(yaw_err_==0.0) continue;
		float turning_radius = (0.5 * dis_yaw.first)/sin(yaw_err_);                     		// 转弯半径  l/2sin(a)
		//使用i控制,消除转向间隙引起的稳态误差
        float theta = Ki_ * sumlateral_err_;													// 假想转角和总横向偏差为线性模型
        if(theta > steer_clearance_)
            theta = steer_clearance_;
        else if(theta < -steer_clearance_)
            theta = -steer_clearance_;
		float t_roadWheelAngle = generateRoadwheelAngleByRadius(turning_radius);         
		t_roadWheelAngle = t_roadWheelAngle + theta;
		t_roadWheelAngle = limitRoadwheelAngleBySpeed(t_roadWheelAngle,vehicle_speed_);  
		t_roadWheelAngle += steer_offset_;
		
		//ROS_INFO("nearest_point_index_:%d" ,nearest_point_index_);
		//find the index of a path point x meters from the current point
		size_t index = findIndexForGivenDis(path_points_,nearest_point_index_,disThreshold_ + 3); 
		if(index ==0)
		{
			ROS_INFO("findIndexForGivenDis faild!");
			break;
		}
		float max_curvature = maxCurvatureInRange(path_points_, nearest_point_index_, index);
		float max_speed = generateMaxTolarateSpeedByCurvature(max_curvature, max_side_accel_);
		goal_speed = goal_speed > max_speed ? max_speed : goal_speed;
    
        if((object_data) && (fabs(t_roadWheelAngle) <= 10))
        {      
        /*
         *@fuc: 判断是否有障碍物
         *
         */ 
            if(goal_speed > 0 && object_data < safety_distance_)
            {   
                //float acceleration = (track_speed_ * track_speed_) / (2 * fabs(object_data - safety_distance_));
                //car_goal.goal_speed = track_speed_ - acceleration;
                //track_speed_ = car_goal.goal_speed;
                goal_speed = 0;
            }
            double now_break = now - ros::Time::now().toSec(); 
            if(fabs(now_break) > timeout)
                object_data = 0;
        }
        if(!isLocationValid)
        	goal_speed = 0.0;
        
        int sign = 1;
        if(t_roadWheelAngle > theta_true_) sign = 1;
        else if(t_roadWheelAngle < theta_true_) sign = -1;
        else sign = 0;
		float goal_angle = theta_true_ + sign * omega_ * dt ;
	    car_goal.goal_speed = goal_speed;
		car_goal.goal_angle = goal_angle * 1.3;
		theta_true_ = goal_angle;
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

/*
 *@fuc:  offset避障
 *
 */
 
void PathTracking::avoiding_flag_callback(const std_msgs::Float32::ConstPtr& msg)
{
	//avoid to left(-) or right(+) the value presents the offset
    if(is_offset_)
    {
        avoiding_offset_ = msg->data;
    }
}


void PathTracking::pub_car_goal_callback(const ros::TimerEvent&)
{
	pub_car_goal.publish(car_goal);
}  

/*
 *@fuc:  判断是否有障碍物体闯入(具体距离值可在livox里面修改)
 *
 */
 
void PathTracking::is_object_callback(const std_msgs::Float32::ConstPtr& msg)
{       
        object_data = msg->data;
        //std::cout << " 有障碍物体,距离为:" << object_data << std::endl;
        now = ros::Time::now().toSec();
}

/*
 *@fuc:  获取实际车辆状态
 *
 */

//void PathTracking::car_state_callback(const logistics_msgs::RealState::ConstPtr& msg)
//{
//	car_state.real_speed = msg->real_speed;
//	car_state.real_angle = msg->real_angle;
//	car_state.real_brake = msg->real_brake;
//	car_state.real_park  = msg->real_park;
//	
//	vehicle_speed_ = car_state.real_speed; //  km/h

//}

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
