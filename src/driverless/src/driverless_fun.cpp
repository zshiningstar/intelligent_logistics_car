#include "ros/ros.h"
#include "driverless/driverless_node.h"
#define __NAME__ "driverless"

/*@function 在此文件中定义AutoDrive初始化成员函数、功能函数等
 *          在driverless_node.cpp对关键函数进行定义，分布存储以提高可读性
*/

AutoDrive::AutoDrive():
	AutoDriveBase(__NAME__),
    avoid_offset_(0.0),
    task_running_(false),
	system_state_(State_Idle),
	last_system_state_(State_Idle),
	has_new_task_(false),
	request_listen_(false),
	as_(nullptr),
	avoid_min_obj_distance_(2000),
	decisionMakingDuration_(0.05)
{
	controlCmd2_.set_speed = 0.0;
	controlCmd2_.set_roadWheelAngle = 0.0;
}

AutoDrive::~AutoDrive()
{

}

bool AutoDrive::isGpsPointValid(const GpsPoint& point)
{
	//std::cout << point.x  << "\t"  <<point.y << std::endl;
	if(fabs(point.x) > 100 && fabs(point.y) > 100)
		return true;
	return false;
}

bool AutoDrive::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	if(is_initialed_) return true;  //避免重复初始化

	nh_ = nh;  nh_private_ = nh_private;

	//获取参数
	nh_private_.param<float>("max_speed",expect_speed_,5.0);// m/s
	nh_private_.param<bool>("use_car_following",use_car_following_,false);
	nh_private_.param<bool>("use_avoiding",use_avoiding_,false);
	nh_private_.param<bool>("is_offline_debug",is_offline_debug_,false);
	nh_private_.param<bool>("use_extern_controller", use_extern_controller_, false);
	nh_private_.param<bool>("use_car_follower", use_car_follower_, false);
	std::string odom_topic = nh_private_.param<std::string>("odom_topic","/ll2utm_odom");
	std::string tracking_info_topic = nh_private.param<std::string>("tracking_info_topic","/driverless/state");
	pub_driverless_state_ = nh.advertise<driverless_common::SystemState>(tracking_info_topic,1);
	
	initDiagnosticPublisher(nh_,__NAME__);
	
	if(!loadVehicleParams())
	{
		ROS_ERROR("[%s] Load vehicle parameters failed!", __NAME__);
		return false;
	}
	
	std::string which_tracker = nh_private.param<std::string>("which_path_tracking_controller", "");
	if(which_tracker == "pure_tracking_controller")
	{
		tracker_ = new PureTracking();
		//初始化路径跟踪控制器
		if(!tracker_->init(nh_, nh_private_))
		{
			ROS_ERROR("[%s] path tracker init false!",__NAME__);
			publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR,"Init path tracker failed!");
			return false;
		}
		ROS_INFO("[%s] path tracker init ok",__NAME__);
		tracker_->setVehicleParams(vehicle_params_);//初始化完毕跟踪控制器之后,才可传入参数
	}
	else
	{
		ROS_ERROR("[%s] Unknown path_planning controller: %s!", __NAME__, which_tracker.c_str());
		return false;
	}
	std::cout << "初始化路径跟踪控制器开始!" << std::endl;
	
//	std::cout << "获取初步参数成功 !"<< std::endl;
	//订阅公用传感器数据
	sub_odom_ = nh_.subscribe(odom_topic, 1,&AutoDrive::odom_callback,this);
	sub_vehicle_speed_ = nh_.subscribe("/car_state",1,&AutoDrive::vehicleSpeed_callback,this);
	sub_new_goal_      = nh_.subscribe("/driverless/expect_path",1,&AutoDrive::goal_callback,this);
	sub_is_object_ = nh.subscribe(nh_private.param<std::string>("is_object","/is_object"),10,
								  &AutoDrive::is_object_callback, this);

	//发布
	pub_cmd2_ = nh_.advertise<logistics_msgs::ControlCmd2>("/controlCmd2",1);
	pub_new_goal_ = nh_.advertise<driverless_common::DoDriverlessTaskActionGoal>("/do_driverless_task/goal", 1);
	
	//定时器                                                                           one_shot, auto_start
	cmd2_timer_ = nh_.createTimer(ros::Duration(0.05), &AutoDrive::sendCmd2_callback,this, false, false);
	timer_100ms_ = nh_.createTimer(ros::Duration(0.1), &AutoDrive::timer100ms_callback,this);
	
	/*+初始化自动驾驶请求服务器*/
	as_  = new DoDriverlessTaskServer(nh_, "do_driverless_task", 
                              boost::bind(&AutoDrive::executeDriverlessCallback,this, _1), false);
    as_->start();
	/*-初始化自动驾驶请求服务器*/
	std::cout << "初始化自动驾驶服务器！" << std::endl;
		// 车辆状态检查，等待初始化
	while(ros::ok() && !is_offline_debug_ ) //若离线调试,无需系统检查
	{
		std::string info;
		if(!vehicle_state_.validity(info))
		{
			ROS_INFO("[%s] %s",__NAME__, info.c_str());
			publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::WARN, info);
			ros::Duration(0.5).sleep();
		}
		else
			break;
	}
	std::cout << "车辆状态检查完成!" << std::endl;
	
    //初始化跟车行驶控制器
    if(use_car_follower_)
	{
		if(!car_follower_.init(nh_, nh_private_))
		{
			ROS_ERROR("[%s] car follower init false!",__NAME__);
			publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR,"Init car follower failed!");
			return false;
		}
		else
    		ROS_INFO("[%s] car follower init ok",__NAME__);
	}
    //初始化外部控制器
	if(use_extern_controller_)
	{
		if(extern_controler_.init(nh_, nh_private_) && extern_controler_.start())
			ROS_INFO("[%s] extern controller init and start ok",__NAME__);
		else
		{
			ROS_ERROR("[%s] Initial or Start extern controller failed!", __NAME__);
			return false;
		}
		capture_extern_cmd_timer_ = nh_.createTimer(ros::Duration(0.05), &AutoDrive::captureExernCmd_callback, this);
	}

    switchSystemState(State_Idle);
	//启动工作线程，等待新任务唤醒
	std::thread t(&AutoDrive::workingThread, this);
	t.detach();
	
	is_initialed_ = true;
	return true;
}


void AutoDrive::is_object_callback(const std_msgs::Float32::ConstPtr& msg)
{       
	avoid_min_obj_distance_ = msg->data;
    last_valid_obj_time_ = ros::Time::now().toSec();
}

void AutoDrive::setSendControlCmdEnable(bool flag)
{
	static bool last_flag = false;
	if(flag == last_flag)
		return;
	last_flag = flag;

	if(flag)
	{
		cmd2_timer_.start();
	}
	else
	{
		cmd2_timer_.stop();
	}
}

void AutoDrive::sendCmd2_callback(const ros::TimerEvent&)
{
	std::lock_guard<std::mutex> lock(cmd2_mutex_);
	pub_cmd2_.publish(controlCmd2_);
}

void AutoDrive::timer100ms_callback(const ros::TimerEvent&)
{
	publishDriverlessState();
}


/*@brief 定时捕获外部控制指令,  
  -当系统正在执行前进任务时，由driveDecisionMaking更新终端控制指令
  -当系统正在执行后退任务时，由reverseDecisionMaking更新终端控制指令
  -而当系统处于其他状态时由captureExernCmd_callback定时更新终端控制指令,以确保外部控制器随时生效

 - 当外部控制指令有效时，将系统状态置为强制使用外部指令，并将指令更新到终端控制指令
 - 当外部控制指令失效时，将系统状态切换到原来的状态
 */
void AutoDrive::captureExernCmd_callback(const ros::TimerEvent&)
{
	static bool last_validity = false;
	extern_cmd_mutex_.lock();
	extern_cmd_ = extern_controler_.getControlCmd();
	//std::cout << "extern_cmd_.validity: " << extern_cmd_.validity << std::endl;
	if(extern_cmd_.validity) //当前外部指令有效
 	{
		setSendControlCmdEnable(true); //使能控制指令发送
		if(system_state_ != State_ForceExternControl)
			switchSystemState(State_ForceExternControl);
		//extern_cmd_.display("Extern Cmd ");
	}
	if(!extern_cmd_.validity && last_validity) //当前无效，上次有效，切换为历史状态
		switchSystemState(last_system_state_);

	last_validity = extern_cmd_.validity;
	extern_cmd_mutex_.unlock();
}

void AutoDrive::switchSystemState(int state)
{
	ROS_ERROR("[%s] NOT ERROR switchSystemState: %s", __NAME__, StateName[state].c_str());//
	if(system_state_ == state) return; //防止重复操作
	
	last_system_state_ = system_state_;
    system_state_ = state;

    //状态为前进，自动驾驶模式开，档位置D
	if(state == State_Drive) 
	{
		cmd2_mutex_.lock();
		controlCmd2_.set_speed = 0.0;
		controlCmd2_.set_roadWheelAngle = 0.0;
		cmd2_mutex_.unlock();
        //启动控制指令发送
		setSendControlCmdEnable(true);
	}
    //状态为后退，自动驾驶模式开，档位置R
	else if(state == State_Reverse) 
	{
		cmd2_mutex_.lock();
		controlCmd2_.set_speed = 0.0;
		controlCmd2_.set_roadWheelAngle = 0.0;
		cmd2_mutex_.unlock();
        //启动控制指令发送
		setSendControlCmdEnable(true);
	}
    //状态为空闲，停止发送控制指令
	else if(state == State_Idle)  //空闲
	{
		cmd2_mutex_.lock();
		controlCmd2_.set_speed = 0.0;
		controlCmd2_.set_roadWheelAngle = 0.0;
		cmd2_mutex_.unlock();
		setSendControlCmdEnable(true);
	}
    //状态为停止，自动驾驶模式开, 速度置零，拉手刹
    //车辆停止后，切换为空挡
	else if(state == State_Stop)  
	{
		cmd2_mutex_.lock();
		controlCmd2_.set_speed = 0.0; 
		controlCmd2_.set_roadWheelAngle = 0.0;
		cmd2_mutex_.unlock();
		setSendControlCmdEnable(true);
		waitSpeedZero(); //等待汽车速度为0
        ROS_ERROR("[%s] NOT ERROR. set_gear: GEAR_NEUTRAL", __NAME__);
		//等待正在执行的任务彻底退出后，将系统置为空闲
		while(task_running_) 
		{
			ROS_INFO("Waiting %s exit...", StateName[system_state_].c_str());
			ros::Duration(0.05).sleep();
		}
        switchSystemState(State_Idle); //递归调用， 状态置为空闲  !!!!
	}
    //准备切换到前进状态
    else if(state == State_SwitchToDrive)
    {
    	setSendControlCmdEnable(true);  //启动控制指令发送
		cmd2_mutex_.lock();
		controlCmd2_.set_speed = 0.0;
		controlCmd2_.set_brake = 0.0;
		cmd2_mutex_.unlock();
        waitSpeedZero();                //等待速度为0
        switchSystemState(State_Drive); //递归调用，状态置为前进
    }
    //切换到倒车状态
    else if(state == State_SwitchToReverse)
    {
    	setSendControlCmdEnable(true);  //启动控制指令发送
		cmd2_mutex_.lock();
		controlCmd2_.set_speed = 0.0;
		controlCmd2_.set_brake = 0.0;
		cmd2_mutex_.unlock();
        waitSpeedZero();                //等待速度为0
        switchSystemState(State_Reverse); //递归调用，状态置为倒车
    }
	else if(state == State_ForceExternControl)
	{
		//No operation
	}
}
void AutoDrive::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	bool odom_valid = msg->pose.covariance[4] >= 9;
	
	vehicle_state_.setPoseValid(odom_valid);
	if(!odom_valid)
		return ;
	
	Pose pose;
	pose.x =  msg->pose.pose.position.x;
	pose.y =  msg->pose.pose.position.y;
	pose.yaw = msg->pose.covariance[0];
	vehicle_state_.setPose(pose);
	
}

void AutoDrive::goal_callback(const pathplaning_msgs::expected_path::ConstPtr& msg)
{
	driverless_common::DoDriverlessTaskActionGoal::Ptr  actionGoal = 
		driverless_common::DoDriverlessTaskActionGoal::Ptr(new driverless_common::DoDriverlessTaskActionGoal);
	actionGoal->header.stamp = ros::Time::now();

	driverless_common::DoDriverlessTaskGoal& goal = actionGoal->goal;
	if(msg->direction == msg->DIRECTION_DRIVE)
		goal.task = goal.DRIVE_TASK;
	else if(msg->direction == msg->DIRECTION_REVERSE)
		goal.task = goal.REVERSE_TASK;
	goal.type = goal.PATH_TYPE;
	goal.target_path = msg->points;
	goal.expect_speed = msg->expect_speed;
	goal.path_resolution = msg->path_resolution;
	pub_new_goal_.publish(actionGoal);

	ROS_INFO("[%s] Receive expect path from extern path planner.", __NAME__);
}

void AutoDrive::vehicleSpeed_callback(const logistics_msgs::RealState::ConstPtr& msg)
{
	if(msg->real_speed >10.0)
	{
		vehicle_state_.speed_validity = false;
		return;
	}
	float _vehicle_speed = msg->real_speed * 3.6;  //km/h->m/s
	vehicle_state_.setSteerAngle(msg->real_angle);
	vehicle_state_.setSpeed(_vehicle_speed); //  km/h
	vehicle_state_.speed_validity = true;
	vehicle_state_.steer_validity = true;
}

bool AutoDrive::loadVehicleParams()
{
	bool ok = true;
	vehicle_params_.max_roadwheel_angle = nh_private_.param<float>("vehicle/max_roadwheel_angle",0.0);
	vehicle_params_.min_roadwheel_angle = nh_private_.param<float>("vehicle/min_roadwheel_angle",0.0);
	vehicle_params_.min_radius          = nh_private_.param<float>("vehicle/min_radius",0.0);
	vehicle_params_.max_speed = nh_private_.param<float>("vehicle/max_speed",0.0);
	vehicle_params_.wheel_base = nh_private_.param<float>("vehicle/wheel_base",0.0);
	vehicle_params_.wheel_track = nh_private_.param<float>("vehicle/wheel_track",0.0);
	vehicle_params_.width = nh_private_.param<float>("vehicle/width",0.0);
	vehicle_params_.length = nh_private_.param<float>("vehicle/length",0.0);
	vehicle_params_.steer_clearance = nh_private_.param<float>("vehicle/steer_clearance",0.0);
	
	std::string node = ros::this_node::getName();
	if(vehicle_params_.max_roadwheel_angle == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/max_roadwheel_angle.",__NAME__,node.c_str());
		ok = false;
	}
	if(vehicle_params_.min_roadwheel_angle == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/min_roadwheel_angle.",__NAME__,node.c_str());
		ok = false;
	}
	if(vehicle_params_.min_radius == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/min_radius.",__NAME__,node.c_str());
		ok = false;
	}
	if(vehicle_params_.max_speed == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/max_speed.",__NAME__,node.c_str());
		ok = false;
	}
	if(vehicle_params_.wheel_base == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehice/wheel_base.",__NAME__,node.c_str());
		ok = false;
	}
	if(vehicle_params_.wheel_track == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/wheel_track.",__NAME__,node.c_str());
		ok = false;
	}
	if(vehicle_params_.width == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/width.",__NAME__,node.c_str());
		ok = false;
	}
	if(vehicle_params_.length == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/length.",__NAME__,node.c_str());
		ok = false;
	}
	if(ok) vehicle_params_.validity = true;

	std::cout << "载入车辆自身参数成功！" << std::endl;
	return ok;
}

/*@brief 载入前进任务文件，路径点位信息/停车点信息/拓展路径
		 若路径附加信息文件不存在，不返回错误，以应对非常规路线
*/
bool AutoDrive::loadDriveTaskFile(const std::string& file, bool flip)
{
	//载入路网文件
	if(! loadPathPoints(file, global_path_))
	{
	    ROS_ERROR("[%s] Load path file failed!",__NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR,"Load path file failed!");
		return false;
	}
	
	if(flip)
	{
		Path temp_path;
        temp_path.points.reserve(global_path_.points.size());

        for(int i=global_path_.points.size()-1; i >0; --i)
        {
            GpsPoint& point = global_path_.points[i];
            //point.yaw += M_PI;
            //if(point.yaw > 2*M_PI) point.yaw -= 2*M_PI;
            temp_path.points.push_back(point);
        }
        global_path_.points.swap(temp_path.points);
	}

	//载入路径附加信息
	std::string path_infos_file = file.substr(0,file.find_last_of(".")) + "_info.xml";
	if(!loadPathAppendInfos(path_infos_file, global_path_, __NAME__))
	{
		ROS_ERROR("[%s] Load path infomation failed!",__NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR,"Load path infomation failed!");
		//return false;
	}
	
	std::cout <<  "global_path_.size(): " << global_path_.size() << std::endl;
	return extendPath(global_path_, 20.0); //路径拓展延伸
}


/*@brief 设置前进任务目标路径点集， 自行计算路径曲率信息
 */
bool AutoDrive::setDriveTaskPathPoints(const driverless_common::DoDriverlessTaskGoalConstPtr& goal)
{
	size_t len = goal->target_path.size();
	if(len == 0) 
		return false;
		
	global_path_.clear();
	global_path_.points.resize(len);
	for(size_t i=0; i<len; ++i)
	{
		const geometry_msgs::Pose2D& pose = goal->target_path[i];
		GpsPoint& point = global_path_.points[i];
		point.x = pose.x;
		point.y = pose.y;
		point.yaw = pose.theta;
	}
	calPathCurvature(global_path_); //计算路径曲率
    global_path_.resolution = goal->path_resolution;
	global_path_.final_index = global_path_.points.size() - 1 ;  //设置终点索引为最后一个点
	//算法根据停车点距离控制车速，若没有附加路径信息将导致到达终点前无法减速停车！
	//因此，此处将终点设为一个永久停车点，
	global_path_.park_points.points.emplace_back(global_path_.final_index, 0.0); 
	return true;
}

/*@brief 等待车速减为0
*/
void AutoDrive::waitSpeedZero()
{
    while(ros::ok() && vehicle_state_.getSpeed(LOCK)!=0.0)
		ros::Duration(0.2).sleep();
}

/*@brief 发布自动驾驶状态信息
*/
void AutoDrive::publishDriverlessState()
{
	//定时发布函数,使用tracker_,所以务必需要确保函数执行前,tracker_已经准备好
	g_trackingError = tracker_->getTrackingErr();
	if(pub_driverless_state_.getNumSubscribers())
	{
		const Pose pose = vehicle_state_.getPose(LOCK);
		const float speed = vehicle_state_.getSpeed(LOCK);
		driverless_state_.header.stamp = ros::Time::now();
		driverless_state_.position_x = pose.x;
		driverless_state_.position_y = pose.y;
		driverless_state_.yaw = pose.yaw;
		driverless_state_.vehicle_speed =  speed;
		driverless_state_.roadwheel_angle = vehicle_state_.getSteerAngle(LOCK);
		driverless_state_.lateral_error = g_trackingError.first;
		driverless_state_.yaw_error = g_trackingError.second;
		driverless_state_.task_state = StateName[system_state_];
		driverless_state_.nearest_object_distance = avoid_min_obj_distance_;
		driverless_state_.command_speed = controlCmd2_.set_speed;

		pub_driverless_state_.publish(driverless_state_);
	}
}
