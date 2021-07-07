#include "ros/ros.h"
#include "driverless/driverless_node.h"

#define __NAME__ "driverless"

/*
	step1. actionlib服务器回调函数(as_callback)收到新目标请求.
	step2. as_callback唤醒工作线程(workingThread)开始工作，然后as_callback挂起.
	step3. workingThread任务完成后继续挂起，唤醒as_callback判断是否有新目标.
	step4. 如果有新任务，返回step2, 否则as_callback退出并再次等待step1.
	
	as_callback使用work_cv_条件变量唤醒workingThread;
	workingThread使用listen_cv_条件变量唤醒as_callback.
 */

void AutoDrive::executeDriverlessCallback(const driverless_common::DoDriverlessTaskGoalConstPtr& goal)
{
	if(!handleNewGoal(goal)) return;
	
	std::unique_lock<std::mutex> lock(listen_cv_mutex_);
	listen_cv_.wait(lock, [&](){return request_listen_;});
	request_listen_ = false;
	listen_cv_mutex_.unlock();
	
	while(ros::ok())
	{
		if(as_->isNewGoalAvailable())
		{
			//if we're active and a new goal is available, we'll accept it, but we won't shut anything down
			ROS_ERROR("[%s] NOT ERROR. The current work was interrupted by new request!", __NAME__);
			driverless_common::DoDriverlessTaskGoalConstPtr new_goal = as_->acceptNewGoal();

			if(!handleNewGoal(new_goal)) return;
			
			std::unique_lock<std::mutex> lock(listen_cv_mutex_);
			listen_cv_.wait(lock, [&](){return request_listen_;});
			request_listen_ = false;
		}
	}
}

/*@brief 处理新目标，
	①有效目标 对新目标进行预处理/载入相关文件/切换系统状态/唤醒工作线程，返回true
	②无效目标 返回false
 *@param goal 目标信息
*/
bool AutoDrive::handleNewGoal(const driverless_common::DoDriverlessTaskGoalConstPtr& goal)
{
	std::cout << "goal->type: " << int(goal->type)  << "\t"
			      << "goal->task: " << int(goal->task)  << "\t"
				  << "goal->file: " << goal->roadnet_file << "\t" 
				  << "expect_speed: " << goal->expect_speed << "\t"
				  << "path_resolution: " << goal->path_resolution << "\t"     
				  << std::endl;
	switchSystemState(State_Stop); //新请求，无论如何先停止, 暂未解决新任务文件覆盖旧文件导致的自动驾驶异常问题，
                                   //因此只能停车后开始新任务
                                   //实则，若新任务与当前任务驾驶方向一致，只需合理的切换路径文件即可！
                                   //已经预留了切换接口，尚未解决运行中清空历史文件带来的隐患

	ROS_ERROR("[%s] NOT ERROR. new task ready, vehicle has speed zero now.", __NAME__);
	this->expect_speed_ = goal->expect_speed;
    //给定目标点位置，调用路径规划
    if(goal->type == goal->POSE_TYPE) 
    {
		ROS_ERROR("[%s] The forward path planning function has not been developed!", __NAME__);
		as_->setSucceeded(driverless_common::DoDriverlessTaskResult(), 
			"Aborting on drive task, because The forward path planning function has not been developed!");
		return false;
    }
    //指定驾驶路径点集
    else if(goal->type == goal->PATH_TYPE)
    {
		if(!setDriveTaskPathPoints(goal))
		{
			ROS_ERROR("[%s] The target path is invalid!", __NAME__);
            as_->setSucceeded(driverless_common::DoDriverlessTaskResult(), "Aborting on drive task, because The target path is invalid!");
			return false;
		}
    }
    //指定路径文件
    else if(goal->type == goal->FILE_TYPE)
    {
        if(!loadDriveTaskFile(goal->roadnet_file, goal->path_filp))
        {
            ROS_ERROR("[%s] Load drive path file failed!", __NAME__);
            driverless_common::DoDriverlessTaskResult res;
            res.success = false;
            as_->setSucceeded(res, "Aborting on drive task, because load drive path file failed! ");
            return false;
        }
        tracker_->setPath(global_path_);
    }
    else
    {
        ROS_ERROR("[%s] Request type error!", __NAME__);
		as_->setAborted(driverless_common::DoDriverlessTaskResult(), "Aborting on unknown goal type! ");
        return false;
    }
    
    if(goal->task == goal->DRIVE_TASK)
    {
	    //切换系统状态为: 切换到前进
	    switchSystemState(State_SwitchToDrive);
	}
	else if(goal->task == goal->REVERSE_TASK)
	{
		switchSystemState(State_SwitchToReverse);
	}
	else
	{
		ROS_ERROR("[%s] Unknown task type!", __NAME__);
		as_->setAborted(driverless_common::DoDriverlessTaskResult(), "Aborting on unknown task! ");
		return false;
	}

    std::unique_lock<std::mutex> lck(work_cv_mutex_);
    has_new_task_ = true;
	work_cv_.notify_one(); //唤醒工作线程
	return true;
  
}

void AutoDrive::workingThread()
{
	is_running_ = true;
	while(ros::ok() && is_running_)
	{
		/*使用条件变量挂起工作线程，等待其他线程请求唤醒，
		 *为防止虚假唤醒(系统等原因)，带有谓词参数的wait函数，唤醒的同时判断谓词是否满足，否则继续挂起
		 *条件变量与独占指针搭配使用，首先使用独占指针加锁，wait函数内部进行解锁并等待唤醒信号，线程唤醒后再次加锁
		 *当前线程被唤醒并开始工作且任务结束前，其他线程无法获得锁，当新任务到达
		 */
		std::unique_lock<std::mutex> lock(work_cv_mutex_);
		work_cv_.wait(lock, [&](){return has_new_task_;});
		has_new_task_ = false;

		int state = system_state_;
		ROS_INFO("[%s] Current system_state: %d", __NAME__, state);
		
		doWork();

		if(state!=State_Drive && state!=State_Reverse)
			ROS_ERROR("[%s] Unknown task type in current state: %d.", __NAME__, state);

		std::unique_lock<std::mutex> lck(listen_cv_mutex_);
		request_listen_ = true;
		listen_cv_.notify_one(); //唤醒监听线程

		//Can not call switchSystemState(State_Stop) here !!!
		//Because the expect state has set by doDriveWork/doReverseWork
		/*此处不可切换系统状态，而需要在上述子任务函数(doDriveWork/doReverseWork)中进行切换,
		 *子任务中系统状态切换包含两种情况，
		 *1. 任务完成，切换系统为停止状态
		 *2. 被新任务打断，切换系统为指定状态
		 *因此！此处不能再切换系统状态，否则状态机制将被打破，导致任务无法正常运行！
		 */

		//Add by zsx on 2021/6/4
		//因为物流车的前进后退都为工作状态，因此前进后退任务分开写维护较为麻烦，此处按照师兄指导将其合并，
		//修改地方有：
		/*1. 进入工作线程函数后，直接启动doWork() [函数名字从doDriveWork修改为doWork]
		 *2. 将decisionMaking(bool)函数修改为decisionMaking()无参类型函数
		*/
	}
}

void AutoDrive::doWork()
{
	//配置路径跟踪控制器
	tracker_->setExpectSpeed(expect_speed_);
	tracker_->start();//路径跟踪控制器
	//配置跟车控制器

	ros::Rate loop_rate(1.0/decisionMakingDuration_);
	
	ROS_ERROR("NOT ERROR: doWork-> task_running_= true");
	task_running_ = true;

	while(ros::ok() && system_state_ != State_Stop && tracker_->isRunning())
	{
		ReadLock lck(vehicle_state_shared_mutex_);
		tracker_->updataVehicleState(vehicle_state_);
		lck.unlock();
		
		tracker_cmd_ = tracker_->getControlCmd();
		follower_cmd_= car_follower_.getControlCmd();
		
		auto cmd = this->decisionMaking();

		if(as_->isActive()) //判断action server是否为活动，防止函数的非服务调用导致的错误
		{
			driverless_common::DoDriverlessTaskFeedback feedback;
			feedback.speed = cmd.set_speed;
			feedback.steer_angle = cmd.set_roadWheelAngle;
			as_->publishFeedback(feedback);

			if(as_->isPreemptRequested()) 
			{
				ROS_INFO("[%s] isPreemptRequested.", __NAME__);
				as_->setPreempted(); //自主触发中断请求
				break;
			}
		}

		loop_rate.sleep();
	}
	ROS_INFO("[%s] drive work  completed...", __NAME__); 
	tracker_->stop();
	car_follower_.stop();
	if(as_->isActive())
	{
		as_->setSucceeded(driverless_common::DoDriverlessTaskResult(), "drive work  completed");
	}
	task_running_ = false;
	switchSystemState(State_Stop);
}

/*
void AutoDrive::doReverseWork()
{
	reverse_controler_.setExpectSpeed(expect_speed_);
	reverse_controler_.start();
	
	ros::Rate loop_rate(1.0/decisionMakingDuration_);
	
	ROS_ERROR("NOT ERROR: doReverseWork-> task_running_= true");
	task_running_ = true;
	while(ros::ok() && system_state_ != State_Stop && reverse_controler_.isRunning())
	{
		//ROS_INFO("[%s] new cycle.", __NAME__);
		reverse_cmd_ = reverse_controler_.getControlCmd();
		
		//ROS_INFO("[%s] speed: %.2f\t angle: %.2f", __NAME__, reverse_cmd_.speed, reverse_cmd_.roadWheelAngle);
		
		if(reverse_cmd_.validity)
			decisionMaking(false);
		
		//如果actionlib服务器处于活跃状态，则进行状态反馈并判断是否外部请求中断
		//如果actionlib服务器未active表明是其他方式请求的工作，比如测试例
		if(as_->isActive())
		{
			driverless::DoDriverlessTaskFeedback feedback;
			feedback.speed = reverse_cmd_.speed;
			feedback.steer_angle = reverse_cmd_.roadWheelAngle;
			as_->publishFeedback(feedback);
			
			if(as_->isPreemptRequested())  //外部请求中断
			{
				ROS_INFO("[%s] isPreemptRequested.", __NAME__);
				as_->setPreempted(); //自主中断当前任务
				break;
			}
		}

		loop_rate.sleep();
	}
	reverse_controler_.stop();
	ROS_INFO("[%s] reverse work complete.", __NAME__);
	if(as_->isActive())
	{
		as_->setSucceeded(driverless::DoDriverlessTaskResult(), "drive work  completed");
	}
	task_running_ = false;
	switchSystemState(State_Stop);
}
*/

/*@brief 前进控制指令决策
 * 指令源包括: 避障控速/跟车控速/路径跟踪控转向和速度
 * 控制指令优先级 ①外部控制指令gv
                ②避障速度控制
				③跟车速度控制
 */
 logistics_msgs::ControlCmd2 AutoDrive::decisionMaking()
 {
 	std::lock_guard<std::mutex> lock2(cmd2_mutex_);
 	/*if(isDrive)  //drive
 	{*/
	controlCmd2_.set_roadWheelAngle = tracker_cmd_.roadWheelAngle;//前轮转角
	controlCmd2_.set_speed = fabs(tracker_cmd_.speed); //优先使用跟踪器速度指令
 	/*}
 	else //reverse
 	{
 		controlCmd2_.set_speed = fabs(reverse_cmd_.speed);
		controlCmd2_.set_roadWheelAngle = reverse_cmd_.roadWheelAngle;
 	}*/
 	
	//若当前状态为强制使用外部控制指令，则忽悠其他指令源
	if(system_state_ == State_ForceExternControl)
	{
		std::lock_guard<std::mutex> lock_extern_cmd(extern_cmd_mutex_);
		if(extern_cmd_.speed_validity)
			controlCmd2_.set_speed = extern_cmd_.speed;

		if(extern_cmd_.steer_validity)
			controlCmd2_.set_roadWheelAngle = extern_cmd_.roadWheelAngle;
	}
	//非外部控制工况且定位状态无效，停车！
	else if(!vehicle_state_.getPoseValid())
	{
		controlCmd2_.set_speed = 0;
	}
	
//	controlCmd2_.set_roadWheelAngle = steerPidCtrl(controlCmd2_.set_roadWheelAngle) + 0.0;

	static float lastCtrlSpeed = controlCmd2_.set_speed;
	float speed_now = vehicle_state_.getSpeed(LOCK);
	float deltaT = decisionMakingDuration_;
	static float safety_distance_ = 4.0; //m

	float maxAccel = 1.0;
	float minAccel = -2.0;
	float expectAccel = 0.0; //m/s2

	int accelSign = sign(avoid_min_obj_distance_- safety_distance_ - speed_now*speed_now/(2*maxAccel));
	
	if(accelSign > 0)
		expectAccel = maxAccel;
	else if(accelSign < 0)
		expectAccel = minAccel;
	
	float expectSpeed = lastCtrlSpeed + expectAccel*deltaT;
//	std::cout << expectSpeed << "  " << lastCtrlSpeed << "  " << expectAccel << std::endl;

	if(expectSpeed <= 0) 
		expectSpeed = 0;
		
	else if(expectSpeed > controlCmd2_.set_speed)
		expectSpeed = controlCmd2_.set_speed;
	controlCmd2_.set_speed = expectSpeed;
	lastCtrlSpeed = controlCmd2_.set_speed;

	if(system_state_ == State_Drive) 
		controlCmd2_.set_speed = fabs(controlCmd2_.set_speed);
	else if(system_state_ == State_Reverse)
	{
		controlCmd2_.set_speed = -fabs(controlCmd2_.set_speed);
//		controlCmd2_.set_roadWheelAngle = -controlCmd2_.set_roadWheelAngle;
	}

	//std::cout << controlCmd2_.set_speed << "\t" << expectSpeed << "\t" << deltaT << "\t" << expectAccel << std::endl;

	return controlCmd2_;
 }


/*@brief 由于物流车机械结构待优化,需要通过PID控制循迹
 *@param kp 
 *@param ki
 *@param kd
 *@return t_roadWheelAngle 前轮转角
 */
float AutoDrive::steerPidCtrl(float expectAngle)
{
	static float steerPidKi = nh_private_.param<float>("steer_pid_ki", 0.1);
	static float tolerateLatErr = nh_private_.param<float>("tolerate_laterr", 0.05);
	
	static float lastErr = 0.0;
	static float lastLastErr = 0.0;
	static float sumErr = 0.0;
	
	float angle_now = vehicle_state_.getSteerAngle(LOCK);
	float err = g_trackingError.first; //from global var
	
	if(fabs(err) > tolerateLatErr)		//横向偏差较小时不修正,因为稳态误差不能全部消除,否则会转向激进
		sumErr = sumErr + err;
	if(sumErr * err < 0)				//横向偏差变号时,说明行驶靠右/左,总横向偏差需要置0
		sumErr = 0;

	float theta = steerPidKi * sumErr;	// 假想转角和总横向偏差为线性模型
    if(theta > vehicle_params_.steer_clearance)
    	theta = vehicle_params_.steer_clearance;
    else if(theta < -vehicle_params_.steer_clearance)
    	theta = -vehicle_params_.steer_clearance;

    return expectAngle - theta;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "auto_drive_node");
	ros::AsyncSpinner spinner(5);
	spinner.start(); //非阻塞

	ros::NodeHandle nh, nh_private("~");
    AutoDrive auto_drive;
    if(auto_drive.init(nh, nh_private))
    	ros::waitForShutdown();
    return 0;
}  


	
