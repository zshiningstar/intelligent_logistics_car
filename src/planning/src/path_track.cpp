#include "path_track.h"

/*@fuc:     车辆目标状态初始化
 *param:    object_data:前方障碍物最近距离
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
}

PathTracking::~PathTracking()
{
}

/*@fuc:   获取launch文件参数并定义话题消息
 */
bool PathTracking::init_params(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	nh_private.param<std::string>("path_points_file",path_points_file_,"");                             // 路径文件
	nh_private.param<float>("speed",track_speed_,1.0);                                                  // 行驶速度
	nh_private.param<float>("foreSightDis_speedCoefficient", foreSightDis_speedCoefficient_,1.8);       // 前视距离速度系数
	nh_private.param<float>("foreSightDis_latErrCoefficient", foreSightDis_latErrCoefficient_,0.3);     // 前视距离横向误差系数
	nh_private.param<float>("omega", omega_,5.0);  													    // 转向角速度 PID-D
	nh_private.param<float>("Ki", Ki_,0.3);  														    // 系数 PID-I
	nh_private.param<float>("steer_clearance", steer_clearance_,0.3);  								    // 转向间隙补偿
	nh_private.param<float>("steer_offset", steer_offset_,0.3);  									    // 转角补偿 | 车辆不会正常直线
	nh_private.param<float>("tolerate_laterror", tolerate_laterror_,0.3);  							    // 容忍横向偏差
	nh_private.param<float>("min_foresight_distance",min_foresight_distance_,3.0);                      // 最小前视距离
	nh_private.param<float>("safety_distance",safety_distance_,4);                                      // 安全距离
	nh_private.param<float>("max_side_accel",max_side_accel_,1.5);                                      // 最大侧向加速度
	nh_private.param<bool>("is_offset_",is_offset_,false);                                              // 是否换道
	nh_private.param<int>("control_rate",control_rate,30);                                              // 控制速率
	nh_private.param<double>("timeout",timeout,0.3);                                                    // 时间延迟
	nh_private.param<bool>("is_back",is_back_,false);                                                   // 是否倒车
    std::string odom_topic = nh_private.param<std::string>("odom_topic","/odom");   
    sub_utm_odom  = nh.subscribe(odom_topic, 5,&PathTracking::gps_odom_callback,this);               
	
    std::string gps_topic = nh_private.param<std::string>("gps_topic","/gps");   
    sub_gps  = nh.subscribe(gps_topic, 5,&PathTracking::gps_callback,this);               
	
    std::string is_object = nh_private.param<std::string>("is_object","/is_object");                    // 是否有障碍物
    sub_is_object = nh.subscribe(is_object,10,&PathTracking::is_object_callback, this);      
	
    std::string is_offset = nh_private.param<std::string>("is_offset","/is_offset");                    // 换道offset
    sub_is_offset = nh.subscribe(is_object,10,&PathTracking::avoiding_flag_callback, this);
	
    std::string car_state = nh_private.param<std::string>("car_state","/car_state");  
    //	sub_car_state = nh.subscribe(car_state,10,&PathTracking::car_state_callback, this);             // 车辆实际状态
    pub_car_goal  =
       nh.advertise<logistics_msgs::GoalState>(nh_private.param<std::string>("car_goal","/car_goal"),10);           
    timer_ = nh.createTimer(ros::Duration(0.01),&PathTracking::pub_car_goal_callback,this);             // 设置一个定时器每0.01秒发布一次
	return true;
}

/*@fuc:   等待设备启动完成
 */
bool PathTracking::init_work(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
    if(init_params(nh,nh_private))
    {
	    if(path_points_file_.empty())
	    {
		    ROS_ERROR("no input path points file !!");
		    return false;
	    }
	    rosSpin_thread_ptr_ = boost::shared_ptr<boost::thread >(new boost::thread(boost::bind(&PathTracking::rosSpinThread, this)));
	    ros::Duration(2.0).sleep();                                                                     // 防止错过GPS话题消息
	    if(loadPathPoints(path_points_file_, path_points_))
	    {	
		    if(is_back_)                                                                                // 如果倒车，翻转路径文件
			{ 
			    reverse(path_points_.begin(), path_points_.end());
		        if(!extendPath(path_points_, 20.0))
		        {
		            ROS_ERROR("extend the path failed !!");
		            return false;
		        }
			}
			else
			    if(!extendPath(path_points_, 20.0))
		        {
		            ROS_ERROR("extend the path failed !!");
		            return false;
		        }
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
	    if(target_point_index_ > path_points_.size() - 2)
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
}

/*@fuc:  路径延伸
 */
bool PathTracking::extendPath(std::vector<gpsMsg_t>& path, float extendDis)
{
	//取最后一个点与倒数第n个点的连线向后插值
	//总路径点不足n个,退出
	int n = 5;
	//std::cout << "extendPath: " << path.size() << "\t" << path.size()-1 << std::endl;
	if(path.size()-1 < n)
	{
		ROS_ERROR("path points is too few (%lu), extend path failed",path.size()-1);
		return false;
	}
	int endIndex = path.size()-1;
	
	float dx = (path[endIndex].x - path[endIndex-n].x)/n;
	float dy = (path[endIndex].y - path[endIndex-n].y)/n;
	float ds = sqrt(dx*dx+dy*dy);

	gpsMsg_t point;
	float remaind_dis = 0.0;
	for(size_t i=1;;++i)
	{
		point.x = path[endIndex].x + dx*i;
		point.y = path[endIndex].y + dy*i;
		point.curvature = 0.0;
		path.push_back(point);
		remaind_dis += ds;
		if(remaind_dis > extendDis)
			break;
	}
	return true;
}

/*@fuc:    计算获取目标前轮转角和车速
 *@param:  t_roadWheelAngle:目标前轮转角
 */
void PathTracking::run()
{
	size_t i =0;
	float dt = 1.0/control_rate;
	ros::Rate loop_rate(control_rate);  //30Hz
//	std::cout << target_point_index_ << " / "  << path_points_.size() << std::endl;
	while(ros::ok() /*&& target_point_index_ < path_points_.size()-2*/)       
	{
		/*if( avoiding_offset_ != 0.0)
		        target_point_ = pointOffset(path_points_[target_point_index_],avoiding_offset_); */
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
        {//判断是否有障碍物
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
		if(goal_angle>max_roadwheelAngle_)
		    goal_angle = max_roadwheelAngle_;
	    else if(goal_angle<(-max_roadwheelAngle_))
	        goal_angle = (-max_roadwheelAngle_);
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

/*@fuc:  offset
 */
gpsMsg_t PathTracking::pointOffset(const gpsMsg_t& point,float offset)
{
	gpsMsg_t result = point;
	result.x =  offset * cos(point.yaw) + point.x;
	result.y = -offset * sin(point.yaw) + point.y;
	return result;
}

/*@fuc:  判断是否正确的GPS数据:不为0即为有效数据
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

/*@fuc: 订阅来自GPS节点的消息
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


/*@fuc:  offset避障
 */
 
void PathTracking::avoiding_flag_callback(const std_msgs::Float32::ConstPtr& msg)
{
	//avoid to left(-) or right(+) the value presents the offset
    if(is_offset_)
    {
        avoiding_offset_ = msg->data;
    }
}
/*@fuc:  发布车辆行驶命令
 */
void PathTracking::pub_car_goal_callback(const ros::TimerEvent&)
{
	pub_car_goal.publish(car_goal);
}  

/*@fuc:  判断是否有障碍物体闯入(具体距离值可在livox里面修改)
 */
void PathTracking::is_object_callback(const std_msgs::Float32::ConstPtr& msg)
{       
        object_data = msg->data;
        //std::cout << " 有障碍物体,距离为:" << object_data << std::endl;
        now = ros::Time::now().toSec();
}

/*@fuc:  获取实际车辆状态
 */
void PathTracking::car_state_callback(const logistics_msgs::RealState::ConstPtr& msg)
{
}

int main(int argc,char**argv)
{
	ros::init(argc,argv,"path_tracking");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	PathTracking path_tracking;
	if(!path_tracking.init_work(nh,nh_private))
		return 1;
	path_tracking.run();
	ROS_INFO("path tracking completed.");
	
	ros::shutdown();
	return 0;
}
