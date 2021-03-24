//#include <stdafx.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <thread>
#include <ctime>
#include <cstdlib>
#include <cmath>
#include <sstream>
#include <iostream>
#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <logistics_msgs/RealState.h>
#include <logistics_msgs/GoalState.h>

#include "std_msgs/Float32.h"

#define	N 999	

boost::shared_ptr<boost::thread> rosSpin_thread_ptr_;

float object_data_;
void rosSpinThread(){ros::spin();}

void is_object_callback(const std_msgs::Float32::ConstPtr& msg)           
{
    object_data_ = msg->data;
    std::cout << " 有障碍物体,距离为:" << object_data_ << std::endl;
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"try_node");
	
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	ros::Publisher  m_pub_goal = nh.advertise<logistics_msgs::GoalState>(nh_private.param<std::string>("car_goal","/car_goal"),10);
	
	ros::Subscriber  sub_is_object = nh.subscribe("/is_object",10,&is_object_callback);
	rosSpin_thread_ptr_ = boost::shared_ptr<boost::thread >(new boost::thread(boost::bind(&rosSpinThread)));
	
	
	ros::Rate loop_rate(10);
	
	std::cout << "数据发送开始......" << std::endl;
	while (ros::ok())
	{	
		object_data_ = 0;
		float num;
		int i;
//		srand(time(NULL));
				
		logistics_msgs::GoalState goal;
//		while(object_data_ == 0)
//		{
//		    goal.goal_speed = 0;
//		    goal.goal_angle = 0;
          goal.goal_speed = 0;        
          goal.goal_angle = 0;
            goal.goal_light = 0;
		    goal.goal_brake = 3;
		    goal.goal_park  = 3;
    		sleep(1);
		    m_pub_goal.publish(goal);
//		}
		
//		while(object_data_ != 0)
//		{
//            goal.goal_speed = 0;
//		    goal.goal_angle = 0;
//    //      goal.goal_angle = rand()%11 - 5;        
//    //      goal.goal_angle = 0;
//            goal.goal_light = 0;
//		    goal.goal_brake = 0;
//		    goal.goal_park  = 0;
//    		sleep(1);
//		    m_pub_goal.publish(goal);
//		    
//		    sleep(20);
//		    object_data_ = 0;
//        }
	
	}
	return 0;
}
