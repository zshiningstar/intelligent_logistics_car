//#include <stdafx.h>
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
#define	N 999	
	

int main(int argc,char** argv)
{
	ros::init(argc,argv,"try_node");
	
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	ros::Publisher  m_pub_goal = nh.advertise<logistics_msgs::GoalState>(nh_private.param<std::string>("car_goal","/car_goal"),10);
	ros::Rate loop_rate(10);
	std::cout << "数据发送开始......" << std::endl;
	while (ros::ok())
	{
		float num;
		int i;
		srand(time(NULL));
				
		logistics_msgs::GoalState goal;
//		for (int i = 0;i <10;i++)
{

		goal.goal_speed = 0;
//		sleep(5);
//		goal.goal_speed = rand()%11 + 5;
		goal.goal_angle = rand()%21 - 10;
//goal.goal_angle = 0;
		goal.goal_brake = 3;
		goal.goal_park  = 3;
		sleep(1);
}
		//发布消息
		m_pub_goal.publish(goal);

//		std::cout << "speed:" << goal.goal_speed <<"  " << "angle:" << goal.goal_angle << "  " 
//		          << "goal.goal_brake:" << goal.goal_brake << "  " << "goal.goal_park:" << goal.goal_park << std::endl;

//		std::cout << "数据发送开始......" << std::endl;
		
		//按照循环频率延时
		loop_rate.sleep();
	
	}
	return 0;
}
