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
		goal.goal_speed = 0;
		goal.goal_angle = 0;
//      goal.goal_angle = rand()%11 - 5;        
//      goal.goal_angle = 0;
        goal.goal_light = 0;
		goal.goal_brake = 3;
		goal.goal_park  = 3;
		sleep(1);
		m_pub_goal.publish(goal);
		loop_rate.sleep();
	
	}
	return 0;
}
