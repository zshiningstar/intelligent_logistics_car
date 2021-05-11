#ifndef PC_STM_H
#define PC_STM_H

#include <sstream>
#include <iostream>
#include <iostream>
#include <ros/ros.h>
#include <cmath>
#include <serial/serial.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <logistics_msgs/RealState.h>
#include <logistics_msgs/PidParams.h>
#include <logistics_msgs/GoalState.h>
#include <stdio.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <logistics_msgs/ControlCmd2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <string>
#include <vector>

class Uppercontrol
{
public:
	Uppercontrol();
	~Uppercontrol();
	
	void run();
	bool init();
	void startReading();
	void stopReading();
private:
	bool openSerial(const std::string& port,int baudrate);
	void closeSerial();
	void readSerialThread();
	
	void parseIncomingData(uint8_t* buffer,size_t len);
	void parseFromStmVehicleState(const unsigned char* buffer);
	
	void handle_speed_msg(uint8_t* buffer_data);
	double generate_real_speed(double& temp1,double& temp2);
	uint8_t sumCheck(const uint8_t* pkg_buffer, int pkg_len);
	
	void GoalState_callback(const logistics_msgs::GoalState::ConstPtr& msg);
	void Cmd2_callback(const logistics_msgs::ControlCmd2::ConstPtr& msg);
	void Pid_callback(const logistics_msgs::PidParams::ConstPtr& pid);
	
	void print(const uint8_t* buf, int len)
	{
		for(int i=0; i< len; ++i)
			std::cout << std::hex << int(buf[i]) << "\t";
		std::cout << std::endl;
	}
private:
	//! shared pointer to Boost thread for listening for data from novatel
	boost::shared_ptr<boost::thread> m_read_thread_ptr;
	bool m_reading_status;  //!< True if the read thread is running, false otherwise.
	uint8_t * const m_pkg_buffer;
	
	serial::Serial *m_serial_port;
	
	ros::Publisher odom_pub;
	ros::Publisher m_pub_state;
	ros::Subscriber m_sub_state;
	ros::Subscriber m_sub_controlCmd2;
	
	ros::Subscriber m_sub_goal;
	ros::Subscriber m_sub_pid_params;
	
	std::string car_state;
	tf::TransformBroadcaster odom_broadcaster;
	ros::Time current_time, last_time;
	nav_msgs::Odometry odom;
	logistics_msgs::RealState m_state;
	
	bool prase_flag_;
	double x,y,th,vx,vy,vth;
	double real_speed_left,real_speed_right,real_angle,real_speed,real_touque;
	double left_wheel_speed,right_wheel_speed,speed;
};

#endif
