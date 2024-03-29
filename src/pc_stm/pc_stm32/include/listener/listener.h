#ifndef LISTENER_H
#define LISTENER_H

#include <sstream>
#include <iostream>
#include <iostream>
#include <ros/ros.h>
#include <cmath>
#include <serial/serial.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <logistics_msgs/RealState.h>
#include <stdio.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <string>
#include <vector>

#define Wheel_Radius  0.25
#define AXIS_DISTANCE 0.88  // 轴距

class Listener
{
public:
	Listener();
	~Listener();
	
	void run();
	bool init();
	void startReading();
	void stopReading();
private:
	bool openSerial(const std::string& port,int baudrate);
	void closeSerial();
	void readSerialThread();
	void parseIncomingData(uint8_t* buffer,size_t len);
	void parseFromStm(const unsigned char* buffer);
	bool sumCheck(const unsigned char* pkg_buffer, size_t pkg_len);
	
	void handle_speed_msg(uint8_t* buffer_data);
	double generate_real_speed(double& temp1,double& temp2);
	
private:
	ros::Publisher m_pub_state;
	
	serial::Serial *m_serial_port;
	//! shared pointer to Boost thread for listening for data from novatel
	boost::shared_ptr<boost::thread> m_read_thread_ptr;
	bool m_reading_status;  //!< True if the read thread is running, false otherwise.
	uint8_t * const m_pkg_buffer;
//	logistics_msgs::RealState m_state;
	
	ros::Subscriber m_sub_state;
	ros::Publisher odom_pub;
	std::string car_state;
	tf::TransformBroadcaster odom_broadcaster;
	ros::Time current_time, last_time;
	nav_msgs::Odometry odom;
	logistics_msgs::RealState m_state;
	
	bool prase_flag_;
	double x,y,th,vx,vy,vth;
	double real_speed_left,real_speed_right,real_angle,real_speed;
	double left_wheel_speed,right_wheel_speed,speed;
};


#endif
