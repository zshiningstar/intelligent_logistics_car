#ifndef DAOYUAN_H_
#define DAOYUAN_H_
#include<iostream>
#include<ros/ros.h>
#include<cmath>
#include<serial/serial.h>
#include<boost/thread.hpp>
#include<boost/bind.hpp>
#include"gps_msgs/Daoyuan.h"
#include"gps_msgs/Rotation.h"
#include<nav_msgs/Odometry.h>
#include"gps_msgs/Satellite.h"
#include"gps_msgs/Satellites.h"
#include <tf2_ros/transform_broadcaster.h>


#define coefficient1  0.010986328125
#define coefficient2  0.0091552734375
#define coefficient3  0.0003662109375
#define coefficient4  0.0000001
#define coefficient5  0.001
#define coefficient6  0.0030517578125
#define coefficient7  0
#define coefficient8  0.25
#define NATURE        2.718281

#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))


PACK(
struct pkgRS422_t
{
	uint8_t header0;
	uint8_t header1;
	uint8_t header2;
	
	float roll;
	float pitch;
	float azimuth;
	
	float top_velocity_x;
	float top_velocity_y;
	float top_velocity_z;
	
	float table_x;
	float table_y;
	float table_z;
	
	double latitude;
	double longitude;
	double height;
	
	float north_velocity;
	float east_velocity;
	float down_velocity;
	
//	int pose_state:1;
//	int velocity_state:1;
//	int gesture_state:1;
//	int azimuth_state:1;
//	int blank:4;            		    // not use
	
	float wheel_data1;
	float wheel_data2;	
	float wheel_data3;

	uint32_t gps_time; 
	int rotation_type;
	uint8_t xorcheck_value;             // 异或校验值
	uint32_t gps_zhou;
	uint8_t check_value;
	
	int gps_state;
});

double deg2rad(const double& deg)
{
	return deg*M_PI/180.0;
}

class Daoyuan
{
public:
	Daoyuan();
	~Daoyuan();
	Daoyuan(const Daoyuan& obj) = delete;
	Daoyuan& operator=(const Daoyuan& obj) = delete;
	bool init();
	void startReading();
	void stopReading();
private:
	bool openSerial(const std::string& port,int baudrate);
	void closeSerial();
	void readSerialThread();
	void parseIncomingData(uint8_t* buffer,size_t len);
	void parse(const uint8_t* buffer);
	
	uint8_t XOR(const uint8_t* buf,size_t len)
	{
		uint8_t sum = buf[0];
		for(size_t i=1; i<len; ++i)
		{
			sum = sum ^ buf[i];
			//cout << hex << int(buf[i]) << " ";
		}
		return sum;
	}
	float complement(int buf, float a)
	{
		int buf_num = 0;
		double value = 0;
		buf_num = *((int*)(&buf));
		value = (double)buf_num * a;
		
		return value;
	}
	
	
private:
	ros::Publisher m_pub_rs422;
	ros::Publisher m_pub_ll2utm;
	
	ros::Publisher m_pub_wheel;
	
	serial::Serial *m_serial_port;
	//! shared pointer to Boost thread for listening for data from novatel
	boost::shared_ptr<boost::thread> m_read_thread_ptr;
	bool m_reading_status;  //!< True if the read thread is running, false otherwise.
	uint8_t * const m_pkg_buffer;
	
	gps_msgs::Daoyuan m_inspax;
	gps_msgs::Rotation m_wheel;
	bool m_is_pub_ll2utm;
	bool m_is_pub_tf;
	bool m_is_pub_wheel;
	
	tf2_ros::TransformBroadcaster m_tf_br;
	std::string m_child_frame_id, m_parent_frame_id;
	
};


#endif
