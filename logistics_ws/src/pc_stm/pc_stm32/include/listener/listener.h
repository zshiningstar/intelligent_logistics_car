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
#include <logistics_msgs/GoalState.h>

class Listener
{
public:
	Listener();
	~Listener();
	
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
	
private:
	ros::Publisher m_pub_state;
	
	serial::Serial *m_serial_port;
	//! shared pointer to Boost thread for listening for data from novatel
	boost::shared_ptr<boost::thread> m_read_thread_ptr;
	bool m_reading_status;  //!< True if the read thread is running, false otherwise.
	uint8_t * const m_pkg_buffer;
	logistics_msgs::RealState m_state;

};


#endif
