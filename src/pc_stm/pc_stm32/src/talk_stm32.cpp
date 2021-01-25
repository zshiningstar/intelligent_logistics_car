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

using namespace std;

class Talk
{
public:
	Talk()
	{};
	~Talk()
	{};
	
	bool init();
private:
	bool openSerial(const std::string& port,int baudrate);
	void closeSerial();
	
	void GoalState_callback(const logistics_msgs::GoalState::ConstPtr& msg);
private:
	ros::Subscriber m_sub_goal;
	serial::Serial *m_serial_port;
	
};

/*
 *@fuc: 初始化
 *
 */
bool Talk::init()
{
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	std::string car_goal = nh_private.param<std::string>("car_goal","/car_goal");
	m_sub_goal = nh.subscribe(car_goal ,1,&Talk::GoalState_callback, this);
	
	std::string port_name = nh_private.param<std::string>("port_name","/dev/ttyUSB0");  // launch文件对应
	
	int baudrate = nh_private.param<int>("baudrate",115200);
	if(!openSerial(port_name,baudrate))
		return false;
	return true;
}

/*
 *@fuc: 订阅 car_goal,写入串口
 *
 */
 
void Talk::GoalState_callback(const logistics_msgs::GoalState::ConstPtr& msg)
{	
	static uint8_t buf[11];

	buf[0]  = 0x66;
	buf[1]  = 0xcc;
	buf[2]  = 0x01;   //数据包id
	buf[3]  = 0x07;   //数据长度（包含校验位）
	buf[4]  = 0x01;
	
	uint16_t sum = msg->goal_speed * 100.0 + 30000;
	buf[5]  = sum >> 8;  
	buf[6]  = sum;  
	
	uint16_t sun = msg->goal_angle * 100.0 + 30000;
	buf[7]  = sun >> 8;
	buf[8]  = sun;
	buf[9]  = msg->goal_light;
	buf[10] = buf[2] + buf[3] + buf[4] + buf[5] + buf[6] + buf[7] + buf[8] + buf[9];   //校验位
	
	m_serial_port-> write(buf,11);
}

/*
 *@fuc:  打开串口
 *
 */
 
bool Talk::openSerial(const std::string& port,int baudrate)              // 打开串口
{   
    m_serial_port = new serial::Serial(port,baudrate,serial::Timeout::simpleTimeout(10)); 
	if (!m_serial_port->isOpen())
	{
		std::stringstream output;
        output << "Serial port: " << port << " failed to open." << std::endl;
		delete m_serial_port;
		m_serial_port = NULL;
		return false;
	} 
	else 
	{
		std::stringstream output;
		output << "Serial port: " << port << " opened successfully." << std::endl;
	}

	m_serial_port->flush();
	return true;
}     

/*
 *@fuc:  关闭串口
 *
 */

void Talk::closeSerial()                                              // 关闭串口
{
	if(m_serial_port!=NULL)
	{
		delete m_serial_port;
		m_serial_port = NULL;
	}
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"talker_node");
	
	Talk talk;
	
	if(talk.init())
	{
		ROS_INFO("%s initial ok...",ros::this_node::getName().c_str());
	}
	
	ros::spin();
}
