#include<iostream>
#include<ros/ros.h>
#include<serial/serial.h>
#include<boost/thread.hpp>
#include<boost/bind.hpp>
#include"gps_msgs/Inspvax.h"
#include"gps_msgs/Satellite.h"
#include"gps_msgs/Satellites.h"
#include <arpa/inet.h>

using namespace std;

#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
PACK(
struct sv_t
{
	uint8_t prn ;
	uint8_t navigation_system ;
	uint8_t flags1 ;
	uint8_t flags2 ;
	uint8_t elevation ;
	uint16_t azimuth ;
	uint8_t frequency[3];
});


class Satellite
{
public:
	Satellite();
	~Satellite();
	Satellite(const Satellite& obj) = delete;
	Satellite& operator=(const Satellite& obj) = delete;
	bool init(int argc,char** argv);
	void startReading();
	void stopReading();
private:
	bool openSerial(const std::string& port,int baudrate);
	void closeSerial();
	void readSerialThread();
	void parseIncomingData(uint8_t* buffer,size_t len);
	void parsePackage(const uint8_t* buffer);
	void processAllDetailedSVInfo(const uint8_t* buffer, int len);
	bool check(const uint8_t* buffer, int len);

	
private:
	ros::Publisher m_pub1_;
	
	serial::Serial *m_serial_port;
	//! shared pointer to Boost thread for listening for data from novatel
	boost::shared_ptr<boost::thread> m_read_thread_ptr;
	bool m_reading_status;  //!< True if the read thread is running, false otherwise.
	uint8_t * const m_pkg_buffer;
	gps_msgs::Inspvax m_inspax;
};

Satellite::Satellite():
	m_reading_status(false),
	m_pkg_buffer(new uint8_t[500])
{
}

Satellite::~Satellite()
{
	this->closeSerial();
	
}

bool Satellite::openSerial(const std::string& port,int baudrate)
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

void Satellite::closeSerial()
{
	if(m_serial_port!=NULL)
	{
		delete m_serial_port;
		m_serial_port = NULL;
	}
		
}

bool Satellite::init(int argc,char** argv)
{
	ros::init(argc,argv,"satellite_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	m_pub1_ = nh.advertise<gps_msgs::Satellites>(nh_private.param<string>("satellite_topic","/satellite"),1);
	std::string port_name = nh_private.param<std::string>("port_name","/dev/ttyUSB0");
	int baudrate = nh_private.param<int>("baudrate",115200);
	if(!openSerial(port_name,baudrate))
		return false;
	return true;
}

void Satellite::startReading()
{
	if(m_reading_status)
		return ;
	m_read_thread_ptr = boost::shared_ptr<boost::thread>(new boost::thread(&Satellite::readSerialThread,this));
}

void Satellite::readSerialThread()
{
	m_reading_status = true;
	
	const int read_size = 500;
	
	uint8_t * const raw_data_buf = new uint8_t[read_size];
	
	size_t get_len;
	
	while(ros::ok() && m_reading_status)
	{
		try
		{
			get_len = m_serial_port->read(raw_data_buf, read_size);
		}
		catch(std::exception &e)
		{
			ROS_ERROR("Error reading from serial port: %s",e.what());
		}
		
		if(get_len == 0)
			continue;
		
//		for(size_t i=0; i<get_len; ++i)
//				cout << hex << int(raw_data_buf[i]) << " ";
//			cout << endl << endl;
////		
		parseIncomingData(raw_data_buf, get_len);
		
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	}
	
	delete [] raw_data_buf;
}

void Satellite::parseIncomingData(uint8_t* buffer,size_t len)
{
	static size_t pkg_buffer_index = 0; //数据包缓存定位索引
	static size_t pkg_len = 0;			//数据包长度
	
	
	for(size_t i=0; i<len; ++i)
	{
		//cout << pkg_buffer_index << ":" <<hex << int(buffer[i]) << endl;
		if(pkg_buffer_index==0)
		{
			if(buffer[i] == 0x02) //start flag
				m_pkg_buffer[pkg_buffer_index++] = buffer[i];
		}
		else if(pkg_buffer_index==1)
			m_pkg_buffer[pkg_buffer_index++] = buffer[i]; //status
		else if(pkg_buffer_index==2)
		{
			if(buffer[i]==0x40) //pkgtype 0x40
			{
				m_pkg_buffer[pkg_buffer_index++] = buffer[i];
			}
			else
				pkg_buffer_index = 0;
		}
		else if(pkg_buffer_index == 3)
			pkg_len = m_pkg_buffer[pkg_buffer_index++] = buffer[i]; //len
		else if(pkg_buffer_index == pkg_len+5)
		{
			pkg_buffer_index = 0;
			
			if(buffer[i] != 0x03) //end flag
				continue;
				
			if(check(m_pkg_buffer+1, pkg_len+4))
				parsePackage(m_pkg_buffer+7);
			else
				ROS_INFO("check failed");
		}
		else
			m_pkg_buffer[pkg_buffer_index++] = buffer[i];
	}
}

bool Satellite::check(const uint8_t* buffer, int len)
{
	uint8_t sum = 0;
	for(size_t i=0; i<len-1; ++i)
	{
		sum += buffer[i];
	}
	cout<< endl;
		
	if(sum != buffer[len-1])
		return false;
	return true;
}

void Satellite::parsePackage(const uint8_t* buffer)
{
	char record_type = buffer[0];
	int record_len = buffer[1];
	
	ROS_INFO("type = %x",record_type);
	if(record_type == 0x22)
		processAllDetailedSVInfo(buffer+2, record_len);
	
}

void Satellite::processAllDetailedSVInfo(const uint8_t* buffer, int len)
{
	int sv_num = buffer[0];
	
	gps_msgs::Satellite satellite;
	gps_msgs::Satellites satellites;
	satellites.header.stamp = ros::Time::now();
	satellites.header.frame_id = "gps";
	satellites.satellites.resize(sv_num);
	satellites.count = sv_num;
	
	for(int i=0; i<sv_num; ++i)
	{
		sv_t *svInfo = (sv_t *)(buffer+1+10*i);
		satellite.num = svInfo->prn;
		satellite.system = svInfo->navigation_system;
		satellite.flags1 = svInfo->flags1;
		satellite.flags2 = svInfo->flags2;
		satellite.elevation = svInfo->elevation;
		satellite.azimuth = htons(svInfo->azimuth);
		satellite.frequency[0] = svInfo->frequency[0];
		satellite.frequency[1] = svInfo->frequency[1];
		satellite.frequency[2] = svInfo->frequency[2];
		satellites.satellites[i] = satellite;
	}
	m_pub1_.publish(satellites);
}


void Satellite::stopReading()
{
	m_reading_status = false;
}


int main(int argc,char** argv)
{
	Satellite gps;
	if(gps.init(argc,argv))
	{
		gps.startReading();
		ROS_INFO("%s initial ok...",ros::this_node::getName().c_str());
	}
	
	ros::spin();
}



