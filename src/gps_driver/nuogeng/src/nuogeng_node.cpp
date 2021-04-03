#include<iostream>
#include"nuogeng/nuogeng.h"
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <geographic_msgs/GeoPoint.h>
#include <Eigen/Dense>

Nuogeng::Nuogeng():
	m_reading_status(false),
	m_pkg_buffer(new uint8_t[500])
{
}

Nuogeng::~Nuogeng()
{
	this->closeSerial();
	if(m_pkg_buffer!=NULL)
		delete [] m_pkg_buffer;
}

bool Nuogeng::openSerial(const std::string& port,int baudrate)
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

void Nuogeng::closeSerial()
{
	if(m_serial_port!=NULL)
	{
		delete m_serial_port;
		m_serial_port = NULL;
	}
}

bool Nuogeng::init()
{
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	m_pub_id20 = nh.advertise<gps_msgs::Inspvax>(nh_private.param<std::string>("gps_topic","/gps"),1);
	m_pub_ll2utm = nh.advertise<nav_msgs::Odometry>(nh_private.param<std::string>("odom_topic","/odom"),1);
	
	nh_private.param<std::string>("parent_frame_id", m_parent_frame_id, "world");
	nh_private.param<std::string>("child_frame_id", m_child_frame_id, "gps");
	
	nh_private.param<bool>("pub_odom", m_is_pub_ll2utm, false);
	nh_private.param<bool>("pub_tf", m_is_pub_tf, true);
	
	std::string port_name = nh_private.param<std::string>("port_name","/dev/ttyUSB0");
	int baudrate = nh_private.param<int>("baudrate",115200);
	if(!openSerial(port_name,baudrate))
		return false;
	return true;
}

void Nuogeng::startReading()
{
	if(m_reading_status)
		return ;
	m_read_thread_ptr = boost::shared_ptr<boost::thread>(new boost::thread(&Nuogeng::readSerialThread,this));
}

void Nuogeng::readSerialThread()
{
	m_reading_status = true;
	
	const int read_size = 200;
	const int buffer_size = read_size * 2;
	const int left_reserve_len = 5;
	uint8_t * const raw_data_buf = new uint8_t[buffer_size+left_reserve_len];
	uint8_t * const buffer =  raw_data_buf+ left_reserve_len;
	
	size_t offset = 0, get_len, total_len;
		   
	while(ros::ok() && m_reading_status)
	{
		try
		{
			get_len = m_serial_port->read(buffer+offset,read_size);
		}
		catch(std::exception &e)
		{
			ROS_ERROR("Error reading from serial port: %s",e.what());
		}
		
		total_len = offset + get_len;
		if(total_len < read_size)
			offset = total_len;
		else
		{
			//cout << total_len << endl;
			parseIncomingData(buffer, total_len);
			offset = 0;
		}
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	
	delete [] raw_data_buf;
}

void Nuogeng::parseIncomingData(uint8_t* buffer,size_t len)
{
	static size_t pkg_buffer_index = 0; //数据包缓存定位索引
	static size_t pkg_len = 0;			//数据包长度
	static size_t remainded = 0;		//包头(5bytes)搜索完毕后buffer的剩余长度
										//remainded <=4
	//real_buffer指针位于buffer指针之前
	//上次搜索剩余的数据位于buffer之前
	//再次搜索时应从real_buffer开始
	static uint8_t *real_buffer = buffer - remainded;
	
	size_t real_buffer_len = len + remainded;
	remainded = 0; //复位
	for(size_t i=0; i<real_buffer_len; ++i)
	{
		if(pkg_buffer_index ==0) //还未找到数据头
		{
			if(real_buffer_len-i > 4 ) //剩余数据可能包含数据头(剩余bytes>=数据头bytes)
			{
				if(LRC(real_buffer+i+1,4)==real_buffer[i]) //LRC校验查找数据头
				{
					m_pkg_buffer[pkg_buffer_index++] = real_buffer[i];
					pkg_len = real_buffer[i+2];
					//cout << pkg_len << endl;
				}
			}
			else if(remainded ==0) // 剩余数据不足且暂未开始拼接
			{
				remainded = real_buffer_len-i; //剩余个数
				//开始拼接，剩余bytes有序放在buffer前面，且保证数据的连续性
				buffer[i-real_buffer_len] = real_buffer[i];
			}
			else
				buffer[i-real_buffer_len] = real_buffer[i]; //继续拼接
		}
		//数据头已经找到，根据包长逐个拷贝(不包括最后一个字节)
		else if(pkg_buffer_index < pkg_len+4)
			m_pkg_buffer[pkg_buffer_index++] = real_buffer[i];
		else
		{	//拷贝最后一个字节,over
			m_pkg_buffer[pkg_buffer_index] = real_buffer[i];
			pkg_buffer_index = 0; //定位索引复位
			
			if(m_pkg_buffer[3]+m_pkg_buffer[4]*256 != getCrc16ccittFalseByTable(m_pkg_buffer+5,pkg_len))
				continue; //校验失败
				
			if(20==m_pkg_buffer[1]) //ID
				parseId20Pkg(m_pkg_buffer);
			else if(31 == m_pkg_buffer[1])
				;//parseId31Pkg(m_pkg_buffer);
		}
	}
}

void Nuogeng::parseId20Pkg(const uint8_t* buffer)
{
	auto gpsPtr = (const pkg20Msgs_t *)buffer;
	
	m_inspax.header.stamp = ros::Time::now();
	m_inspax.header.frame_id = "gps";
	m_inspax.latitude = gpsPtr->latitude *180.0/M_PI;
	m_inspax.longitude = gpsPtr->longitude *180.0/M_PI;
	m_inspax.height = gpsPtr->height;
	m_inspax.north_velocity = gpsPtr->north_velocity;
	m_inspax.east_velocity = gpsPtr->east_velocity;
	m_inspax.up_velocity = gpsPtr->down_velocity;
	m_inspax.roll = gpsPtr->roll;
	m_inspax.pitch = gpsPtr->pitch;
	m_inspax.azimuth = gpsPtr->yaw *180.0/M_PI;
	m_inspax.latitude_standard_deviation = gpsPtr->latitude_std_deviation;
	m_inspax.longitude_standard_deviation = gpsPtr->longitude_std_deviation;
	m_inspax.height_standard_deviation = gpsPtr->height_std_deviation;
	
	m_pub_id20.publish(m_inspax);
	
	geographic_msgs::GeoPoint point;
	point.latitude = m_inspax.latitude;
	point.longitude = m_inspax.longitude;
	point.altitude = m_inspax.height;
	
	geodesy::UTMPoint utm;
	geodesy::fromMsg(point, utm);
	
	Eigen::AngleAxisd xAngle(deg2rad(m_inspax.roll), Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd yAngle(deg2rad(m_inspax.pitch), Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd zAngle(-deg2rad(m_inspax.azimuth-90.0), Eigen::Vector3d::UnitZ());
	
	Eigen::Quaterniond quat = xAngle * yAngle * zAngle;
	quat.normalized();
	
	if(m_is_pub_ll2utm)    // 判断位：如果这个值为true，就发布这个话题消息
	{
		nav_msgs::Odometry odom;
		
		odom.header.stamp = m_inspax.header.stamp;
		odom.header.frame_id = m_parent_frame_id;
		odom.child_frame_id  = m_child_frame_id;
		
		odom.pose.pose.position.x = utm.easting;
		odom.pose.pose.position.y = utm.northing;
		odom.pose.pose.position.z = utm.altitude;
		
		odom.pose.covariance[0] = m_inspax.azimuth *M_PI / 180.0;
		odom.pose.covariance[1] = point.longitude;
		odom.pose.covariance[2] = point.latitude;
		
		
		odom.pose.pose.orientation.x = quat.x();
		odom.pose.pose.orientation.y = quat.y();
		odom.pose.pose.orientation.z = quat.z();
		odom.pose.pose.orientation.w = quat.w();
		
		m_pub_ll2utm.publish(odom);
	}
	
	if(m_is_pub_tf)
	{
		geometry_msgs::TransformStamped transformStamped;
		transformStamped.header.stamp = ros::Time::now();
		transformStamped.header.frame_id = m_parent_frame_id;
		transformStamped.child_frame_id = m_child_frame_id;
		transformStamped.transform.translation.x = utm.easting;
		transformStamped.transform.translation.y = utm.northing;
		transformStamped.transform.translation.z = 0.0;

		transformStamped.transform.rotation.x = quat.x();
		transformStamped.transform.rotation.y = quat.y();
		transformStamped.transform.rotation.z = quat.z();
		transformStamped.transform.rotation.w = quat.w();

		m_tf_br.sendTransform(transformStamped);
	}
	
}

/*
void Nuogeng::parseId31Pkg(const uint8_t* buffer)
{
	int satellite_count = buffer[2]/7; //pkglen/7
	gps_msgs::Satellite satellite;
	gps_msgs::Satellites satellites;
	
	satellites.header.stamp = ros::Time::now();
	satellites.header.frame_id = "gps";
	satellites.satellites.resize(satellite_count);
	satellites.count = satellite_count;
	
	for(int i=0; i<satellite_count; ++i)
	{
		auto satellitePtr = (const pkg31Msgs_t*)(buffer+5+i*7);
		satellite.navigation_system = satellitePtr->navigation_system;
		satellite.satellite_num = satellitePtr->satellite_num;
		satellite.satellite_frequency = satellitePtr->satellite_frequency;
		satellite.elevation = satellitePtr->elevation;
		satellite.azimuth = satellitePtr->azimuth;
		satellite.snr = satellitePtr->snr;
		
		satellites.satellites[i] = satellite;
	}
	m_pub_id31.publish(satellites);
}
*/

void Nuogeng::stopReading()
{
	m_reading_status = false;
}


int main(int argc,char** argv)
{
	ros::init(argc,argv,"nuogeng_node");
	
	Nuogeng gps;
	
	if(gps.init())
	{
		gps.startReading();
		ROS_INFO("%s initial ok...",ros::this_node::getName().c_str());
	}
	
	ros::spin();
}



