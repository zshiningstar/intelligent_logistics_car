#include"daoyuan/daoyuan.h"

Daoyuan::Daoyuan():
	m_reading_status(false),
	m_pkg_buffer(new uint8_t[500])
{
}

Daoyuan::~Daoyuan()
{
	this->closeSerial();
	if(m_pkg_buffer!=NULL)
		delete [] m_pkg_buffer;
}

bool Daoyuan::openSerial(const std::string& port,int baudrate)
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

void Daoyuan::closeSerial()
{
	if(m_serial_port!=NULL)
	{
		delete m_serial_port;
		m_serial_port = NULL;
	}
}

bool Daoyuan::init()
{
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	m_pub_rs422 = nh.advertise<gps_msgs::Daoyuan>(nh_private.param<std::string>("gps_topic","/gps"),1);
	m_pub_wheel = nh.advertise<gps_msgs::Rotation>(nh_private.param<std::string>("rotation_topic","/rotation"),1);
	//m_pub_id20 = nh.advertise<gps_msgs::Inspvax>(nh_private.param<std::string>("gps_topic","/gps"),1);
	
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

void Daoyuan::startReading()
{
	if(m_reading_status)
		return ;
	m_read_thread_ptr = boost::shared_ptr<boost::thread>(new boost::thread(&Daoyuan::readSerialThread,this));
}

void Daoyuan::readSerialThread()
{
	m_reading_status = true;
	const int Max_read_size = 200;
	uint8_t * const raw_data_buf = new uint8_t[Max_read_size];
	size_t get_len;
		   
	while(ros::ok() && m_reading_status)
	{
		try
		{
			get_len = m_serial_port->read(raw_data_buf,Max_read_size);
		}
		catch(std::exception &e)
		{
			ROS_ERROR("Error reading from serial port: %s",e.what());
			ros::Duration(0.01).sleep();
			continue;
		}
		
		if(get_len == 0) 
		{
			ros::Duration(0.01).sleep();
			continue;
		}
		
		parseIncomingData(raw_data_buf, get_len);      // 解析数据
		ros::Duration(0.01).sleep();
		
//		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	
	delete [] raw_data_buf;
}

void Daoyuan::parseIncomingData(uint8_t* buffer,size_t len)
{
	static uint8_t pkg_buffer[63];
	static size_t pkg_buffer_index = 0; //数据包缓存定位索引
	
	size_t pkg_len1 = 0;
	size_t pkg_len2 = 0;
	
	bool XOR1 = false;
	bool XOR2 = false;
	
	for(size_t i=0; i<len; ++i)
	{
		if(0 == pkg_buffer_index) 
		{
			if(0xBD == buffer[i])
				pkg_buffer[pkg_buffer_index++] = buffer[i];
		}
		
		else if(1 == pkg_buffer_index)
		{
			if(0xDB == buffer[i])
			{
				pkg_buffer[pkg_buffer_index++] = buffer[i];
			}
			else
				pkg_buffer_index = 0;
		}
		
		else if(2 == pkg_buffer_index)   //BD DB  0B
		{
			if(0x0B == buffer[i])
			{
				pkg_buffer[pkg_buffer_index++] = buffer[i];
			}
			else
				pkg_buffer_index = 0;
		}
		
		else
		{
			pkg_buffer[pkg_buffer_index++] = buffer[i];  //  ++ ->63
			
			if(pkg_buffer_index == 63)
			{
				XOR1 = (XOR(pkg_buffer,57) == pkg_buffer[57]) ? 1 : 0;   //check 0~56
				XOR2 = (XOR(pkg_buffer,62) == pkg_buffer[62]) ? 1 : 0;   //check 0~61
				//std::cout << std::hex << int(pkg_buffer[57])  << "\t"   <<  int(pkg_buffer[62])  << std::endl;
				
				if(XOR1 && XOR2)
				{
					parse(pkg_buffer);
				}
				else
					ROS_ERROR("check failed.");

				pkg_buffer_index = 0;   //recovery
			}
			
		}
	}
}

void Daoyuan::parse(const uint8_t* buffer)
{
//	auto gpsPtr = (const pkgRS422_t *)buffer;
	
	m_inspax.header.stamp = ros::Time::now();
	m_inspax.header.frame_id = "gps";
	
	m_inspax.roll = *(int16_t*)(buffer+3) * coefficient1;
	m_inspax.pitch = *(int16_t*)(buffer+5) * coefficient1;
	m_inspax.azimuth = *(int16_t*)(buffer+7) * coefficient1;
	
	m_inspax.gyroscope_velocity_x = *(int16_t*)(buffer+9) * coefficient2;
	m_inspax.gyroscope_velocity_y =  *(int16_t*)(buffer+11) * coefficient2;
	m_inspax.gyroscope_velocity_z =  *(int16_t*)(buffer+13) * coefficient2;
	
	m_inspax.accelerator_x = *(int16_t*)(buffer+15) * coefficient3;
	m_inspax.accelerator_y =  *(int16_t*)(buffer+17) * coefficient3;
	m_inspax.accelerator_z = *(int16_t*)(buffer+19) * coefficient3;
	
	m_inspax.latitude = *(int32_t*)(buffer+21) * coefficient4;
	m_inspax.longitude = *(int32_t*)(buffer+25) * coefficient4;
	m_inspax.height = *(int32_t*)(buffer+29) * coefficient5;
	
	m_inspax.north_velocity =  *(int16_t*)(buffer+33) * coefficient6;
	m_inspax.east_velocity =  *(int16_t*)(buffer+35) * coefficient6;
	m_inspax.down_velocity =  *(int16_t*)(buffer+37) * coefficient6;
	
	m_inspax.gps_state = *(buffer+37);
	
	int16_t wheel_data1 = *(int16_t*)(buffer+46);
	int16_t wheel_data2 = *(int16_t*)(buffer+48);
	int16_t wheel_data3 = *(int16_t*)(buffer+50);
	
	m_inspax.gps_time = *(uint32_t*)(buffer+52) * coefficient7;
	m_inspax.rotation_type = buffer[56];
	m_inspax.gps_week = *(uint32_t*)(buffer+58);
	
	/*
	//m_inspax.roll				 = complement(buffer[3] + buffer[4] * 256, coefficient1);
	m_inspax.pitch 				 = complement(buffer[5] + buffer[6] * 256, coefficient1);
	m_inspax.azimuth 			 = complement(buffer[7] + buffer[8] * 256, coefficient1);
	
	m_inspax.gyroscope_velocity_x 	 = complement(buffer[9] + buffer[10] * 256, coefficient2);
	m_inspax.gyroscope_velocity_y 	 = complement(buffer[11] + buffer[12] * 256, coefficient2);
	m_inspax.gyroscope_velocity_z 	 = complement(buffer[13] + buffer[14] * 256, coefficient2);
	
	m_inspax.accelerator_x 			 = complement(buffer[15] + buffer[16] * 256, coefficient3);
	m_inspax.accelerator_y 	 		 = complement(buffer[17] + buffer[18] * 256, coefficient3);
	m_inspax.accelerator_z 			 = complement(buffer[19] + buffer[20] * 256, coefficient3);
	
	m_inspax.latitude 			 = complement(buffer[21] + buffer[22] * 256 + buffer[23] * 256 *256 + buffer[24] * 256 * 256 * 256, coefficient4);
	m_inspax.longitude			 = complement(buffer[25] + buffer[26] * 256 + buffer[27] * 256 *256 + buffer[28] * 256 * 256 * 256, coefficient4);
	m_inspax.height 			 = complement(buffer[29] + buffer[30] * 256 + buffer[31] * 256 *256 + buffer[32] * 256 * 256 * 256, coefficient5);
	
	m_inspax.north_velocity 	 = complement(buffer[33] + buffer[34] * 256, coefficient6);
	m_inspax.east_velocity 	     = complement(buffer[35] + buffer[36] * 256, coefficient6);
	m_inspax.down_velocity 		 = complement(buffer[37] + buffer[38] * 256, coefficient6);
	
	m_inspax.gps_state 		     = complement(buffer[39], coefficient7);
	
	int16_t wheel_data1		     = complement(buffer[46] + buffer[47] * 256, coefficient7);
	int16_t wheel_data1		     = complement(buffer[48] + buffer[49] * 256, coefficient7);
	int16_t wheel_data1		     = complement(buffer[50] + buffer[51] * 256, coefficient7);
	
	m_inspax.gps_time			 = complement(buffer[52] + buffer[53] * 256 + buffer[54] * 256 *256 + buffer[55] * 256 * 256 * 256, coefficient8);
	m_inspax.rotation_type       = buffer[56];
	
	m_inspax.gps_week            = complement(buffer[58] + buffer[59] * 256 + buffer[60] * 256 *256 + buffer[61] * 256 * 256 * 256, coefficient7);
	*/
	float latitude = m_inspax.latitude;
	float longitude = m_inspax.longitude;
	if(m_inspax.gps_state != 0x0f)
		m_inspax.Initializing_State = 0;
	else 
		m_inspax.Initializing_State = 1;
	if((fabs(latitude) <=3) && (fabs(longitude) <=3))
		return;
	else
		m_pub_rs422.publish(m_inspax);  // 发布原始解析程序
	
	uint8_t wheel_type = buffer[56];
	
	switch(wheel_type)
	{	
		case 0: //
			m_wheel.latstd 			= pow(NATURE, wheel_data1 / 100.0);
			m_wheel.lonstd 			= pow(NATURE, wheel_data2 / 100.0);
			m_wheel.hstd 			= pow(NATURE, wheel_data3 / 100.0);
			break;
		case 1:
			m_wheel.vn_std 			= pow(NATURE, wheel_data1 / 100.0);
			m_wheel.ve_std 			= pow(NATURE, wheel_data2 / 100.0);
			m_wheel.vd_std 			= pow(NATURE, wheel_data3 / 100.0);
			break;
		case 2:
			m_wheel.rollstd 		= pow(NATURE, wheel_data1 / 100.0);
			m_wheel.pitchstd 		= pow(NATURE, wheel_data2 / 100.0);
			m_wheel.yawstd 			= pow(NATURE, wheel_data3 / 100.0);
			break;
		case 22:
			m_wheel.temperature 	= wheel_data1 * 200.0 / 32768;
			break;
		case 32:
		{	
			m_wheel.num_satellites	= wheel_data2;
			switch (wheel_data1)
			{
				case 0:
					m_wheel.gps_location_state = "NONE";
					break;
				case 1:
					m_wheel.gps_location_state = "FIXEDPOS";
					break;
				case 2:
					m_wheel.gps_location_state = "FIXEDHEIGHT";
					break;
				case 8:
					m_wheel.gps_location_state = "SINGLE";
					break;
				case 16:
					m_wheel.gps_location_state = "DOPPLER_VELOCITY";
					break;
				case 17:
					m_wheel.gps_location_state = "PSRDIFF";
					break;
				case 18:
					m_wheel.gps_location_state = "SBAS";
					break;
				case 32:
					m_wheel.gps_location_state = "L1_FLOAT";
					break;
				case 33:
					m_wheel.gps_location_state = "IONOFREE_FLOAT";
					break;
				case 34:
					m_wheel.gps_location_state = "NARROW_FLOAT";
					break;
				case 48:
					m_wheel.gps_location_state = "L1_INT";
					break;
				case 49:
					m_wheel.gps_location_state = "WIDE_INT";
					break;
				case 50:
					m_wheel.gps_location_state = "NARROW_INT";
					break;
			}
		}
		case 33:
			m_wheel.is_wheel_speed  = wheel_data2;
			break; 
	}
	
	m_pub_wheel.publish(m_wheel);     // 发布轮循信息
	
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

void Daoyuan::stopReading()
{
	m_reading_status = false;
}


int main(int argc,char** argv)
{
	ros::init(argc,argv,"daoyuan_node");
	
	Daoyuan gps;
	
	if(gps.init())
	{
		gps.startReading();
		ROS_INFO("%s initial ok...",ros::this_node::getName().c_str());
	}
	
	ros::spin();
}



