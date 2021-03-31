#include"listener/listener.h"

using namespace std;
	
Listener::Listener():				
	m_reading_status(false),
	prase_flag_(true),
	m_pkg_buffer(new uint8_t[500])
{
}

Listener::~Listener()
{
	this->closeSerial();
	if(m_pkg_buffer!=NULL)
	delete [] m_pkg_buffer;
}

double Listener::generate_real_speed(double& temp1,double& temp2)
{	
	left_wheel_speed = 2 * M_PI * Wheel_Radius * temp1;
	right_wheel_speed = 2 * M_PI * Wheel_Radius * temp2;
	speed = (left_wheel_speed + right_wheel_speed) / 2;
	return speed;
}

bool Listener::openSerial(const std::string& port,int baudrate)
{
	try
	{
		m_serial_port = new serial::Serial(port,baudrate,serial::Timeout::simpleTimeout(10)); 
	}
	catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
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

void Listener::closeSerial()
{
	if(m_serial_port!=NULL)
	{
		delete m_serial_port;
		m_serial_port = NULL;
	}
}

bool Listener::init()
{
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	odom_pub = nh.advertise<nav_msgs::Odometry>("/wheel_odom", 50);
	m_pub_state = nh.advertise<logistics_msgs::RealState>(nh_private.param<std::string>("car_state","/car_state"),10);
	std::string port_name = nh_private.param<std::string>("port_name","/dev/pts/23");
	int baudrate = nh_private.param<int>("baudrate",115200);
	if(!openSerial(port_name,baudrate))
		return false;
	return true;
}

void Listener::startReading()
{
	if(m_reading_status)
		return ;
	m_read_thread_ptr = boost::shared_ptr<boost::thread>(new boost::thread(&Listener::readSerialThread,this));   // 智能指针赋值时 调用读取线程函数
}

void Listener::readSerialThread()
{
	m_reading_status = true;
	
	const int Max_read_size = 200;
	uint8_t * const raw_data_buf = new uint8_t[Max_read_size];
	
	size_t get_len;
		   
	while(ros::ok() && m_reading_status)
	{	
		try
		{
			get_len = m_serial_port->read(raw_data_buf, Max_read_size);
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
		parseIncomingData(raw_data_buf, get_len);
		ros::Duration(0.01).sleep();
		//boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	delete [] raw_data_buf;
}

void Listener::parseIncomingData(uint8_t* buffer,size_t len)
{	
	static unsigned char pkg_buffer[11];
	static size_t pkg_buffer_index = 0; //数据包缓存定位索引
	size_t pkg_len = 0;
	
	for(size_t i=0; i<len; ++i)
	{
		if(0 == pkg_buffer_index) 
		{
			if(0x66 == buffer[i])
				pkg_buffer[pkg_buffer_index++] = buffer[i];
		}
		else if(1 == pkg_buffer_index)
		{
			if(0xcc == buffer[i])
			{
				pkg_buffer[pkg_buffer_index++] = buffer[i];
			}
			else
				pkg_buffer_index = 0;
		}
		else
		{
			pkg_buffer[pkg_buffer_index++] = buffer[i];
			if(pkg_buffer_index == 10)
			{		
				pkg_len = pkg_buffer[3];
				if((pkg_buffer[2] == 0x02) && sumCheck(pkg_buffer,pkg_len))    //0x02代表下位机向上位机发布的消息包id; 0xe0代表车速信号、制动信号、转角信号有效
				if(1)
				{
					parseFromStm(pkg_buffer);
				}					
				pkg_buffer_index = 0;
			}
		}
	}
}

void Listener::run()
{
	if(init())
	{	
		prase_flag_ = true;
		startReading();
		ROS_INFO("%s initial ok...",ros::this_node::getName().c_str());
	}
	ros::spin();
}

bool Listener::sumCheck(const unsigned char* pkg_buffer, size_t pkg_len)
{	
	size_t sum = 0;
	for(int i = 2;i<pkg_len + 3;i++)
	{	
		sum = sum + pkg_buffer[i];
	}
	if(sum == pkg_buffer[10])
	{	
		std::cout << sum << endl;
		return true;
	}

	else
		return true;
}

void Listener::parseFromStm(const unsigned char* buffer)
{	
	m_state.header.stamp = ros::Time::now();
	m_state.header.frame_id = "car_state";
	
	m_state.real_speed_left = ((buffer[5] * 256 + buffer[6]) - 30000) * 0.01;
	m_state.real_speed_right = ((buffer[7] * 256 + buffer[8]) - 30000) * 0.01;
	m_state.real_angle = ((buffer[9] * 256 + buffer[10]) - 30000) * 0.01;
	m_state.real_brake = buffer[11] & 0xf0;
	m_state.real_park  = buffer[11] & 0x0f;
	
	m_pub_state.publish(m_state);
	printf("speed:%0.2f\t angle:%0.2f\n",m_state.real_speed,m_state.real_angle);
	
	real_speed_left = m_state.real_speed_left;
	real_speed_right = m_state.real_speed_right;
	real_angle = m_state.real_angle;
	real_speed = generate_real_speed(real_speed_left,real_speed_right);
	
	if(prase_flag_)
	{
		double x = 0.0;
		double y = 0.0;
		double th = 0.0;
		last_time = ros::Time::now();
		prase_flag_ = false;
	}
		double dt = (current_time - last_time).toSec();
		vx = real_speed; // forward
		vy = 0;
		vth = (right_wheel_speed - left_wheel_speed) / AXIS_DISTANCE;  //左转角度为正
		
		double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
		double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
		double delta_th = vth * dt;
		ros::Rate r(1.0);
		ros::spinOnce();               // check for incoming messages
		current_time = ros::Time::now();
		
		x += delta_x;
		y += delta_y;
		th += delta_th;
		//里程计的偏航角转为四元数
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
		//首先,创建一个TransformStamped消息,通过tf发送;在current_time发布"odom"坐标到"base_link"的转换
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";

		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		odom_broadcaster.sendTransform(odom_trans);

		//next, we'll publish the odometry message over ROS
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_link";
		odom.header.stamp = current_time;
		//里程数据填充消息
		//set the position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//set the velocity
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = vth;

		//publish the message
		odom_pub.publish(odom);
		ROS_DEBUG_STREAM("accumulation_x: " << x << "; accumulation_y: " << y <<"; accumulation_th: " << vth);
		last_time = current_time;
		r.sleep();
}

void Listener::stopReading()
{
	m_reading_status = false;
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"listener_node");
	Listener lis;
	lis.run();
}
