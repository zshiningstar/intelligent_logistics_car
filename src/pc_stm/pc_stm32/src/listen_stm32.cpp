#include"listener/listener.h"
#include<logistics_msgs/RealState.h>
#include<stdio.h>

using namespace std;

Listener::Listener():				
	m_reading_status(false),
	m_pkg_buffer(new uint8_t[500])
{
}				

Listener::~Listener()
{
	this->closeSerial();
	if(m_pkg_buffer!=NULL)
		delete [] m_pkg_buffer;
}            


/*----------------------打开串口--------------------*/

bool Listener::openSerial(const std::string& port,int baudrate)              // 打开串口
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

/*----------------------关闭串口-------------------*/
void Listener::closeSerial()                                              // 关闭串口
{
	if(m_serial_port!=NULL)
	{
		delete m_serial_port;
		m_serial_port = NULL;
	}
}


/*---------------------初始化---------------------*/

bool Listener::init()  // 初始化
{
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	m_pub_state = nh.advertise<logistics_msgs::RealState>(nh_private.param<std::string>("car_state","/car_state"),10);

	std::string port_name = nh_private.param<std::string>("port_name","/dev/pts/23");  // launch文件对应
	
	int baudrate = nh_private.param<int>("baudrate",115200);
	if(!openSerial(port_name,baudrate))
		return false;
	return true;
}


/*---------------开始从串口读取数据----------------*/
void Listener::startReading()              // 开始读取
{
	if(m_reading_status)   // 判断是否开始运行读取线程，此标志位初始时设置为false
		return ;
	m_read_thread_ptr = boost::shared_ptr<boost::thread>(new boost::thread(&Listener::readSerialThread,this));   // 智能指针赋值时 调用读取线程函数
}

void Listener::readSerialThread()
{
	m_reading_status = true;
	
	const int Max_read_size = 200;
	uint8_t * const raw_data_buf = new uint8_t[Max_read_size];
	
	size_t get_len;
		   
	while(ros::ok() && m_reading_status)    // 不停的读取数据
	{	
		try									// 异常处理
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
		parseIncomingData(raw_data_buf, get_len);      // 解析数据
		
		ros::Duration(0.01).sleep();
		
		//boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	
	delete [] raw_data_buf;
}

void Listener::parseIncomingData(uint8_t* buffer,size_t len)
{	
//	std::cout << 2222222 << endl;
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
//				std::cout << 333333 << endl;
				pkg_len = pkg_buffer[3];
//				std::cout << hex << pkg_buffer[11] << endl;
				if((pkg_buffer[2] == 0x02) && sumCheck(pkg_buffer,pkg_len))    //0x02代表下位机向上位机发布的消息包id; 0xe0代表车速信号、制动信号、转角信号有效
				if(1)
					{
//					std::cout << 4444444 << endl;
					parseFromStm(pkg_buffer);
					}					
				pkg_buffer_index = 0;
			}
		}
	}
}

/*-------------------------求和校验-------------------*/
bool Listener::sumCheck(const unsigned char* pkg_buffer, size_t pkg_len)
{	
//	std::cout << 4444444 << endl;
	size_t sum = 0;
	for(int i = 2;i<pkg_len + 3;i++)
	{	
		sum = sum + pkg_buffer[i];
	}
//	std::cout << sum << endl;
	if(sum == pkg_buffer[10])
	{	
		std::cout << sum << endl;
		return true;
	}

	else
		return true;
}

/*---------------------解析当前车辆状态信息-----------*/
void Listener::parseFromStm(const unsigned char* buffer)
{	
	m_state.header.stamp = ros::Time::now();
	m_state.header.frame_id = "car_state";
	
	
	m_state.real_speed = ((buffer[5] * 256 + buffer[6]) - 30000) * 0.01;
	m_state.real_angle = ((buffer[7] * 256 + buffer[8]) - 30000) * 0.01;
	m_state.real_brake = buffer[9] & 0xf0;
	m_state.real_park  = buffer[9] & 0x0f;
	
	
	m_pub_state.publish(m_state);
//	std::cout << "上位机接收数据ok" << std::endl;

//	printf("buffer[5]:%d\t buffer[6]:%d\t buffer[7]:%d\t buffer[8]:%d\n",buffer[5],buffer[6],buffer[7],buffer[8]);
	printf("speed:%0.2f\t angle:%0.2f\n",m_state.real_speed,m_state.real_angle);
//	std::cout << "real_speed:" << setm_state.real_speed << std::endl;
//	std::cout << "real_angle：" << m_state.real_angle << std::endl;
}

/*--------------------停止读取----------------------*/
void Listener::stopReading()
{
	m_reading_status = false;
}

int main(int argc,char** argv)
{
//	std::cout << 0000000 << endl;
	ros::init(argc,argv,"listener_node");
	
	Listener lis;
	
	if(lis.init())
	{
		lis.startReading();
		ROS_INFO("%s initial ok...",ros::this_node::getName().c_str());
	}
	
	ros::spin();
}

