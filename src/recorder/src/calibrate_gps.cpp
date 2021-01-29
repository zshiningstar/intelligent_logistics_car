#include <ros/ros.h>
#include <unistd.h>
#include <cmath>
#include <nav_msgs/Odometry.h> 

typedef struct
{
	double longitude;  // 经度
	double latitude;   // 维度
	double yaw;        // 航向角
	double x;
	double y;
	float curvature;   // 曲率
}gpsMsg_t;

std::pair<float, float> get_dis_yaw(gpsMsg_t &point1,gpsMsg_t &point2)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	
	std::pair<float, float> dis_yaw;
	dis_yaw.first = sqrt(x * x + y * y);
	dis_yaw.second = atan2(x,y);
	
	if(dis_yaw.second <0)
		dis_yaw.second += 2*M_PI;
	return dis_yaw;
}

/*
 *@fuc:  计算两点间的距离
 *
 */

float dis2Points(const gpsMsg_t& point1, const gpsMsg_t& point2,bool is_sqrt)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;

	if(is_sqrt)
		return sqrt(x*x +y*y);
	return x*x+y*y;
}

class Record
{
	private:
		void gps_callback(const nav_msgs::Odometry::ConstPtr& msg);    // nav_msgs::Odometry：里程计消息类型
		std::string file_path_;
		std::string	file_name_;
		FILE *fp;                               // 定义一个文件指针
		gpsMsg_t last_point , current_point;    // 上一个点，下一个点
		
		float sample_distance_;
		ros::Subscriber sub_gps_;
		
	public:
		Record();
		~Record();
		bool init();
		void recordToFile();
};

Record::Record()
{
	last_point = {0.0,0.0,0.0,0.0,0.0};     // 上一点的经度、纬度、航向角、x、y坐标（没有道路曲率的赋值）
	current_point = last_point;
}

Record::~Record()
{
	if(fp != NULL)
		fclose(fp);
}

bool Record::init()
{
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	
	
	private_nh.param<float>("sample_distance",sample_distance_,0.1);          
	std::string utm_topic = private_nh.param<std::string>("utm_topic","/odom");      // 订阅的是gps节点里面的话题
	
	sub_gps_ = nh.subscribe(utm_topic ,1,&Record::gps_callback,this);                       // 回调
	return true;
}

void Record::gps_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	static size_t  row_num = 0;
	current_point.x = msg->pose.pose.position.x;
	current_point.y = msg->pose.pose.position.y;
	current_point.yaw = msg->pose.covariance[0];        // 计算航向角的偏差
	std::pair<float, float> dis_yaw = get_dis_yaw(current_point, last_point);   
	
	static double sum_yaw_err = 0;
	static int count = -1;
	if(sample_distance_ <= dis_yaw.first)
	{	
		count++;
		
		if(count == 0)
		{
			last_point = current_point;
			return;
		}
		
		double yaw_err = dis_yaw.second - current_point.yaw;
		
		if(yaw_err > M_PI)
			yaw_err -= 2 * M_PI;
		else if(yaw_err < -M_PI)
			yaw_err += 2 * M_PI;
		
		sum_yaw_err += yaw_err;
		std::cout << "mean_yaw_err:" << sum_yaw_err/count * 180 / M_PI << std::endl;
		
		last_point = current_point;                     // 更新上一个点的信息
	}
}

int main(int argc,char**argv)
{
	ros::init(argc,argv,"record_node");
	
	Record record;
	
	if(!record.init())
		return 1;

	ros::spin();
	
	return 0;
}


