#include <ros/ros.h>
#include <unistd.h>
#include <cmath>
#include<nav_msgs/Odometry.h> 

typedef struct
{
	double longitude;
	double latitude;
	double yaw;
	double x;
	double y;
	float curvature;
}gpsMsg_t;

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
		void gps_callback(const nav_msgs::Odometry::ConstPtr& msg);
		std::string file_path_;
		std::string	file_name_;
		FILE *fp;
		gpsMsg_t last_point , current_point;
		
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
	last_point = {0.0,0.0,0.0,0.0,0.0};
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
	
	private_nh.param<std::string>("file_path",file_path_,"");
	private_nh.param<std::string>("file_name",file_name_,"");
	
	if(file_path_.empty() || file_name_.empty())
	{
		ROS_ERROR("please input record file path and file name in launch file!!!");
		return false;
	}
	
	private_nh.param<float>("sample_distance",sample_distance_,0.1);
	std::string utm_topic = private_nh.param<std::string>("utm_topic","/ll2utm_odom");
	
	sub_gps_ = nh.subscribe(utm_topic ,1,&Record::gps_callback,this);

	fp = fopen((file_path_+file_name_).c_str(),"w");
	
	if(fp == NULL)
	{
		ROS_INFO("open record data file %s failed !!!",(file_path_+file_name_).c_str());
		return false;
	}
	return true;
}

void Record::gps_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	static size_t  row_num = 0;
	current_point.x = msg->pose.pose.position.x;
	current_point.y = msg->pose.pose.position.y;
	current_point.yaw = msg->pose.covariance[0];
	
	if(sample_distance_*sample_distance_ <= dis2Points(current_point,last_point,false))
	{
		fprintf(fp,"%.3f\t%.3f\t%.4f\n",current_point.x,current_point.y,current_point.yaw);
		fflush(fp);
		
		ROS_INFO("row:%d\t%.3f\t%.3f\t%.3f",row_num++,current_point.x,current_point.y,current_point.yaw);
		last_point = current_point;
	}
}


int main(int argc,char**argv)
{
	ros::init(argc,argv,"record_data_node");
	
	Record record;
	
	if(!record.init())
		return 1;

	ros::spin();
	
	return 0;
}


