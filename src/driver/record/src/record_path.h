#ifndef RECORD_H
#define RECORD_H

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

/*@fuc:  计算两点间的距离
 */
float dis2Points(const gpsMsg_t& point1, const gpsMsg_t& point2,bool is_sqrt)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;

	if(is_sqrt)
		return sqrt(x*x +y*y);
	return x*x+y*y;
}
float generateCurvature(const gpsMsg_t& point1, const gpsMsg_t& point2)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	
	return y / x * 1.0;
}
class Record
{
	private:
		void gps_callback(const nav_msgs::Odometry::ConstPtr& msg);    // nav_msgs::Odometry：里程计消息类型
		std::string file_path_;
		std::string	file_name_;
		FILE *fp;                               // 定义一个文件指针
		gpsMsg_t last_point , current_point ,next_point;    // 上一个点，下一个点
		
		float sample_distance_;
		ros::Subscriber sub_gps_;
		
	public:
		Record();
		~Record();
		bool init();
		void recordToFile();
};

#endif
