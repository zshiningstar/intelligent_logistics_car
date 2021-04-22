#include "record_path.h"

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
	
	private_nh.param<std::string>("file_path",file_path_,"");          // 文件路径
	private_nh.param<std::string>("file_name",file_name_,"");          // 文件名字
	
	if(file_path_.empty() || file_name_.empty())
	{
		ROS_ERROR("please input record file path and file name in launch file!!!");   // launch文件中输入参数数值
		return false;
	}
	
	private_nh.param<float>("sample_distance",sample_distance_,0.1);          
	std::string utm_topic = private_nh.param<std::string>("utm_topic","/odom");      // 订阅的是gps节点里面的话题
	
	sub_gps_ = nh.subscribe(utm_topic ,1,&Record::gps_callback,this);                       // 回调

	fp = fopen((file_path_+file_name_).c_str(),"w");                                        // 新建一个文件只允许写
	
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
	current_point.yaw = msg->pose.covariance[0];        // 计算航向角的偏差
	
	if(sample_distance_*sample_distance_ <= dis2Points(current_point,last_point,false))
	{
		fprintf(fp,"%.3f\t%.3f\t%.4f\n",current_point.x,current_point.y,current_point.yaw);
		fflush(fp);                                     //把缓冲区的内容强制输出到fp文件里面
		
		ROS_INFO("row:%d\t%.3f\t%.3f\t%.3f",row_num++,current_point.x,current_point.y,current_point.yaw);
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
