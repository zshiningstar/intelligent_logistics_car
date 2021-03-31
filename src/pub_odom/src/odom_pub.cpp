#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <string>
#include <vector>
#include <logistics_msgs/RealState.h>

using namespace std;

class Odometry
{
private:

	bool init();
	void handle_speed_msg(uint8_t* buffer_data);
	void RealState_callback(const logistics_msgs::RealState::ConstPtr& msg);
	
public:

	Odometry(){};
	~Odometry(){};
	void run();
private:
	ros::Subscriber m_sub_state;
	ros::Publisher odom_pub;
	ros::NodeHandle node;
	std::string car_state;
	tf::TransformBroadcaster odom_broadcaster;
	ros::Time current_time, last_time;
	nav_msgs::Odometry odom;
	logistics_msgs::RealState m_state;
	
	bool prase_flag_;
	double x,y,th,vx,vy,vth;
	double real_speed_left,real_speed_right,real_angle,real_speed;
};
void Odometry::RealState_callback(const logistics_msgs::RealState::ConstPtr& msg)
{	
	real_speed_left = msg->real_speed_left;
	real_speed_right = msg->real_speed_right;
	real_angle = msg->real_angle;
	real_speed = (real_speed_left + real_speed_right) / 2;
}
bool Odometry::init()
{	
	ros::NodeHandle private_node("~");
	car_state = private_node.param<std::string>("car_state","/car_state");
	odom_pub = node.advertise<nav_msgs::Odometry>("/wheel_odom", 50);
	return true;
}

void Odometry::run()
{
	if(init())
	{
		m_sub_state = node.subscribe(car_state ,1,&Odometry::RealState_callback, this);
		prase_flag_ = true;
	}
	if(prase_flag_)
	{
		//假设机器人最初从"odom"坐标系的原点开始
		double x = 0.0;
		double y = 0.0;
		double th = 0.0;
		//设置一些速度，其将导致“base_link”坐标系相对于“odom”坐标系，在x方向上以0.1m/s，在y方向上以-0.1m/s的速率和在th方向以0.1rad/s角度移动。这让虚拟机器人走一个圆圈。
		double vx = real_speed;
		double vy = 0;
		double vth = real_angle;
	}
	
	while(ros::ok())
	{
		current_time = ros::Time::now();
		last_time = ros::Time::now();
		//1hz的速度发布里程计信息(里程计包含2个方面的信息：1.位姿（位置和转角）即（x,y,θ）2.速度（前进速度和转向速度))
		ros::Rate r(1.0);
		ros::spinOnce();               // check for incoming messages
		current_time = ros::Time::now();

		//compute odometry in a typical way given the velocities of the robot
		double dt = (current_time - last_time).toSec();
		double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
		double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
		double delta_th = vth * dt;

		x += delta_x;
		y += delta_y;
		th += delta_th;

		//里程计的偏航角转为四元数
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

		//首先,创建一个TransformStamped消息,通过tf发送;在current_time发布"odom"坐标到"base_link"的转换
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";//父坐标系
		odom_trans.child_frame_id = "base_link";//子

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
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pub_odom");
	Odometry odom;
	odom.run();
	return 0;
}

