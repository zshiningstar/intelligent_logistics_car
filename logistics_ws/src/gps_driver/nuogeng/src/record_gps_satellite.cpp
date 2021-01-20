#include<iostream>
#include<fstream>
#include<ros/ros.h>
#include<serial/serial.h>
#include<boost/thread.hpp>
#include<boost/bind.hpp>
#include"gps_msgs/Inspvax.h"
#include<nav_msgs/Odometry.h>
#include"gps_msgs/Satellite.h"
#include"gps_msgs/Satellites.h"
#include<message_filters/subscriber.h>
#include<message_filters/time_synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>


using namespace std;

class Recorder
{
public:
	typedef message_filters::sync_policies::ApproximateTime<gps_msgs::Inspvax, nav_msgs::Odometry, gps_msgs::Satellites> MySyncPolicy;
	Recorder(){}
	~Recorder()
	{
		if(mOutFileInspvax.is_open())
			mOutFileInspvax.close();
		if(mOutFileSatellite.is_open())
			mOutFileSatellite.close();
	}
	
	void gps_callback(const gps_msgs::Inspvax::ConstPtr& msg)
	{
		mInspvaxMsg = *msg;
	}
	
	void satellite_callback(const gps_msgs::Satellites::ConstPtr& msgs)
	{
		mSatellites = *msgs;
	}
	
	void timer_callback(const ros::TimerEvent&)
	{
		double gps_time = mInspvaxMsg.header.stamp.toSec();
		double satellite_time = mSatellites.header.stamp.toSec();
		
		static size_t count = 0;
		mOutFileInspvax << fixed << setprecision(3) << gps_time << "\t" << setprecision(7)
						<< mInspvaxMsg.latitude << "\t" << mInspvaxMsg.longitude << "\t"
						<< setprecision(2)
						<< mInspvaxMsg.azimuth << "\t" << mInspvaxMsg.height <<"\r\n";
		
		mOutFileSatellite << fixed << setprecision(3) << satellite_time  << "\r\n";
		for(auto& satellite:mSatellites.satellites)
		{
			mOutFileSatellite << setw(6)
			 				  << int(satellite.system) << "\t" << int(satellite.num) << "\t"
							  << int(satellite.frequency[0]) <<"\t" << int(satellite.frequency[1]) <<"\t" << int(satellite.frequency[2]) << "\t"
							  << int(satellite.elevation) << "\t" << int(satellite.azimuth) << "\r\n";
		}
		mOutFileSatellite << "--------------------------\r\n";
		ROS_INFO("%5ld: recording...",++count);
	}
	
	void record_callback(const gps_msgs::Inspvax::ConstPtr& gpsMsg, 
						 const nav_msgs::Odometry::ConstPtr& odomMsg,
						 const gps_msgs::Satellites::ConstPtr& satelliteMSg)
	{
		double gps_time = gpsMsg->header.stamp.toSec();
		double satellite_time = satelliteMSg->header.stamp.toSec();
		
		static size_t count = 0;
		mOutFileInspvax << fixed << setprecision(3) << gps_time << "\t" << setprecision(7)
						<< gpsMsg->latitude << "\t" << gpsMsg->longitude << "\t"
						<< setprecision(3)
						<< gpsMsg->azimuth << "\t" 
						<< odomMsg->pose.pose.position.x << "\t" 
						<< odomMsg->pose.pose.position.y << "\t" 
						<< odomMsg->pose.pose.position.z <<"\r\n";
						
		mOutFileInspvax.flush();
		
		mOutFileSatellite << fixed << setprecision(3) << satellite_time  << "\r\n";
		for(auto& satellite:satelliteMSg->satellites)
		{
			mOutFileSatellite << setw(6)
			 				  << int(satellite.system) << "\t" << int(satellite.num) << "\t"
							  << int(satellite.frequency[0]) <<"\t" << int(satellite.frequency[1]) <<"\t" << int(satellite.frequency[2]) << "\t"
							  << int(satellite.elevation) << "\t" << int(satellite.azimuth) << "\r\n";
		}
		mOutFileSatellite << "--------------------------\r\n";
		mOutFileSatellite.flush();
		ROS_INFO("%5ld: recording...",++count);
	
	}
	
	bool init(int argc,char** argv)
	{
		ros::init(argc,argv,"record_data_node");
		ros::NodeHandle nh;
		ros::NodeHandle nh_private("~");
		
		std::string gps_topic = nh_private.param<string>("gps_topic","/gps");
		std::string odom_topic = nh_private.param<string>("odom_topic","/odom");
		std::string satellite_topic = nh_private.param<string>("satellite_topic","/satellite");
		
		bool use_synchronizer = nh_private.param<bool>("use_synchronizer",true);
		if(!use_synchronizer)
		{
			mSubGpsMsg = nh.subscribe(gps_topic, 1, &Recorder::gps_callback,this);
			mSubSatelliteMsg = nh.subscribe(satellite_topic ,1, &Recorder::satellite_callback, this);
			mTimer = nh.createTimer(ros::Duration(nh_private.param<float>("record_duration",1.0)),&Recorder::timer_callback, this);
		}
		else
		{
			mSubGpsPtr.reset(new message_filters::Subscriber<gps_msgs::Inspvax>(nh,gps_topic,50));
			mSubSatellitePtr.reset(new message_filters::Subscriber<gps_msgs::Satellites>(nh,satellite_topic,1));
			mSubOdomPtr.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh,odom_topic,1));
			mSyncPtr.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(50),*mSubGpsPtr, *mSubOdomPtr, *mSubSatellitePtr));
			mSyncPtr->registerCallback(boost::bind(&Recorder::record_callback, this, _1,_2, _3));
		}
		
		string file_name1 = nh_private.param<string>("satellite_file", "satellite.txt");
		string file_name2 = nh_private.param<string>("gps_file", "gps.txt");
		
		mOutFileSatellite.open(file_name1);
		if(!mOutFileSatellite.is_open()) 
		{
			ROS_ERROR("open %s failed!!!",file_name1.c_str());
			return false;
		}
		
		mOutFileInspvax.open(file_name2);
		if(!mOutFileInspvax.is_open())
		{
			ROS_ERROR("open %s failed!!!",file_name2.c_str());
			return false;
		}
		
		
		ROS_INFO("open %s and %s ok ...",file_name1.c_str(),file_name2.c_str());
		
			
		mOutFileInspvax << "stamp\t" << "latitude\t" << "longitude\t" <<"yaw\t" << "height\r\n";
		mOutFileSatellite << "stamp\t" <<  "navigation_system\t" << "satellite_num\t" << "satellite_frequency 1 3 3\t" << "elevation\t" << "azimuth\t" << "\r\n"; 
		return true;
	}


private:
	ros::Subscriber mSubGpsMsg;
	ros::Subscriber mSubSatelliteMsg;
	
	std::unique_ptr<message_filters::Subscriber<gps_msgs::Inspvax> > mSubGpsPtr;
	std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry> > mSubOdomPtr;
	std::unique_ptr<message_filters::Subscriber<gps_msgs::Satellites> > mSubSatellitePtr;
	std::unique_ptr<message_filters::Synchronizer<MySyncPolicy> >mSyncPtr;
	
	ros::Timer mTimer;
	gps_msgs::Inspvax mInspvaxMsg;
	gps_msgs::Satellites mSatellites;
	
	ofstream mOutFileInspvax;
	ofstream mOutFileSatellite;
	
};

int main(int argc, char** argv)
{
	Recorder recorder;
	if(!recorder.init(argc,argv))
		return 0;
	ROS_INFO("%s initial ok...",ros::this_node::getName().c_str());
	ros::spin();
	return 0;
}


