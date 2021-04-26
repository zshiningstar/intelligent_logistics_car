
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/qnode.hpp"

namespace av_console {

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv),
    is_init(false),
    as_online_(false),
    task_state_(Idle),
    ac_(nullptr)
{
    sensors.resize(4);
    sensors[Sensor_Gps] = &gps;
    sensors[Sensor_Livox] = &livox;
    sensors[Sensor_Lidar] = &lidar;
    sensors[Sensor_Camera1] = &camera1;
}

QNode::~QNode()
{
    if(ac_ != nullptr)
    {
        delete ac_;
        ac_ = nullptr;
    }
    if(ros::isStarted())
    {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init()
{
	ros::init(init_argc,init_argv,"av_console");
    if ( ! ros::master::check() )
    {
        std::thread t(&QNode::roscoreThread, this);
        t.detach();
		return false;
	}
    is_init = true;

    //subscribe sensor msg to listen its status.
    ros::NodeHandle nh;
    gps_sub = nh.subscribe("/Rotation",1,&QNode::gpsCheck_callback,this);
    lidar_sub =  nh.subscribe("/lslidar_point_cloud",1,&QNode::lidar_callback,this);
    livox_sub =  nh.subscribe("/livox/lidar",1,&QNode::livox_callback,this);
    sensorStatus_timer = nh.createTimer(ros::Duration(1), &QNode::sensorStatusTimer_callback,this);

    if(ac_ != nullptr)
    {
        delete ac_;
        ac_ = nullptr;
    }

    ac_ = new DoDriverlessTaskClient("/do_driverless_task", true); // true -> don't need ros::spin()

	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url)
{
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"av_console");
    if ( ! ros::master::check())
		return false;
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
    start();
	is_init = true;
	return true;
}

/*QThread 线程函数，start后开始执行*/
void QNode::run()
{
    ros::AsyncSpinner spinner(5);
    spinner.start(); //非阻塞

    while(ros::ok() && ros::master::check())
    {
        //std::cout << ac_->getState().toString() << std::endl;
        ros::Duration(1.0).sleep();
    }
    if(!ros::master::check())
    {
        stampedLog(Warn, "Ros Master is shutdown. You Must Reconnect Before Any Task!");
        Q_EMIT rosmasterOffline();
        is_init = false;
    }
}

bool QNode::serverConnected()
{
    return ac_->isServerConnected();

}

void QNode::cancleAllGoals()
{
    std::cout << "cancleAllGoals" << std::endl;
    ac_->cancelAllGoals();
}

void QNode::requestDriverlessTask(const driverless::DoDriverlessTaskGoal& goal)
{
    ac_->sendGoal(goal, boost::bind(&QNode::taskDoneCallback,this,_1,_2),
                        boost::bind(&QNode::taskActivedCallback,this),
                        boost::bind(&QNode::taskFeedbackCallback,this,_1));
}

void QNode::taskFeedbackCallback(const driverless::DoDriverlessTaskFeedbackConstPtr& fd)
{
    qDebug() << "taskFeedbackCallback ";
}

void QNode::taskDoneCallback(const actionlib::SimpleClientGoalState& state,
                             const driverless::DoDriverlessTaskResultConstPtr& res)
{
    qDebug() << "taskDoneCallback ";
    Q_EMIT taskStateChanged(Driverless_Complete);
}

void QNode::taskActivedCallback()
{
    qDebug() << "taskActivedCallback ";
    Q_EMIT taskStateChanged(Driverless_Running);
}

/*===================传感器状态监测相关函数================*/
/*官方驱动则使用消息回调检测，用户自定义驱动则使用故障诊断消息检测*/
void QNode::gpsCheck_callback(const gps_msgs::Rotation::ConstPtr& gps_msg)
{
    //qDebug() << "gpsFix_callback ";
    double time = ros::Time::now().toSec();
    if(gps_msg->num_satellites > 25)
        gps.last_update_time = time;
}

void QNode::lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& )
{
    //qDebug() << "lidar_callback ";
    static int i = 0;
    if((++i)%5 == 0) //降低更新时间覆盖频率
        lidar.last_update_time = ros::Time::now().toSec();
}
void QNode::livox_callback(const sensor_msgs::PointCloud2::ConstPtr& )
{
    //qDebug() << "livox_callback ";
    static int i = 0;
    if((++i)%5 == 0) //降低更新时间覆盖频率
        livox.last_update_time = ros::Time::now().toSec();
}
//void QNode::diagnostic_callback(const diagnostic_msgs::DiagnosticStatus::ConstPtr& msg)
//{
//    if(msg->hardware_id=="esr_radar")
//        esr.last_update_time = ros::Time::now().toSec();
//    else if(msg->hardware_id=="camera")
//        camera1.last_update_time = ros::Time::now().toSec();
//}
void QNode::sensorStatusTimer_callback(const ros::TimerEvent& )
{
    //qDebug() << "sensorStatusTimer_callback\r\n" ;
    static float tolerateInterval = 1.0;
    double now = ros::Time::now().toSec();
    for(size_t i=0; i<sensors.size(); ++i)
    {
        float interval = now - sensors[i]->last_update_time;
        if(sensors[i]->status==true && interval > tolerateInterval)
        {
            sensors[i]->status = false;
            Q_EMIT sensorStatusChanged(i,false);
            //qDebug() << "sensorStatusTimer_callback " << i << "\t" << sensors[i]->status;
        }
        else if(sensors[i]->status==false && interval < tolerateInterval)
        {
            sensors[i]->status = true;
            Q_EMIT sensorStatusChanged(i,true);
            //qDebug() << "sensorStatusTimer_callback " << i << "\t" << sensors[i]->status;
        }
    }
}

void QNode::log(const std::string &msg)
{
    logging_model.insertRows(logging_model.rowCount(),1);

    QVariant new_row(QString(msg.c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar

}

void QNode::stampedLog( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
    double now = ros::Time::now().toSec();

    logging_model_msg << std::fixed << std::setprecision(2);

	switch ( level ) {
        case(Debug) :
            ROS_DEBUG_STREAM(msg);
            logging_model_msg << "[DEBUG] [" << now << "]: " << msg;
            break;
        case(Info) :
            ROS_INFO_STREAM(msg);
            logging_model_msg << "[INFO] [" << now << "]: " << msg;
            break;
        case(Warn) :
            ROS_WARN_STREAM(msg);
            logging_model_msg << "[INFO] [" << now << "]: " << msg;
            break;
        case(Error) :
            ROS_ERROR_STREAM(msg);
            logging_model_msg << "[ERROR] [" << now << "]: " << msg;
            break;
        case(Fatal) :
            ROS_FATAL_STREAM(msg);
            logging_model_msg << "[FATAL] [" << now << "]: " << msg;
            break;
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace av_console
