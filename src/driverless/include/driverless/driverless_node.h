
#include <ros/ros.h>
#include <memory>
#include "path_tracking.h"
#include "car_following.h"
#include "reverse_drive.h"
#include "extern_control/extern_control.h"
#include <pathplaning_msgs/expected_path.h>

#include <ant_msgs/ControlCmd1.h>
#include <ant_msgs/ControlCmd2.h>
#include <ant_msgs/State1.h>  //gear
#include <ant_msgs/State3.h>  //
#include <ant_msgs/State2.h>  //speed
#include <ant_msgs/State4.h>  //steerAngle
#include "auto_drive_base.h"
#include <condition_variable>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <driverless/DoDriverlessTaskAction.h>   // Note: "Action" is appended


class AutoDrive : public AutoDriveBase
{
public:
    typedef actionlib::SimpleActionClient<driverless::DoDriverlessTaskAction> DoDriverlessTaskClient;
    typedef actionlib::SimpleActionServer<driverless::DoDriverlessTaskAction> DoDriverlessTaskServer;

    AutoDrive();
    ~AutoDrive();
    virtual bool init(ros::NodeHandle nh,ros::NodeHandle nh_private) override;
    void run(){};

private:
    bool loadVehicleParams();
    bool loadDriveTaskFile(const std::string& file);
    bool setDriveTaskPathPoints(const driverless::DoDriverlessTaskGoalConstPtr& goal);
	void publishPathTrackingState();
    bool isGpsPointValid(const GpsPoint& point);
    void vehicleSpeed_callback(const ant_msgs::State2::ConstPtr& msg);
    void vehicleState4_callback(const ant_msgs::State4::ConstPtr& msg);
    void vehicleState1_callback(const ant_msgs::State1::ConstPtr& msg);

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void sendCmd1_callback(const ros::TimerEvent&);
	void sendCmd2_callback(const ros::TimerEvent&);
    void captureExernCmd_callback(const ros::TimerEvent&);
    void setSendControlCmdEnable(bool flag);
    void goal_callback(const pathplaning_msgs::expected_path::ConstPtr& msg);
    void executeDriverlessCallback(const driverless::DoDriverlessTaskGoalConstPtr& goal);
    bool handleNewGoal(const driverless::DoDriverlessTaskGoalConstPtr& goal);

    ant_msgs::ControlCmd2 driveDecisionMaking();
    ant_msgs::ControlCmd2 reverseDecisionMaking();
    

    bool isReverseGear();
    bool isDriveGear();
    bool isNeutralGear();

    void workingThread();
    void doDriveWork();
    void doReverseWork();

    void waitGearOk(int gear);
    void waitSpeedZero();

    enum State
    {
        State_Stop    = 0,  //停止,速度置零/切空挡/拉手刹/车辆停止后跳转到空闲模式
        State_Drive   = 1,  //前进,前进档
        State_Reverse = 2,  //后退,后退档
        State_Idle    = 3,  //空闲, 停止控制指令发送，退出自动驾驶模式
        State_SwitchToDrive  = 4,  //任务切换为前进，
                                   //①若当前为R挡，速度置零->切N挡->切D档
                                   //②若当前为D档，不进行其他操作
                                   //跳转到前进模式
        State_SwitchToReverse= 5,  //任务切换为倒车
                                   //①若当前为R档，不进行其他操作
                                   //②若当前为D档，速度置零->切N档->切R档
                                   //跳转到后退模式
        State_ForceExternControl=6, //强制使用外部控制器状态
    };
    
    std::vector<std::string> StateName = {"State_Stop", "State_Drive", "State_Reverse",
    									  "State_Idle", "State_SwitchToDrive", "State_SwitchToReverse",
    									  "State_ForceExternControl"};
    
    void switchSystemState(int state);
    
private:
    float expect_speed_;
    bool  use_avoiding_;
    bool  use_car_following_;
    bool  is_offline_debug_;

    std::atomic<int> system_state_;
    int last_system_state_;
    std::atomic<bool> task_processing_;
    
    //工作线程条件变量
    bool has_new_task_;
    std::mutex work_cv_mutex_;
    std::condition_variable work_cv_;
    std::atomic<bool> task_running_;  //任务正在执行？
    
    //任务监听线程条件变量
    bool request_listen_;
    std::mutex listen_cv_mutex_;
    std::condition_variable listen_cv_;
    

	ros::Timer cmd1_timer_, cmd2_timer_;
    ros::Timer capture_extern_cmd_timer_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_vehicleState1_;
	ros::Subscriber sub_vehicleState2_;
	ros::Subscriber sub_vehicleState4_;

    ros::Publisher pub_cmd1_, pub_cmd2_;

    ros::Subscriber sub_new_goal_;   //订阅外部目标任务请求
    ros::Publisher  pub_new_goal_;   //发布目标请求到actionlib服务 
    
    std::mutex cmd1_mutex_, cmd2_mutex_;
	ant_msgs::ControlCmd1 controlCmd1_;
	ant_msgs::ControlCmd2 controlCmd2_;

    DoDriverlessTaskServer* as_;
    
    float avoid_offset_;
    PathTracking tracker_;
    controlCmd_t tracker_cmd_;

    bool use_car_follower_;
    CarFollowing car_follower_;
    controlCmd_t follower_cmd_;
 
    bool use_extern_controller_;
    ExternControl extern_controler_;
    controlCmd_t  extern_cmd_;
    std::mutex extern_cmd_mutex_;

    ReverseDrive reverse_controler_;
    controlCmd_t  reverse_cmd_;

    //AvoidObstacle avoider_;
    controlCmd_t avoid_cmd_;
};

