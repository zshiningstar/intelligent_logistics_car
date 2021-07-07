
#include <ros/ros.h>
#include <memory>
#include "car_following.h"
#include "reverse_drive.h"
#include "extern_control/extern_control.h"
#include <pathplaning_msgs/expected_path.h>
#include <logistics_msgs/ControlCmd2.h>
#include <logistics_msgs/RealState.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <ant_msgs/State1.h>  //gear
#include <ant_msgs/State3.h>  //
#include <ant_msgs/State2.h>  //speed
#include <ant_msgs/State4.h>  //steerAngle
#include "auto_drive_base.h"
#include <condition_variable>
#include <driverless_common/SystemState.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <driverless_common/DoDriverlessTaskAction.h>   // Note: "Action" is appended
#include "driverless/path_tracking/pure_tracking/pure_tracking.hpp"


class AutoDrive : public AutoDriveBase
{
public:
    typedef actionlib::SimpleActionClient<driverless_common::DoDriverlessTaskAction> DoDriverlessTaskClient;
    typedef actionlib::SimpleActionServer<driverless_common::DoDriverlessTaskAction> DoDriverlessTaskServer;

    AutoDrive();
    ~AutoDrive();
    virtual bool init(ros::NodeHandle nh,ros::NodeHandle nh_private) override;
    void run(){};

private:
    bool loadVehicleParams();
    bool loadDriveTaskFile(const std::string& file, bool flip=false);
    bool setDriveTaskPathPoints(const driverless_common::DoDriverlessTaskGoalConstPtr& goal);
	void publishPathTrackingState();
    bool isGpsPointValid(const GpsPoint& point);
    void vehicleSpeed_callback(const logistics_msgs::RealState::ConstPtr& msg);

    void is_object_callback(const std_msgs::Float32::ConstPtr& msg);
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
	void sendCmd2_callback(const ros::TimerEvent&);
    void timer100ms_callback(const ros::TimerEvent&);
    void captureExernCmd_callback(const ros::TimerEvent&);
    void setSendControlCmdEnable(bool flag);
    void goal_callback(const pathplaning_msgs::expected_path::ConstPtr& msg);
    void executeDriverlessCallback(const driverless_common::DoDriverlessTaskGoalConstPtr& goal);
    bool handleNewGoal(const driverless_common::DoDriverlessTaskGoalConstPtr& goal);
   	float steerPidCtrl(float expectAngle);

    logistics_msgs::ControlCmd2 decisionMaking();
    

    bool isReverseGear();
    bool isDriveGear();
    bool isNeutralGear();

    void workingThread();
    void doWork();
    //void doReverseWork();

    void waitGearOk(int gear);
    void waitSpeedZero();
    void publishDriverlessState();

    //状态机,数字决不可改变,如需添加，向后排序
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
    
    std::vector<std::string> StateName = {"Stop", "Drive", "Reverse",
    									  "Idle", "SwitchToDrive", "SwitchToReverse",
    									  "ForceExternControl"};
    
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
    
    //avoid
    float avoid_min_obj_distance_;
	double last_valid_obj_time_;
    
	ros::Timer cmd2_timer_, timer_100ms_;
    ros::Timer capture_extern_cmd_timer_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_vehicleState1_;
	ros::Subscriber sub_vehicleState2_;
	ros::Subscriber sub_vehicleState4_;

    ros::Subscriber sub_is_object_;

    ros::Publisher pub_cmd2_;
    ros::Publisher pub_driverless_state_;

    ros::Subscriber sub_new_goal_;   //订阅外部目标任务请求
    ros::Publisher  pub_new_goal_;   //发布目标请求到actionlib服务 
    ros::Subscriber  sub_vehicle_speed_;
    
    std::mutex cmd2_mutex_;
	logistics_msgs::ControlCmd2 controlCmd2_;

    DoDriverlessTaskServer* as_;
    
    float avoid_offset_;
    PathTrackingBase*  tracker_;
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
    
    float steer_offset_;
    
    driverless_common::SystemState driverless_state_;
    float decisionMakingDuration_;
    
	VehicleState vehicle_state_;   //汽车状态
    SharedMutex  vehicle_state_shared_mutex_; // 汽车状态读写锁
};

