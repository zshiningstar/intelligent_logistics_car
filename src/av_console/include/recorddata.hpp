#ifndef RECORDDATA_H
#define RECORDDATA_H
#include <QString>
#include <QObject>
#include <QStringListModel>

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ant_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <gps_msgs/Inspvax.h>
#include <nav_msgs/Odometry.h>
#endif

#include <cstdio>
#include <fstream>
#include <mutex>
#include <memory>
#include <sstream>
#include <utils.hpp>

/*@brief 数据记录类
 *@note  记录各种传感器数据
 * 1.车辆状态：车速，前轮转角
 * 2.gps定位状态： 经纬度,UTM，姿态角(RPY)
 * 3.imu数据： 三轴加速度，三轴角速度
 */

class Subscriber
{
public:
    Subscriber(const std::string& n, ros::Subscriber* s)
    {
        name = n;
        obj = s;
    }
    std::string name;
    ros::Subscriber* obj;
};

class RecordData  : public QObject
{
    Q_OBJECT
public:
    RecordData();
    ~RecordData();

    void setDataFile(const QString& file);
    void setRecordFrequency(int hz);
    void setLaunchSensorWaitTime(float t);
    void setRecordVehicleState(const std::string &topic, bool steerAngle, bool speed);
    void setRecordGps(const std::string &topic, bool yaw, bool wgs84);
    void setRecordUtm(const std::string &topic, bool yaw, bool utm);
    void setRecordStamp(bool stamp);
    void setRecordImu(const std::string &topic, bool angular_v, bool accel);
    void setDisable(bool flag){m_disable = flag;}
    bool start();
    void stop();

    QStringListModel* loggingModel() { return &logging_model; }
    void clearLog();

private:
    void vehicleStateCallback(const ant_msgs::State::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void inspvaxCallback(const gps_msgs::Inspvax::ConstPtr& msg);
    void utmOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void recordTimerUpdate(const ros::TimerEvent  &event);
    void addLog(const QString& log);

Q_SIGNALS:

    void loggingUpdated();

private:
    QString m_fileName;
    int m_record_frequency;

    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    ros::Subscriber m_sub_gps, m_sub_utm, m_sub_imu, m_sub_vehicle_state;
    ros::Timer m_recorder_timer, m_wait_timer;

    std::mutex m_vehicle_state_mutex, m_gps_mutex, m_utm_mutex, m_imu_mutex;

    double m_speed, m_steer_angle; //vehicle state
    double m_lon, m_lat, m_yaw_NED, m_yaw_ENU , m_x, m_y, m_z; //gps
    double m_omega_x, m_omega_y, m_omega_z, m_accel_x, m_accel_y, m_accel_z; //imu

    std::vector<Subscriber> m_subscribers;
    std::vector<double *> m_datas;
    std::string m_dataTitle;  //数据标题
    std::vector<std::string> m_data_formats; //
    bool m_recordTime;

    FILE * m_fp;
    QStringListModel logging_model;
    int m_dataLineCnt;
    int m_validPublisherCnt;
    bool m_waitNodeLaunched; //是否需要等待节点启动？
    double m_launchSensorTime; //开始启动节点的时间
    float m_launchSensorWaitTime; //启动传感器的等待时间

    bool m_disable; //是否禁用功能

};

#endif // RECORDDATA_H
