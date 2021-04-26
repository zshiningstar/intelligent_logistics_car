#include "recorddata.hpp"

RecordData::RecordData():
    nh_private("~"),
    m_fp(NULL),
    m_disable(true)
{

}

RecordData::~RecordData()
{

}

void RecordData::setDataFile(const QString& file)
{
    m_fileName = file;
}

void RecordData::setRecordFrequency(int hz)
{
    m_record_frequency = hz;
}

void RecordData::setRecordVehicleState(const std::string& topic, bool steerAngle, bool speed)
{
    if(steerAngle || speed)
        m_sub_vehicle_state = nh.subscribe<ant_msgs::State>(topic, 1, &RecordData::vehicleStateCallback, this);
    else
        return;

    m_subscribers.emplace_back("m_sub_vehicle_state", &m_sub_vehicle_state);

    if(steerAngle)
    {
        m_datas.push_back(&m_steer_angle);
        m_dataTitle += "steer_angle\t";
        m_data_formats.push_back("%.2f\t");
    }
    if(speed)
    {
        m_datas.push_back(&m_speed);
        m_dataTitle += "speed(m/s)\t";
        m_data_formats.push_back("%.2f\t");
    }

}

void RecordData::setRecordGps(const std::string &topic, bool yaw, bool wgs84)
{
    if(yaw || wgs84)
        m_sub_gps = nh.subscribe<gps_msgs::Inspvax>(topic, 1, &RecordData::inspvaxCallback, this);
    else
        return;
    m_subscribers.emplace_back("m_sub_gps", &m_sub_gps);
    if(yaw)
    {
        m_datas.push_back(&m_yaw_NED);
        m_dataTitle += "yaw_NED(deg)\t";
        m_data_formats.push_back("%.2f\t");
    }

    if(wgs84)
    {
        m_datas.push_back(&m_lon);
        m_datas.push_back(&m_lat);

        m_dataTitle += "longitude\tlatitude\t";
        m_data_formats.push_back("%.7f\t");
        m_data_formats.push_back("%.7f\t");
    }
}

void RecordData::setRecordUtm(const std::string &topic, bool yaw, bool utm)
{
    if(utm || yaw)
        m_sub_utm = nh.subscribe<nav_msgs::Odometry>(topic, 1, &RecordData::utmOdomCallback, this);
    else
        return;
    m_subscribers.emplace_back("m_sub_utm", &m_sub_utm);

    if(yaw)
    {
        m_datas.push_back(&m_yaw_ENU);
        m_dataTitle += "yaw_ENU(deg)\t";
        m_data_formats.push_back("%.2f\t");
    }

    if(utm)
    {
        m_datas.push_back(&m_x);
        m_datas.push_back(&m_y);
        m_datas.push_back(&m_z);

        m_dataTitle += "utm_x\tutm_y\tutm_z\t";
        m_data_formats.push_back("%.3f\t");
        m_data_formats.push_back("%.3f\t");
        m_data_formats.push_back("%.3f\t");
    }
}

void RecordData::setRecordImu(const std::string &topic, bool angular_v, bool accel)
{
    if(angular_v || accel)
        m_sub_imu = nh.subscribe(topic, 1, &RecordData::imuCallback, this);
    else
        return;

    m_subscribers.emplace_back("m_sub_imu", &m_sub_imu);
    if(angular_v)
    {
        m_datas.push_back(&m_omega_x);
        m_datas.push_back(&m_omega_y);
        m_datas.push_back(&m_omega_z);

        m_dataTitle += "omega_x\tomega_y\tomega_z\t";
        m_data_formats.push_back("%.3f\t");
        m_data_formats.push_back("%.3f\t");
        m_data_formats.push_back("%.3f\t");
    }
    if(accel)
    {
        m_datas.push_back(&m_accel_x);
        m_datas.push_back(&m_accel_y);
        m_datas.push_back(&m_accel_z);

        m_dataTitle += "accel_x\taccel_y\taccel_z\t";
        m_data_formats.push_back("%.3f\t");
        m_data_formats.push_back("%.3f\t");
        m_data_formats.push_back("%.3f\t");
    }
}

void RecordData::setRecordStamp(bool stamp)
{
    m_recordTime = stamp;
    if(m_recordTime)
    {
        m_dataTitle = "time\t" + m_dataTitle;
    }
}
// 启动传感器的等待时间
void RecordData::setLaunchSensorWaitTime(float t)
{
    m_launchSensorWaitTime = t;
}

bool RecordData::start()
{
    if(m_disable)
        return false;

    m_dataLineCnt = 0;
    m_validPublisherCnt = 0;
    m_waitNodeLaunched = false;

    if(m_datas.size()==0)
    {
        Q_EMIT addLog("No items were seleted!");
        return false;
    }

    m_fp = fopen(m_fileName.toStdString().c_str(), "w");
    if(m_fp == NULL)
    {
        Q_EMIT addLog(QString("Open ") + m_fileName + " failed!");
        return false;
    }
    fprintf(m_fp, "%s\r\n", m_dataTitle.c_str());

    ros::Duration(1.0).sleep();
    m_recorder_timer = nh.createTimer(ros::Duration(1.0/m_record_frequency), &RecordData::recordTimerUpdate, this);

    return true;
}

void RecordData::stop()
{
    m_recorder_timer.stop();
    m_sub_gps.shutdown();
    m_sub_imu.shutdown();
    m_sub_utm.shutdown();
    m_sub_vehicle_state.shutdown();

    if(m_fp)
    {
        fflush(m_fp);
        fclose(m_fp);
        m_fp = NULL;
    }

    m_datas.clear();
    m_dataTitle.clear();
    m_data_formats.clear();
    m_subscribers.clear();
}

void RecordData::vehicleStateCallback(const ant_msgs::State::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lck(m_vehicle_state_mutex);
    m_steer_angle = msg->roadwheelAngle;
    m_speed = msg->vehicle_speed;
}

void RecordData::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lck(m_imu_mutex);
    m_omega_x = msg->angular_velocity.x;
    m_omega_y = msg->angular_velocity.y;
    m_omega_z = msg->angular_velocity.z;

    m_accel_x = msg->linear_acceleration.x;
    m_accel_y = msg->linear_acceleration.y;
    m_accel_z = msg->linear_acceleration.z;

}

void RecordData::inspvaxCallback(const gps_msgs::Inspvax::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lck(m_gps_mutex);
    m_lon = msg->longitude;
    m_lat = msg->latitude;
    m_yaw_NED = msg->azimuth;
}

void RecordData::utmOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lck(m_utm_mutex);
    auto& position = msg->pose.pose.position;

    m_x = position.x;
    m_y = position.y;
    m_z = position.z;

    m_yaw_ENU = msg->pose.covariance[0]*180.0/M_PI; //yaw
}

void RecordData::recordTimerUpdate(const ros::TimerEvent &event)
{
    //有效的发布者与期望的订阅者个数相同，可以开始记录，否则需要启动发布者节点
    if(m_validPublisherCnt == m_subscribers.size())
    {
        m_waitNodeLaunched = false;

        if(m_recordTime)
            fprintf(m_fp, "%.3f\t", ros::Time::now().toSec());

        std::lock_guard<std::mutex> lck1(m_utm_mutex);
        std::lock_guard<std::mutex> lck2(m_gps_mutex);
        std::lock_guard<std::mutex> lck3(m_imu_mutex);
        std::lock_guard<std::mutex> lck4(m_vehicle_state_mutex);

        for(int i=0; i<m_datas.size(); ++i)
            fprintf(m_fp, m_data_formats[i].c_str(), *m_datas[i]);
        fprintf(m_fp, "\r\n");

        addLog(QString::number(++m_dataLineCnt));
    }
    else //存在期望的发布者不存在
    {
        if(m_waitNodeLaunched && (ros::Time::now().toSec() - m_launchSensorTime < m_launchSensorWaitTime))
            return;

        m_validPublisherCnt = 0;
        for(const Subscriber& sub : m_subscribers )
        {
            if(sub.obj->getNumPublishers() > 0)
            {
                ++m_validPublisherCnt;
                continue;
            }

            //m_sub_gps, m_sub_utm, m_sub_imu, m_sub_vehicle_state;
            if(sub.name == "m_sub_gps")
            {
                av_console::launchRosNodes("gps");
                addLog("launching gps node...");
            }
            //if(sub.name == "m_sub_utm")
            //    av_console::launchUtmNode()
            if(sub.name == "m_sub_imu")
            {
                av_console::launchRosNodes("imu");
                addLog("launching imu node...");
            }
            if(sub.name == "m_sub_vehicle_state")
            {
                av_console::launchRosNodes("base_control");
                addLog("launching vehicle state node...");
            }

            m_waitNodeLaunched = true;
            m_launchSensorTime = ros::Time::now().toSec();
        }

    }
}

void RecordData::addLog(const QString& log)
{
  if(logging_model.rowCount() >400)
  {
      logging_model.removeRows(0,200);
  }

  logging_model.insertRows(logging_model.rowCount(),1);
  std::stringstream logging_model_msg;

  logging_model_msg << std::fixed << std::setprecision(3)
                    << "[" << ros::Time::now() << "]: " << log.toStdString();

  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void RecordData::clearLog()
{
    logging_model.removeRows(0,logging_model.rowCount());
    Q_EMIT loggingUpdated();
}


