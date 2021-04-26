#ifndef RECORDPATH_H
#define RECORDPATH_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <QObject>
#include <QThread>
#include <QDebug>
#include <QDir>
#include <nav_msgs/Odometry.h>
#include <unistd.h>
#include <cmath>
#include <QTimer>

#include <vector>
#include <QStringListModel>
#include <fstream>
#include <sstream>

struct gpsPoint
{
  double x;
  double y;
  //double z;
  double yaw;
  gpsPoint()
  {
    x = y = yaw = 0.0;
  }
};

class RecordPath : public QObject
{
 Q_OBJECT
private:
    float dis2Points(gpsPoint& p1,gpsPoint&p2,bool isSqrt);
    void gps_callback(const nav_msgs::Odometry::ConstPtr& msg);

    std::string file_path_;
    std::string file_name_;

    gpsPoint last_point_;
    float sample_distance_;
    ros::Subscriber sub_gps_;
    std::string odom_topic_;

    std::vector<gpsPoint> path_points_;
    QStringListModel logging_model;

    QTimer  wait_gps_topic_timer_;
    size_t  row_num_;
    gpsPoint current_point_;

Q_SIGNALS:
    void loggingUpdated();

public Q_SLOTS:
    void waitGpsTopicTimeout();

public:
    RecordPath();
    virtual ~RecordPath();
    bool start();
    void stop();
    void RecordPathToFile();
    std::string odomTopic(){return odom_topic_;}
    QStringListModel* loggingModel() { return &logging_model; }
    bool savePathPoints(const std::string& file_name);
    bool generatePathInfoFile(const std::string& file_name);
    size_t pathPointsSize(){return path_points_.size();}
    void log( const std::string &level, const std::string &msg);
};


#endif
