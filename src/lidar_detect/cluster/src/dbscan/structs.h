#ifndef STRUCTS_H_
#define STRUCTS_H_

#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

class Pose
{
public:
    float x, y;
    float yaw;
    Pose(){}
    Pose(float _x, float _y, float _yaw)
    {
        x = _x;
        y = _y;
        yaw = _yaw;
    }
    float disTo(const Pose& p) const 
    {
        float dx = p.x - x;
        float dy = p.y - y;
        return sqrt(dx*dx+dy*dy);
    }
};

class Point
{
public:
    float x, y;
    Point(){}

    Point(float _x, float _y)
    {
        x = _x;
        y = _y;
    }
    float disTo(const Point& p) const 
    {
        float dx = p.x - x;
        float dy = p.y - y;
        return sqrt(dx*dx+dy*dy);
    }

    Point toLocal(const Pose& local_pose) const
    {
        Point point;
        float sinYaw = sin(local_pose.yaw);
        float cosYaw = cos(local_pose.yaw);
        point.x  = (x-local_pose.x)*cosYaw + (y-local_pose.y)*sinYaw;
	    point.y = -(x-local_pose.x)*sinYaw + (y-local_pose.y)*cosYaw;
        return point;
    }

    Point toGlobal(const Pose& local_pose) const
    {
        Point point;
        float sinYaw = sin(local_pose.yaw);
        float cosYaw = cos(local_pose.yaw);
        point.x  = x*cosYaw - y*sinYaw + local_pose.x;
	    point.y  = x*sinYaw + y*cosYaw + local_pose.y;
        return point;
    }
};

class Points
{ 
public:
    std::vector<Point> points;

    size_t size() const
    {
        return points.size();
    }

    Point& operator[](size_t i) 
    {
        return points[i];
    }
    const Point& operator[](size_t i) const  
    {
        return points[i];
    }

    sensor_msgs::PointCloud2::Ptr toPointCloud() const 
    {
         pcl::PointCloud<pcl::PointXYZ> cloud;
         cloud.reserve(points.size());

         for(const Point& p:points)
         {
            pcl::PointXYZ point;
            point.x = p.x;
            point.y = p.y;
            point.z = 0.0;
            cloud.push_back(point);
         }

        sensor_msgs::PointCloud2::Ptr out(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(cloud, *out);
        return out;
    }

    Point getCenter() const
    {
        if(points.size() == 0)
            return Point(0,0);
        float sum_x = 0;
        float sum_y = 0;

        for(const Point& point:points)
        {
            sum_x += point.x;
            sum_y += point.y;
        }

        return Point(sum_x/points.size(), sum_y/points.size());
    }
};



#endif