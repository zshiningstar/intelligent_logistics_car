#pragma once

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/PointCloud2.h>
#include <fstream>

#define RADIAL_DIVIDER_ANGLE 0.2

#define concentric_divider_distance_ 0.01 //0.1 meters default
#define reclass_distance_threshold_ 0.2

class PclTestCore
{

private:
  ros::Subscriber sub_point_cloud_;
  ros::Publisher pub_ground_, pub_no_ground_;

  float sensor_height_;
  float clip_height_;
  float x_min_,y_min_,z_min_,x_max_,y_max_,z_max_; //阈值分割参数
  bool  use_threshold_filter_;

  float local_max_slope_; //局部最大坡度，相邻两点之间
  float global_max_slope_;//全局最大坡度，相对于原点
  float tan_local_max_slope_, tan_global_max_slope_;
  float min_height_threshold_;

  struct PointXYZIRTColor
  {
    pcl::PointXYZI point;

    float radius; //cylindric coords on XY Plane
    float theta;  //angle deg on XY plane

    size_t radial_div;     //index of the radial divsion to which this point belongs to
    size_t concentric_div; //index of the concentric division to which this points belongs to

    size_t original_index; //index of this point in the source pointcloud
  };
  typedef std::vector<PointXYZIRTColor> PointCloudXYZIRTColor;

  size_t radial_dividers_num_;
  size_t concentric_dividers_num_;

  void point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud);

  void clip_above(double clip_height, const pcl::PointCloud<pcl::PointXYZI>::Ptr in, const pcl::PointCloud<pcl::PointXYZI>::Ptr out);

  void remove_close_pt(double min_distance, const pcl::PointCloud<pcl::PointXYZI>::Ptr in, const pcl::PointCloud<pcl::PointXYZI>::Ptr out);

  void clip_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,const pcl::PointCloud<pcl::PointXYZI>::Ptr out);

  void saveDividedPointsToFile(const std::vector<PointCloudXYZIRTColor> &out_radial_ordered_clouds);

  void XYZI_to_RTZColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                        std::vector<pcl::PointIndices> &out_radial_divided_indices,
                        std::vector<PointCloudXYZIRTColor> &out_radial_ordered_clouds);

  void classify_pc(std::vector<PointCloudXYZIRTColor> &in_radial_ordered_clouds,
                   pcl::PointIndices &out_ground_indices);
  void classify_pc2(std::vector<PointCloudXYZIRTColor> &in_radial_ordered_clouds,
                   pcl::PointIndices &out_ground_indices);

  void publish_cloud(const ros::Publisher &in_publisher,
                     const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr,
                     const std_msgs::Header &in_header);

public:
  PclTestCore(ros::NodeHandle &nh);
  ~PclTestCore();
  void Spin();
};
