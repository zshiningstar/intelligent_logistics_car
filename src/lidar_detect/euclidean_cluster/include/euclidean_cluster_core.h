#pragma once

#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include <iostream>
#include <algorithm>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/common/common.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include <std_msgs/Header.h>
#include <math.h>
#include <vector>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <euclidean_cluster/ObjectPolygon.h>
#include <euclidean_cluster/ObjectPolygonArray.h>

#include <sensor_msgs/PointCloud2.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#define LEAF_SIZE 0.2 //定义降采样的leaf size，聚类费时，为减少计算量，通常先进行降采样

class EuClusterCore
{

	private:
		//全局障碍物array 单个box作为局部变量
		jsk_recognition_msgs::BoundingBoxArray bbox_array_;

		visualization_msgs::MarkerArray marker_array_;
		visualization_msgs::Marker marker_;
		
		euclidean_cluster::ObjectPolygonArray polygon_array_;
		euclidean_cluster::ObjectPolygon polygon_;

		std::string obj_pub_;
		double y_max_,y_min_,x_min_,x_max_,z_min_,z_max_;
		
		
		
/*--------------------------------------------------------------*/
        double min_dection_long;
        double distance;
		bool is_min_dection_long;
		std_msgs::Float32 min_dis_object_;
		std_msgs::Float32 offset;
		
		ros::Publisher pub_min_dis_obj_;
		ros::Publisher pub_offset_;
		
		std::vector<double> dis_list;
/*--------------------------------------------------------------*/



		bool pose_estimation_;
		bool use_threshold_filter_;
		bool use_downsample_;
		int min_cluster_size_,max_cluster_size_;

		ros::Subscriber sub_point_cloud_;
		ros::Publisher pub_filtered_points_;
		ros::Publisher pub_bounding_boxs_;
		ros::Publisher pub_polygon_;
		ros::Publisher pub_object_marker_;
		
		

		std::vector<double> seg_distance_, cluster_distance_; //分区方式1：区域分割以及各区域聚类半径
		std::vector<int> regions_;//分区方式2

		std_msgs::Header point_cloud_header_;

		void clip_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr in,const pcl::PointCloud<pcl::PointXYZ>::Ptr out);
		void voxel_grid_filer(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, double leaf_size);

		jsk_recognition_msgs::BoundingBox getBbox(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,const std::vector<int>& indice);
		void cluster_by_distance(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc);
		void cluster_by_distance2(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc);

		void cluster_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc,
				           double in_max_cluster_distance);

		void setcluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_origin_cloud_ptr,
								 const std::vector<int>& in_cluster_indices);

		void point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr);

		void publish_cloud(const ros::Publisher &in_publisher,const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr,const std_msgs::Header &in_header);


	public:
		EuClusterCore(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
		~EuClusterCore();
};
