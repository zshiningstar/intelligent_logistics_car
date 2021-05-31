#include "euclidean_cluster_core.h"

EuClusterCore::EuClusterCore(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
{
    seg_distance_ = {15, 25, 35}; //按照距离分割区间划分
    cluster_distance_ = {0.4, 0.8, 1.4, 1.8}; //各区间的聚类半径
    private_nh.param<std::string>("obj_pub",obj_pub_,"/detection/lidar_objects");
    std::string raw_points_topic = private_nh.param<std::string>("in_points","/pandar_points");
	
    sub_point_cloud_     = nh.subscribe(raw_points_topic, 1, &EuClusterCore::point_cb, this);//point_cb
    pub_filtered_points_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 1);
    pub_bounding_boxs_   = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>(obj_pub_, 1);
    pub_polygon_         = nh.advertise<euclidean_cluster::ObjectPolygonArray>("/objtect_polygon",1);
    pub_object_marker_   = nh.advertise<visualization_msgs::MarkerArray>("/object_marker",1);
    pub_min_dis_obj_     = nh.advertise<std_msgs::Float32>(private_nh.param<std::string>("min_dis_obj","/min_dis_obj"),1);
    pub_offset_          = nh.advertise<std_msgs::Float32>(private_nh.param<std::string>("obj_offset","/obj_offset"),1);
	
    private_nh.param<double>("x_max",x_max_,1.0);
    private_nh.param<double>("x_min",x_min_,1.0);
    private_nh.param<double>("y_max",y_max_,1.0);
    private_nh.param<double>("y_min",y_min_,1.0);
    private_nh.param<double>("z_max",z_max_,1.0);
    private_nh.param<double>("z_min",z_min_,1.0);
    private_nh.param<bool>("pose_estimation",pose_estimation_, true);
    private_nh.param<bool>("use_downsample",use_downsample_, false);
    private_nh.param<bool>("use_threshold_filter",use_threshold_filter_, false);
    private_nh.param<int> ("min_cluster_size",min_cluster_size_,20);
    private_nh.param<int> ("max_cluster_size",max_cluster_size_,500);

    private_nh.param<bool>("is_min_dection_long",is_min_dection_long, true);

    regions_.resize(14);
    //regions_[0] = 4; regions_[1] = 5; regions_[2] = 4; regions_[3] = 5; regions_[4] = 4;
    //regions_[5] = 5; regions_[6] = 5; regions_[7] = 4; regions_[8] = 5; regions_[9] = 4;
    //regions_[10]= 5; regions_[11]= 5; regions_[12]= 4; regions_[13]= 5;
    regions_[0] = 4; regions_[1] = 3; regions_[2] = 3; regions_[3] = 3; regions_[4] = 3;
    regions_[5] = 3; regions_[6] = 3; regions_[7] = 3; regions_[8] = 3; regions_[9] = 3;
    regions_[10]= 3; regions_[11]= 3; regions_[12]= 3; regions_[13]= 3;

    marker_.type = visualization_msgs::Marker::LINE_STRIP;
    marker_.scale.x = 0.1;
    marker_.color.a = 1.0;
    marker_.color.r = 0.0;
    marker_.color.g = 1.0;
    marker_.color.b = 0.5;
    marker_.lifetime = ros::Duration(0.1);
}

EuClusterCore::~EuClusterCore() {}

//阈值滤波
void EuClusterCore::clip_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr in,const pcl::PointCloud<pcl::PointXYZ>::Ptr out)
{
    pcl::ExtractIndices<pcl::PointXYZ> cliper;
    cliper.setInputCloud(in);
    pcl::PointIndices indices;
#pragma omp for
    for (size_t i = 0; i < in->points.size(); i++)
    {
    	//double distance = sqrt(in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y);
    	//去除过高、过低、过远点云
        if (   in->points[i].x<x_min_  ||in->points[i].x> x_max_
            || in->points[i].z<z_min_ || in->points[i].z>z_max_		
            || in->points[i].y<y_min_ || in->points[i].y>y_max_)
        {
            indices.indices.push_back(i);
        }
        //去除车身附近点云
  /*      else if(in->points[i].x<2.0 && in->points[i].x>-1.2 && 
                in->points[i].y>-1.5 && in->points[i].y<1.5 )
             indices.indices.push_back(i);*/
             
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true); //ture to remove the indices
    cliper.filter(*out);
}


void EuClusterCore::publish_cloud(const ros::Publisher &in_publisher,const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr,const std_msgs::Header &in_header)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = in_header;
    in_publisher.publish(cloud_msg);
}


void EuClusterCore::voxel_grid_filer(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, double leaf_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(in);
    filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    filter.filter(*out);
}

jsk_recognition_msgs::BoundingBox EuClusterCore::getBbox(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                                                         const std::vector<int>& indice)
{
    jsk_recognition_msgs::BoundingBox bbox;
		
    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();

    for (auto pit = indice.begin(); pit != indice.end(); ++pit)
    {
        //找最大最小
        pcl::PointXYZ p;
        p.x = in_cloud_ptr->points[*pit].x;
        p.y = in_cloud_ptr->points[*pit].y;
        p.z = in_cloud_ptr->points[*pit].z;

        if (p.x < min_x)  min_x = p.x;
        if (p.y < min_y)  min_y = p.y;
        if (p.z < min_z)  min_z = p.z;
        if (p.x > max_x)  max_x = p.x;
        if (p.y > max_y)  max_y = p.y;
        if (p.z > max_z)  max_z = p.z;
    }
    double length = fabs(max_x-min_x);
    double width = fabs(max_y-min_y);
    double height = fabs(max_z-min_z);
    
    bbox.dimensions.x =  length;
    bbox.dimensions.y =  width;
    bbox.dimensions.z =  height;

    //calculate centroid
    //障碍物中心
    bbox.pose.position.x= min_x + length/2;
    bbox.pose.position.y= min_y + width /2;
    bbox.pose.position.z= min_z + height/2;
    
    offset.data = bbox.pose.position.y;
    pub_offset_.publish(offset);
/*---------------------------------计算障碍物和雷达之间的距离-------------------------------------*/            
//    std::cout << "前方障碍物x坐标:" << bbox.pose.position.x << "前方障碍物y坐标:" <<  bbox.pose.position.y << std::endl;
    double x = bbox.pose.position.x * bbox.pose.position.x;
    double y = bbox.pose.position.y * bbox.pose.position.y;
    double z = bbox.pose.position.z * bbox.pose.position.z;
    distance = sqrt(x+y+z);
	dis_list.push_back(distance);
    bbox.header = point_cloud_header_;
    bbox_array_.boxes.push_back(bbox);
    return bbox;
}

/*
 *@param in_origin_cloud_ptr 点云
 *@param in_cluster_indices  聚类簇点云索引
 */
void EuClusterCore::setcluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_origin_cloud_ptr,
								 const std::vector<int>& in_cluster_indices)
{
    //投影到二维
    std::vector<cv::Point2f> points_2d;
    points_2d.reserve(in_cluster_indices.size());
    
    for(auto pit = in_cluster_indices.begin(); pit != in_cluster_indices.end(); ++pit)
    {
        //每个点投到二维
        cv::Point2f pt;
        pt.x = in_origin_cloud_ptr->points[*pit].x;
        pt.y = in_origin_cloud_ptr->points[*pit].y;
        points_2d.push_back(pt);
    }
    
    std::vector<cv::Point2f> hull;
    cv::convexHull(points_2d, hull); //获取点集凸包

    if(pub_bounding_boxs_.getNumSubscribers())
    {
        jsk_recognition_msgs::BoundingBox bbox = getBbox(in_origin_cloud_ptr,in_cluster_indices);
        if (pose_estimation_)
        {
            cv::RotatedRect box = minAreaRect(hull); //查找凸包最小包络矩形
            
            bbox.pose.position.x = box.center.x;
            bbox.pose.position.y = box.center.y;
            bbox.dimensions.x = box.size.width;
            bbox.dimensions.y = box.size.height;
            
            

            // set bounding box direction
            tf::Quaternion quat = tf::createQuaternionFromRPY(0.0, 0.0, box.angle * M_PI / 180);
            tf::quaternionTFToMsg(quat, bbox.pose.orientation);
            
        }
    }

    if(pub_polygon_.getNumSubscribers())
    {
        polygon_.points.clear();
        polygon_.core.x = polygon_.core.y = 0.0;
        for (size_t i = 0; i < hull.size(); i++)
        {
            geometry_msgs::Point32 point;
            point.x = hull[i].x;
            point.y = hull[i].y;
            point.z = 0;
            polygon_.points.push_back(point);
            polygon_.core.x += point.x;
            polygon_.core.y += point.y;
        }
        polygon_.core.x /= polygon_.points.size();
        polygon_.core.y /= polygon_.points.size();

        polygon_array_.polygons.push_back(polygon_);
    }

    if(pub_object_marker_.getNumSubscribers())
    {
        marker_.points.clear();
        //std::cout << "hull size: " << hull.size() << std::endl;
        for (size_t i = 0; i < hull.size() + 1; i++)
        {
            geometry_msgs::Point point;
            point.x = hull[i%hull.size()].x;
            point.y = hull[i%hull.size()].y;
            point.z = 0;
            marker_.points.push_back(point);
        }
        marker_.header = point_cloud_header_;
        marker_.id = marker_array_.markers.size();
        marker_array_.markers.push_back(marker_);
    }		
}

//分割后聚类
void EuClusterCore::cluster_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc, double in_max_cluster_distance)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    //create 2d pc
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*in_pc, *cloud_2d);
    //make it flat
    for (size_t i = 0; i < cloud_2d->points.size(); i++)
    {
        cloud_2d->points[i].z = 0;
    }

    if (cloud_2d->points.size() > 0)
        tree->setInputCloud(cloud_2d);

    std::vector<pcl::PointIndices> clusters;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclid;
    euclid.setInputCloud(cloud_2d);
    euclid.setClusterTolerance(in_max_cluster_distance);
    euclid.setMinClusterSize(min_cluster_size_);
    euclid.setMaxClusterSize(max_cluster_size_);
    euclid.setSearchMethod(tree);
    euclid.extract(clusters);//聚类一次的结果 size个类 每一类存放索引

	for (auto cluster = clusters.begin(); cluster != clusters.end(); ++cluster) 
		setcluster(in_pc, cluster->indices);
    
}

void EuClusterCore::cluster_by_distance(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc)
{
    //cluster the pointcloud according to the distance of the points using different thresholds (not only one for the entire pc)
    //0 => 0-15m  d=0.5
    //1 => 15-30  d=1
    //2 => 30-45  d=1.6
    //3 => 45---- d=2.1
    
    //新建分割后的各区点云容器
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segment_pc_array(4);

    for (size_t i = 0; i < segment_pc_array.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        segment_pc_array[i] = tmp;
    }

    for (size_t i = 0; i < in_pc->points.size(); i++)
    {
        pcl::PointXYZ current_point;
        current_point.x = in_pc->points[i].x;
        current_point.y = in_pc->points[i].y;
        current_point.z = in_pc->points[i].z;

        //距离
        float origin_distance = sqrt(pow(current_point.x, 2) + pow(current_point.y, 2));

        if (origin_distance < seg_distance_[0])
            segment_pc_array[0]->points.push_back(current_point);
        else if (origin_distance < seg_distance_[1])
            segment_pc_array[1]->points.push_back(current_point);
        else if (origin_distance < seg_distance_[2])
            segment_pc_array[2]->points.push_back(current_point);
        else 
            segment_pc_array[3]->points.push_back(current_point);
    }

    for (size_t i = 0; i < segment_pc_array.size(); i++)
        cluster_segment(segment_pc_array[i], cluster_distance_[i]);//两个参数　一个分区的点云　一个不同区聚类半径
}

void EuClusterCore::cluster_by_distance2(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc)
{
    //新建分割后的各区点云容器
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segment_pc_array(regions_.size());

    for (size_t i = 0; i < segment_pc_array.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        segment_pc_array[i] = tmp;
    }

    for (size_t i = 0; i < in_pc->points.size(); i++)
    {
        pcl::PointXYZ current_point;
        current_point.x = in_pc->points[i].x;
        current_point.y = in_pc->points[i].y;
        current_point.z = in_pc->points[i].z;

        //距离
        float radius = pow(current_point.x, 2) + pow(current_point.y, 2);

        float range = 0.0;
        for(int j = 0; j < regions_.size(); j++) 
        {
            if(radius > range * range && radius <= (range+regions_[j]) * (range+regions_[j])) 
            {
                segment_pc_array[j]->push_back(current_point);
                break;
            }
            range += regions_[j];
        }
    }

    for (size_t i = 0; i < segment_pc_array.size(); i++)
    {
        float cluster_distance = (i+1)*0.1;
        cluster_segment(segment_pc_array[i], cluster_distance);//点云、聚类半径
    }
       
}

//收到点云后的处理
void EuClusterCore::point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
	bbox_array_.boxes.clear();
	marker_array_.markers.clear();
	polygon_array_.polygons.clear();

	point_cloud_header_ = in_cloud_ptr->header;
	
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>); //降采样

    pcl::fromROSMsg(*in_cloud_ptr, *filtered_pc_ptr);
    
    //阈值滤波
    if(use_threshold_filter_)
	    clip_filter(filtered_pc_ptr, filtered_pc_ptr);
    
    //降采样
    if(use_downsample_)
        // down sampling the point cloud before cluster
        voxel_grid_filer(filtered_pc_ptr, filtered_pc_ptr, LEAF_SIZE);

    if(pub_filtered_points_.getNumSubscribers())
	    publish_cloud(pub_filtered_points_,filtered_pc_ptr,point_cloud_header_);

    cluster_by_distance2(filtered_pc_ptr); //按照距离聚类

    if(marker_array_.markers.size())
        pub_object_marker_.publish(marker_array_);

    if(polygon_array_.polygons.size())
    {
        polygon_array_.header = point_cloud_header_;
        pub_polygon_.publish(polygon_array_);
    }

    

    if(bbox_array_.boxes.size())
    {
        bbox_array_.header = point_cloud_header_;
        pub_bounding_boxs_.publish(bbox_array_);
        //if(is_min_dection_long)
        {
		    min_dis_object_.data =  *min_element(dis_list.begin(), dis_list.end());
		    dis_list.clear();
        }
    }
    else
        min_dis_object_.data = 2000.0;
    pub_min_dis_obj_.publish(min_dis_object_);
}
