#include "ground_filter.h"

PclTestCore::PclTestCore(ros::NodeHandle &nh)
{
    ros::NodeHandle nh_private("~");
    std::string raw_points_topic = 
    nh_private.param<std::string>("raw_points","/velodyne_points");
    nh_private.param<float>("sensor_height",sensor_height_,1.8);
    
    nh_private.param<bool> ("use_threshold_filter",use_threshold_filter_,true);
    nh_private.param<float>("clip_x_min",x_min_, 0.0);
    nh_private.param<float>("clip_x_max",x_max_, 50.0);
    nh_private.param<float>("clip_y_min",y_min_, -15.0);
    nh_private.param<float>("clip_y_max",y_max_, 15.0);
    nh_private.param<float>("clip_z_min",z_min_, -2.0);
    nh_private.param<float>("clip_z_max",z_max_, 0.2);

    nh_private.param<float>("min_height_threshold",min_height_threshold_, 0.05);
    nh_private.param<float>("local_max_slope",local_max_slope_, 5.0);
    nh_private.param<float>("global_max_slope",global_max_slope_, 7.0);
    tan_local_max_slope_ = tan(local_max_slope_*M_PI/180.0);
    tan_global_max_slope_ = tan(global_max_slope_*M_PI/180.0);

    sub_point_cloud_ = nh.subscribe(raw_points_topic, 1, &PclTestCore::point_cb, this);
    pub_ground_ = nh.advertise<sensor_msgs::PointCloud2>("/ground_points", 1);
    pub_no_ground_ = nh.advertise<sensor_msgs::PointCloud2>("/ground_filtered_points", 1);

    radial_dividers_num_ = ceil(360 / RADIAL_DIVIDER_ANGLE);

    ros::spin();
}

PclTestCore::~PclTestCore() {}

void PclTestCore::Spin()
{
}

//滤除过高点
void PclTestCore::clip_above(double clip_height, const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                             const pcl::PointCloud<pcl::PointXYZI>::Ptr out)
{
    pcl::ExtractIndices<pcl::PointXYZI> cliper;

    cliper.setInputCloud(in);
    pcl::PointIndices indices;
#pragma omp for
    for (size_t i = 0; i < in->points.size(); i++)
    {
        if (in->points[i].z > clip_height)
        {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true); //ture to remove the indices
    cliper.filter(*out);
}
//滤出过近点 
void PclTestCore::remove_close_pt(double min_distance, const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                                  const pcl::PointCloud<pcl::PointXYZI>::Ptr out)
{
    pcl::ExtractIndices<pcl::PointXYZI> cliper;

    cliper.setInputCloud(in);
    pcl::PointIndices indices;
#pragma omp for
    for (size_t i = 0; i < in->points.size(); i++)
    {
        double distance = sqrt(in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y);

        if (distance < min_distance)
        {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true); //ture to remove the indices
    cliper.filter(*out);
}

//阈值滤波，过远，过近，过高点云
void PclTestCore::clip_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,const pcl::PointCloud<pcl::PointXYZI>::Ptr out)
{
    pcl::ExtractIndices<pcl::PointXYZI> cliper;
    cliper.setInputCloud(in);
    pcl::PointIndices indices;
    indices.indices.reserve(in->points.size()/2);
#pragma omp for
    for (size_t i = 0; i < in->points.size(); i++)
    {
    	//double distance = sqrt(in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y);
    	//去除过高、过低、过远点云
        if (   in->points[i].x<x_min_ || in->points[i].x> x_max_
            || /*in->points[i].z<z_min_ ||*/ in->points[i].z>z_max_		
            || in->points[i].y<y_min_ || in->points[i].y>y_max_)
            indices.indices.push_back(i);
        //去除车身附近点云
        else if(in->points[i].x<2.0 && in->points[i].x>-1.2 && 
                in->points[i].y>-1.5 && in->points[i].y<1.5 )
             indices.indices.push_back(i);   
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true); //ture to remove the indices
    cliper.filter(*out);
}

void PclTestCore::saveDividedPointsToFile(const std::vector<PointCloudXYZIRTColor> &out_radial_ordered_clouds)
{
    static int seq = 0;
    for(int i=0; i<out_radial_ordered_clouds.size(); ++i)
    {
        std::ofstream out_file;
        std::string file_name = std::string("/home/wuconglei/data/") + std::to_string(seq)+"_"+std::to_string(i)+".txt";
        std::cout << file_name << std::endl;
        out_file.open(file_name);
        if(!out_file.is_open()) return;

        for(int j=0; j<out_radial_ordered_clouds[i].size(); ++j)
            out_file << out_radial_ordered_clouds[i][j].point.x << "\t"
                    << out_radial_ordered_clouds[i][j].point.y << "\t"
                    << out_radial_ordered_clouds[i][j].point.z << "\r\n";
        out_file.close();
    }
    seq++;
}

/*
 * @brief 将点云按水平旋转角分组，并每一组点云按距离排序
 * @param[in] in_cloud Input Point Cloud to be organized in radial segments
 * @param[out] out_radial_divided_indices Indices of the points in the original cloud for each radial segment
 * @param[out] out_radial_ordered_clouds Vector of Points Clouds, each element will contain the points ordered
 */
void PclTestCore::XYZI_to_RTZColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                                   std::vector<pcl::PointIndices> &out_radial_divided_indices,
                                   std::vector<PointCloudXYZIRTColor> &out_radial_ordered_clouds)
{
    //out_radial_divided_indices.clear();
    //out_radial_divided_indices.resize(radial_dividers_num_);
    out_radial_ordered_clouds.resize(radial_dividers_num_);

    //把点云的按旋转角分类/分组
    for (size_t i = 0; i < in_cloud->points.size(); i++)
    {
        if(in_cloud->points[i].z+sensor_height_ > 0.3) //基本不可能属于路面的点不参与分类
            continue;
        auto radius = (float)sqrt(
            in_cloud->points[i].x * in_cloud->points[i].x + in_cloud->points[i].y * in_cloud->points[i].y);
        auto theta = (float)atan2(in_cloud->points[i].y, in_cloud->points[i].x) * 180 / M_PI;
        if (theta < 0)
            theta += 360;
        //角度的微分
        size_t radial_div = round(theta / RADIAL_DIVIDER_ANGLE);
        //半径的微分
        size_t concentric_div = floor(fabs(radius / concentric_divider_distance_));

        PointXYZIRTColor new_point;
        new_point.point = in_cloud->points[i];
        new_point.radius = radius;
        new_point.theta = theta;
        new_point.radial_div = radial_div;
        new_point.concentric_div = concentric_div;
        new_point.original_index = i;

        //radial divisions更加角度的微分组织射线
        //out_radial_divided_indices[radial_div%radial_dividers_num_].indices.push_back(i);
        out_radial_ordered_clouds[radial_div%radial_dividers_num_].push_back(new_point);
    } //end for

#pragma omp for
    //将同一根射线上的点按照半径（距离）排序
    for (size_t i = 0; i < radial_dividers_num_; i++)
    {
        std::sort(out_radial_ordered_clouds[i].begin(), out_radial_ordered_clouds[i].end(),
                  [](const PointXYZIRTColor &a, const PointXYZIRTColor &b) { return a.radius < b.radius; });
    }
    //saveDividedPointsToFile(out_radial_ordered_clouds);
}

/*!
 * Classifies Points in the PointCoud as Ground and Not Ground
 * @param in_radial_ordered_clouds Vector of an Ordered PointsCloud ordered by radial distance from the origin
 * @param out_ground_indices Returns the indices of the points classified as ground in the original PointCloud
 */
void PclTestCore::classify_pc(std::vector<PointCloudXYZIRTColor> &in_radial_ordered_clouds,
                              pcl::PointIndices &out_ground_indices)
{
    out_ground_indices.indices.clear();

    for (size_t i = 0; i < in_radial_ordered_clouds.size(); i++) //sweep through each radial division 遍历每一根射线
    {
        float prev_radius = 0.f;
        float prev_height = -sensor_height_;
        bool prev_ground = false;
        bool current_ground = false;
        for (size_t j = 0; j < in_radial_ordered_clouds[i].size(); j++) //loop through each point in the radial div
        {
            float points_distance = in_radial_ordered_clouds[i][j].radius - prev_radius;//前后点半径差
            float height_threshold = tan_local_max_slope_ * points_distance;  //高度阈值
            float current_height = in_radial_ordered_clouds[i][j].point.z;
            float global_height_threshold = tan_global_max_slope_ * in_radial_ordered_clouds[i][j].radius;
            
            // 过近的点将导致高度阈值height_threshold过小，用使用自定义最小阈值代替
            //for points which are very close causing the height threshold to be tiny, set a minimum value
            if (points_distance > concentric_divider_distance_ && height_threshold < min_height_threshold_)
                height_threshold = min_height_threshold_;

            //check current point height against the LOCAL threshold (previous point)
            //当前点高度与上一点的高度差在允许范围内
            if (current_height <= (prev_height + height_threshold) && current_height >= (prev_height - height_threshold))
            {
                //Check again using general geometry (radius from origin) if previous points wasn't ground
                if (!prev_ground) //如果上一点不是地面点，根据全局坡度判定
                {
                    if (current_height <= (-sensor_height_ + global_height_threshold) && 
                        current_height >= (-sensor_height_ - global_height_threshold))
                        current_ground = true;
                    else
                        current_ground = false;
                } //如果上一点是地面点，当前点也是地面点
                else
                    current_ground = true;
            }
            else //当前点高度与上一点的高度差超出阈值
            {
                //check if previous point is too far from previous one, if so classify again
                //利用传感器安装高度和高度容差判定
                if (points_distance > reclass_distance_threshold_ &&
                    (current_height <= (-sensor_height_ + height_threshold) && current_height >= (-sensor_height_ - height_threshold)))
                    current_ground = true;
                else
                    current_ground = false;
            }

            if (current_ground)
            {
                out_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
                prev_ground = true;
            }
            else
                prev_ground = false;

            prev_radius = in_radial_ordered_clouds[i][j].radius;
            prev_height = in_radial_ordered_clouds[i][j].point.z;
        }
    }
}

//使用局部坡度和高度差阈值进行点云分类
void PclTestCore::classify_pc2(std::vector<PointCloudXYZIRTColor> &in_radial_ordered_clouds,
                              pcl::PointIndices &out_ground_indices)
{
    out_ground_indices.indices.clear();

    for (size_t i = 0; i < in_radial_ordered_clouds.size(); ++i) //sweep through each radial division 遍历每一根射线
    {
        if(in_radial_ordered_clouds[i].size()==0)
            continue;
        
        //若射线上的点太少，全部滤出
        if(in_radial_ordered_clouds[i].size()<3)
        {
            for(size_t j=0; j<in_radial_ordered_clouds[i].size(); ++j)
                out_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
            continue;
        }

        PointXYZIRTColor lastGoundPoint; //上一个地面点
        lastGoundPoint.point.x = lastGoundPoint.point.y = 0.0;
        lastGoundPoint.point.z = -sensor_height_;
        lastGoundPoint.radius = 0; 

        //PointXYZIRTColor lastNoGroundPoint; //上一个非地面点

        for(size_t j=0; j<in_radial_ordered_clouds[i].size()-1; ++j)
        {
            const pcl::PointXYZI& point  = in_radial_ordered_clouds[i][j].point;
            float dz = fabs(point.z - lastGoundPoint.point.z);
            if(dz < 1.0) //高度差较小，地面点
            {
                out_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
                lastGoundPoint = in_radial_ordered_clouds[i][j];
                continue;
            }
        }
    }
}

void PclTestCore::publish_cloud(const ros::Publisher &in_publisher,
                                const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr,
                                const std_msgs::Header &in_header)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = in_header;
    in_publisher.publish(cloud_msg);
}

void PclTestCore::point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*in_cloud_ptr, *raw_pc_ptr);

    if(use_threshold_filter_)
        clip_filter(raw_pc_ptr, filtered_pc_ptr);
    else
        filtered_pc_ptr = raw_pc_ptr;

    std::vector<pcl::PointIndices> radial_division_indices;
    std::vector<PointCloudXYZIRTColor> radial_ordered_clouds;

    //将点云按水平旋转角分组，并每一组点云按距离排序
    XYZI_to_RTZColor(filtered_pc_ptr, radial_division_indices, radial_ordered_clouds);

    pcl::PointIndices ground_indices;

    //点云分类，地面/非地面
    classify_pc(radial_ordered_clouds, ground_indices);

    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::ExtractIndices<pcl::PointXYZI> extract_ground;
    extract_ground.setInputCloud(filtered_pc_ptr);
    extract_ground.setIndices(boost::make_shared<pcl::PointIndices>(ground_indices));

    extract_ground.setNegative(false); //true removes the indices, false leaves only the indices
    extract_ground.filter(*ground_cloud_ptr);

    extract_ground.setNegative(true); //true removes the indices, false leaves only the indices
    extract_ground.filter(*no_ground_cloud_ptr);

    if(pub_ground_.getNumSubscribers())
        publish_cloud(pub_ground_, ground_cloud_ptr, in_cloud_ptr->header);
    if(pub_no_ground_.getNumSubscribers())
        publish_cloud(pub_no_ground_, no_ground_cloud_ptr, in_cloud_ptr->header);
}
