#include <ros/ros.h>
#include "dbscan/dbscan.h"
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>


class Cluster
{
public:

    void laserCallback(const sensor_msgs::LaserScanConstPtr& laserScan)
    {
        Points points;

        for (int i = 0; i < laserScan->ranges.size(); ++i) 
        {
                float angle = laserScan->angle_increment * i + laserScan->angle_min;
                float distance = laserScan->ranges[i];
                float x = distance*cos(angle);
                float y = distance*sin(angle);
                
                points.points.emplace_back(x,y);
        }

        /*use dbscan to cluster object*/
        dbscan_->setInputPoints(points);
        dbscan_->run();
        std::vector<std::vector<int> > clusters = dbscan_->getClusters();
        publishClusters(points, clusters, laserScan->header.frame_id);

    }

    Cluster()
    {
        ros::NodeHandle nh, nh_private("~");
        scan_sub_ = nh.subscribe("/scan", 10, &Cluster::laserCallback, this);
        pub_cluster_ = nh.advertise<visualization_msgs::MarkerArray>("/clusters", 1);

        float eps  = nh_private.param<float>("dbscan_eps", 1.0);  //邻域距离
        int minPts = nh_private.param<int>("dbscan_minPts", 5);   //核心点的最小邻居个数
        dbscan_ = new DBSCAN(eps, minPts);
    }

    void publishClusters(const Points& points, const std::vector<std::vector<int> >& clusters, const std::string& frame)
    {
        visualization_msgs::MarkerArray markerArray;
        visualization_msgs::Marker marker;

        marker.header.frame_id = frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "ns";
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.1;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration(0.1);

        for(const std::vector<int>& cluster : clusters)
        {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            for(int index : cluster)
            {
                const Point& p = points[index];
                geometry_msgs::Point point;
                point.x = p.x;
                point.y = p.y;
                point.z = 0.0;

                marker.points.push_back(point);
            }
            markerArray.markers.push_back(marker);
        }
        pub_cluster_.publish(markerArray);
    }

    

private:
    ros::Subscriber scan_sub_;
    ros::Publisher  pub_cluster_;

    DBSCAN *dbscan_;


};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cluster_node");
    Cluster app;
    ros::spin();
    return 0;

}
