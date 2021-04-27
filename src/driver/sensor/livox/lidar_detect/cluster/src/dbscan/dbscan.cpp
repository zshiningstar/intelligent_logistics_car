
#include "dbscan.h"

/*@brief DBSCAN clustering algorithm
 *@param _n  

*/

DBSCAN::DBSCAN(float _eps, int _minPts):
    eps(_eps),
    minPts(_minPts),
    size(0)
{
}

void DBSCAN::setInputPoints(const Points& input)
{
    clusterIdx = 0;

    clusters.clear();
    adjPoints.clear();

    points.clear();
    points.reserve(input.size());
    for(const Point& point : input.points)
    {
        DBscanPoint db_point;
        db_point.x = point.x;
        db_point.y = point.y;
        points.push_back(db_point);
    }
    
    size = points.size();
    adjPoints.resize(size);
}

void DBSCAN::run () 
{
    if(!checkNearPoints())
    {
        //ROS_ERROR("No points to clustering!");
        return ;
    }
    
    for(int i=0; i<size; i++) 
    {
        if(points[i].which != NOT_CLASSIFIED) 
            continue;
        
        if(isCoreObject(i)) 
            dfs(i, clusterIdx++);
        else //not core point, marked as noise temporarily.
            points[i].which = NOISE;
    }
    
    clusters.resize(clusterIdx+1);

    for(int i=0; i<size; i++)
    {
        if(points[i].which != NOISE)
            clusters[points[i].which].push_back(i);
    }

    if(clusters.back().size() == 0)  //delte the empty cluster
        clusters.resize(clusters.size()-1);
}

void DBSCAN::dfs(int now, int c) 
{
    points[now].which = c;
    if(!isCoreObject(now)) return;
    
    for(auto& next : adjPoints[now]) 
    {
        if(points[next].which != NOT_CLASSIFIED) continue;
        dfs(next, c);
    }
}

bool DBSCAN::checkNearPoints() 
{
    if(points.size() == 0)
        return false;

    for(int i=0; i<points.size(); i++) 
    {
        for(int j=0; j<points.size(); j++) 
        {
            if(i==j) continue;
            float dis = points[i].disTo(points[j]);
            //std::cout << dis << "\t";
            if(dis <= eps) 
            {
                points[i].ptsCnt++;
                adjPoints[i].push_back(j);
            }
        }
        //std::cout << std::endl;
    }
    return true;
}

bool DBSCAN::isCoreObject(int idx) 
{
    return points[idx].ptsCnt >= minPts;
}

std::vector<std::vector<int> > DBSCAN::getClusters() 
{
    return clusters;
}
