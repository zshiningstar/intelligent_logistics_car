#ifndef __DBSCAN_H_
#define __DBSCAN_H_

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <map>
#include <algorithm>
#include "structs.h"

#define NOISE -2
#define NOT_CLASSIFIED -1

class DBscanPoint : public Point
{
public:
    int ptsCnt = 0;
    int which = NOT_CLASSIFIED;
};

class DBSCAN 
{
public:
    DBSCAN(float eps, int minPts);
    void setInputPoints(const Points& pts);
    void run ();
    void dfs(int now, int c) ;
    bool checkNearPoints() ;
    bool isCoreObject(int idx) ;
    std::vector<std::vector<int> > getClusters() ;

private:
    int minPts;
    float eps;
    std::vector<DBscanPoint> points;
    int size;
    std::vector<std::vector<int> > adjPoints;
    std::vector<std::vector<int> > clusters;
    int clusterIdx;

};



#endif
