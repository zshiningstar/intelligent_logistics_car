#ifndef UTILS_HPP
#define UTILS_HPP
#include <iostream>
#include <fstream>
#include <cstdio>
#include "structs.hpp"
#include "globalvariables.hpp"
#include <sstream>

namespace av_console {

//启动ros nodes节点
static bool launchRosNodes(const std::string& nodes_name)
{
    std::string launch_cmd = g_rosNodesArray[nodes_name].launch_cmd;
    if(launch_cmd.empty())
        return false;

    std::string cmd = /*std::string("gnome-terminal -x ") +*/ launch_cmd;
    std::cout <<"run: [" << cmd << "]" << std::endl;
    system(cmd.c_str());
    //std::cout <<"run: [" << cmd << "]" << std::endl;
}

static bool loadPathPoints(std::string file_path,PoseArray& path)
{
    std::ifstream in_file(file_path.c_str());
    if(!in_file.is_open())
    {
        printf("open %s failed",file_path.c_str());
        return false;
    }
    Pose pose;
    std::string line;
    path.poses.clear();
    while(in_file.good())
    {
        getline(in_file,line);
        std::stringstream ss(line);
        if(line.length() == 0)
            break;
        ss >> pose.x >> pose.y >> pose.yaw;
        std::cout << pose.x << "\t" <<  pose.y << "\t" << pose.yaw << std::endl;
        path.poses.push_back(pose);
    }
    return true;
}

}
#endif // UTILS_HPP
