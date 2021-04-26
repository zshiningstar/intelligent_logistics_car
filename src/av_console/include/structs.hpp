#ifndef STRUCTS_HPP
#define STRUCTS_HPP
#include <vector>
#include <unordered_map>

namespace av_console {

typedef struct Sensor
{
    bool status;
    double last_update_time;
    Sensor()
    {
        status = false;
        last_update_time = 0;
    }
} sensor_t;

enum SensorId
{
    Sensor_Gps =   0,
    Sensor_Livox = 1,
    Sensor_Lidar = 2,
    Sensor_Camera1=3,
    //Sensor_Rtk    =4,
};

typedef struct _Pose
{
    double x,y,yaw;
}Pose;

typedef struct _PoseArray
{
    std::vector<Pose> poses;
}PoseArray;

//
typedef struct _RosNodes
{
    std::string name;
    std::string launch_cmd;

    //topic_name, topic_value
    std::unordered_map<std::string, std::string> topics;
}RosNodes;

//rosNode_name, rosNodes
typedef std::unordered_map<std::string, RosNodes> RosNodesArray;

}

#endif // STRUCTS_HPP
