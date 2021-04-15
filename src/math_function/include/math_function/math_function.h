#ifndef MATH_H_
#define MATH_H_

#include<cstring>
#include<cmath>
#include<assert.h>
#include<string>
#include<vector>
#include<cstdio>
#include<ros/ros.h>
#include<limits.h>
#include<exception>
#include<fstream>

#define MAX_ROAD_WHEEL_ANGLE 25.0
#define IS_POLAR_COORDINATE_GPS 0

#define WHEEL_DISTANCE  1.2
#define AXIS_DISTANCE 0.88  // 轴距
#define MAX_SPEED 30.0

const float g_vehicle_width = 1 ;// m
const float g_vehicle_length = 1.5; 
const float g_max_deceleration = 5.0; // m/s/s
static const float max_side_acceleration = 1.5; // m/s/s

enum
{
	TrafficSign_None = 0,
	TrafficSign_TrafficLight =1,
	TrafficSign_Avoid = 2,
	TrafficSign_TurnLeft = 3,
	TrafficSign_CarFollow = 4,//?
	TrafficSign_LaneNarrow = 5,
	TrafficSign_IllegalPedestrian = 6,
	TrafficSign_NoTrafficLight = 7,
	TrafficSign_PickUp = 8,
	TrafficSign_Ambulance = 9,//?
	TrafficSign_Railway = 10,
	TrafficSign_TempStop = 11,//?
	TrafficSign_UTurn = 12,
	TrafficSign_School = 13,
	TrafficSign_AvoidStartingCar = 14,
	TrafficSign_OffDutyPerson = 15,
	TrafficSign_Bridge = 16,
	TrafficSign_AccidentArea = 17,
	TrafficSign_JamArea = 18,
	TrafficSign_BusStop = 19,
	TrafficSign_NonVehicle = 20,
	TrafficSign_StopArea = 21, //?
	
	TrafficSign_CloseTurnLight = 22,
	TrafficSign_TurnRight = 23,
	TrafficSign_Stop = 24,
};

typedef struct
{
	double longitude;
	double latitude;
	double yaw;
	double x;
	double y;
	float curvature;
	
	float maxOffset_left;
	float maxOffset_right;
	uint8_t traffic_sign;
	uint8_t other_info;
	
	void show()
	{
		std::cout << std::fixed << x << "\t" << y << "\t" << yaw*180.0/M_PI << std::endl;
	}
	
}gpsMsg_t;

extern const float g_vehicle_width;
extern const float g_vehicle_length;
extern const float g_max_deceleration;

inline float saturationEqual(float value,float limit)
{
	//ROS_INFO("value:%f\t limit:%f",value,limit);
	assert(limit>=0);
	if(value>limit)
		value = limit;
	else if(value < -limit)
		value = -limit;
	return value;
}

inline float generateRoadwheelAngleByRadius(const float& radius)
{
	assert(radius!=0);
	//return asin(AXIS_DISTANCE /radius)*180/M_PI;  //the angle larger
	return atan(AXIS_DISTANCE/radius)*180/M_PI;    //correct algorithm 
}

inline double sinDeg(const double& deg)
{
	return sin(deg*M_PI/180.0);
}


inline int sign(float num)
{
	return num > 0? 1 : -1;
}

inline float deg2rad(float deg)
{
	return  (deg/180.0)*M_PI;
}

bool loadPathPoints(std::string file_path,std::vector<gpsMsg_t>& points)
{
	std::ifstream in_file(file_path.c_str());
	if(!in_file.is_open())
	{
		ROS_ERROR("open %s failed",file_path.c_str());
		return false;
	}
	gpsMsg_t point;
	std::string line;
	
	while(in_file.good())
	{
		getline(in_file,line);
		std::stringstream ss(line);
		ss >> point.x >> point.y >> point.yaw >> point.curvature;
		points.push_back(point);
	}
	
	in_file.close();
	return true;
}

float dis2Points(const gpsMsg_t& point1, const gpsMsg_t& point2,bool is_sqrt)
{
	float x = point1.x - point2.x;
 	float y = point1.y - point2.y;
	
	if(is_sqrt)
		return sqrt(x*x +y*y);
	return x*x+y*y;
}

size_t findNearestPoint(const std::vector<gpsMsg_t>& path_points, const gpsMsg_t& current_point)
{
	size_t index = 0;
	float min_dis2 = FLT_MAX;
	
	for(size_t i=0; i<path_points.size(); ++i)
	{
		float dis2 = dis2Points(path_points[i],current_point,false);
		//std::cout << "findNearestPoint dis: " << sqrt(dis2) << std::endl;
		if(dis2 < min_dis2)
		{
			min_dis2 = dis2;
			index = i;
		}
	}
	if(min_dis2 > 15*15)
	{
		ROS_ERROR("current_point x:%f\ty:%f",current_point.x,current_point.y);
		ROS_ERROR("find correct nearest point failed! the nearest point distance over 15 meters");
		return path_points.size();
	}
		
	return index;
}

float disBetweenPoints(const gpsMsg_t& point1, const gpsMsg_t& point2)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	
	return sqrt(x*x+y*y);
}

float calculateDis2path(const double& X_,const double& Y_,
						 const std::vector<gpsMsg_t>& path_points, 
						 const size_t& target_point_index,
						 size_t * const nearest_point_index_ptr)
{
	if(target_point_index == 0)
	{
		if(nearest_point_index_ptr != NULL)
			*nearest_point_index_ptr = 0;
	
		//the direction of side c 
		//float yaw_of_c = (path_points[first_point_index].yaw + path_points[second_point_index].yaw)/2;
		float yaw_of_c = atan2(path_points[1].x-path_points[0].x, path_points[1].y-path_points[0].y);
	
		//object : world coordination to local coordination
		float x = (X_-path_points[0].x) * cos(yaw_of_c) - (Y_-path_points[0].y) * sin(yaw_of_c);
		//float y = (X_-path_points[first_point_index].x) * sin(yaw_of_c) + (Y_-path_points[first_point_index].y) * cos(yaw_of_c);
	
		return x;
	}
	
	//ROS_INFO("path_points.size:%d\t target_point_index:%d",path_points.size(),target_point_index);
	
	//this target is tracking target,
	//let the target points as the starting point of index
	//Judging whether to index downward or upward
	float dis2target = pow(path_points[target_point_index].x - X_, 2) + 
					   pow(path_points[target_point_index].y - Y_, 2) ;
	
	float dis2next_target = pow(path_points[target_point_index+1].x - X_, 2) + 
							pow(path_points[target_point_index+1].y - Y_, 2) ;
							
	float dis2last_target = pow(path_points[target_point_index-1].x - X_, 2) + 
					        pow(path_points[target_point_index-1].y - Y_, 2) ;
	
	//std::cout << sqrt(dis2target)<<"\t"<< sqrt(dis2next_target) <<"\t"<< sqrt(dis2last_target) << std::endl;
	
	float first_dis ,second_dis ;  //a^2 b^2 
	size_t first_point_index,second_point_index;
	
	first_dis = dis2target;
	first_point_index = target_point_index;
	
	int is_yawReverse = 0;
	
	if(dis2last_target <dis2target && dis2next_target > dis2target) //downward
	{
		is_yawReverse = 1;
		for(size_t i=1;true;i++)
		{
		/*   prevent size_t index 0-1 data overflow    */
			if(target_point_index >= i)
				second_point_index = target_point_index-i;
			else
			{
				first_point_index = 1;
				second_point_index = 0;
				break;
			}
		/*   prevent size_t index 0-1 data overflow    */
			
			second_dis = pow(path_points[second_point_index].x - X_, 2) + 
						 pow(path_points[second_point_index].y - Y_, 2) ;
			
			if(second_dis < first_dis) //continue 
			{
				first_dis = second_dis;
				first_point_index = second_point_index;
			}
			else  //end
				break;
		}
	}
	else if(dis2next_target < dis2target && dis2last_target > dis2target) //upward
	{
		for(size_t i=1;true;i++)
		{
			second_point_index = target_point_index + i;
			if(second_point_index >= path_points.size())
			{
				throw "point index out of range";
				return 0;
			}
			
			second_dis = pow(path_points[second_point_index].x - X_, 2) + 
						 pow(path_points[second_point_index].y - Y_, 2) ;

			if(second_dis < first_dis) //continue
			{
				first_dis = second_dis;
				first_point_index = second_point_index;
			}
			else  //end
				break;
		}
	}
	else //midile
	{
		first_point_index = target_point_index-1;
		//first_point_index = target_point_index;
		
		second_point_index = target_point_index +1;
	}
		
	if(nearest_point_index_ptr != NULL)
		*nearest_point_index_ptr = (first_point_index+second_point_index)/2;
	
	//the direction of side c
	//float yaw_of_c = (path_points[first_point_index].yaw + path_points[second_point_index].yaw)/2;
	float yaw_of_c = is_yawReverse*M_PI + atan2(path_points[second_point_index].x-path_points[first_point_index].x,
									   path_points[second_point_index].y-path_points[first_point_index].y);
				
	//object : world coordination to local coordination
	float x = (X_-path_points[first_point_index].x) * cos(yaw_of_c) - (Y_-path_points[first_point_index].y) * sin(yaw_of_c);
	//float y = (X_-path_points[first_point_index].x) * sin(yaw_of_c) + (Y_-path_points[first_point_index].y) * cos(yaw_of_c);
	
	//ROS_ERROR("index1:%d\t index2:%d",first_point_index,second_point_index);
	return x;
}

std::pair<float, float> get_dis_yaw(gpsMsg_t &target,gpsMsg_t &current)
{
	float x = target.x - current.x;
	float y = target.y - current.y;
	
	std::pair<float, float> dis_yaw;
	dis_yaw.first = sqrt(x * x + y * y);
	dis_yaw.second = atan2(y,x);
	
	if(dis_yaw.second <0)
		dis_yaw.second += 2*M_PI;
	
	//std::cout << "dx dy: " << x << "\t" << y << std::endl;
	//std::cout <<"(" << target.x << "," << target.y << ")\t(" << current.x << "," << current.y << ")" << std::endl;
	//std::cout << "t_yaw:" << dis_yaw.second * 180.0/M_PI << std::endl; 
	
	return dis_yaw;
}

float limitRoadwheelAngleBySpeed(const float& angle, const float& speed)
{
	float min_steering_radius = speed*speed/max_side_acceleration;
	if(min_steering_radius <3.0)  //radius = 3.0 -> steeringAngle = 30.0
		min_steering_radius = 3.0;
	
	float max_roadwheelAngle = fabs(generateRoadwheelAngleByRadius(min_steering_radius));
	if(max_roadwheelAngle > MAX_ROAD_WHEEL_ANGLE - 5.0)
	   max_roadwheelAngle = MAX_ROAD_WHEEL_ANGLE -5.0;
	//ROS_INFO("max_angle:%f\t angle:%f",max_roadwheelAngle,angle);
	return saturationEqual(angle,max_roadwheelAngle);
}

size_t findIndexForGivenDis(const std::vector<gpsMsg_t>& path_points, size_t startIndex,float dis)
{
	float sum_dis = 0.0;
	size_t points_size = path_points.size()-1;
	while(ros::ok())
	{
		if(startIndex+5 >= points_size)
			return 0;//error
		sum_dis	+= disBetweenPoints(path_points[startIndex],path_points[startIndex+5]);
		startIndex += 5;
		if(sum_dis > dis)
			return startIndex;
	}
}

float maxCurvatureInRange(const std::vector<gpsMsg_t>& path_points, size_t startIndex,size_t endIndex)
{
	float max = 0.;
	for(size_t i=startIndex; i<endIndex; i++)
	{
		if(fabs(path_points[i].curvature) > max)
			max = fabs(path_points[i].curvature);
	}
	return max;
}

float generateMaxTolarateSpeedByCurvature(const float& curvature, const float& max_accel)
{
	float abs_cur = fabs(curvature);
	if(abs_cur < 0.001)
		return 100.0;

	return sqrt(1.0/abs_cur*max_accel) *3.6;
}

float generateMaxTolarateSpeedByCurvature(const std::vector<gpsMsg_t>& path_points,
											const size_t& nearest_point_index,
											const size_t& target_point_index)
{
	float max_cuvature = 0.0001;
	size_t endIndex = target_point_index + 10;
	if(endIndex >= path_points.size())
		endIndex = path_points.size() -1;
		
	for(size_t i=nearest_point_index; i < endIndex; i++)
	{
		if(fabs(path_points[i].curvature) > max_cuvature)
			max_cuvature = fabs(path_points[i].curvature);
	}
	return sqrt(1.0/max_cuvature*1.5) *3.6;
}

#endif


