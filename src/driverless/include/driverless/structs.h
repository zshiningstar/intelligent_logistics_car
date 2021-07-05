#ifndef STRUCTS_H_
#define STRUCTS_H_

#include <atomic>
#include <utility>
#include <boost/thread/locks.hpp>    
#include <boost/thread/shared_mutex.hpp>    

typedef boost::shared_mutex SharedMutex;
typedef boost::unique_lock<SharedMutex> WriteLock;
typedef boost::shared_lock<SharedMutex> ReadLock;


/*@brief 车辆控制信息*/
typedef struct ControlCmd
{
	ControlCmd()
	{
		validity = false;
		speed_validity = false;
		speed = 0.0;
		steer_validity = false;
		roadWheelAngle = 0.0;
		hand_brake = false;
		turnLight = 0;
	}

	//当validity有效时,无需在对speed_validity进行判断
	//当validity无效时，需判断speed_validity是否有效!
	bool  validity;       //指令的全局有效性
	bool  speed_validity; //仅速度指令的有效性
	float speed;
	bool  steer_validity; //转向指令有效
	float roadWheelAngle;

	bool hand_brake;
	uint8_t gear;
	uint8_t brake;
	uint8_t turnLight; // 0 关灯,1左转,2右转
	uint8_t stopLight; // 0 关灯

	void display(const std::string& prefix)
	{
		std::cout << prefix << "\t["
			<< "valid:" << validity << "\t" 
			<< "speed:" << speed << "\t"
			<< "brake:" << int(brake) << "\t"
			<< "angle:" << roadWheelAngle 
			<< "]\r\n"; 
	}
} controlCmd_t;

/*@brief 停车点信息*/
class ParkingPoint
{
public:
	ParkingPoint()
	{
		index = 0;
		parkingDuration = 0;
		isParking = false;
	}
	ParkingPoint(size_t _index,float _duration)
	{
		index = _index;
		parkingDuration = _duration;
		isParking = false;
	}
	
	size_t index; //停车点在全局路径中的索引
	float  parkingDuration; //停车时长s,若为0,则表示一直停车
	double parkingTime;     //停车时刻
	bool   isParking;       //正在停车
};

class ParkingPoints
{
public:
	std::vector<ParkingPoint> points;
	size_t next_index = 0;
	bool sorted = false;

	size_t size() const {return points.size();}
	void push_back(const ParkingPoint& point)
	{
		points.push_back(point);
	} 
	const ParkingPoint& operator[](size_t i)const  {return points[i];}
	ParkingPoint& operator[](size_t i)             {return points[i];}

	bool available() const { return next_index  < points.size();}

	void sort() //停车点由小到大排序
	{
		std::sort(points.begin(),points.end(),
			[](const ParkingPoint& point1,const ParkingPoint& point2)
			{return point1.index < point2.index;});
		sorted = true;
	}

	bool isSorted() const {return sorted;}

	void print(const std::string& prefix) const 
	{
		for(auto &point:points)
			printf("[%s] parking point index: %lu  duration: %.1f",prefix.c_str(), point.index,point.parkingDuration);
	}
	
	ParkingPoint& next()
	{
		if(!available())
			next_index = 0;

		return points[next_index];
	}

	void clear()
	{
		points.clear();
		next_index = 0;
		sorted = false;
	}

};


/*@brief 路径转向区间信息 */
class TurnRange
{
public:
	enum TurnType
	{
		TurnType_Left = -1,
		TurnType_None = 0,
		TurnType_Right = 1,
	};

	int type;
	size_t start_index;
	size_t end_index;

	TurnRange(int _type, size_t _start_index, size_t _end_index)
	{
		type = _type;
		start_index = _start_index;
		end_index = _end_index;
	}
};

class TurnRanges
{
public:
	std::vector<TurnRange> ranges;

	size_t size() const {return ranges.size();}
	void clear()
	{
		ranges.clear();
	}
};

/*@brief 位置信息*/
class Point
{
public:
	double x,y,z;
	Point(){}
	Point(double _x, double _y, double _z=0.0):
		x(_x), y(_y), z(_z){}
};

/*@brief 位姿信息*/
class Pose : public Point
{
public:
	double yaw;
};

/*@brief 路径点信息*/
class GpsPoint : public Pose
{
public:
	double longitude;
	double latitude;
	float curvature;
};

class Path
{
public:
	std::vector<GpsPoint> points;
	float resolution;
	bool  has_curvature;               //是否包含路径曲率

	std::atomic<size_t> pose_index;    //距离车辆最近路径点的索引
	size_t final_index;                //终点索引

	ParkingPoints park_points;         //停车点信息
	TurnRanges    turn_ranges;		   //转向区间信息

public:
	size_t size() const {return points.size();}
	const GpsPoint& operator[](size_t i) const {return points[i];}
	GpsPoint& operator[](size_t i)             {return points[i];}
	void clear()                       //清空路径信息
	{
		points.clear();
		resolution = 0.0;
		has_curvature = false;
		pose_index = 0;
		final_index = 0;
		park_points.clear();
		turn_ranges.clear();
	}

	bool finish() const {return pose_index>=final_index;}
	float remaindDis()
	{
		float dis = (final_index - pose_index) * resolution;
		if(dis < 0) dis = 0;
		return dis;
	}
};

/*@brief 车辆参数 */
class VehicleParams
{
public:
	float max_roadwheel_angle;
	float min_roadwheel_angle;
	float min_radius;
	float max_speed;
	float wheel_base;
	float wheel_track;
	float width;
	float length;
	float steer_clearance;
	float steer_offset;

	bool validity;
	VehicleParams()
	{
		validity = false;
	}
};

/*@brief 车辆状态 内部状态+外部状态
 * 更新车辆状态的线程利用类方法进行更新
 * 读取车辆状态的线程先创建副本，然后直接访问副本成员
*/
#define LOCK true
#define UNLOCK  false
class VehicleState 
{
public:
	uint8_t gear;         //档位
	float   speed;        //车速
	float   steer_angle;  //前轮转角
	Pose    pose;         //车辆位置

	bool speed_validity = false;
	bool steer_validity = false;
	bool pose_validity  = false;

	SharedMutex wr_mutex;//读写锁

	void setSpeed(const float& val)
	{
		WriteLock writeLock(wr_mutex);
		speed = val;
	}

	void setSteerAngle(const float& val)
	{
		WriteLock writeLock(wr_mutex);
		steer_angle = val;
	}

	void setPose(const Pose& val)
	{
		WriteLock writeLock(wr_mutex);
		pose = val;
	}
	
	void setPoseValid(bool flag)
	{
		WriteLock writeLock(wr_mutex);
		pose_validity = flag;
	}

	void setGear(uint8_t g)
	{
		ReadLock writeLock(wr_mutex);
		gear = g;
	}

	uint8_t getGear()
	{
		ReadLock readLock(wr_mutex);
		return gear;
	}

	float getSpeed(bool lock = UNLOCK)
	{
		if(lock)
		{
			ReadLock readLock(wr_mutex);
			return speed;
		}
		return speed;
	}
	float getSteerAngle(bool lock = UNLOCK)
	{
		if(lock)
		{
			ReadLock readLock(wr_mutex);
			return steer_angle;
		}
		return steer_angle;
	}
	
	bool getPoseValid() const
	{
		return pose_validity;
	}

	Pose getPose(bool lock = UNLOCK)
	{
		if(lock)
		{
			ReadLock readLock(wr_mutex);
			return pose;
		}
		return pose;
	}

	VehicleState(){} //当定义了拷贝构造函数时，编译器将不提供默认构造函数，需显式定义

	VehicleState(const VehicleState& obj)
	{
		ReadLock readLock(wr_mutex);
		this->speed       = obj.speed;
		this->steer_angle = obj.steer_angle;
		this->pose        = obj.pose;
		this->speed_validity    = obj.speed_validity;
		this->steer_validity    = obj.steer_validity;
		this->pose_validity     = obj.pose_validity;
	};
	const VehicleState& operator=(const VehicleState& obj)
	{
		ReadLock readLock(wr_mutex);
		this->speed       = obj.speed;
		this->steer_angle = obj.steer_angle;
		this->pose        = obj.pose;
		this->speed_validity    = obj.speed_validity;
		this->steer_validity    = obj.steer_validity;
		this->pose_validity     = obj.pose_validity;
		return *this;
	};

	bool validity(std::string& info)
	{
		bool ok = true;
		if(!speed_validity)
		{
			info += "Vehicle speed is invalidity!\t";
			ok = false;
		}
		if(!steer_validity)
		{
			info += "Vehicle steer angle is invalidity!\t";
			ok = false;
		}
		if(!pose_validity)
		{
			info += "Vehicle pose is invalidity!";
			ok = false;
		}
		return ok;
	}


};


#endif
