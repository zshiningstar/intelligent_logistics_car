#ifndef __LOCAL_SOCKET_H_
#define __LOCAL_SOCKET_H_

#include <ros/ros.h>
#include <vector>
#include <thread>
#include <mutex>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "driverless/extern_control/extern_control_base.hpp"

/*本地局域网socket通讯外部控制器
 *为避免自动驾驶过程中出现意外情况，可在必要时对车速进行干预
 *当收到外部控制指令后，系统将按照外部控制器速度指令自动驾驶，直到外部控制器主动请求失能(disable)
*/

#ifndef PACK
#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

enum ExternCmdType
{
	ExternCmdType_Speed = 0,   //速度指令
	ExternCmdType_Disable = 1, //禁用外部指令
	ExternCmdType_turnLight=2, //转向灯指令
};

PACK(
typedef struct ExternCmd
{
	uint8_t header[2];
	uint8_t type; 
	uint8_t len;
	uint8_t data[];
	
}) externCmd_t;


class LanExternControl : public ExternControlBase
{
public:
    LanExternControl():
		ExternControlBase("LocalControl"){}

    ~LanExternControl(){}

    virtual bool init(ros::NodeHandle nh,ros::NodeHandle nh_private) override
    {
		local_ip_     = nh_private.param<std::string>("lan_control/local_ip","0.0.0.0");
		local_port_   = nh_private.param<int>("lan_control/local_port", 5000);
		return initSocket();
    }

    virtual bool start() override
    {
		is_running_ = true;
		std::thread t(&LanExternControl::workThread,this);
		t.detach();
		return true;
    }

    virtual void stop() override
    {
		is_running_ = false;
		cmd_mutex_.lock();
		cmd_.validity = false;
		cmd_mutex_.unlock();
    }

	const uint8_t Header1 = 0x66;
	const uint8_t Header2 = 0xCC;

private:
    bool initSocket()//初始化创建了服务器端
	{
		struct sockaddr_in local_addr;
		bzero(&local_addr,sizeof(local_addr));//init 0  //置&local_addr开始前sizeof(local_addr)的值为0
        //设置端口
		local_addr.sin_port = htons(local_port_);//将IP端口号转化为网络字节序
		//设置协议族
		local_addr.sin_family = AF_INET;//代表Internet协议族
		//设置IP地址
		{
		int convert_ret = inet_pton(AF_INET, local_ip_.c_str(), &local_addr.sin_addr);//inet_pton支持IPV4也支持IPV6
		if(convert_ret !=1)
		{
			ROS_ERROR("convert socket ip failed, please check the format!");
			return false;
		}
		else
			ROS_INFO("convert socket ip complete .");
		}
		
		//创建套接字
		{
		udp_fd_ = socket(PF_INET,SOCK_DGRAM , 0);//SOCK_DGRAM：UDP; PF_INET：IPV4网络通讯
		if(udp_fd_ < 0)
		{
			ROS_ERROR("build socket error");
			return false;
		}
		else
			ROS_INFO("build socket ok .");
		}
		
		//SO_REUSEADDR：设置端口复用
		
		int udo_opt = 1;
		setsockopt(udp_fd_, SOL_SOCKET, SO_REUSEADDR, &udo_opt, sizeof(udo_opt));
		
		// 设置超时
	//	struct timeval timeout;
	//	timeout.tv_sec = 0;  timeout.tv_usec = 0;
	//	setsockopt(udp_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
		
		int ret = bind(udp_fd_, (struct sockaddr*)&local_addr, sizeof(local_addr));
		if(ret < 0)
		{
			ROS_ERROR("udp bind ip: %s port: %d failed!",local_ip_.c_str(),local_port_);
			return false;
		}
		
		return true;
	}

	void workThread()
	{
		const int BufLen = 100;
		uint8_t *recvbuf = new uint8_t [BufLen+1];
		struct sockaddr_in client_addr;//客户端
		socklen_t clientLen = sizeof(client_addr);
		int len = 0;

		while(ros::ok() && is_running_)
		{
			len = recvfrom(udp_fd_, recvbuf, BufLen,0,(struct sockaddr*)&client_addr, &clientLen);
			if(len <=0) continue;//小于0则接收不成功
			
			//for(int i=0; i<len; ++i)
			//	std::cout << std::hex << int(recvbuf[i]) << "\t";
			//std::cout << std::endl;
			
			if(recvbuf[0] != Header1 ||
			   recvbuf[1] != Header2 )
			continue;
			
			std::string answer;
			externCmd_t *extern_cmd = (externCmd_t*)recvbuf;//防止字节对齐之后直接将extern_cmd指向recvbuf开始地址位置，并且此时解析已经完成
			if(extern_cmd->type == ExternCmdType_Speed)//外部速度指令
			{
				cmd_mutex_.lock();
				cmd_.validity = true;//指令有效
				cmd_.speed_validity = true;//速度指令有效
				cmd_.steer_validity = false;
				cmd_.speed = extern_cmd->data[0];//速度值
				cmd_mutex_.unlock();
				answer = std::string("speed:") + std::to_string(extern_cmd->data[0]) + std::string("km/h");
			}
			else if(extern_cmd->type == ExternCmdType_Disable)//外部指令禁用
			{
				cmd_mutex_.lock();
				cmd_.validity = false;
				cmd_.speed_validity = false;
				cmd_.steer_validity = false;
				cmd_mutex_.unlock();
				answer = "disabled";
			}
			else if(extern_cmd->type == ExternCmdType_turnLight)//转向灯指令
			{
				cmd_.validity = false;//转向灯指令不修改其他指令
				cmd_.speed_validity = false;
				cmd_.steer_validity = false;
				
				cmd_mutex_.lock();
				cmd_.turnLight = extern_cmd->data[0];
				cmd_mutex_.unlock();
				if(extern_cmd->data[0] == 0)
					answer = "close all light";
				else if(extern_cmd->data[0] == 1)
					answer = std::string("left light");
				else if(extern_cmd->data[0] == 2)
					answer = std::string("right light");
			}
			
			sendto(udp_fd_, answer.c_str(), answer.length(),0, 
							(struct sockaddr*)&client_addr, sizeof(client_addr));//
		}
		is_running_ = false;
	}

private:
	int udp_fd_;
	struct sockaddr_in sockaddr_;  //服务器地址
	std::string local_ip_;
	int         local_port_;
};

#endif
