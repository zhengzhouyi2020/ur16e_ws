#include <cmath>
#include <iostream>
#include <pthread.h>
#include"ros/ros.h"
#include"ros_haption/HaptionData.h"

#include <time.h>
#include <unistd.h>
//#include "VirtuoseAPI.h"

#include "HaptionRobot.h"

int num=0;

class Haption
{
public:
	Haption()
	{
                 //sub1和pub1的话题名称需要根据实际的话题进行更改，sub2的话题名称不能更改

		sub1 = n.subscribe("JointPos", 1000, &Haption::Callback,this); //接收外部节点位置数据的订阅方
		sub2 = n.subscribe("HaptionFrequency", 1000, &Haption::Callback1,this); //接收自定义的发布频率
		pub1 = n.advertise<ros_haption::HaptionData>("DataFromHaption",1000); //发布Franka的节点位置数据的发布方
        
	};

	void Callback(const ros_haption::HaptionData &data)	//接收外部数据的回调函数
	{
	   
	};   

	void Callback1(const ros_haption::HaptionData &data)   //以订阅的频率控制发布的频率
	{

        	//Haption数据类型实例化
		ros_haption::HaptionData haptiondata;

        	//将Haption自身数据传递给将要发布的消息
       		for (int i = 0; i < 7; i++)
        	{
              		haptiondata.Position[i]=VH.M_Data.Position[i];
        	}
        	for (int i = 0; i < 6; i++)
        	{
              		haptiondata.Force[i]=VH.M_Data.Force[i];
        	}
        	for (int i = 0; i < 6; i++)
        	{
              		haptiondata.Speed[i]=VH.M_Data.Speed[i];
        	}
        	for (int i = 0; i < 6; i++)
        	{
              		haptiondata.ArticularPosition[i]=VH.M_Data.Position1[i];
        	}
        	for (int i = 0; i < 6; i++)
        	{
              		haptiondata.ArticularSpeed[i]=VH.M_Data.Speed1[i];
        	}

        	pub1.publish(haptiondata);//发布消息
        }
     
public:
	ros::NodeHandle n;  
	ros::Subscriber sub1; 
	ros::Subscriber sub2;
	ros::Publisher pub1; 
	VirHaptions VH;
};
 


 
int main(int argc, char** argv)
 {

	//初始化ros节点
	ros::init(argc, argv, "HaptionData");

	Haption ha;

	//初始化数据
	ha.VH.HaptionInit();

	ha.VH.start_data_pth();

	
  
	ros::spin();
	//关闭haption
        
	ha.VH.HaptionClose();

	std::cout  << "    control finish!    " << std::endl;

 	return 0;
 }
