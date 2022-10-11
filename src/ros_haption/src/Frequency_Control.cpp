#include"ros/ros.h"
#include"ros_haption/HaptionData.h"





int main(int argc, char *argv[])
{
    	ros::init(argc,argv,"HaptionFrequency");
    	ros::NodeHandle nh;
	int FREQ;
	nh.param<int>("/freq",FREQ,10);
    	ros::Publisher pub1=nh.advertise<ros_haption::HaptionData>("HaptionFrequency",1000);

    	//发布的消息内容是无意义，用作控制频率
    	ros_haption::HaptionData data;

	//主程序发布频率以该频率定义的
	ros::Rate rate(FREQ);
	ROS_INFO("FREQ:       %d",FREQ);
	int num =0;
	while(ros::ok())
	{

		for(int i=0;i<7;i++)
		{
			data.Position[i] = 0;
		}
		for(int i=0;i<6;i++)
		{
			data.Speed[i] = 0;
		}
		for(int i=0;i<6;i++)
		{
			data.Force[i] = 0;
		}
		for(int i=0;i<6;i++)
		{
			data.ArticularPosition[i] = 0;
		}
		for(int i=0;i<6;i++)
		{
			data.ArticularSpeed[i] = 0;
		}
		pub1.publish(data);
		rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
