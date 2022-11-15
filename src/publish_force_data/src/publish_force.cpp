#include <iostream>
#include <string>
#include <ros/ros.h>
#include <math.h>
#include <sys/time.h>
#include "OptoForceAPI/OptoDAQ.h"
#include "OptoForceAPI/OptoDAQDescriptor.h"
#include "OptoForceAPI/OptoPacket6D.h"
#include "OptoForceAPI/OptoDAQWatcher.h"

#include "publish_force_data/Force.h"




int main(int argc,char** argv){
    ros::init(argc,argv,"publish_force");
    ros::NodeHandle nh; //ROS句柄对象

    ros::Publisher ForcePub=nh.advertise<publish_force_data::Force>("/force_data",10); //发布信息对象,力数据和当前位置信息发布到matlab
    /*创建一个OptoDAQWatcher实例，通过USB连接获得DAQs*/
    OptoDAQWatcher watcher;
    watcher.Start(); // 开始监控不同线程
    OptoDAQDescriptor descriptor;// 初始化一个对象
    while(true){
        if (watcher.GetFirstDAQ(&descriptor) == true) {  //发现的第一个数据采集卡
            std::cout<<descriptor.GetAddress()<<std::endl;
            std::cout<<descriptor.GetSerialNumber()<<std::endl;
            std::cout<<descriptor.GetTypeName()<<std::endl;
            break;
        }
    }
    descriptor.SetTypeName("64");//"64": 6-axis DAQ 设置为六维力的模式

    OptoDAQ  *optoDAQ = new OptoDAQ(descriptor,10);//初始化数据采集对象,数据包的最大计数10个
    if(optoDAQ->Open()){
         std::cout<<"DAQ is now opened!"<<std::endl;
    }else{
        std::cout << "DAQ could not be opened!" << std::endl;
    }

    OptoConfig config = OptoConfig(500, 6, 0); //设置采样频率100HZ,1s获取100个数据,6表示15HZ的过滤频率
    if(optoDAQ->SendConfig(config)){
        std::cout << " DAQ successfully configured." << std::endl;
    }else{
        std::cout << ". DAQ could not be configured." << std::endl;
    }
    optoDAQ->RequestSensitivityReport();


    //数据的读写部分
    int packetNum=5; //阻塞获取数据包的个数 现在是0.001s的频率发送
    OptoPackets6D packets(packetNum);
    std::vector<double> force;

    OptoPacket6D packet;   //单个数据包
    long long timeStame;   //packet获得的事件戳
    clock_t start,finish;  //获取包开始和结束的时间
    double totaltime;      //运行的总时间
    publish_force_data::Force forceMsg;  //要发布的力位置信息


    try{
        long long timeTemp=0;
        while(optoDAQ->IsValid()&&optoDAQ->Is6D()&&ros::ok()){
            packets.Clear();
            optoDAQ->GetPackets6D(&packets,true);//false表示不需要阻塞调用,一直等到十个数据包收集完
            packet=packets.GetPacket(0);//获取数据包的第一个数据
            timeStame=packet.GetTimeStamp();

            if(timeTemp==0){
                totaltime=0.0;
                timeTemp=timeStame;
            }else{
                totaltime=(timeStame-timeTemp)/1000000.0;
            }

            /*发布位置和力信息,位置信息采用四元数的方法进行发布*/
            forceMsg.timeStamp=totaltime;
            forceMsg.forceData[0]=packet.GetFxInNewton();
            forceMsg.forceData[1]=packet.GetFyInNewton();
            forceMsg.forceData[2]=packet.GetFzInNewton();
            forceMsg.forceData[3]=packet.GetTxInNewtonMeter();
            forceMsg.forceData[4]=packet.GetTyInNewtonMeter();
            forceMsg.forceData[5]=packet.GetTzInNewtonMeter();

            ForcePub.publish(forceMsg);//向matlab发送信息
        }

    }catch(std::exception exception){
        delete optoDAQ;
        std::cout<<exception.what()<<std::endl;
    }
}
