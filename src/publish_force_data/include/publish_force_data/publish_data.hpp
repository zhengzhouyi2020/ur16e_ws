#pragma once

#include <ros/ros.h>
#include <string>
#include <math.h>
#include <sys/time.h>

#include "OptoForceAPI/OptoDAQ.h"
#include "OptoForceAPI/OptoDAQDescriptor.h"
#include "OptoForceAPI/OptoPacket6D.h"
#include "OptoForceAPI/OptoDAQWatcher.h"

#include <publish_force_data/ForceAndTorque.h>

using namespace ros;

class ForcePublish {
    public:
        explicit ForcePublish(NodeHandle& nh, int frequency = 500, int packetNum = 5, int maxPacketNum = 10,int filter = 6);
        ~ForcePublish();

        void getForceData();
        bool publishData(publish_force_data::ForceAndTorque::Request &request, publish_force_data::ForceAndTorque::Response& response);


    private:
        NodeHandle nh;
        ServiceServer service;
        double totaltime;   // 距离开始采集的总时间
        long long timeTemp;  // 从采集卡中得到的时间,单位是微秒

        /*创建一个OptoDAQWatcher实例，通过USB连接获得DAQs*/
        OptoDAQWatcher watcher;  
        OptoDAQ  optoDAQ; // 初始化数据采集对象
        OptoPackets6D packets; // 采集力的数据包,
        OptoPacket6D packet;  // 单个数据包
        
        
};
