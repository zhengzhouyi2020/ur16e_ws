#include <publish_force_data/publish_data.hpp>

ForcePublish::ForcePublish(NodeHandle nh, int frequency, int packetNum, int maxPacketNum,int filter):nh(nh) {
    // 注册服务

    service =nh.advertiseService("server_for_force",&ForcePublish::publishData,this);
    timeTemp = 0L;

    watcher.Start();
    OptoDAQDescriptor descriptor;// 初始化一个采集卡描述对象
    //"64": 6-axis DAQ 设置为六维力的模式
    while(true){
        if (watcher.GetFirstDAQ(&descriptor) == true) {  //发现的第一个数据采集卡,一直等待连接
            std::cout<<"connected to the force sensor"<<std::endl;
            break;
        }
    }

    descriptor.SetTypeName("64");
    //初始化数据采集对象,能够持有数据包的最大计数10个,多的将被舍去
    optoDAQ = OptoDAQ(descriptor,maxPacketNum);
    if(optoDAQ.Open()){
        std::cout<<"DAQ is now opened!"<<std::endl;
    }else{
        std::cout << "DAQ could not be opened!" << std::endl;
    }
    //设置采集对象的参数属性
    OptoConfig config = OptoConfig(frequency, filter, 0); //设置采样频率100HZ,1s获取100个数据,6表示1.5HZ的过滤频率
       if(optoDAQ.SendConfig(config)){
        std::cout << " DAQ successfully configured." << std::endl;
    }else{
        std::cout << ". DAQ could not be configured." << std::endl;
    }
    // 请求DAQ提供传感器数据
    optoDAQ.RequestSensitivityReport();
    // 初始化数据包的数量
    packets = OptoPackets6D(packetNum);

}

void ForcePublish::getForceData() {
     if (optoDAQ.IsValid() && optoDAQ.Is6D() && ok()) {
        optoDAQ.GetPackets6D(&packets,true);//false表示不需要阻塞调用,一直等到十个数据包收集完
        packet=packets.GetPacket(0);//获取数据包的第一个数据
        long long timeStame=packet.GetTimeStamp();
        if(timeTemp==0){
            totaltime=0.0;
            timeTemp=timeStame;
        }else{
            totaltime=(timeStame-timeTemp)/1000000.0;
        }
        packets.Clear();
    } else {
        std::cout <<  "collect error, please check your computer"<< std::endl;
    }
}

bool ForcePublish::publishData( publish_force_data::ForceAndTorque::Request& request, publish_force_data::ForceAndTorque::Response& response) {
    if (optoDAQ.IsValid() && optoDAQ.Is6D() && ok()) {
        optoDAQ.GetPackets6D(&packets,true);//false表示不需要阻塞调用,一直等到十个数据包收集完
        packet=packets.GetPacket(0);//获取数据包的第一个数据
        long long timeStame=packet.GetTimeStamp();
        if(timeTemp==0){
            totaltime=0.0;
            timeTemp=timeStame;
        }else{
            totaltime=(timeStame-timeTemp)/1000000.0;
        }
        response.timeStamp = totaltime;
        response.forceData[0] = packet.GetFxInNewton();
        response.forceData[1] = packet.GetFyInNewton();
        response.forceData[2] = packet.GetFzInNewton();
        response.forceData[3] = packet.GetTxInNewtonMeter();
        response.forceData[4] = packet.GetTyInNewtonMeter();
        response.forceData[5] = packet.GetTzInNewtonMeter();
        packets.Clear();
    } else {
        std::cout <<  "collect error, please check your computer"<< std::endl;
        return false;
    }
    return true;
}

ForcePublish::~ForcePublish() {
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "force_service_node");
    ros::NodeHandle nh;
    ForcePublish forcePublish(nh);
    std::cout<<"OK"<<std::endl;
    while (ok()) {
        forcePublish.getForceData();
        spinOnce();
    }
    return 0;
}