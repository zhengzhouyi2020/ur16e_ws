#include <ros/ros.h> 
#include <publish_force_data/ForceAndTorque.h>

using namespace ros;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "force_service_node");
    ros::NodeHandle *nh;

    // 发现服务后，创建一个服务客户端，连接service
	ros::service::waitForService("/server_for_force");//查询是否有spawn的服务，如果没有的话就会一直等待，只有服务在系统上存在了，才会请求这个服务
    ros::ServiceClient client = nh->serviceClient<publish_force_data::ForceAndTorque>("server_for_force"); //创建客户端ServiceClient，这个客户端是用来给服务发送请求的，<>是数据类型，“”是服务的名字
    
    publish_force_data::ForceAndTorque srv; // 定义收到的力数据
    while (ok()) {
         client.call(srv);
         ROS_INFO("x:%f y:%f z:%f", srv.response.forceData.at(0),srv.response.forceData.at(0),srv.response.forceData.at(0));
    }
    return 0;
}
