#pragma once

// 时间
#include <ros/package.h>
#include <ros/ros.h>
#include <string>
#include <math.h>
#include <sys/time.h>
#include <vector>

// 六维力传感器接口
#include "OptoForceAPI/OptoDAQ.h"
#include "OptoForceAPI/OptoDAQDescriptor.h"
#include "OptoForceAPI/OptoPacket6D.h"
#include "OptoForceAPI/OptoDAQWatcher.h"

// 矩阵运算库
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"

// 文件流输出
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>

// RTDE接口
#include "ur_rtde/rtde_receive_interface.h"
#include <ur_rtde/rtde_control_interface.h>

#include <thread>
#include <chrono>

using namespace std::chrono;

using namespace ur_rtde;
using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;

// PID类
class PID {
    private:
        double KP;
        double KI;
        double KD;
        double exp_fz;
        double dt;
        double now_fz;
        double sum_err;
        double now_err;
        double last_err;
public:
    PID(double exp_fz,double kp,double ki,double kd,double dt){
        this->KP = kp;
        this->KI = ki;
        this->KD = kd;
        this->exp_fz = exp_fz;
        this->dt = dt;
        this->now_fz = 0;
        this->sum_err = 0;
        this->now_err = 0;
        this->last_err = 0;
    }
    double pid_force_position(const double exp_fz,const double now_fz ){
        double Xe;
        this->exp_fz = exp_fz;
        this->now_fz = now_fz;
        this->last_err = this->now_err;
        this->now_err = this->now_fz-this->exp_fz;
        this->sum_err += this->now_err;

        if (this->now_fz > -5){
            Xe = this->KP * (this->now_err);
        }
        else{
            Xe = this->KP * (this->now_err) \
                        + this->KI * this->sum_err * this->dt;
        }
        return Xe;
    };
    ~PID(){}
};

// 力数据采集的类
class FORCE{
    private:
        double totaltime;   // 距离开始采集的总时间
        long long timeTemp;  // 从采集卡中得到的时间,单位是微秒

        /*创建一个OptoDAQWatcher实例，通过USB连接获得DAQs*/
        OptoDAQWatcher watcher;  
        OptoDAQ*  optoDAQ; // 初始化数据采集对象
        OptoPackets6D packets; // 采集力的数据包,
        OptoPacket6D packet;  // 单个数据包
    public:
        FORCE(int frequency = 500, int packetNum = 5, int maxPacketNum = 10,int filter = 6){
            timeTemp = 0L;
            watcher.Start();
            OptoDAQDescriptor descriptor;// 初始化一个采集卡描述对象
            //"64": 6-axis DAQ 设置为六维力的模式
            descriptor.SetTypeName("64");
            while(true){
                if (watcher.GetFirstDAQ(&descriptor) == true) {  //发现的第一个数据采集卡,一直等待连接
                    std::cout<<"connected to the force sensor"<<std::endl; 
                    break;
                }
            }
            //初始化数据采集对象,能够持有数据包的最大计数10个,多的将被舍去
            optoDAQ = new OptoDAQ(descriptor,maxPacketNum);
            if(optoDAQ->Open()){
                std::cout<<"DAQ is now opened!"<<std::endl;
            }else{
                std::cout << "DAQ could not be opened!" << std::endl;
            }
            //设置采集对象的参数属性
            OptoConfig config = OptoConfig(frequency, filter, 0); //设置采样频率100HZ,1s获取100个数据,6表示1.5HZ的过滤频率
            if(optoDAQ->SendConfig(config)){
                std::cout << " DAQ successfully configured." << std::endl;
            }else{
                std::cout << ". DAQ could not be configured." << std::endl;
            }
            // 请求DAQ提供传感器数据
            optoDAQ->RequestSensitivityReport();
            // 初始化数据包的数量
            packets = OptoPackets6D(packetNum);
        }

        bool getForce(Vector6d &forceData, double &timeStamp){
            if (optoDAQ->IsValid() && optoDAQ->Is6D() && ros::ok()) {
            packets.Clear();
            optoDAQ->GetPackets6D(&packets,true);//false表示不需要阻塞调用,一直等到十个数据包收集完
            packet=packets.GetPacket(0);//获取数据包的第一个数据
            long long timeStame=packet.GetTimeStamp();
            if(timeTemp==0){
                totaltime=0.0;
                timeTemp=timeStame;
            }else{
                totaltime=(timeStame-timeTemp)/1000000.0;
            }
            timeStamp = totaltime;
            forceData[0] = packet.GetFxInNewton();
            forceData[1] = packet.GetFyInNewton();
            forceData[2] = packet.GetFzInNewton();
            forceData[3] = packet.GetTxInNewtonMeter();
            forceData[4] = packet.GetTyInNewtonMeter();
            forceData[5] = packet.GetTzInNewtonMeter();
            } else {
                std::cout <<  "collect error, please check your computer"<< std::endl;
                return false;
            }
                return true;
        }
        ~FORCE() {
            delete optoDAQ;
        }   
};


// 力/轨迹控制的类
class ForceControl {
    public:
        PID *pid;
        FORCE *force;
        Vector3d G0;
        Vector3d p;
        Vector6d F0;

        ForceControl(PID* pid, FORCE* force,Vector3d G0,Vector3d p,Vector6d F0);
        ForceControl(FORCE* force,Vector3d G0,Vector3d p,Vector6d F0);
        Vector6d calculateGravityCompensation(const std::vector<double>& vector,Vector6d F);
        ~ForceControl();
    private:
   
          
};


// 角度到旋转矩阵计算
void q2matrix(const std::vector<double>& q,Matrix<double,3,3>& TEB){
    double s1 = sin(q[0]), c1 = cos(q[0]); // q1
    double q23 = q[1], q234 = q[1], s2 = sin(q[1]), c2 = cos(q[1]); // q2
    double s3 = sin(q[2]), c3 = cos(q[2]); q23 += q[2]; q234 += q[2]; // q3
    q234 += q[3]; // q4
    double s5 = sin(q[4]), c5 = cos(q[4]); // q5
    double s6 = sin(q[5]), c6 = cos(q[5]); // q6
    double s23 = sin(q23), c23 = cos(q23);
    double s234 = sin(q234), c234 = cos(q234);
     TEB(0,0) = c6*(s1*s5 + c234*c1*c5) - s234*c1*s6;
     TEB(0,1) = -s6*(s1*s5 + c234*c1*c5) - s234*c1*c6;
     TEB(0,2) = c5*s1 - c234*c1*s5;
     TEB(1,0) = -c6*(c1*s5 - c234*c5*s1) - s234*s1*s6;
     TEB(1,1) = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1;
     TEB(1,2) =- c1*c5 - c234*s1*s5;
     TEB(2,0) =c234*s6 + s234*c5*c6;
     TEB(2,1) =c234*c6 - s234*c5*s6;
     TEB(2,2) =-s234*s5;
}

// 正运动学计算
void ur16e_Kinematic(const std::vector<double>& q,Matrix<double,4,4>& TEB){
     double d1 =  0.1807;
     double a2 = -0.4784;
     double a3 = -0.36;
     double d4 =  0.17415;
     double d5 =  0.11985;
     double d6 =  0.11655;

    double s1 = sin(q[0]), c1 = cos(q[0]); // q1
    double q23 = q[1], q234 = q[1], s2 = sin(q[1]), c2 = cos(q[1]); // q2
    double s3 = sin(q[2]), c3 = cos(q[2]); q23 += q[2]; q234 += q[2]; // q3
    q234 += q[3]; // q4
    double s5 = sin(q[4]), c5 = cos(q[4]); // q5
    double s6 = sin(q[5]), c6 = cos(q[5]); // q6
    double s23 = sin(q23), c23 = cos(q23);
    double s234 = sin(q234), c234 = cos(q234);
     TEB(0,0) = c6*(s1*s5 + c234*c1*c5) - s234*c1*s6;
     TEB(0,1) = -s6*(s1*s5 + c234*c1*c5) - s234*c1*c6;
     TEB(0,2) = c5*s1 - c234*c1*s5;
     TEB(0,3) = d6*(c5*s1 - c234*c1*s5) + c1*(a3*c23 + a2*c2) + d4*s1 + d5*s234*c1;
     TEB(1,0) = -c6*(c1*s5 - c234*c5*s1) - s234*s1*s6;
     TEB(1,1) = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1;
     TEB(1,2) =- c1*c5 - c234*s1*s5;
     TEB(1,3) =s1*(a3*c23 + a2*c2) - d4*c1 - d6*(c1*c5 + c234*s1*s5) + d5*s234*s1;
     TEB(2,0) =c234*s6 + s234*c5*c6;
     TEB(2,1) =c234*c6 - s234*c5*s6;
     TEB(2,2) =-s234*s5;
     TEB(2,3) =d1 + a3*s23 + a2*s2 - d5*c234 - d6*s234*s5;
     TEB(3,0) =0;
     TEB(3,1) =0;
     TEB(3,2) =0;
     TEB(3,3) =1;
}

// CSV文件读取double数据
Eigen::MatrixXd readCSV(std::string file, int rows, int cols) {

  std::ifstream in(file);
  
  std::string line;

  int row = 0;
  int col = 0;

  Eigen::MatrixXd res = Eigen::MatrixXd(rows, cols);

  if (in.is_open()) {

    while (std::getline(in, line)) {

      char *ptr = (char *) line.c_str();
      int len = line.length();

      col = 0;

      char *start = ptr;
      for (int i = 0; i < len; i++) {

        if (ptr[i] == ',') {
          res(row, col++) = atof(start);
          start = ptr + i + 1;
        }
      }
      res(row, col) = atof(start);

      row++;
    }

    in.close();
  }
  return res;
}

std::vector<double> stringToNum(const std::vector<std::string>& strs) {
    std::vector<double> result;
    double num;
    for(int i = 0; i < strs.size(); i++) {
        std::istringstream iss(strs[i]); //将整行字符串line读入到字符串流istringstream中
        iss >> num;
        result.push_back(num);
    }
    return result;
}

void getRecodeData(std::string path, std::vector<std::vector<double>>& result){
    std::string lineStr;
    std::ifstream file(path, std::ios::in);
    std::vector<std::vector<std::string>> strArray;
    getline(file, lineStr); //  直接跳过第一行
    while(getline(file, lineStr)) {
        std::stringstream ss(lineStr);
        std::string str;
        std::vector<std::string> lineArray;
        while(getline(ss,str,',')){
            lineArray.push_back(str);   // 字符串分割,直接将分割后的字符保存在容器中
        }
        strArray.push_back(lineArray); // 添加到矩阵中
        
    }
    int length = strArray.size();
    for(int i = 0; i < length; i++) {
        result.push_back(stringToNum(strArray[i]));  //将字符型的数组转化为浮点数
    }
}

// 力控初始化
ForceControl::ForceControl(PID* pid, FORCE* force,Vector3d G0,Vector3d p,Vector6d F0){
    this->pid = pid;
    this->force = force;
    this->F0 = F0;
    this-> G0 = G0;
    this->p = p;
}

// 力控初始化
ForceControl::ForceControl(FORCE* force,Vector3d G0,Vector3d p,Vector6d F0){
    this->force = force;
    this->F0 = F0;
    this-> G0 = G0;
    this->p = p;
}


// 利用旋转矢量得到的重力补偿
// Vector6d ForceControl::calculateGravityCompensation(const std::vector<double>& vector,Vector6d F) {
//     // TODO how to difine vector;
//     double theta=sqrt(vector[0]*vector[0]+vector[1]*vector[1]+vector[2]*vector[2]);
//     double kx=vector[0]/theta;
//     double ky=vector[1]/theta;
//     double kz=vector[2]/theta;
//     double x=kx*sin(theta/2);
//     double y=ky*sin(theta/2);
//     double z=kz*sin(theta/2);
//     double w=cos(theta/2);
//     if(vector[0]<0){
//      x=-x;
//      y=-y;
//      z=-z;
//     }
//     Eigen::Quaterniond quaternion(w,x,y,z);
//     Eigen::Matrix3d R = quaternion.matrix();
//     Eigen::Vector3d Gb = (R.transpose()*G0);
//     Vector6d GF;
//     GF<<Gb(0),Gb(1),Gb(2),
//         Gb(2)*p(1)- Gb(1)*p(2),Gb(0)*p(2)-Gb(2)*p(0),Gb(1)*p(0)-Gb(0)*p(1);
//     return F-GF-F0;
// }

// 重力补偿
Vector6d ForceControl::calculateGravityCompensation(const std::vector<double>& q,Vector6d F) {
    Eigen::Matrix3d R;
    q2matrix(q,R);
    Eigen::Vector3d Gb = (R.transpose()*G0);
    Vector6d GF;
    GF<<Gb(0),Gb(1),Gb(2),
        Gb(2)*p(1)- Gb(1)*p(2),Gb(0)*p(2)-Gb(2)*p(0),Gb(1)*p(0)-Gb(0)*p(1);
    return F-GF-F0;
}

// 析构函数
ForceControl::~ForceControl(){
    delete pid;
    delete force;
}

