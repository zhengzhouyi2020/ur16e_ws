#include"publish_force_data/force_control_grinding.hpp"


void vector2matrix(const std::vector<double>& vector,Matrix<double,3,3>& TEB) {
    // TODO how to difine vector;
    double theta=sqrt(vector[0]*vector[0]+vector[1]*vector[1]+vector[2]*vector[2]);
    double kx=vector[0]/theta;
    double ky=vector[1]/theta;
    double kz=vector[2]/theta;
    double x=kx*sin(theta/2);
    double y=ky*sin(theta/2);
    double z=kz*sin(theta/2);
    double w=cos(theta/2);
    // if(vector[0]<0){
    //  x=-x;
    //  y=-y;
    //  z=-z;
    // }
    Eigen::Quaterniond quaternion(w,x,y,z);
    TEB = quaternion.matrix();

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "data");
    ros::NodeHandle nh;
    std::cout << "OK" << std::endl;
    // rtde connecting robot
    RTDEReceiveInterface rtde_receive("192.168.1.11");
    std::cout << "OK" << std::endl;
    // to write data
    char fileName[256]={0};
    std::ofstream outFile; //创建文件流对象
    time_t timep; //创建时间结构体,包含时分秒,用来作为文件名称
    time(&timep);
    std::string dir_package,dir_param_file;
    //获取包名的绝对地址
    dir_package = ros::package::getPath("publish_force_data");
    //以当前时间作为文件名
    strftime(fileName, sizeof(fileName), "/data/force_%m%d%H%M%S.csv",localtime(&timep) );
    dir_param_file = dir_package + fileName;
    try {
        outFile.open(dir_param_file,std::ios::out);//打开文件,进行读写
        outFile<< "timestamp"<< ","<<"HexForceX"<< ","<<"HexForceY"<< ","<<"HexForceZ"<< ","<<"HexTorqueX"<< ","<<"HexTorqueY"<< ","<<"HexTorqueZ"<< ","<<
                         "ContactFX"<< ","<<"ContactFY"<< ","<<"ContactFZ"<< ","<<"ContactRX"<< ","<<"ContactRY"<< ","<<"ContactRZ"<< ","<<
                         "TCP_pose0"<< ","<<"TCP_pose1"<< ","<<"TCP_pose2"<< ","<< "TCP_pose3"<< ","<<"TCP_pose4"<< ","<<"TCP_pose5"<< ","<<
                         "Angle0"<< ","<<"Angle1"<< ","<<"Angle2"<< ","<<"Angle3"<< ","<<"Angle4"<< ","<<"Angle5"<< 
                                std::endl;
        } catch (std::exception exception) {
            outFile.close();
            std::cout << exception.what() << std::endl;
    }
    //重力补偿参数
    Vector3d G0(-1.104356,0.693975,-11.756118);
    Vector3d p(0.020186,0.000221,0.043238);
    Vector6d F0(1.27712,-0.250479,1.625059,-0.084533,0.018943,0.0293957);
    FORCE* force = new FORCE();
    ForceControl forceControl(force,G0,p,F0);

    Vector6d HexForce;
    double timeStamp;

    while (ros::ok()) {

        forceControl.force->getForce(HexForce, timeStamp); // 得到当前的力数据和事件戳
      
        std::vector<double> actual_p = rtde_receive.getActualTCPPose();

        std::vector<double> rotateVector(actual_p.begin()+3,actual_p.end());
 //     Vector6d actual_Fz = forceControl.calculateGravityCompensation(rotateVector,HexForce); // 经过重力补偿后得到真实的接触力,用四元数计算总有点小问题
        Vector6d actual_Fz = forceControl.calculateGravityCompensation(rtde_receive.getActualQ(),HexForce); // 这里用正运动学计算旋转矩阵
        std::vector<double> actual_q = rtde_receive.getActualQ();
        

        // Matrix<double,4,4> TEB;
        // ur16e_Kinematic(actual_q,TEB);
        // std::cout << "TEB:" << TEB << std::endl; 
        // Matrix<double,3,3> TEB2;
        // vector2matrix(rotateVector,TEB2); 
        // std::cout << "TEB2:" << TEB2 << std::endl; 

     //   TEB=TEB.inverse(); // 不应该转置
        // 写入数据
        try{
            outFile << timeStamp<< ","<< HexForce[0]<< ","<< HexForce[1]<< ","<< HexForce[2]<< ","<<
                                HexForce[3]<< ","<< HexForce[4]<< ","<< HexForce[5]<< ","<<
                                actual_Fz[0]<< ","<< actual_Fz[1]<< ","<< actual_Fz[2]<< ","<< actual_Fz[3]<< ","<< actual_Fz[4]<< ","<< actual_Fz[5]<< ","<<
                                actual_p[0]<< ","<< actual_p[1]<< ","<< actual_p[2]<< ","<< actual_p[3]<< ","<< actual_p[4]<< ","<< actual_p[5]<< ","<<
                                actual_q[0]<< ","<< actual_q[1]<< ","<< actual_q[2]<< ","<< actual_q[3]<< ","<< actual_q[4]<< ","<< actual_q[5]<<
                                std::endl;
        } catch(std::exception exception) {
             outFile.close();
            std::cout << exception.what() << std::endl;
        }
    } 
    outFile.close();

}