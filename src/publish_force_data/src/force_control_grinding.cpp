#include"publish_force_data/force_control_grinding.hpp"



const std::string data_path = "/home/ur16e/hellows/src/publish_force_data/data/force_0107095214.csv";

int main(int argc, char** argv) {
    ros::init(argc, argv, "force");
    ros::NodeHandle nh;

    // rtde connecting robot
    RTDEReceiveInterface rtde_receive("192.168.1.11");
    RTDEControlInterface rtde_control("192.168.1.11");
    std::cout <<"Connected OK!"<<std::endl;

    // to write data
    char fileName[256]={0};
    std::ofstream outFile; //创建文件流对象
    time_t timep; //创建时间结构体,包含时分秒,用来作为文件名称
    time(&timep);
    std::string dir_package,dir_param_file;
    //获取包名的绝对地址
    dir_package = ros::package::getPath("publish_force_data");
    //以当前时间作为文件名
    strftime(fileName, sizeof(fileName), "/data/position_%m%d%H%M%S.csv",localtime(&timep) );
    dir_param_file = dir_package + fileName;
    try {
        outFile.open(dir_param_file,std::ios::out);//打开文件,进行读写
        outFile<< "timestamp"<< ","<<"HexForceX"<< ","<<"HexForceY"<< ","<<"HexForceZ"<< ","<<"HexTorqueX"<< ","<<"HexTorqueY"<< ","<<"HexTorqueZ"<< ","<<
                         "ContactFX"<< ","<<"ContactFY"<< ","<<"ContactFZ"<< ","<<"ContactRX"<< ","<<"ContactRY"<< ","<<"ContactRZ"<< ","<<
                         "TCP_pose0"<< ","<<"TCP_pose1"<< ","<<"TCP_pose2"<< ","<< "TCP_pose3"<< ","<<"TCP_pose4"<< ","<<"TCP_pose5"<< ","<<
                         "Angle0"<< ",G0,p,F0"<<"Angle1"<< ","<<"Angle2"<< ","<<"Angle3"<< ","<<"Angle4"<< ","<<"Angle5"<< 
                                std::endl;
        } catch (std::exception exception) {
            outFile.close();
            std::cout << exception.what() << std::endl;
    }
   
    // TODO 
    // to read data
    std::vector<std::vector<double>> robot_data; // 所有的数据
    getRecodeData(data_path, robot_data); // 获取所有的数据,保存在robot_data


    double Hex_Z_Force_data; // Z方向的力
    std::vector<double> tcp_position_data(6); // tcp工具的位置

    std::copy(robot_data[0].begin() + 13,robot_data[0].begin() + 19,tcp_position_data.begin()); //区间拷贝
    rtde_control.moveL({-0.265236,-0.125685,0.688036,-0.0844924,-2.68991,0.323995},0.08,0.005);  // 运动到起始点
      
    rtde_control.moveL(tcp_position_data,0.08,0.005); // 运动到第一个点
 
    // 重力补偿参数
    Vector3d G0(-1.104356,0.693975,-11.756118);
    Vector3d p(0.020186,0.000221,0.043238);
    Vector6d F0(1.27712,-0.250479,1.625059,-0.084533,0.018943,0.0293957);
    //"""param:期望力,kp,ki,kd,dt"""
    PID* pid = new PID(1, 0.0001, 0.00015, 0.001,0.01); 
    //"""采用频率：500 间隔：5  滤波频率：1.5"""
    FORCE* force = new FORCE(500,5,10,6);
    ForceControl forceControl(pid,force,G0,p,F0);

    Vector6d HexForce;
    double timeStamp;
    std::cout<<"init ok!"<<std::endl;
    for (int i = 0; i < robot_data.size(); ++i) {

        Hex_Z_Force_data = robot_data[i][9]; // 得到轨迹数据的z向力
        std::copy(robot_data[i].begin() + 13,robot_data[i].begin() + 19,tcp_position_data.begin()); //区间拷贝

        forceControl.force->getForce(HexForce, timeStamp); // 得到当前的力数据和事件戳
        std::vector<double> actual_p = rtde_receive.getActualTCPPose();

 //     std::vector<double> rotateVector(actual_p.begin()+3,actual_p.end());
        Vector6d actual_Fz = forceControl.calculateGravityCompensation(rtde_receive.getActualQ(),HexForce); // 经过重力补偿后得到真实的接触力
        std::vector<double> actual_q = rtde_receive.getActualQ();

        Matrix<double,4,4> TEB;
        ur16e_Kinematic(actual_q,TEB);
     //   TEB=TEB.inverse(); // 不应该转置

        double Xe = forceControl.pid->pid_force_position(Hex_Z_Force_data,actual_Fz[2]); //使用末端的z向力做pid控制
        Vector3d XeTemp;
        XeTemp << 0, 0, Xe;
        XeTemp = TEB.block(0,0,3,3) * XeTemp;  // 计算偏置
        
        std::vector<double> desired_position(tcp_position_data);// 得到偏置的位置
        
        desired_position[0] += XeTemp[0];
        desired_position[1] += XeTemp[1];
        desired_position[2] += XeTemp[2];
        std::cout<<"force:"<< actual_Fz[2] << "," <<"data:"<< XeTemp << "," << std::endl;
        rtde_control.moveL(desired_position); // 运动到目标位置
        std::cout << tcp_position_data[0] << tcp_position_data[1] << tcp_position_data[2] << tcp_position_data[3] << tcp_position_data[4] << tcp_position_data[5]<<std::endl;
        std::cout << desired_position[0] << desired_position[1] << desired_position[2] << desired_position[3] << desired_position[4] << desired_position[5]<<std::endl;
        // 写入数据
        try{
            outFile << timeStamp<<","<< HexForce[0]<< ","<< HexForce[1]<< ","<< HexForce[2]<< ","<<
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