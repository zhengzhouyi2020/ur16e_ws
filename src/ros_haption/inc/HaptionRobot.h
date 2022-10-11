#include <iostream>
#include <unistd.h>
#include <pthread.h>
#include <array>

#include "VirtuoseAPI.h"


struct HaptionData
{
    std::array<float, 7>  Position{};  //virtGetPosition – Read the current position of the VIRTUOSE, or of the object attached to the VIRTUOSE.

    std::array<float,6>  Speed{};  //virtGetSpeed(VirtContext VC, float *speed_p_env_r_ps); 

    std::array<float,6>  Force{}; //virtGetForce



    std::array<float, 6>  Position1{};  //virtGetArticularPosition  //关节位置

    std::array<float, 6>  Speed1{};	//virtGetArticularSpeed   //关节速度

    std::array<int,3> Buttonstate{};//virtGetButton(VirtContext VC, int button_number, int *state);//仅支持三个按钮，它们的相应的指标取决于产品



    //测试关于button_number的命名，以及相应的状态
    //virtGetButton(VirtContext VC, int button_number, int *state);

    // Position: meters (m)
    // Angles: radians (rad)
    // Speed: meters per second (m/s)
    // Force: Newtons (N)
    // Torque: Newton-meters (Nm)
    // Mass: kilograms (kg)
    // Inertia: kilograms per square meters (kg/m2)
    // Translation stiffness: Newtons per meter (N/m)
    // Rotation stiffness: Newton-meters per radian (Nm/rad)
    // Translation damping: Newtons per meter per second (N/(m/s))
    // Rotation damping: Newton-meters per radian per second (Nm/(rad/s))

// All integers of standard "int" type
// All floating-point of standard "float" type
// All vectors and matrices of standard "float []" type
// A position is a float[7]
// A speed is a float[6]
// 3 values for the position (X, Y, Z)
// 4 values for the orientation as a quaternion (Qx, Qy, Qz, Qw)
// 3 values for the velocity (Vx, Vy, Vy)
// 3 values for the rotation speed (Rx, Ry, Rz)
// A force is a float[6]
// 3 values for the force itself (Fx, Fy, Fz)
// 3 values for the torques (Tx, Ty, Tz)

};

class VirHaptions 
{
    public:
        VirHaptions();

    public:

    //VirtContext virtOpen(const char *nom);
    ////////////////////////////virtSetArticularForce(VirtContext VC, float *force);
    //virtSetForce(VirtContext VC, float *force);
    ///////////////////////////virtSetArticularPosition(VirtContext VC, float *pos);
    //virtSetPosition(VirtContext VC, float *pos);
    ///////////////////////////virtSetArticularSpeed(VirtContext VC, float *speed);
    //virtSetSpeed(VC, speed); // Write device speed

    int HaptionInit();

    void HaptionClose();

    void HaptionGetData();

    void HaptioninitData();

    void setfrequency(float num);


/* **************** */
    void start_data_pth();

/* **************** */
    void setposition()
    {
            virtSetPosition(M_VC, SendData.Position.data());    
    };
    void start_Pos_pth();

/* **************** */
    void setspeed()
    {
             virtSetSpeed(M_VC, SendData.Speed.data());    
    };
        void start_Speed_pth();

/* **************** */
    void setforce()
    {
            virtSetForce(M_VC, SendData.Force.data());    
    };
    void start_Force_pth();



public:

        bool STATE1;

        float timestep;

        VirtContext M_VC;

        HaptionData M_Data;

        HaptionData SendData;


};
