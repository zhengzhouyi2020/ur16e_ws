#include "HaptionRobot.h"

        VirHaptions::VirHaptions()
        {
        };


    int VirHaptions::HaptionInit()
    {
        M_VC = virtOpen("127.0.0.1#53210");  // Open connection

        if (M_VC == NULL)
	{
		fprintf(stderr, "Erreur dans virtOpen: %s\n",virtGetErrorMessage(virtGetErrorCode(NULL)));
		return -1;
	}



	//常见设置

	//virtSetIndexingMode(VC, INDEXING_ALL);
		//设置比例因子
	virtSetForceFactor(M_VC, 1.0f);
	virtSetSpeedFactor(M_VC, 1.0f);
	//virtSetTimeStep(M_VC, 0.003f);


	//float identity[7] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,1.0f};
	//virtSetBaseFrame(M_VC, identity); //修改虚拟机构基架的位置
	//virtSetObservationFrame(M_VC, identity)； //移动观察框

	//COMMAND_TYPE_NONE 无法移动
	//COMMAND_TYPE_IMPEDANCE 力/位置控制
	//COMMAND_TYPE_VIRTMECH  虚拟机构的位置/力控制
	//COMMAND_TYPE_ARTICULAR 关节位置控制
	//COMMAND_TYPE_ARTICULAR_IMPEDANCE 关节力控制
	//virtSetCommandType(VC, COMMAND_TYPE_VIRTMECH); //设置所需的控制模式

	

        //按1khz 需要测试是否可以调整频率
        timestep=0.001f;                                    //设置循环频率
        
        virtSetCommandType(M_VC, COMMAND_TYPE_IMPEDANCE);  //Choose control mode

        //virtSetTimeStep(M_VC, timestep);                          // Specify cycle time

        //virtAttachVO(M_VC, mass, inertia);                    // Attach to process

        virtSetPowerOn(M_VC, 1);                                        // Enable force-feedback

        HaptionGetData();

        HaptioninitData();

        STATE1 = false;
    }


    void VirHaptions::HaptionClose()
    {

            //此处需考虑加入判断是否有循环
            virtStopLoop(M_VC); 					// Stop update

            virtSetPowerOn(M_VC, 0);                                    // Disable force-feedback
            virtClose(M_VC);                                                        // Close connection
    }

    ////////////////////////////////////////////////////////////////

    void VirHaptions::HaptionGetData()
    {

		
		int state;

            virtGetPosition(M_VC, M_Data.Position.data());

            virtGetSpeed(M_VC, M_Data.Speed.data());

            virtGetForce(M_VC, M_Data.Force.data());

		virtGetArticularPosition(M_VC, M_Data.Position1.data());

		virtGetArticularSpeed(M_VC,M_Data.Speed1.data());

		virtGetButton(M_VC, 1, &state);
		
		M_Data.Buttonstate[0]= state;




    }

    void VirHaptions::HaptioninitData()
    {
            virtGetForce(M_VC, SendData.Force.data());

            virtGetPosition(M_VC, SendData.Position.data());

            virtGetSpeed(M_VC, SendData.Speed.data());
    }

        void VirHaptions::setfrequency(float num)
        {
                timestep=num;  
        }

/* **************** */
    //此处需要测试 1.是否需要加入循环 2.是否加入时间片轮询减小运算量
    void virtGetDataCallback(VirtContext VC,void *__this)
    {
            VirHaptions* _this = (VirHaptions*) __this;
            _this->HaptionGetData();
    };

    void VirHaptions::start_data_pth()
    {
            virtSetPeriodicFunction(M_VC,virtGetDataCallback , &timestep, (void*)this); // Setup callback
            virtStartLoop(M_VC);
    };

/* **************** */
    //此处需要测试 1.是否需要加入循环 2.是否加入时间片轮询减小运算量
    void virtSetPositionCallback(VirtContext VC,void *__this)
    {
            VirHaptions* _this = (VirHaptions*) __this;
             _this->HaptionGetData();
             _this->setposition();        
    };

    void VirHaptions::start_Pos_pth()
    {
            virtSetPeriodicFunction(M_VC,virtSetPositionCallback , &timestep, (void*)this); // Setup callback
            virtStartLoop(M_VC);
    };

/* **************** */
    //此处需要测试 1.是否需要加入循环 2.是否加入时间片轮询减小运算量
     void virtSetForceCallback(VirtContext VC,void *__this)
    {
            VirHaptions* _this = (VirHaptions*) __this;
             _this->HaptionGetData();
             _this->setforce();        
    };  
    void VirHaptions::start_Force_pth()
    {
            virtSetPeriodicFunction(M_VC,virtSetForceCallback , &timestep, (void*)this); // Setup callback
            virtStartLoop(M_VC);
    }

/* **************** */
    //此处需要测试 1.是否需要加入循环 2.是否加入时间片轮询减小运算量
        void virtSetSpeedCallback(VirtContext VC,void *__this)
    {
            VirHaptions* _this = (VirHaptions*) __this;
             _this->HaptionGetData();
             _this->setspeed();        
    };  
        void VirHaptions::start_Speed_pth()
        {
                virtSetPeriodicFunction(M_VC,virtSetSpeedCallback , &timestep, (void*)this); // Setup callback
                virtStartLoop(M_VC);
        }
