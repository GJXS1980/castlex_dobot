#include "ros/ros.h"
#include "ros/console.h"
#include <unistd.h>
#include <cstdlib>

#include "std_msgs/String.h"
#include "dobot/SetCmdTimeout.h"
#include "dobot/SetQueuedCmdClear.h"
#include "dobot/SetQueuedCmdStartExec.h"
#include "dobot/SetQueuedCmdForceStopExec.h"
#include "dobot/GetDeviceVersion.h"

#include "dobot/SetEndEffectorParams.h"
#include "dobot/SetPTPJointParams.h"
#include "dobot/SetPTPCoordinateParams.h"
#include "dobot/SetPTPJumpParams.h"
#include "dobot/SetPTPCommonParams.h"
#include "dobot/SetPTPCmd.h"

#include "dobot/SetHOMEParams.h"
#include "dobot/SetHOMECmd.h"
#include "dobot/GetQueuedCmdCurrentIndex.h"
#include "dobot/DobotAction.h"

#include <iostream>
 
using namespace std;

/*定义机械臂初始化坐标*/
double init_x, init_y, init_z;

/*定义位置坐标*/
double step1_x, step1_y, step1_z; /*第1个动作*/
double step2_x, step2_y, step2_z;	/*第2个动作*/
double step3_x, step3_y, step3_z;	/*第3个动作*/
double step4_x, step4_y, step4_z;	/*第4个动作*/
double step5_x, step5_y, step5_z;	/*第5个动作*/
double step6_x, step6_y, step6_z;	/*第6个动作*/



int main(int argc, char **argv)
{

    ros::init(argc, argv, "DobotClient");
    ros::NodeHandle n;
    ros::NodeHandle node("~");

    ros::ServiceClient client;

// 设置机械臂初始化坐标参数
    /*向上*/
    node.param("init_x", init_x, 200.0);  
    node.param("init_y", init_y, 0.0);  
    node.param("init_z", init_z, 50.0);   


// 设置坐标参数
    /*向上*/
    node.param("step1_x", step1_x, 230.0);  
    node.param("step1_y", step1_y, 0.0);  
    node.param("step1_z", step1_z, 90.0);    

    /*向下*/
    node.param("step2_x", step2_x, 200.0);  
    node.param("step2_y", step2_y, 0.0);  
    node.param("step2_z", step2_z, 40.0);    

    /*向左*/
    node.param("step3_x", step3_x, 200.0);  
    node.param("step3_y", step3_y, 50.0);  
    node.param("step3_z", step3_z, 40.0);   

    /*向右*/
    node.param("step4_x", step4_x, 200.0);  
    node.param("step4_y", step4_y, -50.0);  
    node.param("step4_z", step4_z, 40.0);   

    /*向前*/
    node.param("step5_x", step5_x, 210.0);  
    node.param("step5_y", step5_y, -50.0);  
    node.param("step5_z", step5_z, 40.0);  

    /*向后*/
    node.param("step6_x", step6_x, 190.0);  
    node.param("step6_y", step6_y, -50.0);  
    node.param("step6_z", step6_z, 40.0);  


    // SetCmdTimeout
    client = n.serviceClient<dobot::SetCmdTimeout>("/DobotServer/SetCmdTimeout");
    dobot::SetCmdTimeout srv1;
    srv1.request.timeout = 3000;
    if (client.call(srv1) == false) {
        ROS_ERROR("Failed to call SetCmdTimeout. Maybe DobotServer isn't started yet!");
        return -1;
    }

    // Clear the command queue
    client = n.serviceClient<dobot::SetQueuedCmdClear>("/DobotServer/SetQueuedCmdClear");
    dobot::SetQueuedCmdClear srv2;
    client.call(srv2);

    // Start running the command queue
    client = n.serviceClient<dobot::SetQueuedCmdStartExec>("/DobotServer/SetQueuedCmdStartExec");
    dobot::SetQueuedCmdStartExec srv3;
    client.call(srv3);

    // Get device version information
    client = n.serviceClient<dobot::GetDeviceVersion>("/DobotServer/GetDeviceVersion");
    dobot::GetDeviceVersion srv4;
    client.call(srv4);
    if (srv4.response.result == 0) {
        ROS_INFO("Device version:%d.%d.%d", srv4.response.majorVersion, srv4.response.minorVersion, srv4.response.revision);
    } else {
        ROS_ERROR("Failed to get device version information!");
    }

    // Set end effector parameters
    client = n.serviceClient<dobot::SetEndEffectorParams>("/DobotServer/SetEndEffectorParams");
    dobot::SetEndEffectorParams srv5;
    srv5.request.xBias = 70;
    srv5.request.yBias = 0;
    srv5.request.zBias = 0;
    client.call(srv5);

    // Set PTP joint parameters
    do {
        client = n.serviceClient<dobot::SetPTPJointParams>("/DobotServer/SetPTPJointParams");
        dobot::SetPTPJointParams srv;

        for (int i = 0; i < 4; i++) {
            srv.request.velocity.push_back(100);
        }
        for (int i = 0; i < 4; i++) {
            srv.request.acceleration.push_back(100);
        }
        client.call(srv);
    } while (0);

    // Set PTP coordinate parameters
    do {
        client = n.serviceClient<dobot::SetPTPCoordinateParams>("/DobotServer/SetPTPCoordinateParams");
        dobot::SetPTPCoordinateParams srv;

        srv.request.xyzVelocity = 100;
        srv.request.xyzAcceleration = 100;
        srv.request.rVelocity = 100;
        srv.request.rAcceleration = 100;
        client.call(srv);
    } while (0);

    // Set PTP jump parameters
    do {
        client = n.serviceClient<dobot::SetPTPJumpParams>("/DobotServer/SetPTPJumpParams");
        dobot::SetPTPJumpParams srv;

        srv.request.jumpHeight = 20;
        srv.request.zLimit = 200;
        client.call(srv);
    } while (0);

    // Set PTP common parameters
    do {
        client = n.serviceClient<dobot::SetPTPCommonParams>("/DobotServer/SetPTPCommonParams");
        dobot::SetPTPCommonParams srv;

        srv.request.velocityRatio = 50;
        srv.request.accelerationRatio = 50;
        client.call(srv);
    } while (0);



/*       -----------------     机械臂归零    ---------------------     */

//  执行机械臂归零操作
    do{
        client = n.serviceClient<dobot::SetHOMEParams>("/DobotServer/SetHOMEParams");
        dobot::SetHOMEParams home;
        home.request.x = 200;
        home.request.y = 0;
        home.request.z = 70;
        home.request.r = 0;
        home.request.isQueued = 1;
        client.call(home); 
 
    } while(0);

    do{
        client = n.serviceClient<dobot::SetHOMECmd>("/DobotServer/SetHOMECmd");
        dobot::SetHOMECmd home1;
        client.call(home1);
    } while(0);


// 初始化机械臂的位姿
        client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
        dobot::SetPTPCmd srv;
        do {
            srv.request.ptpMode = 1;
            srv.request.x = init_x;
            srv.request.y = init_y;
            srv.request.z = init_z;
            srv.request.r = 0;
            client.call(srv);
            if (srv.response.result == 0) {
                break;
            }     
            ros::spinOnce();

        } while (0);

	//  获取当前的指令索引
    client = n.serviceClient<dobot::GetQueuedCmdCurrentIndex>("/DobotServer/GetQueuedCmdCurrentIndex");
    dobot::GetQueuedCmdCurrentIndex srv6;
    client.call(srv6);

	//  等待所有的动作完成
	while(srv6.response.queuedCmdIndex < srv.response.queuedCmdIndex)
    	if(client.call(srv6))
    	{
        	//cout << "wait!" << endl;
    	}



/*   -----------------------    机械臂第1个动作运动    -----------------------       */

        client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
        dobot::SetPTPCmd srv7;

        // Initialization Robot pose
        do {
            srv7.request.ptpMode = 1;
            srv7.request.x = step1_x;
            srv7.request.y = step1_y;
            srv7.request.z = step1_z;
            srv7.request.r = 0;
            client.call(srv7);
            if (srv7.response.result == 0) {
                break;
            }     
            ros::spinOnce();
            if (ros::ok() == false) {
                break;
            }
        } while (0);

	//  获取当前的指令索引
    client = n.serviceClient<dobot::GetQueuedCmdCurrentIndex>("/DobotServer/GetQueuedCmdCurrentIndex");
    dobot::GetQueuedCmdCurrentIndex srv8;
    client.call(srv8);

	//  等待所有的动作完成
	while(srv8.response.queuedCmdIndex < srv7.response.queuedCmdIndex)
    	if(client.call(srv8))
    	{
        	//cout << "wait!" << endl;
    	}

/*  ---------------------   机械臂第2个动作运动  -----------------------------  */
        client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
        dobot::SetPTPCmd srv9;

        // Initialization Robot pose
        do {
            srv9.request.ptpMode = 1;
            srv9.request.x = step2_x;
            srv9.request.y = step2_y;
            srv9.request.z = step2_z;
            srv9.request.r = 0;
            client.call(srv7);
            if (srv9.response.result == 0) {
                break;
            }     
            ros::spinOnce();
            if (ros::ok() == false) {
                break;
            }
        } while (0);

	//  获取当前的指令索引
    client = n.serviceClient<dobot::GetQueuedCmdCurrentIndex>("/DobotServer/GetQueuedCmdCurrentIndex");
    dobot::GetQueuedCmdCurrentIndex srv10;
    client.call(srv10);

	//  等待所有的动作完成
	while(srv10.response.queuedCmdIndex < srv9.response.queuedCmdIndex)
    	if(client.call(srv10))
    	{
        	//cout << "wait!" << endl;
    	}


/*   --------------------  机械臂第3个动作运动  -------------------------   */
        client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
        dobot::SetPTPCmd srv11;

        // Initialization Robot pose
        do {
            srv11.request.ptpMode = 1;
            srv11.request.x = step3_x;
            srv11.request.y = step3_y;
            srv11.request.z = step3_z;
            srv11.request.r = 0;
            client.call(srv11);
            if (srv11.response.result == 0) {
                break;
            }     
            ros::spinOnce();
            if (ros::ok() == false) {
                break;
            }
        } while (0);

	//  获取当前的指令索引
    client = n.serviceClient<dobot::GetQueuedCmdCurrentIndex>("/DobotServer/GetQueuedCmdCurrentIndex");
    dobot::GetQueuedCmdCurrentIndex srv12;
    client.call(srv12);

	//  等待所有的动作完成
	while(srv12.response.queuedCmdIndex < srv11.response.queuedCmdIndex)
    	if(client.call(srv12))
    	{
        	//cout << "wait!" << endl;
    	}


/*  --------------------  机械臂第4个动作运动  --------------------------- */
        client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
        dobot::SetPTPCmd srv13;

        // Initialization Robot pose
        do {
            srv13.request.ptpMode = 1;
            srv13.request.x = step4_x;
            srv13.request.y = step4_y;
            srv13.request.z = step4_z;
            srv13.request.r = 0;
            client.call(srv13);
            if (srv13.response.result == 0) {
                break;
            }     
            ros::spinOnce();
            if (ros::ok() == false) {
                break;
            }
        } while (0);

	//  获取当前的指令索引
    client = n.serviceClient<dobot::GetQueuedCmdCurrentIndex>("/DobotServer/GetQueuedCmdCurrentIndex");
    dobot::GetQueuedCmdCurrentIndex srv14;
    client.call(srv14);

	//  等待所有的动作完成
	while(srv14.response.queuedCmdIndex < srv13.response.queuedCmdIndex)
    	if(client.call(srv14))
    	{
        	//cout << "wait!" << endl;
    	}

/*  --------------------  机械臂第5个动作运动  --------------------------- */
        client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
        dobot::SetPTPCmd srv15;

        // Initialization Robot pose
        do {
            srv15.request.ptpMode = 1;
            srv15.request.x = step5_x;
            srv15.request.y = step5_y;
            srv15.request.z = step5_z;
            srv15.request.r = 0;
            client.call(srv15);
            if (srv15.response.result == 0) {
                break;
            }     
            ros::spinOnce();
            if (ros::ok() == false) {
                break;
            }
        } while (0);

	//  获取当前的指令索引
    client = n.serviceClient<dobot::GetQueuedCmdCurrentIndex>("/DobotServer/GetQueuedCmdCurrentIndex");
    dobot::GetQueuedCmdCurrentIndex srv16;
    client.call(srv16);

	//  等待所有的动作完成
	while(srv16.response.queuedCmdIndex < srv15.response.queuedCmdIndex)
    	if(client.call(srv16))
    	{
        	//cout << "wait!" << endl;
    	}

/*  --------------------  机械臂第6个动作运动  --------------------------- */
        client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
        dobot::SetPTPCmd srv17;

        // Initialization Robot pose
        do {
            srv17.request.ptpMode = 1;
            srv17.request.x = step6_x;
            srv17.request.y = step6_y;
            srv17.request.z = step6_z;
            srv17.request.r = 0;
            client.call(srv17);
            if (srv17.response.result == 0) {
                break;
            }     
            ros::spinOnce();
            if (ros::ok() == false) {
                break;
            }
        } while (0);

	//  获取当前的指令索引
    client = n.serviceClient<dobot::GetQueuedCmdCurrentIndex>("/DobotServer/GetQueuedCmdCurrentIndex");
    dobot::GetQueuedCmdCurrentIndex srv18;
    client.call(srv18);

	//  等待所有的动作完成
	while(srv18.response.queuedCmdIndex < srv17.response.queuedCmdIndex)
    	if(client.call(srv18))
    	{
        	//cout << "wait!" << endl;
    	}


    return 0;
    

}

