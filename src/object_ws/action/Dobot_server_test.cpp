#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "dobot/DobotAction.h"

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

#include "dobot/SetEndEffectorSuctionCup.h"

#include "object_detect/Center_msg.h"

#include "dobot/GetQueuedCmdCurrentIndex.h"
#include <iostream>
 
using namespace std;




typedef actionlib::SimpleActionServer<dobot::DobotAction> Server;



// 收到action的goal后调用的回调函数
void execute(const dobot::DobotGoalConstPtr& goal, Server* as)
{
    ros::Rate r(1);
    dobot::DobotFeedback feedback;

    ROS_INFO("Dishwasher %d is working.", goal->dobot_pick_id);

    ros::NodeHandle n;

    ros::ServiceClient client;

// Clear the command queue
    client = n.serviceClient<dobot::SetQueuedCmdClear>("/DobotServer/SetQueuedCmdClear");
    dobot::SetQueuedCmdClear srv2;
    client.call(srv2);

    client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");

    dobot::SetPTPCmd srv;


        // Initialization Robot pose
        do {
            srv.request.ptpMode = 1;
            srv.request.x = 195;
            srv.request.y = 195;
            srv.request.z = 80;
            srv.request.r = 0;
            client.call(srv);
            if (srv.response.result == 0) {
                break;
            }     
            ros::spinOnce();

        } while (0);

        do {
            srv.request.ptpMode = 1;
            srv.request.x = 0;
            srv.request.y = 217;
            srv.request.z = 80;
            srv.request.r = 0;
            client.call(srv);
            if (srv.response.result == 0) {
                break;
            }     
            ros::spinOnce();

        } while (0);


        // ************************** pick up **************************
        client = n.serviceClient<dobot::SetEndEffectorSuctionCup>("/DobotServer/SetEndEffectorSuctionCup");
        dobot::SetEndEffectorSuctionCup sck2;
        // pick up 
        do{
            sck2.request.enableCtrl = 1;
            sck2.request.suck = 1; // Enable suction 
            sck2.request.isQueued = true;
            client.call(sck2);
 
            ros::spinOnce(); //spinOnce() will execute any commands in the queue.

        } while(0);

        client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
        dobot::SetPTPCmd srv1;
        // 防止撞倒其他方块
        do {
            srv.request.ptpMode = 1;
            srv.request.x = 0;
            srv.request.y = 217;
            srv.request.z = 80;
            srv.request.r = 0;
            client.call(srv);
            if (srv.response.result == 0) {
                break;
            }     
            ros::spinOnce();
        } while (0);

        // 把物体运动初始化位置
        do {
            srv1.request.ptpMode = 1;
            srv1.request.x = 195;
            srv1.request.y = 0;
            srv1.request.z = 80;
            srv1.request.r = 0;
            client.call(srv1);  
             if (srv1.response.result == 0) {

		break;                
            }  
            ros::spinOnce();
        } while (0);


        // ************************** drop off **************************

        client = n.serviceClient<dobot::SetEndEffectorSuctionCup>("/DobotServer/SetEndEffectorSuctionCup");
        dobot::SetEndEffectorSuctionCup sck3;
        //drop off 
        do{    
            sck3.request.enableCtrl = 1;
            sck3.request.suck = 0;
            sck3.request.isQueued = true;
            client.call(sck3); 
            ros::spinOnce(); //spinOnce() will execute any commands in the queue.
            if (ros::ok() == false) {
                break;
            }
        }while (0);

/*          GJXS            */

    client = n.serviceClient<dobot::GetQueuedCmdCurrentIndex>("/DobotServer/GetQueuedCmdCurrentIndex");
    dobot::GetQueuedCmdCurrentIndex srv9;
    client.call(srv9);

while(srv9.response.queuedCmdIndex < sck3.response.queuedCmdIndex)
    if(client.call(srv9))
    {
        //cout << "wait!" << endl;
    }

    feedback.percent_complete = 2;
    as->publishFeedback(feedback);

	// 当action完成后，向客户端返回结果
    ROS_INFO("Dishwasher %d finish working.", goal->dobot_pick_id);
    as->setSucceeded();
}



// 主函数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "do_dishes_server");
    ros::NodeHandle n;

    ros::ServiceClient client;

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

    client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");

    do{
        client = n.serviceClient<dobot::SetHOMEParams>("/DobotServer/SetHOMEParams");
        dobot::SetHOMEParams home;
        home.request.x = 200;
        home.request.y = 0;
        home.request.z = 0;
        home.request.r = 0;
        home.request.isQueued = 1;
        client.call(home); 
 
    } while(0);
 
    do{
        client = n.serviceClient<dobot::SetHOMECmd>("/DobotServer/SetHOMECmd");
        dobot::SetHOMECmd home1;
        client.call(home1);
    } while(0);

/* 
    ros::ServiceClient client;
    dobot::SetPTPCmd srv6;


        // Initialization Robot pose
        do {
            srv6.request.ptpMode = 1;
            srv6.request.x = 195;
            srv6.request.y = 195;
            srv6.request.z = 80;
            srv6.request.r = 0;
            client.call(srv6);
            if (srv6.response.result == 0) {
                break;
            }     
            ros::spinOnce();
            if (ros::ok() == false) {
                break;
            }
        } while (0);
*/

	// 定义一个服务器
    Server server(n, "do_dishes", boost::bind(&execute, _1, &server), false);
	
	// 服务器开始运行
    server.start();

    ros::spin();

    return 0;
}
