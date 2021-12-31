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

#include "dobot/SetEndEffectorSuctionCup.h"

#include "object_detect/Center_msg.h"


#include <iostream>
 
using namespace std;

// 接收到订阅的消息后，会进入消息回调函数
void chatterCallback(const object_detect::Center_msg::ConstPtr& msg)
{
  // 将接收到的消息打印出来
  ROS_INFO("x1: [%f]", msg->data[0]);
  ROS_INFO("y1: [%f]", msg->data[1]);

}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "DobotClient");
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




    while (ros::ok()) {

/**
        client = n.serviceClient<dobot::SetEndEffectorSuctionCup>("/DobotServer/SetEndEffectorSuctionCup");
        dobot::SetEndEffectorSuctionCup sck1;   
        do{
            sck1.request.enableCtrl = 1; // When enableCtr == 1 the motor will operate. 
            sck1.request.suck = 1; // When suck == 1 the suction cup will suck.
            sck1.request.isQueued = true; // This command puts the request in the queue.
            ros::spinOnce(); 
        } while(0);

*/
        client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
        dobot::SetPTPCmd srv;

        // Initialization Robot pose
        do {
            srv.request.ptpMode = 1;
            srv.request.x = 195;
            srv.request.y = 0;
            srv.request.z = 80;
            srv.request.r = 0;
            client.call(srv);
            if (srv.response.result == 0) {
                break;
            }     
            ros::spinOnce();
            if (ros::ok() == false) {
                break;
            }
        } while (0);



        // 启动物体识别
        do {
            srv.request.ptpMode = 1;
            srv.request.x = 195;
            srv.request.y = 0;
            srv.request.z = 80;
            srv.request.r = 0;
            client.call(srv);
            if (srv.response.result == 0) {
                break;
            }     
            ros::spinOnce();
            if (ros::ok() == false) {
                break;
            }
        } while (0);




        // 机械臂运动到物体中心位置
        do {
            srv.request.ptpMode = 1;
            srv.request.x = 244.7;
            srv.request.y = -25;
            srv.request.z = -35;
            srv.request.r = 0;
            client.call(srv);
            if (srv.response.result == 0) {
                break;
            }     
            ros::spinOnce();
            if (ros::ok() == false) {
                break;
            }
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
            if (ros::ok() == false) {
                break;
            }
        } while(0);

        client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
        dobot::SetPTPCmd srv1;

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
            if (ros::ok() == false) {
                break;
            }
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


        //ros::spinOnce();
    return 0;
    }

}

