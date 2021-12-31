#include <ros/ros.h>
#include "ros/console.h"

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
#include "dobot/GetQueuedCmdCurrentIndex.h"
#include "dobot/DobotAction.h"

#include <actionlib/server/simple_action_server.h>
#include "object_detect/object_result.h"

#include "std_msgs/Int64.h"
#include "dobot/DobotActionFeedback.h"

#include <iostream>
#include <unistd.h>
#include <cstdlib>

using namespace std;

typedef actionlib::SimpleActionServer<dobot::DobotAction> Server;

double detect_x;
double detect_y;
double detect_z;



/*  ********************************************************************* */
/*      －－－－－－     执行机械臂初始化动作    －－－－－   */
// 收到action的goal后调用的回调函数
void execute_pick_up(const dobot::DobotGoalConstPtr& goal, Server* as)
{
    ros::Rate r(1);
    dobot::DobotFeedback feedback;

    ros::NodeHandle n;
    ros::ServiceClient client;
    ros::NodeHandle node("~");

	ros::Publisher object_pub = n.advertise<std_msgs::Int64>("robotInitDone", 1);
	std_msgs::Int64 tag;

// 设置参数
    node.param("detect_x", detect_x, 230.0);  // 识别物体时机械臂的x坐标(笛卡尔坐标系)
    node.param("detect_y", detect_y, 0.0);  //　识别物体时机械臂的y坐标(笛卡尔坐标系)
    node.param("detect_z", detect_z, 60.0);    //　识别物体时机械臂的z坐标(笛卡尔坐标系)


// 清除指令队列
    client = n.serviceClient<dobot::SetQueuedCmdClear>("/DobotServer/SetQueuedCmdClear");
    dobot::SetQueuedCmdClear srv2;
    client.call(srv2);



// 创建PTP客户端
    client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
    dobot::SetPTPCmd srv;

        // 初始化机械臂的位姿
        do {
            srv.request.ptpMode = 1;
            srv.request.x = detect_x;
            srv.request.y = detect_y;
            srv.request.z = detect_z;
            srv.request.r = 0;
            client.call(srv);
            if (srv.response.result == 0) {
                break;
            }     
            ros::spinOnce();

        } while (0);

//  获取当前的指令索引
    client = n.serviceClient<dobot::GetQueuedCmdCurrentIndex>("/DobotServer/GetQueuedCmdCurrentIndex");
    dobot::GetQueuedCmdCurrentIndex srv8;
    client.call(srv8);

//  等待所有的动作完成
while(srv8.response.queuedCmdIndex < srv.response.queuedCmdIndex)
    if(client.call(srv8))
    {
        //cout << "wait!" << endl;
    }
    //sleep(3);
    tag.data = 8;
    object_pub.publish(tag);


	// 当action完成后，向客户端返回结果
    ROS_INFO("Manipulator arm %d finish working.", goal->dobot_pick_id);
    as->setSucceeded();

}

/*  ********************************************************************* */
/*                       －－－－－－     主函数    －－－－－                    */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "dobot_init_server");
    ros::NodeHandle n;
    ros::ServiceClient client;


/*                   －－－－－－     设置机械臂参数    －－－－－                  */

    // 设置命令超时时间
    client = n.serviceClient<dobot::SetCmdTimeout>("/DobotServer/SetCmdTimeout");
    dobot::SetCmdTimeout srv1;
    srv1.request.timeout = 3000;
    if (client.call(srv1) == false) {
        ROS_ERROR("Failed to call SetCmdTimeout. Maybe DobotServer isn't started yet!");
        return -1;
    }

    // 清除指令队列
    client = n.serviceClient<dobot::SetQueuedCmdClear>("/DobotServer/SetQueuedCmdClear");
    dobot::SetQueuedCmdClear srv2;
    client.call(srv2);

    // 开始运行指令队列
    client = n.serviceClient<dobot::SetQueuedCmdStartExec>("/DobotServer/SetQueuedCmdStartExec");
    dobot::SetQueuedCmdStartExec srv3;
    client.call(srv3);


    // 获取设备的版本信息
    client = n.serviceClient<dobot::GetDeviceVersion>("/DobotServer/GetDeviceVersion");
    dobot::GetDeviceVersion srv4;
    client.call(srv4);
    if (srv4.response.result == 0) {
        ROS_INFO("Device version:%d.%d.%d", srv4.response.majorVersion, srv4.response.minorVersion, srv4.response.revision);
    } else {
        ROS_ERROR("Failed to get device version information!");
    }


    // 设置末端执行器参数
    client = n.serviceClient<dobot::SetEndEffectorParams>("/DobotServer/SetEndEffectorParams");
    dobot::SetEndEffectorParams srv5;
    srv5.request.xBias = 70;
    srv5.request.yBias = 0;
    srv5.request.zBias = 0;
    client.call(srv5);


    // 设置PTP模式下的关节参数
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

    // 设置PTP的坐标参数
    do {
        client = n.serviceClient<dobot::SetPTPCoordinateParams>("/DobotServer/SetPTPCoordinateParams");
        dobot::SetPTPCoordinateParams srv;

        srv.request.xyzVelocity = 100;
        srv.request.xyzAcceleration = 100;
        srv.request.rVelocity = 100;
        srv.request.rAcceleration = 100;
        client.call(srv);
    } while (0);

    // 设置PTP的jump模式下的参数
    do {
        client = n.serviceClient<dobot::SetPTPJumpParams>("/DobotServer/SetPTPJumpParams");
        dobot::SetPTPJumpParams srv;

        srv.request.jumpHeight = 20;
        srv.request.zLimit = 200;
        client.call(srv);
    } while (0);

    // 设置PTP常用的参数
    do {
        client = n.serviceClient<dobot::SetPTPCommonParams>("/DobotServer/SetPTPCommonParams");
        dobot::SetPTPCommonParams srv;

        srv.request.velocityRatio = 50;
        srv.request.accelerationRatio = 50;
        client.call(srv);
    } while (0);

/*  ********************************************************************* */
    // 定义一个抓取物体的服务器
    Server server(n, "initialization", boost::bind(&execute_pick_up, _1, &server), false);
      
    // 服务器开始运行
    server.start();


    ros::spin();

    return 0;
}
