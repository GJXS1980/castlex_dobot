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
#include "object_detect/object_result_msg.h"

#include "std_msgs/Int64.h"

#include <iostream>
#include <unistd.h>
#include <cstdlib>

using namespace std;


typedef actionlib::SimpleActionServer<dobot::DobotAction> Server;

double side_length;
double CamToSucker;
double suck_z;
double CamToSuc_deviation;
double detect_x;
double detect_y;

/*  ********************************************************************* */
/*      －－－－－－     创建物体识别中心坐标返回的类    －－－－－   */
class Listener
{
public:
  int count = 0;
  float x1, y1, ss;
public:
  void callback(const object_detect::object_result_msg::ConstPtr& msg);
  int print_datax()
  {

  return x1;
  }
  int print_datay()
  {

  return y1;
  }
  int print_datas()
  {

  return ss;
  }
};

void Listener::callback(const object_detect::object_result_msg::ConstPtr& msg)
{
  x1 = msg->data[0];
  y1 = msg->data[1];
  ss = msg->data[2];


  print_datax();
  print_datay();
  print_datas();

  ++count;
}



/*  ********************************************************************* */
/*      －－－－－－     执行识别抓取物体动作    －－－－－   */
// 收到action的goal后调用的回调函数
void execute_pick_up(const dobot::DobotGoalConstPtr& goal, Server* as)
{
    float sx, sy, ss, x, y, s;
    std_msgs::Int64 tag;

    ros::Rate r(1);
    dobot::DobotFeedback feedback;

    ROS_INFO("Manipulator arm %d is working.", goal->dobot_pick_id);

    ros::NodeHandle n;

    ros::NodeHandle node("~");


    ros::ServiceClient client;


    ros::Publisher action_pub = n.advertise<std_msgs::Int64>("grasp_success", 1000);

// 设置参数

    node.param("side_length", side_length, 34.45);	// 方块的边长
    node.param("CamToSucker", CamToSucker, 35.58);	//　相机到吸盘的距离（x方向的偏差）
    node.param("suck_z", suck_z, -50.0);	//　吸盘吸取的高度
    node.param("CamToSuc_deviation", CamToSuc_deviation, 10.0);	//　相机与吸盘在y方向的偏差
    node.param("detect_x", detect_x, 230.0);    // 识别物体时机械臂的x坐标(笛卡尔坐标系)
    node.param("detect_y", detect_y, 0.0);    //　识别物体时机械臂的y坐标(笛卡尔坐标系)



    while (ros::ok()) {
// 创建PTP客户端
    client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
    dobot::SetPTPCmd srv;

        do {

            Listener listener;
            ros::Subscriber sub = n.subscribe("object_center", 1000, &Listener::callback, &listener);
            ros::Rate loop_rate(10);

            while(ros::ok() and listener.count <=0){
                ros::spinOnce();
                loop_rate.sleep();
            }

            sx = listener.print_datax();
            sy = listener.print_datay();
            ss = listener.print_datas();

            std::cout << sx << "\n";
            std::cout << sy << "\n";
            std::cout << ss << "\n";

            } while (0);


        // 机械臂运动到物体中心位置
        do {
            x = (239.5 - sy )/(ss/side_length) - CamToSucker + detect_x;       // ss/34.45为像素距离之比
            y = (319.5 - sx)/(ss/side_length) + CamToSuc_deviation + detect_y;
            srv.request.ptpMode = 1;
            srv.request.x = x;
            srv.request.y = y;
            srv.request.z = suck_z;    //-50两快木块演示用
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

 
//  获取当前的指令索引
    client = n.serviceClient<dobot::GetQueuedCmdCurrentIndex>("/DobotServer/GetQueuedCmdCurrentIndex");
    dobot::GetQueuedCmdCurrentIndex srv8;
    client.call(srv8);



//  等待所有的动作完成
while(srv8.response.queuedCmdIndex < srv.response.queuedCmdIndex)
    if(client.call(srv8))
    {
       // cout << "pick_up" << endl;
    }


/*  ********************************************************************* */

//  抓取物体
        //  创建吸盘的客户端
        client = n.serviceClient<dobot::SetEndEffectorSuctionCup>("/DobotServer/SetEndEffectorSuctionCup");
        dobot::SetEndEffectorSuctionCup sck2;
        //  吸取物体
        do{
            sck2.request.enableCtrl = 1;
            sck2.request.suck = 1; // Enable suction 
            sck2.request.isQueued = true;
            client.call(sck2);
 
            ros::spinOnce(); //spinOnce() will execute any commands in the queue.

        } while(0);

// 创建PTP客户端
        client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
        dobot::SetPTPCmd srv1;

        // 把物体运动初始化位置

        do {
            srv1.request.ptpMode = 1;
            srv1.request.x = 200;
            srv1.request.y = 0;
            srv1.request.z = 80;
            srv1.request.r = 0;
            client.call(srv1);  
             if (srv1.response.result == 0) {

        break;                
            }  
            ros::spinOnce();
        } while (0);

        do {
            srv1.request.ptpMode = 1;
            srv1.request.x = 200;
            srv1.request.y = -200;
            srv1.request.z = 80;
            srv1.request.r = 0;
            client.call(srv1);  
             if (srv1.response.result == 0) {

        break;                
            }  
            ros::spinOnce();
        } while (0);

        do {
            srv1.request.ptpMode = 1;
            srv1.request.x = 200;
            srv1.request.y = -180;
            srv1.request.z = -30;
            srv1.request.r = 0;
            client.call(srv1);  
             if (srv1.response.result == 0) {

        break;                
            }  
            ros::spinOnce();
        } while (0);


//  test:松开物体
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


        client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
        dobot::SetPTPCmd srv2;

        // 把物体运动初始化位置

        do {
            srv2.request.ptpMode = 1;
            srv2.request.x = 200;
            srv2.request.y = 0;
            srv2.request.z = 80;
            srv2.request.r = 0;
            client.call(srv2);  
             if (srv2.response.result == 0) {

        break;                
            }  
            ros::spinOnce();
        } while (0);


//  获取当前的指令索引
    client = n.serviceClient<dobot::GetQueuedCmdCurrentIndex>("/DobotServer/GetQueuedCmdCurrentIndex");
    dobot::GetQueuedCmdCurrentIndex srv9;
    client.call(srv9);

//  等待所有的动作完成
while(srv9.response.queuedCmdIndex < srv2.response.queuedCmdIndex)
    if(client.call(srv9))
    {
        //cout << "wait!" << endl;
    }

//  当动作完成之后，反馈回动作完成的标准
    tag.data = 3;
    action_pub.publish(tag);
    
    // 当action完成后，向客户端返回结果
    ROS_INFO("Manipulator arm %d finish working.", goal->dobot_pick_id);
    as->setSucceeded();

}
}



/*  ********************************************************************* */
/*                       －－－－－－     主函数    －－－－－                    */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "Dobot_Grasp_server");
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


/*  ********************************************************************* */
    // 定义一个抓取物体的服务器
    Server server(n, "Grasp", boost::bind(&execute_pick_up, _1, &server), false);
      
    // 服务器开始运行
    server.start();

    ros::spin();

    return 0;

}
