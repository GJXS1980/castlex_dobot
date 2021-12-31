#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "dobot/DobotAction.h"

typedef actionlib::SimpleActionServer<dobot::DobotAction> Server;



// 收到action的goal后调用的回调函数
void execute(const dobot::DobotGoalConstPtr& goal, Server* as)
{
    ros::Rate r(1);
    dobot::DobotFeedback feedback;

    ROS_INFO("Dishwasher %d is working.", goal->dobot_pick_id);

    // 假设洗盘子的进度，并且按照1hz的频率发布进度feedback
    for(int i=1; i<=10; i++)
    {
        feedback.percent_complete = i * 10;
        as->publishFeedback(feedback);
        r.sleep();
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

	// 定义一个服务器
    Server server(n, "do_dishes", boost::bind(&execute, _1, &server), false);
	
	// 服务器开始运行
    server.start();

    ros::spin();

    return 0;
}