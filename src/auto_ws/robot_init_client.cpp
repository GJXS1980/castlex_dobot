#include <actionlib/client/simple_action_client.h>
#include "dobot/DobotAction.h"

#include "std_msgs/Int32.h"

typedef actionlib::SimpleActionClient<dobot::DobotAction> Client;

// 当action完成后会调用次回调函数一次
void doneCb(const actionlib::SimpleClientGoalState& state,
        const dobot::DobotResultConstPtr& result)
{
    ROS_INFO("Yay! Ｉnit Robot is in progress");
    //ros::shutdown();
}

// 当action激活后会调用次回调函数一次
void activeCb()
{
    ROS_INFO("Manipulator arm just went active");
}

// 收到feedback后调用的回调函数
void feedbackCb(const dobot::DobotFeedbackConstPtr& feedback)
{
    ROS_INFO(" percent_complete : %f ", feedback->percent_complete);
}

/*  **************************** 如果到达第一个目标点就执行  **************************************** */
void PickUpCallback(const std_msgs::Int32::ConstPtr& msg)
{
    int tag;
    tag = msg -> data;
    if(tag == 1)
    {
    // 定义一个客户端
    Client client("ZeroBack", true);

    // 等待服务器端
    ROS_INFO("Waiting for action server to start.");
    client.waitForServer();
    ROS_INFO("Action server started ... ");

    // 创建一个action的goal
    dobot::DobotGoal goal;

    goal.dobot_pick_id = tag;

    // 发送action的goal给服务器端，并且设置回调函数
    client.sendGoal(goal,  &doneCb, &activeCb, &feedbackCb);
    }
}


// 主函数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "dobot_zero_client");

    ros::NodeHandle n;

    ros::Subscriber subPick = n.subscribe("grasp_cal_flag", 1000, PickUpCallback);

    ros::spin();

    return 0;
    
}
