#include <ros/ros.h>
#include <signal.h>

#include "rmtt_control.h"

void mySigintHandler(int sig)
{
    ROS_INFO("[rmtt_control_node] exit...");
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rmtt_control_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(20.0);//控制循环频率为 20 Hz

    //可以在 Ctrl+C 时先做 清理工作（关闭文件、释放资源、停 ROS 节点）
    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();

    sleep(8.0);

    // 控制器
    //在rmtt_control.h里面定义的，调用类
    RMTT_CONTROL rmtt_control;
    //传入句柄，去获取参数，初始化发布者和订阅者、定时器
    rmtt_control.init(nh);

    //ros::spinOnce() 在每轮循环中处理一次回调
    ros::spinOnce();
    ros::Duration(1.0).sleep();

    // 主循环
    while (ros::ok())
    {
        // 回调函数
        ros::spinOnce();
        // 主循环函数
        //调用类的函数，这里就是实现控制的关键
        rmtt_control.mainloop();
        // sleep
        rate.sleep();
    }

    return 0;
}
