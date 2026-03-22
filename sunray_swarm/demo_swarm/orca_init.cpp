//
// Created by xin on 23/03/26.
//
#include <ros/ros.h>
#include <sunray_swarm_msgs/orca_cmd.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "orca_init");
    ros::NodeHandle pnh("~");

    std::string agent_prefix = "rmtt";   // rmtt / ugv
    double delay = 5.0;
    int cmd = sunray_swarm_msgs::orca_cmd::SET_HOME; // 0

    pnh.param<std::string>("agent_prefix", agent_prefix, agent_prefix);
    pnh.param("delay", delay, delay);
    pnh.param("cmd", cmd, cmd);

    const std::string topic = "/sunray_swarm/" + agent_prefix + "/orca_cmd";
    ros::Publisher pub = pnh.advertise<sunray_swarm_msgs::orca_cmd>(topic, 1, true);

    ROS_INFO("[orca_init] will publish cmd=%d to %s after %.1fs", cmd, topic.c_str(), delay);
    ros::Duration(delay).sleep();

    sunray_swarm_msgs::orca_cmd msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.cmd_source = ros::this_node::getName();
    msg.orca_cmd = static_cast<uint8_t>(cmd);
    // obs_point 留空即可
    pub.publish(msg);

    ROS_INFO("[orca_init] published orca_cmd=%d. exit.", cmd);
    ros::Duration(0.2).sleep(); // 给 latch/网络一点时间
    return 0;
}