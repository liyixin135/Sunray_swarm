#include <ros/ros.h>
#include <signal.h>
#include <string>

#include <sunray_swarm_msgs/orca_cmd.h>
#include <std_msgs/String.h>

static void mySigintHandler(int sig)
{
    ROS_INFO("[orca_init] exit...");
    ros::shutdown();
}

// 发一条 orca_cmd
static void publishCmd(ros::Publisher& pub,
                       const std::string& source,
                       uint8_t cmd,
                       const std::string& frame_id = "world")
{
    sunray_swarm_msgs::orca_cmd msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;
    msg.cmd_source = source;
    msg.orca_cmd = cmd;
    // obs_point 默认空
    pub.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "orca_init", ros::init_options::NoSigintHandler);
    signal(SIGINT, mySigintHandler);

    ros::NodeHandle nh("~");

    // 参数：rmtt / ugv
    std::string agent_prefix = "rmtt";
    nh.param<std::string>("agent_prefix", agent_prefix, agent_prefix);

    // 可选：先 STOP，防止刚启动时乱跑
    bool stop_first = true;
    nh.param("stop_first", stop_first, stop_first);

    // stop 后等待
    double stop_wait = 0.5;
    nh.param("stop_wait", stop_wait, stop_wait);

    // 运行前等待（给 move_base / bridge / goal_pub 留时间）
    double start_delay = 8.0;
    nh.param("start_delay", start_delay, start_delay);

    // 启动命令（默认 SET_HOME=0；如果你需要场景就改成 11/12/13...）
    int start_cmd_int = sunray_swarm_msgs::orca_cmd::SET_HOME; // 0
    nh.param("start_cmd", start_cmd_int, start_cmd_int);
    const uint8_t start_cmd = static_cast<uint8_t>(start_cmd_int);

    // 发布信息到地面站（可选）
    bool publish_text = true;
    nh.param("publish_text", publish_text, publish_text);

    const std::string node_name = ros::this_node::getName();
    const std::string cmd_topic = "/sunray_swarm/" + agent_prefix + "/orca_cmd";

    ros::Publisher orca_cmd_pub = nh.advertise<sunray_swarm_msgs::orca_cmd>(cmd_topic, 1, true);
    ros::Publisher text_pub;
    if (publish_text)
    {
        text_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1, true);
    }

    auto say = [&](const std::string& s)
    {
        ROS_INFO("%s", s.c_str());
        if (publish_text)
        {
            std_msgs::String t;
            t.data = "[" + node_name + "] ---> " + s;
            text_pub.publish(t);
        }
    };

    say("orca_init start. cmd_topic=" + cmd_topic);

    // 给 ROS 建连一点时间（尤其是多节点同时启动时）
    ros::Duration(0.3).sleep();

    if (stop_first)
    {
        say("Publish ORCA_STOP...");
        publishCmd(orca_cmd_pub, node_name, sunray_swarm_msgs::orca_cmd::ORCA_STOP);
        ros::Duration(stop_wait).sleep();
    }

    if (start_delay > 0.0)
    {
        say("Wait start_delay=" + std::to_string(start_delay) + "s ...");
        ros::Duration(start_delay).sleep();
    }

    // 按 swarm_test 的习惯：发 SET_HOME 就相当于“启动 ORCA”
    say("Publish start_cmd=" + std::to_string((int)start_cmd) + " ...");
    publishCmd(orca_cmd_pub, node_name, start_cmd);

    say("orca_init done. exit.");
    ros::Duration(0.2).sleep();
    return 0;
}