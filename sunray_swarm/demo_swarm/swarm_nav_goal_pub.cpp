//
// Created by xin on 23/03/26.
//
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <XmlRpcValue.h>
#include <cmath>
#include <string>
#include <vector>

struct Goal2D
{
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
};

static geometry_msgs::Quaternion yawToQuat(double yaw)
{
    geometry_msgs::Quaternion q;
    const double h = yaw * 0.5;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(h);
    q.w = std::cos(h);
    return q;
}

static double toDouble(const XmlRpc::XmlRpcValue& v)
{
    if (v.getType() == XmlRpc::XmlRpcValue::TypeInt) return static_cast<int>(v);
    if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble) return static_cast<double>(v);
    return 0.0;
}

// param format:
// goals: [ [8.0, 0.0, 0.0], [-8.0, 0.0, 0.0], [0.0, 8.0, 0.0], [0.0, -8.0, 0.0] ]
static bool loadGoals(ros::NodeHandle& pnh, const std::string& param, int agent_num, std::vector<Goal2D>& out)
{
    XmlRpc::XmlRpcValue goals;
    if (!pnh.getParam(param, goals))
    {
        ROS_ERROR("Missing required param '~%s'.", param.c_str());
        return false;
    }
    if (goals.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("Param '~%s' must be a list: [ [x,y,yaw], ... ].", param.c_str());
        return false;
    }

    const int n = std::min<int>(agent_num, goals.size());
    if (n <= 0)
    {
        ROS_ERROR("Param '~%s' is empty.", param.c_str());
        return false;
    }

    out.clear();
    out.resize(n);

    for (int i = 0; i < n; i++)
    {
        if (goals[i].getType() != XmlRpc::XmlRpcValue::TypeArray || goals[i].size() < 2)
        {
            ROS_ERROR("Param '~%s[%d]' must be [x,y] or [x,y,yaw].", param.c_str(), i);
            return false;
        }
        out[i].x = toDouble(goals[i][0]);
        out[i].y = toDouble(goals[i][1]);
        out[i].yaw = (goals[i].size() >= 3) ? toDouble(goals[i][2]) : 0.0;
    }

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "swarm_nav_goal_pub");
    ros::NodeHandle pnh("~");

    int agent_num = 4;
    pnh.param("agent_num", agent_num, 4);

    std::string agent_prefix = "rmtt";          // rmtt_1..rmtt_N
    std::string goal_topic = "move_base_simple/goal";
    std::string frame_id = "map";

    pnh.param("agent_prefix", agent_prefix, agent_prefix);
    pnh.param("goal_topic", goal_topic, goal_topic);
    pnh.param("frame_id", frame_id, frame_id);

    // Publish reliability: repeat for ~3 seconds by default
    int repeat_times = 30;
    double repeat_hz = 10.0;
    pnh.param("repeat_times", repeat_times, repeat_times);
    pnh.param("repeat_hz", repeat_hz, repeat_hz);

    // wait for subscribers
    double connect_wait = 1.0;
    pnh.param("connect_wait", connect_wait, connect_wait);

    std::vector<Goal2D> goals;
    if (!loadGoals(pnh, "goals", agent_num, goals))
    {
        return 1;
    }

    std::vector<ros::Publisher> pubs;
    pubs.reserve(goals.size());

    for (size_t i = 0; i < goals.size(); i++)
    {
        const std::string agent_name = agent_prefix + "_" + std::to_string(static_cast<int>(i) + 1);
        const std::string topic = "/" + agent_name + "/" + goal_topic;

        // latch=true: new subscriber can still receive last message
        pubs.emplace_back(pnh.advertise<geometry_msgs::PoseStamped>(topic, 1, true));
        ROS_INFO("Publishing to: %s", topic.c_str());
    }

    ros::Duration(connect_wait).sleep();

    ros::Rate r(repeat_hz);
    for (int k = 0; ros::ok() && k < repeat_times; k++)
    {
        for (size_t i = 0; i < pubs.size(); i++)
        {
            geometry_msgs::PoseStamped msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = frame_id;

            msg.pose.position.x = goals[i].x;
            msg.pose.position.y = goals[i].y;
            msg.pose.position.z = 0.0;
            msg.pose.orientation = yawToQuat(goals[i].yaw);

            pubs[i].publish(msg);
        }
        ros::spinOnce();
        r.sleep();
    }

    ROS_INFO("Goals published. Exit.");
    return 0;
}