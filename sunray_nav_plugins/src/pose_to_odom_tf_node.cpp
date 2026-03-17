#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

class PoseToOdomTf {
public:
  explicit PoseToOdomTf(ros::NodeHandle& nh) : nh_(nh) {
    nh_.param<std::string>("pose_topic", pose_topic_, pose_topic_);
    nh_.param<std::string>("odom_topic", odom_topic_, odom_topic_);
    nh_.param<std::string>("map_frame", map_frame_, map_frame_);
    nh_.param<std::string>("odom_frame", odom_frame_, odom_frame_);
    nh_.param<std::string>("base_frame", base_frame_, base_frame_);
    nh_.param<double>("publish_rate", publish_rate_, publish_rate_);

    pose_sub_ = nh_.subscribe(pose_topic_, 10, &PoseToOdomTf::poseCb, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);

    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, publish_rate_)),
                             &PoseToOdomTf::timerCb,
                             this);
  }

private:
  void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    last_pose_ = *msg;
    have_pose_ = true;
  }

  void timerCb(const ros::TimerEvent&) {
    if (!have_pose_) {
      return;
    }

    const ros::Time stamp = ros::Time::now();

    // Publish odom (using pose as ground truth position in map frame).
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose = last_pose_.pose;
    odom_pub_.publish(odom);

    // Broadcast map->odom as identity (simplest for simulation).
    geometry_msgs::TransformStamped t_map_odom;
    t_map_odom.header.stamp = stamp;
    t_map_odom.header.frame_id = map_frame_;
    t_map_odom.child_frame_id = odom_frame_;
    t_map_odom.transform.rotation.w = 1.0;
    tf_broadcaster_.sendTransform(t_map_odom);

    // Broadcast odom->base_link using pose.
    geometry_msgs::TransformStamped t_odom_base;
    t_odom_base.header.stamp = stamp;
    t_odom_base.header.frame_id = odom_frame_;
    t_odom_base.child_frame_id = base_frame_;
    t_odom_base.transform.translation.x = last_pose_.pose.position.x;
    t_odom_base.transform.translation.y = last_pose_.pose.position.y;
    t_odom_base.transform.translation.z = last_pose_.pose.position.z;
    t_odom_base.transform.rotation = last_pose_.pose.orientation;
    tf_broadcaster_.sendTransform(t_odom_base);
  }

  ros::NodeHandle nh_;
  ros::Subscriber pose_sub_;
  ros::Publisher odom_pub_;
  ros::Timer timer_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  bool have_pose_{false};
  geometry_msgs::PoseStamped last_pose_;

  std::string pose_topic_{"/vrpn_client_node/rmtt_1/pose"};
  std::string odom_topic_{"/odom"};
  std::string map_frame_{"map"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  double publish_rate_{30.0};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_to_odom_tf");
  ros::NodeHandle nh("~");
  PoseToOdomTf node(nh);
  ros::spin();
  return 0;
}

