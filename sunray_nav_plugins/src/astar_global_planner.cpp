#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PoseStamped.h>

#include "sunray_nav_plugins/astar_global_planner.h"

namespace sunray_nav_plugins {

AStarGlobalPlanner::AStarGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  initialize(std::move(name), costmap_ros);
}

void AStarGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  if (initialized_) {
    return;
  }
  costmap_ros_ = costmap_ros;

  ros::NodeHandle private_nh("~/" + name);
  plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1, true);

  initialized_ = true;
}

bool AStarGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                  const geometry_msgs::PoseStamped& goal,
                                  std::vector<geometry_msgs::PoseStamped>& plan) {
  if (!initialized_ || costmap_ros_ == nullptr) {
    ROS_ERROR("AStarGlobalPlanner not initialized.");
    return false;
  }

  plan.clear();

  // Minimal skeleton: straight-through plan with just start+goal.
  // Next stage will replace this with real A* on costmap.
  geometry_msgs::PoseStamped s = start;
  geometry_msgs::PoseStamped g = goal;
  const std::string frame = costmap_ros_->getGlobalFrameID();
  s.header.frame_id = frame;
  g.header.frame_id = frame;
  s.header.stamp = ros::Time::now();
  g.header.stamp = s.header.stamp;

  plan.push_back(s);
  plan.push_back(g);

  nav_msgs::Path path;
  path.header.frame_id = frame;
  path.header.stamp = s.header.stamp;
  path.poses = plan;
  plan_pub_.publish(path);

  return true;
}

}  // namespace sunray_nav_plugins

PLUGINLIB_EXPORT_CLASS(sunray_nav_plugins::AStarGlobalPlanner, nav_core::BaseGlobalPlanner)

