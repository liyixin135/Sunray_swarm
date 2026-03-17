#include <pluginlib/class_list_macros.h>

#include <cmath>

#include <geometry_msgs/PoseStamped.h>

#include "sunray_nav_plugins/orca_local_planner.h"

namespace sunray_nav_plugins {

OrcaLocalPlanner::OrcaLocalPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
  initialize(std::move(name), tf, costmap_ros);
}

void OrcaLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
  if (initialized_) {
    return;
  }

  tf_ = tf;
  costmap_ros_ = costmap_ros;

  ros::NodeHandle private_nh("~/" + name);
  private_nh.param("goal_tolerance_xy", goal_tolerance_xy_, goal_tolerance_xy_);

  initialized_ = true;
}

bool OrcaLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
  if (!initialized_) {
    ROS_ERROR("OrcaLocalPlanner not initialized.");
    return false;
  }
  global_plan_ = orig_global_plan;
  return true;
}

bool OrcaLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  if (!initialized_ || costmap_ros_ == nullptr) {
    ROS_ERROR("OrcaLocalPlanner not initialized.");
    return false;
  }

  // Minimal skeleton: output zero velocity.
  // Next stage will integrate ORCA: read local obstacles from costmap and follow global_plan_.
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.linear.z = 0.0;
  cmd_vel.angular.x = 0.0;
  cmd_vel.angular.y = 0.0;
  cmd_vel.angular.z = 0.0;

  return true;
}

bool OrcaLocalPlanner::isGoalReached() {
  if (!initialized_ || costmap_ros_ == nullptr) {
    return false;
  }
  if (global_plan_.empty()) {
    return false;
  }

  geometry_msgs::PoseStamped robot_pose;
  if (!costmap_ros_->getRobotPose(robot_pose)) {
    return false;
  }

  const auto& goal = global_plan_.back();
  const double dx = robot_pose.pose.position.x - goal.pose.position.x;
  const double dy = robot_pose.pose.position.y - goal.pose.position.y;
  const double dist = std::sqrt(dx * dx + dy * dy);
  return dist <= goal_tolerance_xy_;
}

}  // namespace sunray_nav_plugins

PLUGINLIB_EXPORT_CLASS(sunray_nav_plugins::OrcaLocalPlanner, nav_core::BaseLocalPlanner)

