#pragma once

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

namespace sunray_nav_plugins {

class AStarGlobalPlanner final : public nav_core::BaseGlobalPlanner {
public:
  AStarGlobalPlanner() = default;
  AStarGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;

  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override;

private:
  bool initialized_{false};
  costmap_2d::Costmap2DROS* costmap_ros_{nullptr};
  ros::Publisher plan_pub_;
};

}  // namespace sunray_nav_plugins

