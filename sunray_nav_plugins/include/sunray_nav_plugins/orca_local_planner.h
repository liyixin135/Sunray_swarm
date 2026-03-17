#pragma once

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/base_local_planner.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>

namespace sunray_nav_plugins {

    class OrcaLocalPlanner final : public nav_core::BaseLocalPlanner {
    public:
        OrcaLocalPlanner() = default;
        OrcaLocalPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

        void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) override;

        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) override;
        bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
        bool isGoalReached() override;

    private:
        bool initialized_{false};
        tf2_ros::Buffer* tf_{nullptr};
        costmap_2d::Costmap2DROS* costmap_ros_{nullptr};
        std::vector<geometry_msgs::PoseStamped> global_plan_;

        double goal_tolerance_xy_{0.20};

        // ---- minimal "go-to-goal" controller params (for bring-up) ----
        double kp_xy_{0.8};        // proportional gain in XY
        double max_vel_xy_{0.4};   // [m/s] limit for vx, vy in body frame
        double max_yaw_rate_{1.0}; // [rad/s]
        bool use_yaw_control_{false};
    };

}  // namespace sunray_nav_plugins