#include <pluginlib/class_list_macros.h>

#include <algorithm>
#include <cmath>

#include <geometry_msgs/PoseStamped.h>
#include <tf2/utils.h>

#include "sunray_nav_plugins/orca_local_planner.h"

namespace sunray_nav_plugins {

    static double clamp(double v, double lo, double hi) {
        return std::max(lo, std::min(v, hi));
    }

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

        // bring-up controller params
        private_nh.param("kp_xy", kp_xy_, kp_xy_);
        private_nh.param("max_vel_xy", max_vel_xy_, max_vel_xy_);
        private_nh.param("max_yaw_rate", max_yaw_rate_, max_yaw_rate_);
        private_nh.param("use_yaw_control", use_yaw_control_, use_yaw_control_);

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
        cmd_vel = geometry_msgs::Twist();

        if (!initialized_ || costmap_ros_ == nullptr) {
            ROS_ERROR("OrcaLocalPlanner not initialized.");
            return false;
        }

        if (global_plan_.empty()) {
            // No plan: stop
            return true;
        }

        geometry_msgs::PoseStamped robot_pose;
        if (!costmap_ros_->getRobotPose(robot_pose)) {
            ROS_WARN_THROTTLE(1.0, "OrcaLocalPlanner: failed to get robot pose from costmap_ros.");
            return false;
        }

        const auto& goal = global_plan_.back();

        // Compute error in global frame (usually map)
        const double dx = goal.pose.position.x - robot_pose.pose.position.x;
        const double dy = goal.pose.position.y - robot_pose.pose.position.y;
        const double dist = std::sqrt(dx * dx + dy * dy);

        if (dist <= goal_tolerance_xy_) {
            // At goal: stop
            return true;
        }

        // Robot yaw in global frame
        const double yaw = tf2::getYaw(robot_pose.pose.orientation);

        // Rotate global error into body frame (base_link):
        // [x_body] =  cos(yaw)*dx + sin(yaw)*dy
        // [y_body] = -sin(yaw)*dx + cos(yaw)*dy
        const double ex_body =  std::cos(yaw) * dx + std::sin(yaw) * dy;
        const double ey_body = -std::sin(yaw) * dx + std::cos(yaw) * dy;

        // Simple P controller in body frame
        double vx = kp_xy_ * ex_body;
        double vy = kp_xy_ * ey_body;

        // Limit speed
        vx = clamp(vx, -max_vel_xy_, max_vel_xy_);
        vy = clamp(vy, -max_vel_xy_, max_vel_xy_);

        cmd_vel.linear.x = vx;
        cmd_vel.linear.y = vy;
        cmd_vel.linear.z = 0.0;

        // Optional yaw control (off by default)
        if (use_yaw_control_) {
            const double goal_yaw = tf2::getYaw(goal.pose.orientation);
            double yaw_err = goal_yaw - yaw;
            // wrap to [-pi, pi]
            while (yaw_err > M_PI) yaw_err -= 2.0 * M_PI;
            while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;
            cmd_vel.angular.z = clamp(1.0 * yaw_err, -max_yaw_rate_, max_yaw_rate_);
        } else {
            cmd_vel.angular.z = 0.0;
        }

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