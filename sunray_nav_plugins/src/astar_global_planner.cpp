#include <pluginlib/class_list_macros.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

#include "sunray_nav_plugins/astar_global_planner.h"

namespace sunray_nav_plugins {

    namespace {

// pack (x,y) into a single key for hash maps
        inline uint32_t keyOf(unsigned int x, unsigned int y, unsigned int size_x) {
            return static_cast<uint32_t>(y * size_x + x);
        }

        struct NodeRec {
            unsigned int x{0}, y{0};
            float g{std::numeric_limits<float>::infinity()};
            float f{std::numeric_limits<float>::infinity()};
            uint32_t parent{std::numeric_limits<uint32_t>::max()};
            bool closed{false};
        };

        struct OpenItem {
            uint32_t key;
            float f;
            // priority_queue puts "largest" first, so we invert (min-heap behavior)
            bool operator<(const OpenItem& other) const { return f > other.f; }
        };

        inline float heuristic(unsigned int x, unsigned int y, unsigned int gx, unsigned int gy) {
            // Euclidean (works fine for 8-neighborhood)
            const float dx = static_cast<float>(x) - static_cast<float>(gx);
            const float dy = static_cast<float>(y) - static_cast<float>(gy);
            return std::sqrt(dx * dx + dy * dy);
        }

    }  // namespace

    AStarGlobalPlanner::AStarGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
        initialize(std::move(name), costmap_ros);
    }

    void AStarGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
        if (initialized_) return;

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

        costmap_2d::Costmap2D* cm = costmap_ros_->getCostmap();
        if (!cm) {
            ROS_ERROR("AStarGlobalPlanner: costmap is null.");
            return false;
        }

        const std::string frame = costmap_ros_->getGlobalFrameID();

        // Convert start/goal into costmap grid coordinates
        unsigned int sx, sy, gx, gy;
        if (!cm->worldToMap(start.pose.position.x, start.pose.position.y, sx, sy)) {
            ROS_WARN("AStarGlobalPlanner: start is outside costmap bounds.");
            return false;
        }
        if (!cm->worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy)) {
            ROS_WARN("AStarGlobalPlanner: goal is outside costmap bounds.");
            return false;
        }

        const unsigned int size_x = cm->getSizeInCellsX();
        const unsigned int size_y = cm->getSizeInCellsY();

        auto isBlocked = [&](unsigned int x, unsigned int y) -> bool {
            const unsigned char c = cm->getCost(x, y);
            if (c == costmap_2d::NO_INFORMATION) {
                return true;  // conservative: treat unknown as blocked
            }
            return c >= costmap_2d::LETHAL_OBSTACLE;
        };

        if (isBlocked(gx, gy)) {
            ROS_WARN("AStarGlobalPlanner: goal cell is blocked (occupied/unknown).");
            return false;
        }
        if (isBlocked(sx, sy)) {
            ROS_WARN("AStarGlobalPlanner: start cell is blocked (occupied/unknown).");
            return false;
        }

        // A* search
        std::unordered_map<uint32_t, NodeRec> nodes;
        nodes.reserve(size_x * 4);  // heuristic reserve

        auto& startRec = nodes[keyOf(sx, sy, size_x)];
        startRec.x = sx;
        startRec.y = sy;
        startRec.g = 0.0f;
        startRec.f = heuristic(sx, sy, gx, gy);
        startRec.parent = std::numeric_limits<uint32_t>::max();

        std::priority_queue<OpenItem> open;
        open.push(OpenItem{keyOf(sx, sy, size_x), startRec.f});

        const int dx8[8] = {1,  1,  0, -1, -1, -1, 0, 1};
        const int dy8[8] = {0,  1,  1,  1,  0, -1, -1, -1};
        const float stepCost[8] = {1.0f, 1.41421356f, 1.0f, 1.41421356f,
                                   1.0f, 1.41421356f, 1.0f, 1.41421356f};

        uint32_t goalKey = keyOf(gx, gy, size_x);
        bool found = false;

        while (!open.empty()) {
            OpenItem curItem = open.top();
            open.pop();

            auto it = nodes.find(curItem.key);
            if (it == nodes.end()) continue;
            NodeRec& cur = it->second;

            if (cur.closed) continue;
            cur.closed = true;

            if (curItem.key == goalKey) {
                found = true;
                break;
            }

            for (int i = 0; i < 8; ++i) {
                const int nx_i = static_cast<int>(cur.x) + dx8[i];
                const int ny_i = static_cast<int>(cur.y) + dy8[i];
                if (nx_i < 0 || ny_i < 0) continue;
                const unsigned int nx = static_cast<unsigned int>(nx_i);
                const unsigned int ny = static_cast<unsigned int>(ny_i);
                if (nx >= size_x || ny >= size_y) continue;

                if (isBlocked(nx, ny)) continue;

                // Optional: prevent cutting corners through obstacles on diagonals
                if (dx8[i] != 0 && dy8[i] != 0) {
                    // moving diagonally: require both adjacent cardinal cells to be free
                    const unsigned int ax = static_cast<unsigned int>(static_cast<int>(cur.x) + dx8[i]);
                    const unsigned int ay = cur.y;
                    const unsigned int bx = cur.x;
                    const unsigned int by = static_cast<unsigned int>(static_cast<int>(cur.y) + dy8[i]);
                    if (isBlocked(ax, ay) || isBlocked(bx, by)) continue;
                }

                const uint32_t nkey = keyOf(nx, ny, size_x);

                NodeRec& nb = nodes[nkey];
                nb.x = nx;
                nb.y = ny;

                const float tentative_g = cur.g + stepCost[i];

                if (tentative_g < nb.g) {
                    nb.g = tentative_g;
                    nb.f = tentative_g + heuristic(nx, ny, gx, gy);
                    nb.parent = curItem.key;
                    open.push(OpenItem{nkey, nb.f});
                }
            }
        }

        if (!found) {
            ROS_WARN("AStarGlobalPlanner: failed to find a path.");
            return false;
        }

        // Reconstruct path (grid cells) from goal back to start
        std::vector<uint32_t> rev_keys;
        rev_keys.reserve(4096);
        uint32_t k = goalKey;
        while (k != std::numeric_limits<uint32_t>::max()) {
            rev_keys.push_back(k);
            auto it = nodes.find(k);
            if (it == nodes.end()) break;
            k = it->second.parent;
        }

        if (rev_keys.empty()) {
            ROS_WARN("AStarGlobalPlanner: empty reconstructed path.");
            return false;
        }

        std::reverse(rev_keys.begin(), rev_keys.end());

        // Convert grid path to world path
        const ros::Time stamp = ros::Time::now();
        plan.reserve(rev_keys.size());

        for (uint32_t kk : rev_keys) {
            auto it = nodes.find(kk);
            if (it == nodes.end()) continue;

            double wx, wy;
            cm->mapToWorld(it->second.x, it->second.y, wx, wy);

            geometry_msgs::PoseStamped p;
            p.header.frame_id = frame;
            p.header.stamp = stamp;
            p.pose.position.x = wx;
            p.pose.position.y = wy;

            // Ignore height: keep z=0 for path points (you can set to 1.0 if your controller expects it)
            p.pose.position.z = 0.0;

            p.pose.orientation.w = 1.0;
            plan.push_back(p);
        }

        // Publish nav_msgs/Path
        nav_msgs::Path path;
        path.header.frame_id = frame;
        path.header.stamp = stamp;
        path.poses = plan;
        plan_pub_.publish(path);

        return true;
    }

}  // namespace sunray_nav_plugins

PLUGINLIB_EXPORT_CLASS(sunray_nav_plugins::AStarGlobalPlanner, nav_core::BaseGlobalPlanner)