#!/usr/bin/env python3
import math
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from sunray_swarm_msgs.msg import agent_state

class PathToOrcaGoal:
    def __init__(self):
        self.plan_topic = rospy.get_param("~plan_topic", "/rmtt_1/move_base/AStarGlobalPlanner/plan")
        self.state_topic = rospy.get_param("~state_topic", "/sunray_swarm/rmtt_1/agent_state")
        self.goal_topic = rospy.get_param("~goal_topic", "/sunray_swarm/rmtt_1/goal_point")

        self.lookahead = float(rospy.get_param("~lookahead", 1.5))   # meters
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance", 0.25))  # meters
        self.pub_hz = float(rospy.get_param("~pub_hz", 10.0))

        self.latest_path = None
        self.pos = None  # (x,y)

        self.goal_pub = rospy.Publisher(self.goal_topic, Point, queue_size=10)
        rospy.Subscriber(self.plan_topic, Path, self.on_path, queue_size=1)
        rospy.Subscriber(self.state_topic, agent_state, self.on_state, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(1.0/self.pub_hz), self.on_timer)
        rospy.loginfo("PathToOrcaGoal: plan=%s state=%s -> goal=%s lookahead=%.2f",
                      self.plan_topic, self.state_topic, self.goal_topic, self.lookahead)

    def on_path(self, msg: Path):
        # 空 path 直接忽略
        if not msg.poses:
            self.latest_path = None
        else:
            self.latest_path = msg

    def on_state(self, msg: agent_state):
        # 你们 agent_state.pos 是数组：pos[0], pos[1]
        self.pos = (float(msg.pos[0]), float(msg.pos[1]))

    @staticmethod
    def dist(a, b):
        return math.hypot(a[0]-b[0], a[1]-b[1])

    def pick_lookahead_point(self, path: Path, pos_xy):
        pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        # 找离当前位置最近的 path 点索引
        nearest_i = min(range(len(pts)), key=lambda i: self.dist(pts[i], pos_xy))

        # 从 nearest_i 往前累计弧长，找到 lookahead 处的点
        acc = 0.0
        last = pts[nearest_i]
        for i in range(nearest_i + 1, len(pts)):
            cur = pts[i]
            acc += self.dist(cur, last)
            last = cur
            if acc >= self.lookahead:
                return cur, pts[-1]
        # path 太短，直接用终点
        return pts[-1], pts[-1]

    def on_timer(self, _evt):
        if self.latest_path is None or self.pos is None:
            return

        goal_xy, final_xy = self.pick_lookahead_point(self.latest_path, self.pos)

        # 如果已经接近终点，就直接发布终点（避免在终点附近抖动）
        if self.dist(self.pos, final_xy) <= self.goal_tolerance:
            out = Point(x=final_xy[0], y=final_xy[1], z=0.0)
        else:
            out = Point(x=goal_xy[0], y=goal_xy[1], z=0.0)

        self.goal_pub.publish(out)

if __name__ == "__main__":
    rospy.init_node("path_to_orca_goal_point")
    PathToOrcaGoal()
    rospy.spin()