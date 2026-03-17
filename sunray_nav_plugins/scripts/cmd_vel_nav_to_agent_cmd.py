#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sunray_swarm_msgs.msg import agent_cmd

class Bridge:
    def __init__(self):
        self.agent_id = rospy.get_param("~agent_id", 1)
        self.cmd_source = rospy.get_param("~cmd_source", "move_base")
        self.in_topic = rospy.get_param("~cmd_vel_nav_topic", f"/sunray_swarm/rmtt_{self.agent_id}/cmd_vel_nav")
        self.out_topic = rospy.get_param("~agent_cmd_topic", f"/sunray_swarm/rmtt_{self.agent_id}/agent_cmd")

        self.pub = rospy.Publisher(self.out_topic, agent_cmd, queue_size=10)
        self.sub = rospy.Subscriber(self.in_topic, Twist, self.cb, queue_size=10)

        rospy.loginfo("cmd_vel_nav_to_agent_cmd bridge: agent_id=%d sub=%s pub=%s",
                      self.agent_id, self.in_topic, self.out_topic)

    def cb(self, msg: Twist):
        cmd = agent_cmd()
        cmd.header.stamp = rospy.Time.now()
        cmd.agent_id = self.agent_id
        cmd.cmd_source = self.cmd_source
        cmd.control_state = agent_cmd.VEL_CONTROL_BODY

        cmd.desired_vel = msg
        # unused in this mode, keep default zeros
        cmd.desired_pos.x = 0.0
        cmd.desired_pos.y = 0.0
        cmd.desired_pos.z = 0.0
        cmd.desired_yaw = 0.0

        self.pub.publish(cmd)

if __name__ == "__main__":
    rospy.init_node("cmd_vel_nav_to_agent_cmd_bridge")
    Bridge()
    rospy.spin()
