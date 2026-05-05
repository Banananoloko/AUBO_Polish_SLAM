#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
unity_execution_monitor.py
==========================
监控 Unity 仿真器中机器臂的关节是否收敛到目标位置，
对标 src/aubo_linked_execution/scripts/linked_execution_monitor.py。

订阅：
  /linked_execution/monitor_goal      (sensor_msgs/JointState) - 目标关节位置
  /aubo_e5/joint_states               (sensor_msgs/JointState) - Unity 当前关节位置
  /linked_execution/monitor_control   (std_msgs/String)        - "RESET" 信号

发布：
  /linked_execution/monitor_status    (std_msgs/String)
    取值: "IDLE" | "TRACKING" | "SUCCEEDED" | "FAILED"

使用方式：
  与 Gazebo 版 linked_execution_monitor 完全等价的接口，linked_execution_action_server
  无需感知后端是 Gazebo 还是 Unity。
"""

import math
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class UnityExecutionMonitor:
    def __init__(self):
        self.tolerance = rospy.get_param('~tolerance', 0.03)        # 0.03 rad ≈ 1.7°
        self.timeout_sec = rospy.get_param('~timeout_sec', 15.0)
        self.sim_joint_states_topic = rospy.get_param(
            '~sim_joint_states', '/aubo_e5/joint_states')

        self.goal_position = None       # dict: name -> position
        self.tracking_start = None
        self.status = "IDLE"

        self.pub_status = rospy.Publisher(
            '/linked_execution/monitor_status', String, queue_size=10, latch=True)

        rospy.Subscriber('/linked_execution/monitor_goal',
                         JointState, self.on_goal, queue_size=1)
        rospy.Subscriber(self.sim_joint_states_topic,
                         JointState, self.on_state, queue_size=10)
        rospy.Subscriber('/linked_execution/monitor_control',
                         String, self.on_control, queue_size=1)

        self._publish_status("IDLE")
        rospy.loginfo("[unity_execution_monitor] tolerance=%.4f rad, timeout=%.1fs",
                      self.tolerance, self.timeout_sec)

    def on_goal(self, msg):
        if not msg.name or not msg.position:
            return
        self.goal_position = dict(zip(msg.name, msg.position))
        self.tracking_start = rospy.Time.now()
        self._publish_status("TRACKING")
        rospy.loginfo("[unity_execution_monitor] new goal received, tracking started")

    def on_control(self, msg):
        if msg.data == "RESET":
            self.goal_position = None
            self.tracking_start = None
            self._publish_status("IDLE")
            rospy.loginfo("[unity_execution_monitor] reset")

    def on_state(self, msg):
        if self.goal_position is None or self.status != "TRACKING":
            return

        # 检查所有目标关节误差
        max_err = 0.0
        for name, target in self.goal_position.items():
            if name not in msg.name:
                return
            idx = msg.name.index(name)
            err = abs(msg.position[idx] - target)
            max_err = max(max_err, err)

        if max_err < self.tolerance:
            self._publish_status("SUCCEEDED")
            rospy.loginfo("[unity_execution_monitor] converged (max err=%.4f rad)", max_err)
            self.goal_position = None
            return

        # 超时检查
        if self.tracking_start and \
           (rospy.Time.now() - self.tracking_start).to_sec() > self.timeout_sec:
            self._publish_status("FAILED")
            rospy.logwarn("[unity_execution_monitor] timeout (max err=%.4f rad)", max_err)
            self.goal_position = None

    def _publish_status(self, status):
        self.status = status
        self.pub_status.publish(String(data=status))


def main():
    rospy.init_node('unity_execution_monitor', anonymous=False)
    UnityExecutionMonitor()
    rospy.spin()


if __name__ == '__main__':
    main()
