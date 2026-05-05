#!/usr/bin/env python
"""
linked_execution_monitor.py

Monitors whether Gazebo has converged to a target joint position after a
trajectory is executed on the real robot.

Topics:
  Subscribe:
    /linked_execution/monitor_goal    (sensor_msgs/JointState)  — target joints
    /linked_execution/monitor_control (std_msgs/String)         — "RESET" → IDLE
    /aubo_e5/joint_states             (sensor_msgs/JointState)  — Gazebo state
  Publish:
    /linked_execution/monitor_status  (std_msgs/String)         — state string

State machine: IDLE → TRACKING → SUCCEEDED / FAILED
"""

import rospy
import threading
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class LinkedExecutionMonitor(object):
    IDLE      = 'IDLE'
    TRACKING  = 'TRACKING'
    SUCCEEDED = 'SUCCEEDED'
    FAILED    = 'FAILED'

    def __init__(self):
        self._lock = threading.Lock()
        self._state = self.IDLE

        self._goal_positions = {}   # joint_name → position
        self._tracking_start = None
        self._timeout = 0.0
        self._consecutive_ok = 0

        self._tolerance    = rospy.get_param('~joint_tolerance', 0.03)
        self._success_cycles = int(rospy.get_param('~success_cycles', 3))
        self._timeout_margin = rospy.get_param('~timeout_margin', 2.0)

        self._status_pub = rospy.Publisher(
            '/linked_execution/monitor_status', String, queue_size=1, latch=True)

        rospy.Subscriber('/linked_execution/monitor_goal',
                         JointState, self._goal_cb, queue_size=1)
        rospy.Subscriber('/linked_execution/monitor_control',
                         String, self._control_cb, queue_size=1)
        rospy.Subscriber('/aubo_e5/joint_states',
                         JointState, self._gazebo_cb, queue_size=1)

        self._publish_status(self.IDLE)

    # ------------------------------------------------------------------
    def _publish_status(self, status):
        self._state = status
        self._status_pub.publish(String(data=status))

    def _goal_cb(self, msg):
        with self._lock:
            self._goal_positions = dict(zip(msg.name, msg.position))
            # timeout = trajectory_duration hint stored in msg.header stamp seconds field
            # (action server encodes it there); fall back to margin only
            hint = msg.header.stamp.to_sec()
            self._timeout = hint + self._timeout_margin if hint > 0.0 else self._timeout_margin
            self._tracking_start = rospy.Time.now()
            self._consecutive_ok = 0
            self._publish_status(self.TRACKING)
            rospy.loginfo('LinkedExecutionMonitor: TRACKING started, timeout=%.1fs', self._timeout)

    def _control_cb(self, msg):
        if msg.data == 'RESET':
            with self._lock:
                self._goal_positions = {}
                self._consecutive_ok = 0
                self._tracking_start = None
                self._publish_status(self.IDLE)
                rospy.loginfo('LinkedExecutionMonitor: RESET → IDLE')

    def _gazebo_cb(self, msg):
        with self._lock:
            if self._state != self.TRACKING:
                return

            elapsed = (rospy.Time.now() - self._tracking_start).to_sec()
            if elapsed > self._timeout:
                rospy.logwarn('LinkedExecutionMonitor: timeout after %.1fs — FAILED', elapsed)
                self._publish_status(self.FAILED)
                return

            name_to_pos = dict(zip(msg.name, msg.position))
            all_ok = True
            for joint, target in self._goal_positions.items():
                actual = name_to_pos.get(joint)
                if actual is None:
                    all_ok = False
                    break
                if abs(actual - target) > self._tolerance:
                    all_ok = False
                    break

            if all_ok:
                self._consecutive_ok += 1
                if self._consecutive_ok >= self._success_cycles:
                    rospy.loginfo('LinkedExecutionMonitor: SUCCEEDED after %.1fs', elapsed)
                    self._publish_status(self.SUCCEEDED)
            else:
                self._consecutive_ok = 0


if __name__ == '__main__':
    rospy.init_node('linked_execution_monitor')
    LinkedExecutionMonitor()
    rospy.spin()
