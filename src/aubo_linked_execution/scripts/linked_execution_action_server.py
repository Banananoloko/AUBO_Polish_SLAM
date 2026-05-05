#!/usr/bin/env python
"""
linked_execution_action_server.py

Aggregation layer that:
  1. Accepts FollowJointTrajectory goals from MoveIt (as "linked_execution_controller")
  2. Forwards the goal to the real robot action server (aubo_e5_controller)
  3. Publishes the trajectory end-point to the Gazebo convergence monitor
  4. Waits for both the real robot and Gazebo to succeed
  5. Reports overall SUCCESS / ABORT to MoveIt

Action Server: linked_execution_controller/follow_joint_trajectory
Action Client: aubo_e5_controller/follow_joint_trajectory
"""

import rospy
import actionlib
import threading

from control_msgs.msg import (FollowJointTrajectoryAction,
                               FollowJointTrajectoryGoal,
                               FollowJointTrajectoryResult,
                               FollowJointTrajectoryFeedback)
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool

REAL_ACTION_NS  = 'aubo_e5_controller/follow_joint_trajectory'
SERVER_ACTION_NS = 'linked_execution_controller/follow_joint_trajectory'

MONITOR_STATUS_TOPIC  = '/linked_execution/monitor_status'
MONITOR_GOAL_TOPIC    = '/linked_execution/monitor_goal'
MONITOR_CONTROL_TOPIC = '/linked_execution/monitor_control'
SAFETY_MONITOR_TOPIC  = '/safety_monitor/safe_to_execute'

GAZEBO_WAIT_EXTRA = 5.0  # extra seconds beyond trajectory duration to wait for Gazebo


class LinkedExecutionActionServer(object):
    def __init__(self):
        self._monitor_goal_pub = rospy.Publisher(
            MONITOR_GOAL_TOPIC, JointState, queue_size=1)
        self._monitor_control_pub = rospy.Publisher(
            MONITOR_CONTROL_TOPIC, String, queue_size=1)

        self._safe_to_execute = True
        self._last_safety_msg_time = rospy.Time.now()
        self._safety_watchdog_timeout = rospy.get_param('~safety_watchdog_timeout', 5.0)

        self._safety_sub = rospy.Subscriber(
            SAFETY_MONITOR_TOPIC, Bool, self._safety_cb, queue_size=1)

        # 启动看门狗定时器
        self._watchdog_timer = rospy.Timer(rospy.Duration(1.0), self._watchdog_callback)

        self._real_client = actionlib.SimpleActionClient(
            REAL_ACTION_NS, FollowJointTrajectoryAction)
        rospy.loginfo('LinkedExecutionActionServer: waiting for real robot action server...')
        wait_timeout = rospy.get_param('~real_server_wait_timeout', 30.0)
        if not self._real_client.wait_for_server(rospy.Duration(wait_timeout)):
            rospy.logfatal(
                'LinkedExecutionActionServer: timed out waiting for %s after %.0fs. '
                'Is aubo_driver running and connected? '
                'Start with sim_only:=true to skip real robot.',
                REAL_ACTION_NS, wait_timeout)
            rospy.signal_shutdown('real robot action server unavailable')
            return
        rospy.loginfo('LinkedExecutionActionServer: real robot action server connected')

        self._server = actionlib.SimpleActionServer(
            SERVER_ACTION_NS,
            FollowJointTrajectoryAction,
            execute_cb=self._execute_cb,
            auto_start=False)
        self._server.start()
        rospy.loginfo('LinkedExecutionActionServer: ready at %s', SERVER_ACTION_NS)

    # ------------------------------------------------------------------
    def _safety_cb(self, msg):
        self._safe_to_execute = msg.data
        self._last_safety_msg_time = rospy.Time.now()

    def _watchdog_callback(self, event):
        """看门狗：检测 safety_monitor 是否存活"""
        elapsed = (rospy.Time.now() - self._last_safety_msg_time).to_sec()
        if elapsed > self._safety_watchdog_timeout:
            if self._safe_to_execute:  # 只在状态变化时记录
                rospy.logerr('[linked_execution] Safety monitor watchdog timeout (%.1fs)! Blocking execution.', elapsed)
            self._safe_to_execute = False

    def _execute_cb(self, goal):
        rospy.loginfo('LinkedExecutionActionServer: received goal')

        # 0. Check safety monitor status
        if not self._safe_to_execute:
            rospy.logerr('LinkedExecutionActionServer: execution blocked by safety monitor')
            result = FollowJointTrajectoryResult()
            result.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            self._server.set_aborted(result, 'Execution blocked by safety monitor')
            return

        # 1. Compute trajectory duration for timeout budget
        traj = goal.trajectory
        if traj.points:
            duration_secs = traj.points[-1].time_from_start.to_sec()
        else:
            duration_secs = 0.0

        # 2. Send goal to real robot
        self._real_client.send_goal(
            goal,
            done_cb=None,
            active_cb=None,
            feedback_cb=self._forward_feedback)

        # 3. Publish trajectory end-point to monitor
        self._publish_monitor_goal(traj, duration_secs)

        # 4. Wait for real robot (with per-point timeout + safety margin)
        real_timeout = duration_secs + 10.0
        real_done = self._real_client.wait_for_result(
            rospy.Duration(real_timeout))

        if self._server.is_preempt_requested():
            rospy.logwarn('LinkedExecutionActionServer: preempt requested')
            self._real_client.cancel_goal()
            self._monitor_control_pub.publish(String(data='RESET'))
            self._server.set_preempted()
            return

        if not real_done:
            rospy.logerr('LinkedExecutionActionServer: real robot action timed out')
            self._real_client.cancel_goal()
            self._monitor_control_pub.publish(String(data='RESET'))
            result = FollowJointTrajectoryResult()
            result.error_code = FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED
            self._server.set_aborted(result, 'Real robot action timed out')
            return

        real_state = self._real_client.get_state()
        real_result = self._real_client.get_result()

        if real_state != actionlib.GoalStatus.SUCCEEDED:
            rospy.logerr('LinkedExecutionActionServer: real robot action failed, state=%d', real_state)
            self._monitor_control_pub.publish(String(data='RESET'))
            result = real_result if real_result else FollowJointTrajectoryResult()
            self._server.set_aborted(result, 'Real robot execution failed')
            return

        rospy.loginfo('LinkedExecutionActionServer: real robot SUCCEEDED, waiting for Gazebo...')

        # 5. Wait for Gazebo to converge
        gazebo_timeout = rospy.Duration(duration_secs + GAZEBO_WAIT_EXTRA)
        gazebo_ok = self._wait_for_gazebo(gazebo_timeout)

        self._monitor_control_pub.publish(String(data='RESET'))

        if not gazebo_ok:
            rospy.logwarn('LinkedExecutionActionServer: Gazebo did not converge — aborting')
            result = FollowJointTrajectoryResult()
            result.error_code = FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED
            self._server.set_aborted(result, 'Gazebo mirror did not converge')
            return

        rospy.loginfo('LinkedExecutionActionServer: both real and Gazebo SUCCEEDED')
        self._server.set_succeeded(real_result if real_result else FollowJointTrajectoryResult())

    # ------------------------------------------------------------------
    def _forward_feedback(self, feedback):
        self._server.publish_feedback(feedback)

    def _publish_monitor_goal(self, traj, duration_secs):
        if not traj.points:
            return
        end_point = traj.points[-1]
        js = JointState()
        js.header.stamp = rospy.Time(duration_secs)  # encode duration as timestamp hint
        js.name = list(traj.joint_names)
        js.position = list(end_point.positions)
        self._monitor_goal_pub.publish(js)

    def _wait_for_gazebo(self, timeout):
        deadline = rospy.Time.now() + timeout
        rate = rospy.Rate(10)
        while rospy.Time.now() < deadline:
            if self._server.is_preempt_requested():
                return False
            try:
                status_msg = rospy.wait_for_message(
                    MONITOR_STATUS_TOPIC, String, timeout=0.5)
                if status_msg.data == 'SUCCEEDED':
                    return True
                if status_msg.data == 'FAILED':
                    return False
            except rospy.ROSException:
                pass
            rate.sleep()
        return False


if __name__ == '__main__':
    rospy.init_node('linked_execution_action_server')
    LinkedExecutionActionServer()
    rospy.spin()
