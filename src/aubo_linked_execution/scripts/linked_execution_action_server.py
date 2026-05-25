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
import copy
import math

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

GAZEBO_WAIT_EXTRA = 8.0  # extra seconds beyond trajectory duration to wait for Gazebo (increased from 5.0)
JOINT_NAMES = [
    'shoulder_joint',
    'upperArm_joint',
    'foreArm_joint',
    'wrist1_joint',
    'wrist2_joint',
    'wrist3_joint',
]


def _duration_to_sec(duration):
    return duration.to_sec() if hasattr(duration, 'to_sec') else float(duration)


class LinkedExecutionActionServer(object):
    def __init__(self):
        self._monitor_goal_pub = rospy.Publisher(
            MONITOR_GOAL_TOPIC, JointState, queue_size=1)
        self._monitor_control_pub = rospy.Publisher(
            MONITOR_CONTROL_TOPIC, String, queue_size=1)

        self._safe_to_execute = True
        self._last_safety_msg_time = rospy.Time.now()
        self._safety_watchdog_timeout = rospy.get_param('~safety_watchdog_timeout', 5.0)
        self._trajectory_start_tolerance = rospy.get_param('~trajectory_start_tolerance', 0.05)
        self._joint_jump_threshold = rospy.get_param('~joint_jump_threshold', 0.35)
        self._max_retime_scale = rospy.get_param('~max_retime_scale', 6.0)
        self._simulator_sample_dt = rospy.get_param('~simulator_sample_dt', 0.005)
        self._robot_waypoint_dt = rospy.get_param('~robot_waypoint_dt', 0.002)
        self._max_robot_velocity = self._get_float_list_param(
            '~max_robot_velocity', [0.5, 0.5, 0.5, 0.6, 0.6, 0.6])
        self._max_robot_acceleration = self._get_float_list_param(
            '~max_robot_acceleration', [2.0, 2.0, 2.0, 2.4, 2.4, 2.4])
        self._real_timeout_scale = rospy.get_param('~real_timeout_scale', 3.0)
        self._real_timeout_margin = rospy.get_param('~real_timeout_margin', 30.0)

        self._safety_sub = rospy.Subscriber(
            SAFETY_MONITOR_TOPIC, Bool, self._safety_cb, queue_size=1)
        self._joint_lock = threading.Lock()
        self._current_joint_state = None
        self._joint_state_sub = rospy.Subscriber(
            '/real/joint_states', JointState, self._joint_state_cb, queue_size=1)

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
    @staticmethod
    def _get_float_list_param(name, default):
        values = rospy.get_param(name, default)
        try:
            values = [float(v) for v in values]
        except (TypeError, ValueError):
            rospy.logwarn('LinkedExecutionActionServer: invalid %s, using default %s',
                          name, default)
            values = default
        if len(values) < len(default):
            values = list(values) + list(default[len(values):])
        return values[:len(default)]

    def _safety_cb(self, msg):
        self._safe_to_execute = msg.data
        self._last_safety_msg_time = rospy.Time.now()

    def _joint_state_cb(self, msg):
        with self._joint_lock:
            self._current_joint_state = msg

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

        traj = goal.trajectory

        # 1. Reject clearly unsafe trajectories before they reach the driver.
        unsafe_reason = self._unsafe_trajectory_reason(traj)
        if unsafe_reason:
            rospy.logerr('LinkedExecutionActionServer: rejected trajectory: %s', unsafe_reason)
            result = FollowJointTrajectoryResult()
            result.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            self._server.set_aborted(result, unsafe_reason)
            return

        # 2. Retime trajectories for the real AUBO controller.  RViz Plan&Execute
        # uses MoveIt's geometric path directly; this layer adds the dynamic guard
        # that the planner itself does not model (controller collision/torque trip).
        velocity_scale = self._velocity_scale()
        retimed_goal, retime_scale, retime_reason = self._retime_goal_for_real_robot(
            goal, velocity_scale)
        if retimed_goal is None:
            rospy.logerr('LinkedExecutionActionServer: rejected trajectory: %s', retime_reason)
            result = FollowJointTrajectoryResult()
            result.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            self._server.set_aborted(result, retime_reason)
            return
        if retime_scale > 1.001:
            rospy.logwarn('LinkedExecutionActionServer: retimed trajectory by %.2fx (%s)',
                          retime_scale, retime_reason)

        # 3. Compute trajectory duration for timeout budget.
        # aubo_robot_simulator expands time_from_start by
        # /aubo_controller/velocity_scale_factor before streaming to the driver,
        # so the real execution budget must use the expanded duration.
        traj = retimed_goal.trajectory
        if traj.points:
            duration_secs = traj.points[-1].time_from_start.to_sec()
        else:
            duration_secs = 0.0
        effective_duration_secs = duration_secs / velocity_scale

        # 4. Send goal to real robot
        self._real_client.send_goal(
            retimed_goal,
            done_cb=None,
            active_cb=None,
            feedback_cb=self._forward_feedback)

        # 5. Publish trajectory end-point to monitor
        self._publish_monitor_goal(traj, effective_duration_secs)

        # 6. Wait for the real robot.  On the AUBO driver path the controller
        # consumes dense waypoints more slowly than MoveIt's nominal duration,
        # especially after the simulator expands time by velocity_scale_factor.
        # A short timeout here cancels a still-moving robot before it reaches
        # the target, leaving stale queued points behind.
        real_timeout = (
            effective_duration_secs * self._real_timeout_scale +
            self._real_timeout_margin)
        rospy.loginfo('LinkedExecutionActionServer: waiting %.1fs for real action '
                      '(effective trajectory %.1fs)',
                      real_timeout, effective_duration_secs)
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

        # 7. Wait for Gazebo to converge (advisory only — real robot success is what matters)
        gazebo_timeout = rospy.Duration(effective_duration_secs + GAZEBO_WAIT_EXTRA)
        gazebo_ok = self._wait_for_gazebo(gazebo_timeout)

        self._monitor_control_pub.publish(String(data='RESET'))

        if not gazebo_ok:
            rospy.logwarn('LinkedExecutionActionServer: Gazebo did not converge — '
                          'proceeding with real robot success only (advisory warning)')

        rospy.loginfo('LinkedExecutionActionServer: real robot SUCCEEDED, Gazebo %s',
                      'OK' if gazebo_ok else 'TIMEOUT (ignored)')
        self._server.set_succeeded(real_result if real_result else FollowJointTrajectoryResult())

    # ------------------------------------------------------------------
    def _velocity_scale(self):
        try:
            return max(0.05, float(rospy.get_param('/aubo_controller/velocity_scale_factor', 1.0)))
        except (TypeError, ValueError):
            return 1.0

    def _unsafe_trajectory_reason(self, traj):
        if not traj.points:
            return 'empty trajectory'

        if len(traj.joint_names) != len(JOINT_NAMES):
            return 'unexpected joint count: %d' % len(traj.joint_names)

        for name in traj.joint_names:
            if name not in JOINT_NAMES:
                return 'unexpected joint name: %s' % name

        for point_idx, point in enumerate(traj.points):
            if len(point.positions) != len(traj.joint_names):
                return 'point %d has invalid position dimension' % point_idx
            for value in point.positions:
                if math.isnan(value) or math.isinf(value):
                    return 'point %d contains non-finite joint position' % point_idx

        timing_reason = self._trajectory_timing_reason(traj)
        if timing_reason:
            return timing_reason

        start_reason = self._trajectory_start_gap_reason(traj)
        if start_reason:
            return start_reason

        jump_reason = self._joint_jump_reason(traj)
        if jump_reason:
            return jump_reason

        return ''

    @staticmethod
    def _trajectory_timing_reason(traj, min_dt=1e-6):
        last_t = None
        for point_idx, point in enumerate(traj.points):
            t = _duration_to_sec(point.time_from_start)
            if math.isnan(t) or math.isinf(t):
                return 'point %d has non-finite time_from_start' % point_idx
            if last_t is not None:
                dt = t - last_t
                if dt <= 0.0:
                    return 'point %d has non-increasing timestamp: dt=%.9fs' % (
                        point_idx, dt)
                if dt < min_dt:
                    return 'point %d timestamp interval %.9fs < %.9fs' % (
                        point_idx, dt, min_dt)
            last_t = t
        if _duration_to_sec(traj.points[-1].time_from_start) <= 0.0:
            return 'trajectory duration is zero'
        return ''

    def _trajectory_start_gap_reason(self, traj):
        with self._joint_lock:
            current_msg = copy.deepcopy(self._current_joint_state)

        if current_msg is None or not current_msg.position:
            try:
                current_msg = rospy.wait_for_message(
                    '/real/joint_states', JointState, timeout=0.5)
            except rospy.ROSException:
                current_msg = None

        if current_msg is None or not current_msg.position:
            return 'no /real/joint_states available for trajectory start validation'

        current = dict(zip(current_msg.name, current_msg.position))
        first = traj.points[0]
        max_gap = 0.0
        max_joint = ''
        for idx, name in enumerate(traj.joint_names):
            if name not in current:
                continue
            gap = abs(first.positions[idx] - current[name])
            if gap > max_gap:
                max_gap = gap
                max_joint = name
        if max_gap > self._trajectory_start_tolerance:
            return ('trajectory start mismatch on %s: %.3f rad > %.3f rad' %
                    (max_joint, max_gap, self._trajectory_start_tolerance))
        return ''

    def _joint_jump_reason(self, traj):
        last = list(traj.points[0].positions)
        for point_idx, point in enumerate(traj.points[1:], start=1):
            current = list(point.positions)
            for joint_idx, (prev, now) in enumerate(zip(last, current)):
                delta = abs(now - prev)
                if delta > self._joint_jump_threshold:
                    joint_name = traj.joint_names[joint_idx]
                    return ('trajectory branch jump on %s at point %d: %.3f rad > %.3f rad' %
                            (joint_name, point_idx, delta, self._joint_jump_threshold))
            last = current
        return ''

    def _streaming_time_factor(self):
        robot_dt = max(0.0005, float(self._robot_waypoint_dt))
        return max(1.0, float(self._simulator_sample_dt) / robot_dt)

    def _retime_goal_for_real_robot(self, goal, velocity_scale):
        traj = goal.trajectory
        required_scale, reason = self._required_retime_scale(traj, velocity_scale)
        if required_scale > self._max_retime_scale:
            return None, required_scale, (
                'required retime scale %.2fx exceeds limit %.2fx (%s)' %
                (required_scale, self._max_retime_scale, reason))

        if required_scale <= 1.001:
            return goal, 1.0, 'within dynamic limits'

        retimed_goal = copy.deepcopy(goal)
        self._scale_trajectory_time(retimed_goal.trajectory, required_scale)
        return retimed_goal, required_scale, reason

    def _required_retime_scale(self, traj, velocity_scale):
        n_joints = len(traj.joint_names)
        stream_factor = self._streaming_time_factor()
        robot_vel_factor = velocity_scale * stream_factor
        robot_acc_factor = velocity_scale * velocity_scale * stream_factor * stream_factor

        required_scale = 1.0
        limiting_reason = 'within dynamic limits'

        nonlocal_required = [required_scale]
        nonlocal_reason = [limiting_reason]

        def consider_velocity(joint_idx, velocity, source):
            max_velocity = self._max_robot_velocity[joint_idx]
            if max_velocity <= 0.0:
                return
            predicted = abs(velocity) * robot_vel_factor
            ratio = predicted / max_velocity
            if ratio > nonlocal_required[0]:
                nonlocal_required[0] = ratio
                nonlocal_reason[0] = (
                    '%s predicted %s velocity %.3f rad/s > %.3f rad/s' %
                    (traj.joint_names[joint_idx], source, predicted, max_velocity))

        def consider_acceleration(joint_idx, acceleration, source):
            max_acceleration = self._max_robot_acceleration[joint_idx]
            if max_acceleration <= 0.0:
                return
            predicted = abs(acceleration) * robot_acc_factor
            ratio = math.sqrt(predicted / max_acceleration)
            if ratio > nonlocal_required[0]:
                nonlocal_required[0] = ratio
                nonlocal_reason[0] = (
                    '%s predicted %s acceleration %.3f rad/s^2 > %.3f rad/s^2' %
                    (traj.joint_names[joint_idx], source, predicted, max_acceleration))

        for point in traj.points:
            if len(point.velocities) == n_joints:
                for idx, velocity in enumerate(point.velocities):
                    consider_velocity(idx, velocity, 'planned')
            if len(point.accelerations) == n_joints:
                for idx, acceleration in enumerate(point.accelerations):
                    consider_acceleration(idx, acceleration, 'planned')

        last_time = _duration_to_sec(traj.points[0].time_from_start)
        last_pos = list(traj.points[0].positions)
        last_segment_velocity = None
        for point_idx, point in enumerate(traj.points[1:], start=1):
            current_time = _duration_to_sec(point.time_from_start)
            dt = current_time - last_time
            if dt <= 1e-6:
                last_time = current_time
                last_pos = list(point.positions)
                continue

            segment_velocity = []
            for idx, (prev, now) in enumerate(zip(last_pos, point.positions)):
                velocity = (now - prev) / dt
                segment_velocity.append(velocity)
                consider_velocity(idx, velocity, 'segment')

            if last_segment_velocity is not None:
                for idx, (prev_v, now_v) in enumerate(zip(last_segment_velocity, segment_velocity)):
                    acceleration = (now_v - prev_v) / dt
                    consider_acceleration(idx, acceleration, 'segment')

            last_segment_velocity = segment_velocity
            last_time = current_time
            last_pos = list(point.positions)

        return max(1.0, nonlocal_required[0]), nonlocal_reason[0]

    @staticmethod
    def _scale_trajectory_time(traj, time_scale):
        n_joints = len(traj.joint_names)
        for point in traj.points:
            point.time_from_start = rospy.Duration(
                point.time_from_start.to_sec() * time_scale)
            if len(point.velocities) == n_joints:
                point.velocities = [v / time_scale for v in point.velocities]
            if len(point.accelerations) == n_joints:
                point.accelerations = [
                    a / (time_scale * time_scale) for a in point.accelerations]

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
