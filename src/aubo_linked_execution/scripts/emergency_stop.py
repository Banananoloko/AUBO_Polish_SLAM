#!/usr/bin/env python3
"""Minimal emergency stop bridge for the square demo GUI.

This module intentionally does not modify planners, goal validation, or arrival
logic.  It only cuts the active execution path by canceling action goals and
publishing the same stop primitives already understood by the AUBO ROS bridge.
"""

import rospy
import actionlib

from actionlib_msgs.msg import GoalID
from control_msgs.msg import FollowJointTrajectoryAction
from std_msgs.msg import String, UInt8
from trajectory_msgs.msg import JointTrajectory


DEFAULT_JOINT_NAMES = [
    'shoulder_joint',
    'upperArm_joint',
    'foreArm_joint',
    'wrist1_joint',
    'wrist2_joint',
    'wrist3_joint',
]

ACTION_NAMESPACES = [
    'linked_execution_controller/follow_joint_trajectory',
    'aubo_e5_controller/follow_joint_trajectory',
]

CANCEL_TOPICS = [
    '/linked_execution_controller/follow_joint_trajectory/cancel',
    '/aubo_e5_controller/follow_joint_trajectory/cancel',
    '/move_group/cancel',
    '/execute_trajectory/cancel',
]


def _joint_names():
    names = rospy.get_param('/controller_joint_names', DEFAULT_JOINT_NAMES)
    try:
        names = [str(name) for name in names if str(name)]
    except TypeError:
        names = list(DEFAULT_JOINT_NAMES)
    return names or list(DEFAULT_JOINT_NAMES)


def _cancel_action_clients(timeout=0.05):
    canceled = []
    for namespace in ACTION_NAMESPACES:
        try:
            client = actionlib.SimpleActionClient(namespace, FollowJointTrajectoryAction)
            if client.wait_for_server(rospy.Duration(timeout)):
                client.cancel_all_goals()
                canceled.append(namespace)
        except Exception as exc:
            rospy.logwarn('Emergency stop: action cancel failed for %s: %s',
                          namespace, exc)
    return canceled


def issue_emergency_stop(move_group=None, execution_client=None):
    """Request an immediate non-destructive stop of the ROS execution chain.

    Args:
        move_group: optional MoveGroupCommander, used only for stop()/clear.
        execution_client: optional active SimpleActionClient from the GUI control
            path.  If supplied, its current goal is canceled directly.

    Returns:
        dict with a compact summary for GUI logging.
    """
    summary = {
        'actions_canceled': [],
        'empty_trajectory': False,
        'driver_cancel': False,
        'move_group_stop': False,
    }

    rospy.logwarn('Emergency stop requested: canceling planning/execution chain')

    try:
        if execution_client is not None:
            execution_client.cancel_all_goals()
            summary['actions_canceled'].append('active_gui_client')
    except Exception as exc:
        rospy.logwarn('Emergency stop: active GUI action cancel failed: %s', exc)

    summary['actions_canceled'].extend(_cancel_action_clients())

    try:
        for topic in CANCEL_TOPICS:
            pub = rospy.Publisher(topic, GoalID, queue_size=1)
            pub.publish(GoalID())
    except Exception as exc:
        rospy.logwarn('Emergency stop: action cancel topic publish failed: %s', exc)

    try:
        if move_group is not None:
            move_group.stop()
            move_group.clear_pose_targets()
            summary['move_group_stop'] = True
    except Exception as exc:
        rospy.logwarn('Emergency stop: MoveGroup stop failed: %s', exc)

    try:
        empty = JointTrajectory()
        empty.joint_names = _joint_names()
        traj_pub = rospy.Publisher('/joint_path_command', JointTrajectory, queue_size=3)
        rospy.sleep(0.03)
        for _ in range(3):
            traj_pub.publish(empty)
            rospy.sleep(0.01)
        summary['empty_trajectory'] = True
    except Exception as exc:
        rospy.logwarn('Emergency stop: empty trajectory publish failed: %s', exc)

    try:
        cancel_pub = rospy.Publisher('/aubo_driver/cancel_trajectory', UInt8, queue_size=3)
        reset_pub = rospy.Publisher('/linked_execution/monitor_control', String, queue_size=1)
        event_pub = rospy.Publisher('/trajectory_execution_event', String, queue_size=1)
        rospy.sleep(0.03)
        for _ in range(3):
            cancel_pub.publish(UInt8(data=1))
            rospy.sleep(0.01)
        reset_pub.publish(String(data='RESET'))
        event_pub.publish(String(data='stop'))
        summary['driver_cancel'] = True
    except Exception as exc:
        rospy.logwarn('Emergency stop: driver cancel publish failed: %s', exc)

    rospy.logwarn('Emergency stop request sent: %s', summary)
    return summary
