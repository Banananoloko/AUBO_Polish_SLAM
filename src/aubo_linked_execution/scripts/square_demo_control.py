#!/usr/bin/env python3
"""
square_demo_control.py
AUBO E5 正方形轨迹演示 — 虚实同步控制端

用法:
    rosrun aubo_linked_execution square_demo_control.py

前提: 先通过 run_square_demo.sh 启动系统 (RViz + MoveIt + Gazebo)
"""

import sys
import time
import math
import threading
import subprocess
import os
import argparse
import copy

import rospy
import actionlib
import moveit_commander
import geometry_msgs.msg
from actionlib_msgs.msg import GoalStatus
from actionlib_msgs.msg import GoalID
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
from std_msgs.msg import Header, String, UInt8
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ---- ANSI 颜色 ----
C = {
    'R': '\033[0;31m',
    'G': '\033[0;32m',
    'Y': '\033[1;33m',
    'C': '\033[0;36m',
    'B': '\033[0;34m',
    'W': '\033[1;37m',
    'N': '\033[0m',
}

# ---- 坐标系说明 ----
# 示教器 报告的是 base_link 系（无 pedestal 偏移）
# ROS/MoveIt 使用的是 URDF world 系（含 pedestal 0.503m Z 偏移）
# 本脚本所有用户 I/O 使用示教器坐标系，与 MoveIt 交互时通过 to_world/to_teach 转换
PEDESTAL_Z = 0.503               # URDF pedestal_joint 的 Z 偏移 (world系 - base_link系)

# ---- 正方形轨迹参数 ----
# YZ 平面, 固定 X=0.4m, 边长 20cm
# 坐标: 示教器坐标系 (与示教器读数一致)
SQUARE_CORNERS = [
    (0.4, -0.10, 0.50),
    (0.4, -0.10, 0.70),
    (0.4,  0.10, 0.70),
    (0.4,  0.10, 0.50),
]

# ---- 运动参数 ----
# joint_limits.yaml 已将 ITP 速度硬限制在低速区间。
# 发送到底层 action 前再扩展时间戳，避免 MoveIt 执行管理器过早抢占仍在运动的实机。
VELOCITY_SCALING = 1.0
ACCEL_SCALING    = 1.0
EEF_STEP         = 0.005        # 笛卡尔路径步长 5mm
PLANNING_TIME    = 10.0
GOAL_POS_TOL     = 0.01         # 1cm
GOAL_ORI_TOL     = 0.05
GOAL_JOINT_TOL   = 0.01         # 0.01 rad
ARRIVAL_POS_TOL  = 0.020        # 2cm
ARRIVAL_CONSEC   = 2            # 连续采样次数 (go()已确认到达,此处仅冗余校验)
ARRIVAL_TIMEOUT  = 5.0          # 到达超时 (秒)
MAX_RETRIES      = 2            # 仅用于未下发轨迹时的重新规划
POSITION_ONLY_JOINT_STEP_LIMIT = 0.35  # rad, reject fallback plans with branch jumps
DIRECT_EXECUTION_TIME_SCALE = 3.0      # stretch MoveIt timing before handing to robot
POST_FAILURE_ARRIVAL_TIMEOUT = 60.0    # wait instead of retrying while robot may still move
ACTION_WAIT_TIME_SCALE = 3.0           # real arm can lag MoveIt's nominal timing on long moves
ACTION_WAIT_MARGIN = 60.0              # GUI must wait longer than linked action timeout
CARTESIAN_INTERPOLATION_STEP = 0.020    # 2cm user-waypoint densification for line/circle paths
CARTESIAN_MIN_FRACTION = 0.98           # reject partial Cartesian paths before any robot motion
CARTESIAN_JUMP_THRESHOLD = 2.0          # MoveIt relative jump guard; exact guard below is stricter
CARTESIAN_WRIST3_RANGE_LIMIT = 0.12     # rad, keep tool roll/wrist3 effectively locked
PRE_EXECUTION_CLEAR_SETTLE = 0.45       # seconds, let cancel/empty-trajectory clear old queues
FINAL_FK_TARGET_TOL = 0.060             # hard gate: planned final FK must match this stage target
MIN_TRAJECTORY_DT = 0.002               # seconds, real controller consumes waypoints at 2ms
DRIVER_STREAM_DT = 0.002                # aubo_driver differentiates every MAC 2ms waypoint
DRIVER_WAYPOINT_MARGIN = 0.8            # keep below driver hard reject thresholds
JOINT_SOFT_LIMIT = 3.05                 # URDF joint lower/upper absolute value (rad)
EXECUTION_ACTIONS = [
    'linked_execution_controller/follow_joint_trajectory',
    'aubo_e5_controller/follow_joint_trajectory',
]

# ---- 工作空间参数 ----
# 肩关节在 URDF world 系中的 Z = pedestal(0.503) + base_link→shoulder(0.0495)
SHOULDER_Z       = 0.5525       # 肩关节中心 world 系 Z 坐标
MAX_REACH        = 0.886        # AUBO E5 臂展 (从肩关节中心)

# ---- 连续轨迹测试序列 (示教器坐标系) ----
TEST_SEQUENCE = [
    (0.40,  0.00, 0.60),   # 中心高位
    (0.40, -0.15, 0.55),   # 左侧
    (0.40, -0.15, 0.45),   # 左低
    (0.40,  0.15, 0.45),   # 右低
    (0.40,  0.15, 0.55),   # 右侧
    (0.40,  0.00, 0.60),   # 回中心
]

# ---- 预设工件打磨测试轨迹 (示教器坐标系) ----
GRINDING_TEST_WAYPOINTS = [
    (-0.6,  -0.08, 0.18),
    (-0.55, -0.058, 0.18),
    (-0.5,  -0.02, 0.18),
]
GRINDING_APPROACH_Z_OFFSET = 0.08
GRINDING_TOOL_RPY_DEG = (178.0, 4.5, -86.0)
GRINDING_ORI_TOL = math.radians(8.0)
GRINDING_LIFT_GAP = 0.10
GRINDING_ARC_TIME_SCALE = 8.0
GRINDING_ARC_POSES = [
    (-0.5140, 0.0800, 0.1765, 178.00, 0.82, -100.80),
    (-0.5250, 0.0960, 0.1765, 178.00, 1.07, -100.00),
    (-0.5441, 0.0974, 0.1765, 177.45, 1.20, -99.90),
    (-0.5620, 0.1002, 0.1765, 177.05, 1.34, -99.45),
    (-0.5800, 0.0998, 0.1765, 176.60, 1.47, -99.05),
    (-0.5980, 0.0970, 0.1765, 176.00, 1.57, -98.70),
    (-0.6107, 0.0912, 0.1765, 175.80, 1.65, -98.35),
    (-0.6200, 0.0800, 0.1765, 175.55, 1.73, -98.00),
    (-0.6321, 0.0780, 0.1765, 175.30, 1.83, -97.65),
    (-0.6400, 0.0700, 0.1765, 175.10, 1.93, -97.25),
    (-0.6466, 0.0636, 0.1765, 175.07, 1.95, -97.47),
    (-0.6522, 0.0562, 0.1765, 175.03, 1.98, -97.68),
    (-0.6600, 0.0500, 0.1765, 175.00, 2.00, -97.90),
]
ACTION_SERVER_MAX_RETIME_SCALE = 6.0

# ---- README 路径 (相对于本脚本目录的 ../../README.md) ----
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
README_PATH = os.path.normpath(os.path.join(_SCRIPT_DIR, '../../README.md'))


def cprint(tag, text):
    """带颜色标签的日志输出"""
    colours = {
        'INFO': C['C'], 'OK': C['G'], 'WARN': C['Y'],
        'ERROR': C['R'], 'EXEC': C['B'], 'WP': C['W'],
        'INPUT': C['C'],
    }
    prefix = colours.get(tag, C['C'])
    print('%s[%s]%s %s' % (prefix, tag, C['N'], text))


def safe_input(prompt='> '):
    """跨 Python 版本的输入函数 (无 raw_input 问题)"""
    sys.stdout.write(prompt)
    sys.stdout.flush()
    return sys.stdin.readline().strip()


def to_world(tx, ty, tz):
    """示教器坐标系 → URDF world 坐标系（加 pedestal Z 偏移）"""
    return (tx, ty, tz + PEDESTAL_Z)


def to_teach(wx, wy, wz):
    """URDF world 坐标系 → 示教器坐标系（减 pedestal Z 偏移）"""
    return (wx, wy, wz - PEDESTAL_Z)


def rpy_to_quat(roll_deg, pitch_deg, yaw_deg):
    """RPY (度) → geometry_msgs.msg.Quaternion"""
    from tf.transformations import quaternion_from_euler
    q = quaternion_from_euler(
        math.radians(roll_deg),
        math.radians(pitch_deg),
        math.radians(yaw_deg))
    return geometry_msgs.msg.Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def parse_pose_input(parts):
    """解析用户位姿输入，返回 (x, y, z, quat_or_None, error_msg).
    接受 3 个值 (仅位置, 自动选择可达方向) 或 6 个值 (位置 + RPY 度).
    """
    try:
        nums = [float(p) for p in parts]
    except ValueError:
        return None, None, None, None, '数字解析失败'
    if any(math.isinf(n) or math.isnan(n) for n in nums):
        return None, None, None, None, '坐标不允许为 inf 或 nan'
    if len(nums) == 3:
        x, y, z = nums
        return x, y, z, None, None
    elif len(nums) == 6:
        x, y, z = nums[:3]
        roll, pitch, yaw = nums[3:]
        return x, y, z, rpy_to_quat(roll, pitch, yaw), None
    else:
        return None, None, None, None, '需要 3 个数字 (x y z) 或 6 个数字 (x y z roll pitch yaw 度)'


class SquareDemoController:
    def __init__(self):
        # GUI 嵌入控制器时已初始化 ROS 节点；同一进程内不能用不同参数重复 init_node。
        if not rospy.core.is_initialized():
            rospy.init_node('square_demo_control', anonymous=True)

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("manipulator_e5")

        # 运动参数设置
        self.group.set_max_velocity_scaling_factor(VELOCITY_SCALING)
        self.group.set_max_acceleration_scaling_factor(ACCEL_SCALING)
        self.group.set_planning_time(PLANNING_TIME)
        self.group.set_num_planning_attempts(10)
        self.group.set_goal_position_tolerance(GOAL_POS_TOL)
        self.group.set_goal_orientation_tolerance(GOAL_ORI_TOL)
        self.group.set_goal_joint_tolerance(GOAL_JOINT_TOL)

        # 关节状态缓存 (线程安全)
        self._lock = threading.Lock()
        self._joint_positions = {}
        self._joint_velocities = {}
        self._joint_state_time = None
        self._gazebo_joint_time = None
        self._execution_client = None
        self._execution_action_ns = ''
        self._driver_pose = None
        self._driver_pose_wall_time = 0.0
        self._fk_client = None
        self._joint_path_clear_pub = rospy.Publisher(
            '/joint_path_command', JointTrajectory, queue_size=1)
        self._driver_cancel_pub = rospy.Publisher(
            '/aubo_driver/cancel_trajectory', UInt8, queue_size=1)
        self._monitor_control_pub = rospy.Publisher(
            '/linked_execution/monitor_control', String, queue_size=1)
        self._trajectory_event_pub = rospy.Publisher(
            '/trajectory_execution_event', String, queue_size=1)
        self._action_cancel_pubs = [
            rospy.Publisher('/linked_execution_controller/follow_joint_trajectory/cancel',
                            GoalID, queue_size=1),
            rospy.Publisher('/aubo_e5_controller/follow_joint_trajectory/cancel',
                            GoalID, queue_size=1),
            rospy.Publisher('/move_group/cancel', GoalID, queue_size=1),
            rospy.Publisher('/execute_trajectory/cancel', GoalID, queue_size=1),
        ]

        rospy.Subscriber('/joint_states', JointState,
                         self._joint_state_cb, queue_size=1)
        rospy.Subscriber('/aubo_e5/joint_states', JointState,
                         self._gazebo_joint_cb, queue_size=1)
        rospy.Subscriber('/aubo_driver/current_pose', PoseStamped,
                         self._driver_pose_cb, queue_size=1)

        cprint('INFO', 'SquareDemoController 初始化完成')
        cprint('INFO', '规划组: %s | 末端: %s' %
               (self.group.get_name(), self.group.get_end_effector_link()))
        cprint('INFO', '速度: %d%% | 加速度: %d%% | 步长: %dmm' %
               (VELOCITY_SCALING * 100, ACCEL_SCALING * 100, EEF_STEP * 1000))

    # ============================================================
    # 话题回调
    # ============================================================
    def _joint_state_cb(self, msg):
        with self._lock:
            self._joint_positions = dict(zip(msg.name, msg.position))
            if msg.velocity and len(msg.velocity) == len(msg.name):
                self._joint_velocities = dict(zip(msg.name, msg.velocity))
            self._joint_state_time = msg.header.stamp

    def _gazebo_joint_cb(self, msg):
        with self._lock:
            self._gazebo_joint_time = msg.header.stamp

    def _driver_pose_cb(self, msg):
        with self._lock:
            self._driver_pose = copy.deepcopy(msg.pose)
            self._driver_pose_wall_time = time.time()

    # ============================================================
    # 基础方法
    # ============================================================
    def get_current_ee_pose(self):
        """从 moveit_commander 获取当前末端位姿"""
        try:
            return self.group.get_current_pose().pose
        except Exception:
            return None

    def get_current_driver_pose(self, max_age=1.0):
        """Return fresh SDK/teach-pendant pose in base_link frame, if available."""
        with self._lock:
            pose = copy.deepcopy(self._driver_pose)
            age = time.time() - self._driver_pose_wall_time
        if pose is None or age > max_age:
            return None
        return pose

    def _moveit_world_offset_from_driver(self):
        """Current MoveIt wrist3 pose minus SDK pose, expressed in world frame."""
        driver_pose = self.get_current_driver_pose()
        moveit_pose = self.get_current_ee_pose()
        if driver_pose is None or moveit_pose is None:
            return (0.0, 0.0, 0.0)
        driver_wx, driver_wy, driver_wz = to_world(
            driver_pose.position.x,
            driver_pose.position.y,
            driver_pose.position.z)
        return (
            moveit_pose.position.x - driver_wx,
            moveit_pose.position.y - driver_wy,
            moveit_pose.position.z - driver_wz,
        )

    def to_moveit_target_world(self, tx, ty, tz):
        """Convert desired teach-pendant SDK pose to the MoveIt wrist3 target."""
        wx, wy, wz = to_world(tx, ty, tz)
        ox, oy, oz = self._moveit_world_offset_from_driver()
        if abs(ox) > 1e-4 or abs(oy) > 1e-4 or abs(oz) > 1e-4:
            cprint('INFO', 'SDK→MoveIt 末端修正: Δ=(%.3f, %.3f, %.3f)m' %
                   (ox, oy, oz))
        return wx + ox, wy + oy, wz + oz

    def get_current_display_pose_teach(self):
        """Pose position in the same teach-pendant frame shown by the TUI."""
        driver_pose = self.get_current_driver_pose()
        if driver_pose is not None:
            return (driver_pose.position.x,
                    driver_pose.position.y,
                    driver_pose.position.z)
        moveit_pose = self.get_current_ee_pose()
        if moveit_pose is None:
            return None
        return to_teach(moveit_pose.position.x,
                        moveit_pose.position.y,
                        moveit_pose.position.z)

    def estimate_sync_delay(self):
        """估算虚实同步延迟 (秒)"""
        with self._lock:
            jst = self._joint_state_time
            gst = self._gazebo_joint_time
        if jst is None or gst is None:
            return -1.0
        return abs((jst - gst).to_sec())

    def wait_for_system_ready(self, timeout=60.0):
        """等待 /joint_states 和 /aubo_e5/joint_states 有数据"""
        cprint('INFO', '等待系统就绪...')

        # sim_only 模式下 aubo_driver 未启动, /aubo_driver/robot_connected
        # 不存在, aubo_robot_simulator 的运动工作线程会死循环等待该参数=='1'
        # 导致轨迹永远不被插补、Gazebo 不动作、action server 超时。
        # 手动设为 '1' 解除阻塞。
        if not rospy.has_param('/aubo_driver/robot_connected'):
            rospy.set_param('/aubo_driver/robot_connected', '1')
            cprint('INFO', '已设置 /aubo_driver/robot_connected=1 (sim 模式)')

        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                has_js = bool(self._joint_positions)
                has_gz = self._gazebo_joint_time is not None
            if has_js and has_gz:
                # 等待 simulator 的 motion worker 真正启动 (约 0.5s)
                time.sleep(0.5)
                break
            time.sleep(0.5)
        else:
            cprint('ERROR', '等待 joint_states 超时')
            return False
        cprint('OK', '系统就绪 (joint_states + gazebo)')
        return True

    # ============================================================
    # 路径规划与执行 (set_pose_target + go 方式, 走 OMPL 规划)
    # ============================================================
    def verify_arrival(self, wx, wy, wz):
        """go(wait=True) 已通过 C++ action + in_motion + Gazebo monitor 三层确认到达。
        此处仅做单次 FK 采样校验，用于日志显示，不再阻塞等待。
        参数为 world 坐标系。"""
        driver_pose = self.get_current_driver_pose()
        if driver_pose is not None:
            tx, ty, tz = to_teach(wx, wy, wz)
            dx = driver_pose.position.x - tx
            dy = driver_pose.position.y - ty
            dz = driver_pose.position.z - tz
            err = math.sqrt(dx * dx + dy * dy + dz * dz)
            return err < ARRIVAL_POS_TOL, err

        pose = self.get_current_ee_pose()
        if pose is None:
            return False, float('inf')
        dx = pose.position.x - wx
        dy = pose.position.y - wy
        dz = pose.position.z - wz
        err = math.sqrt(dx * dx + dy * dy + dz * dz)
        return err < ARRIVAL_POS_TOL, err

    @staticmethod
    def _orientation_error(current, target):
        """Return shortest angular distance between two quaternions in radians."""
        dot = (
            current.x * target.x +
            current.y * target.y +
            current.z * target.z +
            current.w * target.w)
        dot = max(-1.0, min(1.0, abs(dot)))
        return 2.0 * math.acos(dot)

    def verify_pose_arrival(self, wx, wy, wz, orientation=None,
                            orientation_tolerance=GOAL_ORI_TOL):
        driver_pose = self.get_current_driver_pose()
        if driver_pose is not None:
            tx, ty, tz = to_teach(wx, wy, wz)
            dx = driver_pose.position.x - tx
            dy = driver_pose.position.y - ty
            dz = driver_pose.position.z - tz
            pos_err = math.sqrt(dx * dx + dy * dy + dz * dz)
            if orientation is None:
                return pos_err < ARRIVAL_POS_TOL, pos_err, 0.0
            ori_err = self._orientation_error(driver_pose.orientation, orientation)
            return (pos_err < ARRIVAL_POS_TOL and ori_err < orientation_tolerance,
                    pos_err, ori_err)

        pose = self.get_current_ee_pose()
        if pose is None:
            return False, float('inf'), float('inf')
        dx = pose.position.x - wx
        dy = pose.position.y - wy
        dz = pose.position.z - wz
        pos_err = math.sqrt(dx * dx + dy * dy + dz * dz)
        if orientation is None:
            return pos_err < ARRIVAL_POS_TOL, pos_err, 0.0

        ori_err = self._orientation_error(pose.orientation, orientation)
        return (pos_err < ARRIVAL_POS_TOL and ori_err < orientation_tolerance,
                pos_err, ori_err)

    @staticmethod
    def _normalize_plan_result(plan_result):
        """Handle MoveIt Python API differences across ROS releases."""
        if isinstance(plan_result, tuple):
            if len(plan_result) < 2:
                return False, None
            plan_ok = bool(plan_result[0])
            plan = plan_result[1]
        else:
            plan = plan_result
            plan_ok = True

        traj = getattr(plan, 'joint_trajectory', None)
        has_points = bool(traj and traj.points)
        return plan_ok and has_points, plan

    @staticmethod
    def _large_joint_jump_reason(plan, limit=POSITION_ONLY_JOINT_STEP_LIMIT):
        """Return a reason string if a planned trajectory contains a branch jump."""
        traj = getattr(plan, 'joint_trajectory', None)
        if not traj or not traj.points:
            return '空轨迹'

        last = list(traj.points[0].positions)
        for point_idx, point in enumerate(traj.points[1:], start=1):
            current = list(point.positions)
            if len(current) != len(last):
                return '轨迹点关节维度不一致'
            for joint_idx, (prev, now) in enumerate(zip(last, current)):
                delta = abs(now - prev)
                if delta > limit:
                    joint_name = traj.joint_names[joint_idx] if joint_idx < len(traj.joint_names) else str(joint_idx)
                    return '%s 在相邻轨迹点 %d 处跳变 %.3f rad > %.3f rad' % (
                        joint_name, point_idx, delta, limit)
            last = current
        return ''

    @staticmethod
    def _trajectory_duration(plan):
        traj = getattr(plan, 'joint_trajectory', None)
        if not traj or not traj.points:
            return 0.0
        return traj.points[-1].time_from_start.to_sec()

    @staticmethod
    def _trajectory_timing_reason(plan, min_dt=MIN_TRAJECTORY_DT):
        traj = getattr(plan, 'joint_trajectory', None)
        if not traj or not traj.points:
            return '空轨迹'
        last_t = None
        for idx, point in enumerate(traj.points):
            t = point.time_from_start.to_sec()
            if last_t is not None:
                dt = t - last_t
                if dt <= 0.0:
                    return '轨迹点 %d 时间戳不递增 (dt=%.6fs)' % (idx, dt)
                if dt < min_dt:
                    return '轨迹点 %d 间隔 %.6fs < %.6fs' % (idx, dt, min_dt)
            last_t = t
        if traj.points[-1].time_from_start.to_sec() <= 0.0:
            return '轨迹总时长为 0'
        return ''

    @staticmethod
    def _wrist3_motion_reason(plan, limit=CARTESIAN_WRIST3_RANGE_LIMIT):
        traj = getattr(plan, 'joint_trajectory', None)
        if not traj or not traj.points or 'wrist3_joint' not in traj.joint_names:
            return ''
        wrist_idx = traj.joint_names.index('wrist3_joint')
        values = [p.positions[wrist_idx] for p in traj.points
                  if len(p.positions) > wrist_idx]
        if not values:
            return ''
        span = max(values) - min(values)
        if abs(span) > limit:
            return 'wrist3_joint 规划转动 %.3f rad > %.3f rad' % (abs(span), limit)
        return ''

    @staticmethod
    def _goal_state_name(state):
        names = {
            GoalStatus.PENDING: 'PENDING',
            GoalStatus.ACTIVE: 'ACTIVE',
            GoalStatus.PREEMPTED: 'PREEMPTED',
            GoalStatus.SUCCEEDED: 'SUCCEEDED',
            GoalStatus.ABORTED: 'ABORTED',
            GoalStatus.REJECTED: 'REJECTED',
            GoalStatus.PREEMPTING: 'PREEMPTING',
            GoalStatus.RECALLING: 'RECALLING',
            GoalStatus.RECALLED: 'RECALLED',
            GoalStatus.LOST: 'LOST',
        }
        return names.get(state, str(state))

    def _velocity_scale_factor(self):
        try:
            return float(rospy.get_param('/aubo_controller/velocity_scale_factor', 1.0))
        except (TypeError, ValueError):
            return 1.0

    def _prepare_plan_for_direct_execution(self, plan, time_scale=DIRECT_EXECUTION_TIME_SCALE):
        """Copy and slow a RobotTrajectory before sending it to the low-level action."""
        safe_plan = copy.deepcopy(plan)
        traj = safe_plan.joint_trajectory
        if not traj.points:
            return safe_plan

        n_joints = len(traj.joint_names)
        for point in traj.points:
            point.time_from_start = rospy.Duration(
                point.time_from_start.to_sec() * time_scale)
            if len(point.velocities) == n_joints:
                point.velocities = [v / time_scale for v in point.velocities]
            if len(point.accelerations) == n_joints:
                point.accelerations = [a / (time_scale * time_scale)
                                       for a in point.accelerations]

        # Force clean start/end boundary conditions. The driver ultimately streams
        # positions only, but zero endpoint derivatives avoid stale velocity data.
        for point in (traj.points[0], traj.points[-1]):
            point.velocities = [0.0] * n_joints
            point.accelerations = [0.0] * n_joints

        return safe_plan

    @staticmethod
    def _get_float_list_param(name, default, count):
        values = rospy.get_param(name, default)
        try:
            values = [float(v) for v in values]
        except (TypeError, ValueError):
            values = list(default)
        if len(values) < count:
            values = list(values) + list(default[len(values):count])
        return values[:count]

    def _driver_safe_joint_steps(self, joint_count):
        """Return per-joint max delta accepted by the 2ms driver stream guard."""
        default_limits = [0.5, 0.5, 0.5, 0.6, 0.6, 0.6]
        velocity_limits = self._get_float_list_param(
            '/aubo_driver/velocity_safe_limits', default_limits, joint_count)
        try:
            waypoint_delta = float(rospy.get_param('/aubo_driver/max_waypoint_delta', 0.05))
        except (TypeError, ValueError):
            waypoint_delta = 0.05

        safe_steps = []
        for idx in range(joint_count):
            velocity_step = max(1e-6, velocity_limits[idx] * DRIVER_STREAM_DT)
            delta_step = max(1e-6, waypoint_delta)
            safe_steps.append(min(velocity_step, delta_step) * DRIVER_WAYPOINT_MARGIN)
        return safe_steps

    def _densify_trajectory_for_driver(self, plan):
        """Insert joint-space waypoints so no point relies on simulator-side split.

        The real aubo_driver checks every streamed waypoint against a 2ms MAC
        period.  If a MoveIt segment contains a wrist jump, relying on the
        simulator bridge to split it can still leave the driver seeing a sparse
        jump when queues get out of phase.  Densifying here makes the action goal
        itself compatible with the driver guard.
        """
        traj = getattr(plan, 'joint_trajectory', None)
        if not traj or len(traj.points) < 2:
            return plan

        joint_count = len(traj.joint_names)
        if joint_count == 0:
            return plan

        safe_steps = self._driver_safe_joint_steps(joint_count)
        safe_dt = MIN_TRAJECTORY_DT * 1.05
        new_points = []

        first = copy.deepcopy(traj.points[0])
        first.time_from_start = rospy.Duration(0.0)
        first.velocities = [0.0] * joint_count
        first.accelerations = [0.0] * joint_count
        new_points.append(first)

        last_positions = list(first.positions)
        last_original_t = traj.points[0].time_from_start.to_sec()
        elapsed = 0.0
        inserted = 0

        for point in traj.points[1:]:
            current_positions = list(point.positions)
            if len(current_positions) != joint_count:
                new_points.append(copy.deepcopy(point))
                continue

            steps = 1
            for idx, (prev, now) in enumerate(zip(last_positions, current_positions)):
                steps = max(steps, int(math.ceil(abs(now - prev) / safe_steps[idx])))

            current_original_t = point.time_from_start.to_sec()
            original_dt = max(0.0, current_original_t - last_original_t)
            segment_dt = max(original_dt, steps * safe_dt)
            sub_dt = segment_dt / float(steps)

            for step_idx in range(1, steps + 1):
                ratio = float(step_idx) / float(steps)
                new_point = JointTrajectoryPoint()
                new_point.positions = [
                    prev + (now - prev) * ratio
                    for prev, now in zip(last_positions, current_positions)
                ]
                new_point.velocities = [0.0] * joint_count
                new_point.accelerations = [0.0] * joint_count
                if hasattr(point, 'effort') and point.effort:
                    new_point.effort = list(point.effort)
                new_point.time_from_start = rospy.Duration(elapsed + sub_dt * step_idx)
                new_points.append(new_point)

            inserted += max(0, steps - 1)
            elapsed += segment_dt
            last_original_t = current_original_t
            last_positions = current_positions

        if inserted:
            cprint('INFO', '驱动安全重采样: 插入 %d 个 2ms 安全路点, 总点数 %d → %d, 时长 %.1fs → %.1fs' %
                   (inserted, len(traj.points), len(new_points),
                    traj.points[-1].time_from_start.to_sec(),
                    new_points[-1].time_from_start.to_sec()))

        traj.points = new_points
        return plan

    def _retime_trajectory(self, plan):
        """Run MoveIt time parameterization for Cartesian paths."""
        try:
            current_state = self.robot.get_current_state()
            return self.group.retime_trajectory(
                current_state, plan, VELOCITY_SCALING, ACCEL_SCALING)
        except TypeError:
            current_state = self.robot.get_current_state()
            return self.group.retime_trajectory(
                current_state, plan, VELOCITY_SCALING)
        except Exception as e:
            cprint('WARN', '轨迹时间参数化失败: %s' % e)
            return None

    def _start_gap_reason(self, plan, limit=0.15):
        traj = getattr(plan, 'joint_trajectory', None)
        if not traj or not traj.points:
            return '空轨迹'
        with self._lock:
            current = dict(self._joint_positions)
        if not current:
            return ''
        first = traj.points[0]
        max_gap = 0.0
        max_joint = ''
        for idx, name in enumerate(traj.joint_names):
            if name not in current or idx >= len(first.positions):
                continue
            gap = abs(first.positions[idx] - current[name])
            if gap > max_gap:
                max_gap = gap
                max_joint = name
        if max_gap > limit:
            return '%s 起点偏差 %.3f rad > %.3f rad' % (max_joint, max_gap, limit)
        return ''

    def _current_joint_map(self):
        with self._lock:
            return dict(self._joint_positions)

    @staticmethod
    def _nearest_equivalent_angle(value, reference):
        """Return value shifted by 2pi multiples nearest to reference."""
        if reference is None:
            return value
        candidates = []
        for k in range(-2, 3):
            candidate = value + k * 2.0 * math.pi
            if -JOINT_SOFT_LIMIT <= candidate <= JOINT_SOFT_LIMIT:
                candidates.append(candidate)
        if not candidates:
            candidates = [value]
        return min(candidates, key=lambda candidate: abs(candidate - reference))

    def _anchor_trajectory_to_current_state(self, plan):
        """Anchor the first trajectory point to live joints and unwrap later points.

        The AUBO driver differentiates consecutive streamed positions over a 2ms
        robot servo period. A first point that is only an equivalent +/-2pi branch
        away from the robot can therefore appear as thousands of rad/s. Anchoring
        point 0 to the live joint state prevents a zero-time branch jump from ever
        reaching the driver.
        """
        traj = getattr(plan, 'joint_trajectory', None)
        if not traj or not traj.points:
            return

        current = self._current_joint_map()
        if not current:
            return

        previous = []
        first = traj.points[0]
        first_positions = list(first.positions)
        for idx, name in enumerate(traj.joint_names):
            if idx >= len(first_positions):
                continue
            live_value = current.get(name)
            if live_value is not None:
                first_positions[idx] = live_value
            previous.append(first_positions[idx])
        first.positions = first_positions

        for point in traj.points[1:]:
            adjusted = list(point.positions)
            for idx, value in enumerate(adjusted):
                ref = previous[idx] if idx < len(previous) else None
                adjusted[idx] = self._nearest_equivalent_angle(value, ref)
            point.positions = adjusted
            previous = adjusted

    def _get_execution_client(self):
        if self._execution_client is not None:
            return self._execution_client

        real_driver_configured = rospy.has_param('/aubo_driver/server_host')
        for action_ns in EXECUTION_ACTIONS:
            if real_driver_configured and not action_ns.startswith('linked_execution'):
                cprint('ERROR', '实机模式禁止绕过 linked_execution_controller')
                return None
            client = actionlib.SimpleActionClient(action_ns, FollowJointTrajectoryAction)
            wait_s = 3.0 if action_ns.startswith('linked_execution') else 3.0
            if client.wait_for_server(rospy.Duration(wait_s)):
                self._execution_client = client
                self._execution_action_ns = action_ns
                cprint('INFO', '执行 action: %s' % action_ns)
                return client

        cprint('ERROR', '未找到可用 FollowJointTrajectory action server')
        return None

    def _controller_joint_names(self):
        try:
            names = rospy.get_param('/controller_joint_names', [])
            names = [str(name) for name in names if str(name)]
        except TypeError:
            names = []
        return names or [
            'shoulder_joint', 'upperArm_joint', 'foreArm_joint',
            'wrist1_joint', 'wrist2_joint', 'wrist3_joint',
        ]

    def _flush_execution_pipeline(self, tag):
        """Cancel active goals and clear simulator/driver stream before a new segment."""
        try:
            self.group.stop()
            self.group.clear_pose_targets()
        except Exception:
            pass

        if self._execution_client is not None:
            try:
                self._execution_client.cancel_all_goals()
            except Exception as exc:
                cprint('WARN', '%s: action cancel_all_goals 失败: %s' % (tag, exc))

        empty = JointTrajectory()
        empty.joint_names = self._controller_joint_names()
        cancel = GoalID()
        for _ in range(3):
            for pub in self._action_cancel_pubs:
                pub.publish(cancel)
            self._joint_path_clear_pub.publish(empty)
            self._driver_cancel_pub.publish(UInt8(data=1))
            self._trajectory_event_pub.publish(String(data='stop'))
            rospy.sleep(0.03)
        self._monitor_control_pub.publish(String(data='RESET'))
        cprint('INFO', '%s: 执行前清空旧 action/驱动流队列' % tag)
        rospy.sleep(PRE_EXECUTION_CLEAR_SETTLE)
        stopped = self._wait_until_joint_stopped(tag)
        if not stopped:
            cprint('ERROR', '%s: 清队列后关节仍在运动，拒绝发送新轨迹' % tag)
        return stopped

    def _max_abs_joint_velocity(self):
        with self._lock:
            velocities = dict(self._joint_velocities)
        if not velocities:
            return 0.0
        return max(abs(v) for v in velocities.values())

    def _wait_until_joint_stopped(self, tag, timeout=3.0, threshold=0.02):
        deadline = time.time() + timeout
        stable = 0
        max_vel = 0.0
        while time.time() < deadline and not rospy.is_shutdown():
            max_vel = self._max_abs_joint_velocity()
            if max_vel <= threshold:
                stable += 1
                if stable >= 3:
                    return True
            else:
                stable = 0
            rospy.sleep(0.1)
        cprint('WARN', '%s: 清队列后关节仍可能在动 | max_vel=%.3f rad/s' %
               (tag, max_vel))
        return False

    def _get_fk_client(self):
        if self._fk_client is not None:
            return self._fk_client
        try:
            rospy.wait_for_service('/compute_fk', timeout=1.0)
            self._fk_client = rospy.ServiceProxy('/compute_fk', GetPositionFK)
            return self._fk_client
        except Exception as exc:
            cprint('WARN', 'FK 服务不可用, 跳过终点 FK 门控: %s' % exc)
            return None

    def _fk_pose_for_joint_point(self, joint_names, positions):
        client = self._get_fk_client()
        if client is None:
            return None
        req = GetPositionFKRequest()
        req.header = Header()
        req.header.stamp = rospy.Time.now()
        req.header.frame_id = self.robot.get_planning_frame()
        req.fk_link_names = [self.group.get_end_effector_link()]
        req.robot_state = RobotState()
        req.robot_state.joint_state = JointState()
        req.robot_state.joint_state.header = req.header
        req.robot_state.joint_state.name = list(joint_names)
        req.robot_state.joint_state.position = list(positions)
        try:
            res = client(req)
        except Exception as exc:
            cprint('WARN', 'FK 服务调用失败, 跳过终点 FK 门控: %s' % exc)
            return None
        if not res.pose_stamped:
            cprint('WARN', 'FK 服务未返回末端位姿, 跳过终点 FK 门控')
            return None
        return res.pose_stamped[0].pose

    def _final_fk_target_reason(self, plan, expected_xyz, tag='', limit=FINAL_FK_TARGET_TOL):
        """Reject if a planned trajectory final pose is not this segment's target."""
        if expected_xyz is None:
            return ''
        traj = getattr(plan, 'joint_trajectory', None)
        if not traj or not traj.points:
            return '空轨迹'
        final = traj.points[-1]
        if len(final.positions) != len(traj.joint_names):
            return '终点关节维度不一致'
        pose = self._fk_pose_for_joint_point(traj.joint_names, final.positions)
        if pose is None:
            return ''
        ex, ey, ez = expected_xyz
        dx = pose.position.x - ex
        dy = pose.position.y - ey
        dz = pose.position.z - ez
        err = math.sqrt(dx * dx + dy * dy + dz * dz)
        if err > limit:
            return ('轨迹终点 FK 不匹配当前阶段目标: FK=(%.3f, %.3f, %.3f), '
                    'target=(%.3f, %.3f, %.3f), err=%.3fm > %.3fm' %
                    (pose.position.x, pose.position.y, pose.position.z,
                     ex, ey, ez, err, limit))
        if tag:
            cprint('INFO', '%s: 轨迹终点 FK 门控通过 | err=%.3fm' % (tag, err))
        return ''

    def _wait_for_arrival(self, wx, wy, wz, timeout, consecutive=3,
                          orientation=None,
                          orientation_tolerance=GOAL_ORI_TOL):
        deadline = time.time() + timeout
        ok_count = 0
        last_err = float('inf')
        last_ori_err = float('inf') if orientation is not None else 0.0
        while time.time() < deadline and not rospy.is_shutdown():
            arrived, last_err, last_ori_err = self.verify_pose_arrival(
                wx, wy, wz, orientation, orientation_tolerance)
            if arrived:
                ok_count += 1
                if ok_count >= consecutive:
                    return True, last_err, last_ori_err
            else:
                ok_count = 0
            rospy.sleep(0.2)
        return False, last_err, last_ori_err

    def _execute_plan_direct(self, plan, wx, wy, wz, tag,
                             expected_orientation=None,
                             orientation_tolerance=GOAL_ORI_TOL,
                             time_scale=DIRECT_EXECUTION_TIME_SCALE,
                             expected_plan_xyz=None):
        """Execute through FollowJointTrajectory directly, avoiding MoveIt TEM preemption."""
        prepared = self._prepare_plan_for_direct_execution(plan, time_scale=time_scale)
        self._anchor_trajectory_to_current_state(prepared)
        self._densify_trajectory_for_driver(prepared)

        jump_reason = self._large_joint_jump_reason(prepared)
        if jump_reason:
            cprint('WARN', '%s: 轨迹被拒绝: %s' % (tag, jump_reason))
            return False, False, float('inf'), -1.0

        timing_reason = self._trajectory_timing_reason(prepared)
        if timing_reason:
            cprint('WARN', '%s: 轨迹时间戳不安全: %s' % (tag, timing_reason))
            return False, False, float('inf'), -1.0

        start_reason = self._start_gap_reason(prepared)
        if start_reason:
            cprint('WARN', '%s: 轨迹起点不安全: %s' % (tag, start_reason))
            return False, False, float('inf'), -1.0

        final_reason = self._final_fk_target_reason(prepared, expected_plan_xyz, tag)
        if final_reason:
            cprint('ERROR', '%s: 轨迹被拒绝: %s' % (tag, final_reason))
            return False, False, float('inf'), -1.0

        client = self._get_execution_client()
        if client is None:
            return False, False, float('inf'), -1.0

        traj_duration = self._trajectory_duration(prepared)
        scale = max(0.05, self._velocity_scale_factor())
        effective_duration = traj_duration / scale
        try:
            server_retime_budget = float(rospy.get_param(
                '/linked_execution_action_server/max_retime_scale',
                ACTION_SERVER_MAX_RETIME_SCALE))
        except (TypeError, ValueError):
            server_retime_budget = ACTION_SERVER_MAX_RETIME_SCALE
        wait_timeout = max(
            POST_FAILURE_ARRIVAL_TIMEOUT,
            effective_duration * server_retime_budget * ACTION_WAIT_TIME_SCALE +
            ACTION_WAIT_MARGIN)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = prepared.joint_trajectory
        cprint('EXEC', '%s: direct action execute %.1fs轨迹, timeout %.1fs ...' %
               (tag, traj_duration, wait_timeout))

        if not self._flush_execution_pipeline(tag):
            return False, False, float('inf'), -1.0
        client.send_goal(goal)
        done = client.wait_for_result(rospy.Duration(wait_timeout))
        state = client.get_state() if done else GoalStatus.LOST

        if not done:
            cprint('WARN', '%s: action 等待超时, 取消目标并等待实际到位' % tag)
            client.cancel_goal()
        elif state != GoalStatus.SUCCEEDED:
            cprint('WARN', '%s: action 返回 %s, 等待实际到位' %
                   (tag, self._goal_state_name(state)))

        arrived, err, ori_err = self._wait_for_arrival(
            wx, wy, wz,
            2.0 if done and state == GoalStatus.SUCCEEDED else POST_FAILURE_ARRIVAL_TIMEOUT,
            orientation=expected_orientation,
            orientation_tolerance=orientation_tolerance)
        sync_delay = self.estimate_sync_delay()

        if state == GoalStatus.SUCCEEDED or arrived:
            if not arrived:
                if expected_orientation is not None:
                    cprint('WARN', '%s: 6-DOF 到位校验未通过 | 位置误差=%.3fm | 姿态误差=%.1f°' %
                           (tag, err, math.degrees(ori_err)))
                else:
                    cprint('WARN', '%s: 到位校验未通过 | 位置误差=%.3fm' %
                           (tag, err))
                return False, True, err, sync_delay

            if state != GoalStatus.SUCCEEDED:
                cprint('INFO', '%s: action 未成功, 但实际位姿到位校验通过' % tag)
            if expected_orientation is not None:
                cprint('INFO', '%s: 姿态到位 | 姿态误差=%.1f°' %
                       (tag, math.degrees(ori_err)))
            return True, True, err, sync_delay

        if expected_orientation is not None:
            cprint('WARN', '%s: 6-DOF 到位校验未通过 | 位置误差=%.3fm | 姿态误差=%.1f°' %
                   (tag, err, math.degrees(ori_err)))

        # Do not immediately retry an execution failure: the robot/controller may
        # still be draining buffered points, and preempting here is what produced
        # the joint target speed out-of-range spike.
        return False, True, err, sync_delay

    def _execute_position_only_target(self, plan_wx, plan_wy, plan_wz, tag,
                                      verify_wx=None, verify_wy=None, verify_wz=None):
        """Plan by position only, then reject unsafe IK branch jumps before execution."""
        if verify_wx is None:
            verify_wx, verify_wy, verify_wz = plan_wx, plan_wy, plan_wz

        self.group.clear_pose_targets()
        self.group.set_start_state_to_current_state()
        self.group.set_position_target([plan_wx, plan_wy, plan_wz])
        cprint('EXEC', '%s: 位置目标 fallback 规划 ...' % tag)

        plan_ok, plan = self._normalize_plan_result(self.group.plan())
        self.group.clear_pose_targets()
        if not plan_ok:
            cprint('WARN', '%s: 位置目标 fallback 规划失败' % tag)
            return False, False, float('inf'), -1.0

        jump_reason = self._large_joint_jump_reason(plan)
        if jump_reason:
            cprint('WARN', '%s: 位置目标 fallback 被拒绝: %s' % (tag, jump_reason))
            return False, False, float('inf'), -1.0

        self.group.stop()
        self.group.clear_pose_targets()
        return self._execute_plan_direct(
            plan, verify_wx, verify_wy, verify_wz, tag,
            expected_plan_xyz=(plan_wx, plan_wy, plan_wz))

    def execute_pose_target(self, target_teach, wp_label, orientation=None,
                            prefer_position_only=False,
                            orientation_tolerance=GOAL_ORI_TOL,
                            time_scale=DIRECT_EXECUTION_TIME_SCALE):
        """
        先用 MoveIt 规划, 再直接发 FollowJointTrajectory action 执行。
        这样绕开 MoveIt TrajectoryExecutionManager 的短超时, 避免实机尚未停稳时被脚本重试抢占。

        target_teach: 示教器坐标系 (x, y, z)
        orientation:   geometry_msgs.msg.Quaternion 或 None
        prefer_position_only: True 时 3-DOF 输入直接用位置目标规划
        已发送轨迹但未确认到位时不立即重试, 防止旧轨迹/新轨迹拼接导致速度尖峰。
        """
        tx, ty, tz = target_teach
        verify_wx, verify_wy, verify_wz = to_world(tx, ty, tz)
        plan_wx, plan_wy, plan_wz = self.to_moveit_target_world(tx, ty, tz)

        last_err = float('inf')
        last_delay = -1.0

        for attempt in range(1 + MAX_RETRIES):
            tag = wp_label if attempt == 0 else '%s-R%d' % (wp_label, attempt)
            if attempt > 0:
                cprint('WARN', '%s: 重试 %d/%d' % (tag, attempt, MAX_RETRIES))

            # 获取目标方向: 用户指定 > 保持当前方向 > 报错。
            # 自定义 3-DOF 位置输入直接使用位置目标规划；预设轨迹仍先保持当前方向。
            # 预设轨迹在首次 6-DOF 规划失败后会降级为位置目标，
            # 并在执行前检查轨迹中是否存在关节分支跳变。
            use_pose_target = orientation is not None or (attempt == 0 and not prefer_position_only)
            if orientation is None and use_pose_target:
                current_pose = self.get_current_ee_pose()
                if current_pose is None:
                    cprint('ERROR', '%s: 无法获取当前位姿' % tag)
                    return False, float('inf'), -1.0
                target_orientation = current_pose.orientation
            else:
                target_orientation = orientation

            if use_pose_target:
                target_pose = geometry_msgs.msg.Pose()
                target_pose.position.x = plan_wx
                target_pose.position.y = plan_wy
                target_pose.position.z = plan_wz
                target_pose.orientation = target_orientation

                self.group.set_start_state_to_current_state()
                self.group.set_pose_target(target_pose)
                cprint('EXEC', '%s: 6-DOF 目标规划 ...' % tag)
                plan_ready, plan = self._normalize_plan_result(self.group.plan())
                self.group.stop()
                self.group.clear_pose_targets()

                if plan_ready:
                    plan_ok, sent, last_err, last_delay = self._execute_plan_direct(
                        plan, verify_wx, verify_wy, verify_wz, tag,
                        expected_orientation=target_orientation,
                        orientation_tolerance=orientation_tolerance,
                        time_scale=time_scale,
                        expected_plan_xyz=(plan_wx, plan_wy, plan_wz))
                else:
                    plan_ok, sent = False, False

                if not plan_ok and sent:
                    cprint('WARN', '%s: 已发送轨迹但未确认到位, 不立即重试以避免抢占仍在运动的实机' % tag)
                    return False, last_err, last_delay

                if not plan_ok and orientation is None:
                    cprint('WARN', '%s: 保持当前方向的 6-DOF 目标不可用, 尝试位置目标 fallback' % tag)
                    plan_ok, sent, last_err, last_delay = self._execute_position_only_target(
                        plan_wx, plan_wy, plan_wz, '%s-POS' % tag,
                        verify_wx, verify_wy, verify_wz)
            else:
                plan_ok, sent, last_err, last_delay = self._execute_position_only_target(
                    plan_wx, plan_wy, plan_wz, tag,
                    verify_wx, verify_wy, verify_wz)

            if not plan_ok and sent:
                cprint('WARN', '%s: 已发送轨迹但未确认到位, 不立即重试以避免抢占仍在运动的实机' % tag)
                return False, last_err, last_delay

            if plan_ok:
                return True, last_err, last_delay

        cprint('WARN', '%s: %d 次尝试后仍未到达' % (wp_label, 1 + MAX_RETRIES))
        return False, last_err, last_delay

    # ============================================================
    # 正方形轨迹
    # ============================================================
    def run_square_trajectory(self):
        """执行正方形轨迹 (4 个角点 + 返回起点) — 坐标均已转换为 world 系供 MoveIt 使用"""
        cprint('EXEC', '>>> 正方形轨迹 (20cm x 20cm, YZ 平面) <<<')

        all_waypoints = SQUARE_CORNERS + [SQUARE_CORNERS[0]]
        total_start = time.time()
        ok_count = 0

        for i, corner in enumerate(all_waypoints):
            wp_label = 'WP-%d/%d' % (i + 1, len(all_waypoints))
            # 显示示教器坐标系 (与示教器读数一致)
            cprint('INFO', '%s 目标(示教器系): (%.2f, %.2f, %.2f)' %
                   (wp_label, corner[0], corner[1], corner[2]))

            arrived, err, sync_delay = self.execute_pose_target(
                corner, wp_label, prefer_position_only=True)

            # 等待驱动队列排空、关节状态稳定, 防止相邻轨迹数据混叠
            rospy.sleep(0.5)

            # 读取当前实际位姿, 与 TUI/示教器显示保持同源。
            display_pose = self.get_current_display_pose_teach()
            if display_pose:
                px, py, pz = display_pose
            else:
                px = py = pz = float('nan')

            if arrived:
                cprint('WP', '%s 到达 | 位姿=(%.2f,%.2f,%.2f)示教器系 | '
                       '延迟=%.3fs | 误差=%.3fm | OK' %
                       (wp_label, px, py, pz, sync_delay, err))
                ok_count += 1
            else:
                cprint('WARN', '%s 未达 | 位姿=(%.2f,%.2f,%.2f)示教器系 | '
                       '延迟=%.3fs | 误差=%.3fm | 继续' %
                       (wp_label, px, py, pz, sync_delay, err))

        elapsed = time.time() - total_start
        cprint('OK', '正方形轨迹结束 | 成功 %d/%d 段 | 总耗时 %.1fs' %
               (ok_count, len(all_waypoints), elapsed))
        return ok_count == len(all_waypoints)

    # ============================================================
    # 自定义路径点
    # ============================================================
    def run_custom_waypoint(self, x, y, z, orientation=None):
        """x, y, z 为示教器坐标系 (用户输入), 内部转为 world 系
        orientation: 可选 geometry_msgs.msg.Quaternion, None=仅约束位置"""
        target = (x, y, z)
        orient_desc = '' if orientation is None else ' +指定方向'
        cprint('INFO', '执行自定义目标(示教器系): (%.2f, %.2f, %.2f)%s' % (target + (orient_desc,)))
        arrived, err, sync_delay = self.execute_pose_target(
            target, 'CUSTOM', orientation, prefer_position_only=(orientation is None))

        display_pose = self.get_current_display_pose_teach()
        if display_pose:
            px, py, pz = display_pose
        else:
            px = py = pz = float('nan')

        if arrived:
            cprint('OK', '到达 | 位姿=(%.2f,%.2f,%.2f)示教器系 | 延迟=%.3fs | 误差=%.3fm' %
                   (px, py, pz, sync_delay, err))
        else:
            cprint('WARN', '未达 | 位姿=(%.2f,%.2f,%.2f)示教器系 | 延迟=%.3fs | 误差=%.3fm' %
                   (px, py, pz, sync_delay, err))
        return arrived

    @staticmethod
    def check_waypoint_safety(x, y, z):
        """基于肩关节中心的工作空间检查 (AUBO E5 臂展 0.886m)
        x,y,z: 示教器坐标系"""
        wx, wy, wz = to_world(x, y, z)
        dz = wz - SHOULDER_Z
        dist = math.sqrt(wx * wx + wy * wy + dz * dz)
        if dist > MAX_REACH:
            cprint('WARN', '距肩关节中心 %.2fm > 臂展 %.2fm, 超出工作空间' % (dist, MAX_REACH))
            return False
        if dist > MAX_REACH * 0.92:
            cprint('INFO', '距肩关节中心 %.2fm (接近边界 %d%%)' % (dist, int(MAX_REACH * 100)))
        if wz < 0.02:
            cprint('WARN', 'Z=%.2fm 过低 (基座碰撞风险)' % wz)
            return False
        return True

    # ============================================================
    # [3] 多路径点连续轨迹
    # ============================================================
    def run_multi_waypoint(self):
        """逐行读取用户输入的路径点 (示教器坐标系), 连续笛卡尔执行。

        每行: "x y z"。旧格式中的 RPY 会被解析但在连续模式中忽略，
        执行时锁定开始时的当前末端方向。
        """
        cprint('INFO', '输入路径点 (示教器坐标系):')
        cprint('INFO', '  格式: x y z  （连续模式锁定当前 RPY）')
        cprint('INFO', '  空行或 "done" 结束')

        waypoints = []
        while True:
            try:
                line = safe_input('  wp> ')
            except (EOFError, KeyboardInterrupt):
                break
            if line == '' or line.lower() == 'done':
                break
            parts = line.split()
            try:
                x, y, z, orientation, err = parse_pose_input(parts)
            except ValueError:
                cprint('WARN', '数字解析失败, 跳过: %s' % line)
                continue
            if err:
                cprint('WARN', '%s, 跳过' % err)
                continue
            if not self.check_waypoint_safety(x, y, z):
                cprint('WARN', '(%.2f, %.2f, %.2f) 超出安全范围, 已跳过' % (x, y, z))
                continue
            waypoints.append((x, y, z, orientation))
            cprint('INFO', '已添加 WP-%d: (%.2f, %.2f, %.2f)%s' %
                   (len(waypoints), x, y, z,
                    '' if orientation is None else ' +RPY将被忽略'))

        if not waypoints:
            cprint('WARN', '未输入有效路径点, 取消')
            return

        # 询问是否循环执行
        cprint('INPUT', '循环执行? (y/n) [默认 n]:')
        loop_ans = safe_input().strip().lower()
        loops = 1
        if loop_ans == 'y':
            cprint('INPUT', '循环次数 [默认 1]:')
            try:
                n_str = safe_input().strip()
                loops = int(n_str) if n_str else 1
                loops = max(1, loops)
            except ValueError:
                loops = 1
            cprint('INFO', '将循环执行 %d 次' % loops)

        self.execute_multi_waypoints(waypoints, loops)

    def execute_multi_waypoints(self, waypoints, loops=1):
        """执行已解析好的路径点列表, 供交互模式和脚本菜单共用。

        多路径点默认走连续笛卡尔路径: 锁定当前末端方向, 对用户输入点进行
        稠密化, 一次规划并一次下发。这样避免逐点 stop/start 和 IK 分支切换
        导致的离散突变。
        """
        total_start = time.time()
        if not waypoints:
            cprint('WARN', '空路径点列表, 取消')
            return False

        expanded = []
        for _ in range(max(1, loops)):
            for wp in waypoints:
                x, y, z = wp[:3]
                if not self.check_waypoint_safety(x, y, z):
                    cprint('WARN', '(%.2f, %.2f, %.2f) 超出安全范围, 取消整条连续轨迹' %
                           (x, y, z))
                    return False
                expanded.append((x, y, z))

        if any(len(wp) > 3 and wp[3] is not None for wp in waypoints):
            cprint('WARN', '多路径点连续模式将锁定当前末端方向, 忽略逐点 RPY, 以避免腕部翻转')

        arrived, err, sync_delay = self.execute_cartesian_waypoint_path(
            expanded, 'MULTI-CART')

        display_pose = self.get_current_display_pose_teach()
        if display_pose:
            px, py, pz = display_pose
        else:
            px = py = pz = float('nan')

        elapsed = time.time() - total_start
        if arrived:
            cprint('OK', '连续多路径点结束 | 位姿=(%.2f,%.2f,%.2f)示教器系 | '
                   '误差=%.3fm | 延迟=%.3fs | 总耗时 %.1fs' %
                   (px, py, pz, err, sync_delay, elapsed))
        else:
            cprint('WARN', '连续多路径点未执行/未达 | 位姿=(%.2f,%.2f,%.2f)示教器系 | '
                   '误差=%.3fm | 延迟=%.3fs | 总耗时 %.1fs' %
                   (px, py, pz, err, sync_delay, elapsed))
        return arrived

    def _densify_teach_waypoints(self, teach_points, step=CARTESIAN_INTERPOLATION_STEP):
        dense = []
        last = None
        for point in teach_points:
            if last is None:
                dense.append(point)
                last = point
                continue
            dx = point[0] - last[0]
            dy = point[1] - last[1]
            dz = point[2] - last[2]
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            n = max(1, int(math.ceil(dist / step)))
            for i in range(1, n + 1):
                ratio = float(i) / float(n)
                dense.append((
                    last[0] + dx * ratio,
                    last[1] + dy * ratio,
                    last[2] + dz * ratio,
                ))
            last = point
        return dense

    @staticmethod
    def _solve_3x3(matrix, vector):
        """Solve a small 3x3 linear system with Gaussian elimination."""
        a = [list(row) + [float(vector[i])] for i, row in enumerate(matrix)]
        n = 3
        for col in range(n):
            pivot = max(range(col, n), key=lambda row: abs(a[row][col]))
            if abs(a[pivot][col]) < 1e-9:
                return None
            if pivot != col:
                a[col], a[pivot] = a[pivot], a[col]
            div = a[col][col]
            for j in range(col, n + 1):
                a[col][j] /= div
            for row in range(n):
                if row == col:
                    continue
                factor = a[row][col]
                for j in range(col, n + 1):
                    a[row][j] -= factor * a[col][j]
        return [a[i][n] for i in range(n)]

    def _fit_circle_xy(self, pose_waypoints):
        """Least-squares fit x^2 + y^2 + D*x + E*y + F = 0."""
        if len(pose_waypoints) < 3:
            return None

        ata = [[0.0, 0.0, 0.0] for _ in range(3)]
        atb = [0.0, 0.0, 0.0]
        for x, y, _z, _r, _p, _yaw in pose_waypoints:
            row = [x, y, 1.0]
            b = -(x * x + y * y)
            for i in range(3):
                atb[i] += row[i] * b
                for j in range(3):
                    ata[i][j] += row[i] * row[j]

        solution = self._solve_3x3(ata, atb)
        if solution is None:
            return None
        d, e, f = solution
        cx = -0.5 * d
        cy = -0.5 * e
        radius_sq = cx * cx + cy * cy - f
        if radius_sq <= 1e-8:
            return None
        return cx, cy, math.sqrt(radius_sq)

    @staticmethod
    def _quat_msg_to_list(quat):
        return [quat.x, quat.y, quat.z, quat.w]

    @staticmethod
    def _quat_list_to_msg(quat):
        return geometry_msgs.msg.Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3])

    def _densify_arc_pose_waypoints(self, pose_waypoints, step=EEF_STEP):
        """Densify 6D teach waypoints along a fitted XY circle.

        The measured points are preserved as segment endpoints. Intermediate
        points follow the fitted circle with linearly interpolated radius, z and
        slerped orientation, so the trace stays visibly arc-like without forcing
        the supplied samples off their taught positions.
        """
        if len(pose_waypoints) < 2:
            return pose_waypoints, None

        circle = self._fit_circle_xy(pose_waypoints)
        if circle is None:
            dense = []
            for x, y, z, roll, pitch, yaw in pose_waypoints:
                dense.append((x, y, z, rpy_to_quat(roll, pitch, yaw)))
            return dense, None

        from tf.transformations import quaternion_slerp

        cx, cy, radius = circle
        dense = []
        for idx, start in enumerate(pose_waypoints[:-1]):
            end = pose_waypoints[idx + 1]
            sx, sy, sz, sr, sp, syaw = start
            ex, ey, ez, er, ep, eyaw = end
            start_angle = math.atan2(sy - cy, sx - cx)
            end_angle = math.atan2(ey - cy, ex - cx)
            delta_angle = end_angle - start_angle
            while delta_angle > math.pi:
                delta_angle -= 2.0 * math.pi
            while delta_angle < -math.pi:
                delta_angle += 2.0 * math.pi

            start_radius = math.sqrt((sx - cx) ** 2 + (sy - cy) ** 2)
            end_radius = math.sqrt((ex - cx) ** 2 + (ey - cy) ** 2)
            arc_len = max(start_radius, end_radius, radius) * abs(delta_angle)
            dz = ez - sz
            segment_len = math.sqrt(arc_len * arc_len + dz * dz)
            samples = max(1, int(math.ceil(segment_len / step)))
            q0 = self._quat_msg_to_list(rpy_to_quat(sr, sp, syaw))
            q1 = self._quat_msg_to_list(rpy_to_quat(er, ep, eyaw))

            first_sample = 0 if idx == 0 else 1
            for sample_idx in range(first_sample, samples + 1):
                ratio = float(sample_idx) / float(samples)
                angle = start_angle + delta_angle * ratio
                radius_i = start_radius + (end_radius - start_radius) * ratio
                x = cx + radius_i * math.cos(angle)
                y = cy + radius_i * math.sin(angle)
                z = sz + dz * ratio
                quat = self._quat_list_to_msg(
                    quaternion_slerp(q0, q1, ratio, spin=0, shortestpath=True))
                dense.append((x, y, z, quat))

        return dense, circle

    def _compute_cartesian_path_compat(self, poses):
        """Call compute_cartesian_path across MoveIt Python API variants.

        Some MoveIt builds use:
          compute_cartesian_path(waypoints, eef_step, avoid_collisions=True, ...)
        Older builds use:
          compute_cartesian_path(waypoints, eef_step, jump_threshold, avoid_collisions)
        Passing the old positional arguments to the newer API turns True into a
        path constraint and raises "unknown constraint type <class 'bool'>".
        """
        try:
            return self.group.compute_cartesian_path(
                poses, EEF_STEP, avoid_collisions=True)
        except TypeError as modern_error:
            try:
                return self.group.compute_cartesian_path(
                    poses, EEF_STEP, CARTESIAN_JUMP_THRESHOLD, True)
            except TypeError:
                raise modern_error

    def execute_cartesian_waypoint_path(self, teach_points, label='CARTESIAN',
                                        orientation=None,
                                        orientation_desc='锁定当前 RPY',
                                        orientation_tolerance=GOAL_ORI_TOL,
                                        time_scale=DIRECT_EXECUTION_TIME_SCALE):
        """Plan and execute a continuous Cartesian path with a locked orientation."""
        if len(teach_points) < 1:
            cprint('WARN', '%s: 空笛卡尔路径, 取消' % label)
            return False, float('inf'), -1.0

        current_pose = self.get_current_ee_pose()
        if current_pose is None:
            cprint('ERROR', '%s: 无法读取当前末端位姿, 取消' % label)
            return False, float('inf'), -1.0

        locked_orientation = (
            copy.deepcopy(orientation) if orientation is not None
            else copy.deepcopy(current_pose.orientation))
        dense_teach = self._densify_teach_waypoints(teach_points)

        ox, oy, oz = self._moveit_world_offset_from_driver()
        if abs(ox) > 1e-4 or abs(oy) > 1e-4 or abs(oz) > 1e-4:
            cprint('INFO', 'SDK→MoveIt 末端修正: Δ=(%.3f, %.3f, %.3f)m' %
                   (ox, oy, oz))

        poses = []
        for tx, ty, tz in dense_teach:
            wx, wy, wz = to_world(tx, ty, tz)
            pose = geometry_msgs.msg.Pose()
            pose.position.x = wx + ox
            pose.position.y = wy + oy
            pose.position.z = wz + oz
            pose.orientation = locked_orientation
            poses.append(pose)

        cprint('EXEC', '%s: 连续笛卡尔规划, 输入点=%d, 稠密点=%d, 姿态=%s' %
               (label, len(teach_points), len(poses), orientation_desc))

        self.group.clear_pose_targets()
        self.group.set_start_state_to_current_state()
        try:
            plan, fraction = self._compute_cartesian_path_compat(poses)
        finally:
            self.group.stop()
            self.group.clear_pose_targets()

        if fraction < CARTESIAN_MIN_FRACTION:
            cprint('WARN', '%s: 笛卡尔路径仅完成 %.1f%% < %.1f%%, 拒绝执行' %
                   (label, fraction * 100.0, CARTESIAN_MIN_FRACTION * 100.0))
            return False, float('inf'), -1.0

        retimed = self._retime_trajectory(plan)
        if retimed is None:
            return False, float('inf'), -1.0

        if orientation is None:
            wrist_reason = self._wrist3_motion_reason(retimed)
            if wrist_reason:
                cprint('WARN', '%s: 姿态锁定轨迹被拒绝: %s' % (label, wrist_reason))
                return False, float('inf'), -1.0

        timing_reason = self._trajectory_timing_reason(retimed, min_dt=1e-6)
        if timing_reason:
            cprint('WARN', '%s: retime 后时间戳不安全: %s' % (label, timing_reason))
            return False, float('inf'), -1.0

        final_tx, final_ty, final_tz = dense_teach[-1]
        verify_wx, verify_wy, verify_wz = to_world(final_tx, final_ty, final_tz)
        plan_ok, sent, err, sync_delay = self._execute_plan_direct(
            retimed, verify_wx, verify_wy, verify_wz, label,
            expected_orientation=locked_orientation,
            orientation_tolerance=orientation_tolerance,
            time_scale=time_scale,
            expected_plan_xyz=(
                poses[-1].position.x,
                poses[-1].position.y,
                poses[-1].position.z))
        return plan_ok, err, sync_delay

    def execute_cartesian_pose_path(self, teach_pose_points, label='CART-POSE',
                                    path_desc='6D 笛卡尔路径',
                                    orientation_tolerance=GOAL_ORI_TOL,
                                    time_scale=DIRECT_EXECUTION_TIME_SCALE):
        """Plan and execute a Cartesian path whose waypoints carry orientation."""
        if len(teach_pose_points) < 2:
            cprint('WARN', '%s: 6D 路径点不足, 取消' % label)
            return False, float('inf'), -1.0

        current_pose = self.get_current_ee_pose()
        if current_pose is None:
            cprint('ERROR', '%s: 无法读取当前末端位姿, 取消' % label)
            return False, float('inf'), -1.0

        dense_teach, circle = self._densify_arc_pose_waypoints(teach_pose_points)
        if circle is not None:
            cx, cy, radius = circle
            cprint('INFO', '%s: 圆弧拟合 center=(%.4f, %.4f), radius=%.4fm' %
                   (label, cx, cy, radius))
        else:
            cprint('WARN', '%s: 圆弧拟合失败, 使用原始 6D 点折线补点' % label)

        ox, oy, oz = self._moveit_world_offset_from_driver()
        if abs(ox) > 1e-4 or abs(oy) > 1e-4 or abs(oz) > 1e-4:
            cprint('INFO', 'SDK→MoveIt 末端修正: Δ=(%.3f, %.3f, %.3f)m' %
                   (ox, oy, oz))

        poses = []
        for tx, ty, tz, quat in dense_teach:
            wx, wy, wz = to_world(tx, ty, tz)
            pose = geometry_msgs.msg.Pose()
            pose.position.x = wx + ox
            pose.position.y = wy + oy
            pose.position.z = wz + oz
            pose.orientation = quat
            poses.append(pose)

        cprint('EXEC', '%s: %s, 原始点=%d, 圆弧稠密点=%d' %
               (label, path_desc, len(teach_pose_points), len(poses)))

        self.group.clear_pose_targets()
        self.group.set_start_state_to_current_state()
        try:
            plan, fraction = self._compute_cartesian_path_compat(poses)
        finally:
            self.group.stop()
            self.group.clear_pose_targets()

        if fraction < CARTESIAN_MIN_FRACTION:
            cprint('WARN', '%s: 笛卡尔路径仅完成 %.1f%% < %.1f%%, 拒绝执行' %
                   (label, fraction * 100.0, CARTESIAN_MIN_FRACTION * 100.0))
            return False, float('inf'), -1.0

        retimed = self._retime_trajectory(plan)
        if retimed is None:
            return False, float('inf'), -1.0

        timing_reason = self._trajectory_timing_reason(retimed, min_dt=1e-6)
        if timing_reason:
            cprint('WARN', '%s: retime 后时间戳不安全: %s' % (label, timing_reason))
            return False, float('inf'), -1.0

        final_tx, final_ty, final_tz, final_quat = dense_teach[-1]
        verify_wx, verify_wy, verify_wz = to_world(final_tx, final_ty, final_tz)
        plan_ok, sent, err, sync_delay = self._execute_plan_direct(
            retimed, verify_wx, verify_wy, verify_wz, label,
            expected_orientation=final_quat,
            orientation_tolerance=orientation_tolerance,
            time_scale=time_scale,
            expected_plan_xyz=(
                poses[-1].position.x,
                poses[-1].position.y,
                poses[-1].position.z))
        return plan_ok, err, sync_delay

    def run_menu_action(self, choice, pose_text=None, waypoint_texts=None, loops=1):
        """执行 run_square_demo.sh 传入的单次菜单动作。"""
        if choice == '1':
            return self.run_square_trajectory()

        if choice == '2':
            if not pose_text:
                cprint('ERROR', '缺少目标位姿: x y z [roll pitch yaw]')
                return False
            try:
                x, y, z, orientation, err = parse_pose_input(pose_text.split())
            except ValueError:
                cprint('ERROR', '格式错误')
                return False
            if err:
                cprint('ERROR', err)
                return False
            if not self.check_waypoint_safety(x, y, z):
                cprint('WARN', '可能超出安全范围, 继续? (y/n)')
                if safe_input().strip().lower() != 'y':
                    cprint('INFO', '已取消')
                    return False
            return self.run_custom_waypoint(x, y, z, orientation)

        if choice == '3':
            waypoints = []
            for line in waypoint_texts or []:
                parts = line.split()
                try:
                    x, y, z, orientation, err = parse_pose_input(parts)
                except ValueError:
                    cprint('WARN', '数字解析失败, 跳过: %s' % line)
                    continue
                if err:
                    cprint('WARN', '%s, 跳过: %s' % (err, line))
                    continue
                if not self.check_waypoint_safety(x, y, z):
                    cprint('WARN', '(%.2f, %.2f, %.2f) 超出安全范围, 已跳过' % (x, y, z))
                    continue
                waypoints.append((x, y, z, orientation))

            if not waypoints:
                cprint('WARN', '未输入有效路径点, 取消')
                return False
            return self.execute_multi_waypoints(waypoints, max(1, loops))

        if choice == '4':
            self.run_safety_watchdog_status()
            return True

        if choice == '5':
            return self.run_grinding_test()

        if choice == '6':
            self.run_planning_algorithms_overview()
            return True

        if choice == '7':
            self.run_show_readme()
            return True

        if choice.lower() == 'q':
            cprint('INFO', '退出')
            return True

        cprint('WARN', '未知选项: %s' % choice)
        return False

    # ============================================================
    # [4] 安全审查状态
    # ============================================================
    def run_safety_watchdog_status(self):
        """显示项目全部看门狗/门控耦合点静态表, 并尝试实时 ROS 话题检查"""
        # 静态表 (6 个看门狗)
        watchdogs = [
            {
                'id': 1,
                'name': 'Safety Monitor 心跳',
                'node': 'safety_monitor.py',
                'topic': '/safety_monitor/safe_to_execute',
                'threshold': '5s watchdog in linked_execution_action_server',
                'effect': '阻断执行',
                'check_type': 'topic',
            },
            {
                'id': 2,
                'name': 'C++ Action 看门狗',
                'node': 'joint_trajectory_action.cpp',
                'topic': '/feedback_states',
                'threshold': '1s WATCHDOG_PERIOD_',
                'effect': '中止轨迹',
                'check_type': 'topic',
            },
            {
                'id': 3,
                'name': 'Gazebo 收敛监控',
                'node': 'linked_execution_monitor.py',
                'topic': '/linked_execution/monitor_status',
                'threshold': 'traj_duration + 8s',
                'effect': '联动失败判定',
                'check_type': 'topic',
            },
            {
                'id': 4,
                'name': '实机连接参数',
                'node': 'aubo_robot_simulator',
                'topic': '/aubo_driver/robot_connected',
                'threshold': "阻塞直到 =='1'",
                'effect': '插补桥启动门',
                'check_type': 'rosparam',
            },
            {
                'id': 5,
                'name': 'CAN 缓冲区门控',
                'node': 'aubo_driver.cpp',
                'topic': '/aubo_driver/rib_status',
                'threshold': 'MINIMUM_BUFFER_SIZE=300',
                'effect': '限流发送',
                'check_type': 'topic',
            },
            {
                'id': 6,
                'name': 'Gazebo RTF 监控',
                'node': 'gazebo_rtf_monitor.py',
                'topic': '/gazebo_rtf_monitor/warning',
                'threshold': 'RTF 0.8–1.2',
                'effect': '仿真性能预警',
                'check_type': 'topic',
            },
        ]

        cprint('INFO', '')
        cprint('INFO', '=== 安全审查状态 — 看门狗/门控耦合点 ===')
        cprint('INFO', '')

        # 打印静态表头
        header = '%-2s  %-20s  %-30s  %-20s  %-6s' % (
            '#', '名称', '监听话题/参数', '超时/阈值', '作用')
        print('%s%s%s' % (C['W'], header, C['N']))
        print('%s%s%s' % (C['W'], '-' * len(header), C['N']))

        for wd in watchdogs:
            row = '%-2d  %-20s  %-30s  %-20s  %-6s' % (
                wd['id'], wd['name'][:20], wd['topic'][:30],
                wd['threshold'][:20], wd['effect'][:6])
            print(row)

        cprint('INFO', '')
        cprint('INFO', '--- 实时检查 (timeout=2s) ---')

        for wd in watchdogs:
            name = wd['name']
            topic = wd['topic']
            check_type = wd['check_type']

            if check_type == 'rosparam':
                # 用 rosparam get 检查参数
                try:
                    result = subprocess.run(
                        ['rosparam', 'get', topic],
                        timeout=2,
                        capture_output=True,
                        text=True,
                    )
                    if result.returncode == 0 and result.stdout.strip():
                        status = '%s✓ ACTIVE%s (值=%s)' % (
                            C['G'], C['N'], result.stdout.strip())
                    else:
                        status = '%s✗ 无数据%s' % (C['R'], C['N'])
                except subprocess.TimeoutExpired:
                    status = '%s? 未知%s (超时)' % (C['Y'], C['N'])
                except Exception as e:
                    status = '%s? 未知%s (%s)' % (C['Y'], C['N'], str(e))
            else:
                # 用 rostopic echo -n 1 检查话题活跃度 (收到1条消息即判定为 ACTIVE)
                # 比 rostopic hz 更可靠: hz 需要 window 条消息才输出结果,
                # 低频话题在 2s 内可能收不到足够消息而误报 ✗
                try:
                    result = subprocess.run(
                        ['rostopic', 'echo', '-n', '1', topic],
                        timeout=2,
                        capture_output=True,
                        text=True,
                    )
                    if result.returncode == 0 and result.stdout.strip():
                        status = '%s✓ ACTIVE%s' % (C['G'], C['N'])
                    else:
                        status = '%s✗ 无数据%s' % (C['R'], C['N'])
                except subprocess.TimeoutExpired:
                    # 超时 = 2s 内未收到消息, 话题不活跃或节点未启动
                    status = '%s✗ 无数据%s (2s 内无消息)' % (C['R'], C['N'])
                except Exception as e:
                    status = '%s? 未知%s (%s)' % (C['Y'], C['N'], str(e))

            print('  [%d] %-20s  %s  %s' % (wd['id'], name[:20], topic, status))

        cprint('INFO', '')

    # ============================================================
    # [5] 预设工件打磨测试
    # ============================================================
    def run_grinding_test(self):
        """执行内置 3 点工件打磨测试轨迹 (示教器坐标系)。

        先以固定工具姿态到达第一个点上方的接近点，再执行固定姿态直线段。
        直线结束后抬高 gap，转移到圆弧起点上方，下探到圆弧起点，最后执行
        按示教 6D 点拟合/补密后的圆弧循迹。
        """
        cprint('EXEC', '>>> 预设工件打磨测试 — 直线 + 抬升 + 圆弧循迹 <<<')
        if not self._flush_execution_pipeline('GRIND-PRECHECK'):
            return False
        grind_orientation = rpy_to_quat(*GRINDING_TOOL_RPY_DEG)
        orientation_desc = '固定 RPY(%.1f°, %.1f°, %.1f°)' % GRINDING_TOOL_RPY_DEG
        cprint('INFO', '打磨工具姿态: %s' % orientation_desc)

        first = GRINDING_TEST_WAYPOINTS[0]
        approach = (first[0], first[1], first[2] + GRINDING_APPROACH_Z_OFFSET)

        cprint('INFO', '先移动到打磨接近点(示教器系): (%.2f, %.2f, %.2f)' %
               approach)
        arrived, err, sync_delay = self.execute_pose_target(
            approach, 'GRIND-APPROACH',
            orientation=grind_orientation,
            prefer_position_only=False,
            orientation_tolerance=GRINDING_ORI_TOL)
        if not arrived:
            cprint('WARN', '打磨接近点未到达，取消预设打磨测试 | 误差=%.3fm | 延迟=%.3fs' %
                   (err, sync_delay))
            return False

        cprint('INFO', '尝试连续笛卡尔打磨线段 (%s)' % orientation_desc)
        arrived, err, sync_delay = self.execute_cartesian_waypoint_path(
            GRINDING_TEST_WAYPOINTS, 'GRIND-CART',
            orientation=grind_orientation,
            orientation_desc=orientation_desc,
            orientation_tolerance=GRINDING_ORI_TOL)
        if not arrived:
            cprint('WARN', '连续笛卡尔打磨线段不可用，降级为逐点 6-DOF 固定姿态目标')
            ok_count = 0
            for i, wp in enumerate(GRINDING_TEST_WAYPOINTS):
                label = 'GRIND-WP-%d/%d' % (i + 1, len(GRINDING_TEST_WAYPOINTS))
                arrived, err, sync_delay = self.execute_pose_target(
                    wp, label,
                    orientation=grind_orientation,
                    prefer_position_only=False,
                    orientation_tolerance=GRINDING_ORI_TOL)
                if arrived:
                    ok_count += 1
                    cprint('WP', '%s 到达 | 误差=%.3fm | 延迟=%.3fs' %
                           (label, err, sync_delay))
                else:
                    cprint('WARN', '%s 未达 | 误差=%.3fm | 延迟=%.3fs' %
                           (label, err, sync_delay))
                    break
                rospy.sleep(0.3)

            if ok_count == len(GRINDING_TEST_WAYPOINTS):
                cprint('OK', '直线打磨段逐点降级完成 %d/%d' %
                       (ok_count, len(GRINDING_TEST_WAYPOINTS)))
            else:
                cprint('WARN', '预设工件打磨测试未完成 | 直线段成功 %d/%d 点' %
                       (ok_count, len(GRINDING_TEST_WAYPOINTS)))
                return False

        line_end = GRINDING_TEST_WAYPOINTS[-1]
        lift_target = (
            line_end[0],
            line_end[1],
            line_end[2] + GRINDING_LIFT_GAP)
        cprint('INFO', '直线结束后垂直抬升 %.2fm 到: (%.4f, %.4f, %.4f)' %
               ((GRINDING_LIFT_GAP,) + lift_target))
        arrived, err, sync_delay = self.execute_cartesian_waypoint_path(
            [lift_target], 'GRIND-LIFT',
            orientation=grind_orientation,
            orientation_desc='固定 RPY + 垂直 gap 抬升',
            orientation_tolerance=GRINDING_ORI_TOL)
        if not arrived:
            cprint('WARN', '打磨 gap 抬升未到达，取消圆弧循迹 | 误差=%.3fm | 延迟=%.3fs' %
                   (err, sync_delay))
            return False
        rospy.sleep(0.5)

        arc_start = GRINDING_ARC_POSES[0]
        arc_start_position = arc_start[:3]
        arc_start_orientation = rpy_to_quat(*arc_start[3:])
        arc_approach = (
            arc_start[0],
            arc_start[1],
            arc_start[2] + GRINDING_LIFT_GAP)
        cprint('INFO', '移动到圆弧起点上方 gap 位: (%.4f, %.4f, %.4f)' %
               arc_approach)
        arrived, err, sync_delay = self.execute_cartesian_waypoint_path(
            [arc_approach], 'GRIND-ARC-APPROACH',
            orientation=arc_start_orientation,
            orientation_desc='笛卡尔安全过渡到圆弧 gap 位',
            orientation_tolerance=GRINDING_ORI_TOL,
            time_scale=GRINDING_ARC_TIME_SCALE)
        if not arrived:
            cprint('WARN', '圆弧起点上方 gap 位未到达，取消圆弧循迹 | 误差=%.3fm | 延迟=%.3fs' %
                   (err, sync_delay))
            return False
        rospy.sleep(0.5)

        cprint('INFO', '下探到圆弧起点: (%.4f, %.4f, %.4f)' %
               arc_start_position)
        arrived, err, sync_delay = self.execute_cartesian_waypoint_path(
            [arc_start_position], 'GRIND-ARC-START',
            orientation=arc_start_orientation,
            orientation_desc='圆弧起点姿态 + 垂直下探',
            orientation_tolerance=GRINDING_ORI_TOL,
            time_scale=GRINDING_ARC_TIME_SCALE)
        if not arrived:
            cprint('WARN', '圆弧起点未到达，取消圆弧循迹 | 误差=%.3fm | 延迟=%.3fs' %
                   (err, sync_delay))
            return False

        cprint('INFO', '开始圆弧循迹: 6D 示教点=%d, Z=%.4fm' %
               (len(GRINDING_ARC_POSES), GRINDING_ARC_POSES[0][2]))
        arrived, err, sync_delay = self.execute_cartesian_pose_path(
            GRINDING_ARC_POSES, 'GRIND-ARC',
            path_desc='拟合圆弧 + RPY 插值循迹',
            orientation_tolerance=GRINDING_ORI_TOL,
            time_scale=GRINDING_ARC_TIME_SCALE)
        if arrived:
            cprint('OK', '预设工件打磨测试结束 | 直线 + gap + 圆弧循迹完成')
            return True

        cprint('WARN', '圆弧循迹未完成 | 误差=%.3fm | 延迟=%.3fs' %
               (err, sync_delay))
        return False

    # ============================================================
    # [封存] 连续轨迹测试 (6点包络测试) — 原按钮[6]实现, 已由轨迹生成测试替代
    # ============================================================
    def run_test_sequence(self):
        """[封存] 执行内置 6 点包络测试序列 (示教器坐标系)"""
        cprint('EXEC', '>>> 连续轨迹测试 — 6 点包络序列 <<<')
        labels = ['中心高位', '左侧', '左低', '右低', '右侧', '回中心']
        total_start = time.time()
        ok_count = 0

        for i, wp in enumerate(TEST_SEQUENCE):
            wp_label = 'TEST-%d/%d(%s)' % (i + 1, len(TEST_SEQUENCE), labels[i])
            cprint('INFO', '%s 目标(示教器系): (%.2f, %.2f, %.2f)' %
                   (wp_label, wp[0], wp[1], wp[2]))

            arrived, err, sync_delay = self.execute_pose_target(
                wp, wp_label, prefer_position_only=True)
            rospy.sleep(0.5)

            display_pose = self.get_current_display_pose_teach()
            if display_pose:
                px, py, pz = display_pose
            else:
                px = py = pz = float('nan')

            if arrived:
                cprint('WP', '%s 到达 | 位姿=(%.2f,%.2f,%.2f)示教器系 | 误差=%.3fm | OK' %
                       (wp_label, px, py, pz, err))
                ok_count += 1
            else:
                cprint('WARN', '%s 未达 | 位姿=(%.2f,%.2f,%.2f)示教器系 | 误差=%.3fm' %
                       (wp_label, px, py, pz, err))

        elapsed = time.time() - total_start
        cprint('OK', '包络测试结束 | 成功 %d/%d 点 | 总耗时 %.1fs' %
               (ok_count, len(TEST_SEQUENCE), elapsed))
        return ok_count == len(TEST_SEQUENCE)

    # ============================================================
    # [6] 轨迹生成测试
    # ============================================================
    def run_planning_algorithms_overview(self):
        """纯日志展示：三类规划算法能力概览 + 测试路径点生成 + 规划管道诊断。
        不做任何机器人运动，仅通过 cprint 输出到日志窗口。"""
        cprint('INFO', '')
        cprint('INFO', '╔══════════════════════════════════════════════╗')
        cprint('INFO', '║   AUBO E5 轨迹生成测试 — 规划算法能力概览    ║')
        cprint('INFO', '╚══════════════════════════════════════════════╝')
        cprint('INFO', '')
        cprint('INFO', '采用 OMPL/RRTConnect、CHOMP 和 LERP 三类规划能力：')
        cprint('INFO', '')
        cprint('INFO', '┌─ [RRTConnect]  通用避障规划 ───────────────────┐')
        cprint('INFO', '│  论文定位: 复杂避障时的主要规划器                │')
        cprint('INFO', '│  引擎: OMPL 采样规划引擎 (概率完备)              │')
        cprint('INFO', '│  特点: 高维空间高效探索、适用于复杂环境避障      │')
        cprint('INFO', '│  配置: ompl_planning.yaml (22+ 算法可选)        │')
        cprint('INFO', '│  控制参数: range=0.18, planner=RRTConnect       │')
        cprint('INFO', '└────────────────────────────────────────────────┘')
        cprint('INFO', '')
        cprint('INFO', '┌─ [CHOMP]      轨迹平滑与优化 ───────────────────┐')
        cprint('INFO', '│  论文定位: 减少轨迹突变、提高运动连续性          │')
        cprint('INFO', '│  引擎: CHOMP (Covariant Hamiltonian Opt.)       │')
        cprint('INFO', '│  特点: 梯度优化同时优化平滑度与避障代价          │')
        cprint('INFO', '│  适用: 轨迹后处理优化、高连续性要求场景          │')
        cprint('INFO', '│  配置: ompl_planning.yaml (CHOMP 优化器参数)    │')
        cprint('INFO', '└────────────────────────────────────────────────┘')
        cprint('INFO', '')
        cprint('INFO', '┌─ [LERP]       无障碍简单插值 ───────────────────┐')
        cprint('INFO', '│  论文定位: 简单点到点过渡、标定姿态切换          │')
        cprint('INFO', '│  引擎: Pilz Industrial Motion Planner (LERP)    │')
        cprint('INFO', '│  特点: 关节空间线性插值、计算开销最低、响应最快  │')
        cprint('INFO', '│  适用: 无障碍场景、标定姿态过渡、短距移动        │')
        cprint('INFO', '│  配置: ompl_planning.yaml (LERP 算法参数)       │')
        cprint('INFO', '└────────────────────────────────────────────────┘')
        cprint('INFO', '')

        # 规划管道基础状态检查
        cprint('INFO', '--- 规划管道基础状态 ---')
        try:
            cprint('INFO', '  规划组: %s' % self.group.get_name())
            cprint('INFO', '  末端执行器: %s' % self.group.get_end_effector_link())
            with self._lock:
                js_ok = bool(self._joint_positions)
            cprint('INFO', '  /joint_states: %s' %
                   ('✓ 有数据 (%d 关节)' % len(self._joint_positions) if js_ok else '✗ 无数据'))
            with self._lock:
                gz_ok = self._gazebo_joint_time is not None
            cprint('INFO', '  /aubo_e5/joint_states: %s' %
                   ('✓ 有数据' if gz_ok else '✗ 无数据'))
            pipeline_ok = js_ok and gz_ok
            cprint('INFO', '  规划管道总评: %s' %
                   ('✓ 就绪' if pipeline_ok else '△ 部分就绪 — 等待话题数据'))
        except Exception as e:
            cprint('WARN', '  状态检查异常: %s' % str(e))
            pipeline_ok = False
        cprint('INFO', '')

        # 自定测试路径点
        cprint('INFO', '--- 自定测试路径点 (示教器坐标系) ---')
        test_waypoints = [
            (0.40,  0.00, 0.60, '中心高位 — 典型工作点'),
            (0.40, -0.15, 0.55, '左偏工作点 — 中等偏移'),
            (0.40,  0.15, 0.45, '右低工作点 — 低姿态'),
            (0.35, -0.20, 0.50, '左远工作点 — 较大偏移'),
            (0.50,  0.10, 0.65, '前伸高位 — 边界测试'),
        ]
        for i, (x, y, z, desc) in enumerate(test_waypoints, 1):
            wx, wy, wz = to_world(x, y, z)
            safe = self.check_waypoint_safety(x, y, z)
            flag = '✓' if safe else '✗超界'
            cprint('INFO', '  WP-%d: (%.2f, %.2f, %.2f) → world(%.3f, %.3f, %.3f) [%s] %s' %
                   (i, x, y, z, wx, wy, wz, flag, desc))
        cprint('INFO', '')

        # 规划算法选择策略
        cprint('INFO', '--- 规划算法选择策略 (基于路径点特征) ---')
        for i, (x, y, z, desc) in enumerate(test_waypoints, 1):
            wx, wy, wz = to_world(x, y, z)
            dist = math.sqrt(wx*wx + wy*wy + (wz-SHOULDER_Z)*(wz-SHOULDER_Z))
            if dist > MAX_REACH * 0.85:
                algo = 'RRTConnect ← 距肩 %.3fm (>85%% 臂展), 需采样探索' % dist
            elif abs(z) < 0.15:
                algo = 'RRTConnect ← 低姿态 (Z=%.2fm), 需避障验证' % z
            elif abs(x) > 0.45:
                algo = 'CHOMP      ← 前伸位姿, 建议平滑优化'
            elif abs(y) > 0.15:
                algo = 'RRTConnect ← 横向偏移较大'
            else:
                algo = 'LERP       ← 短距/标定过渡, 线性插值'
            cprint('INFO', '  WP-%d: %s' % (i, algo))

        cprint('INFO', '')
        cprint('INFO', '--- 规划→执行 8 步工作流 ---')
        cprint('INFO', '  ① 示教器系输入 (x y z [r p y])  ② to_world() +0.503m Z')
        cprint('INFO', '  ③ check_waypoint_safety()     ④ set_position/pose_target()')
        cprint('INFO', '  ⑤ plan() / compute_cartesian_path()')
        cprint('INFO', '  ⑥ 轨迹审查: 关节跳变/起点/时间戳/wrist3')
        cprint('INFO', '  ⑦ direct FollowJointTrajectory action')
        cprint('INFO', '  ⑧ 联动层 retime → 实机执行 + 仿真收敛确认')
        cprint('INFO', '')

        cprint('INFO', '--- 规划算法能力矩阵 ---')
        cprint('INFO', '  %-16s %10s %10s %10s' % ('能力', 'RRTConnect', 'CHOMP', 'LERP'))
        cprint('INFO', '  %-16s %10s %10s %10s' % ('─'*16, '─'*10, '─'*10, '─'*10))
        for label, rrt, chomp, lerp in [
            ('避障规划',       '   ✓   ', '   ✓   ', '   ✗   '),
            ('轨迹平滑',       '   △   ', '   ✓   ', '   ✗   '),
            ('计算速度',       '   慢   ', '   中   ', '   快   '),
            ('姿态约束',       '   ✓   ', '   ✓   ', '   ✗   '),
            ('概率完备',       '   ✓   ', '   ✗   ', '   ✓   '),
        ]:
            cprint('INFO', '  %-16s %10s %10s %10s' % (label, rrt, chomp, lerp))
        cprint('INFO', '  ✓=支持  △=部分  ✗=不支持')
        cprint('INFO', '')

        # 当前端点位姿显示（如果有数据）
        display_pose = self.get_current_display_pose_teach()
        if display_pose:
            tx, ty, tz = display_pose
            cprint('INFO', '--- 当前末端位姿 (示教器系) ---')
            cprint('INFO', '  位置: (%.3f, %.3f, %.3f) m' % (tx, ty, tz))
            cprint('INFO', '  (规划起点参考 — 用于验证起点差距检查)')
        cprint('INFO', '')
        cprint('OK', '轨迹生成测试完成 — 规划管道%s, 5 个测试路径点已评估' %
               ('就绪' if pipeline_ok else '部分就绪'))

    # ============================================================
    # [7] 介绍 (README)
    # ============================================================
    def run_show_readme(self):
        """打印 README 全文后直接返回菜单。"""
        if not os.path.isfile(README_PATH):
            cprint('WARN', 'README 未找到: %s' % README_PATH)
            return

        cprint('INFO', '--- README ---')
        try:
            with open(README_PATH, 'r', encoding='utf-8', errors='replace') as f:
                for line in f:
                    print(line, end='')
            print('')
        except Exception as e:
            cprint('ERROR', '读取 README 失败: %s' % str(e))

    # ============================================================
    # 主循环
    # ============================================================
    def print_status(self):
        display_pose = self.get_current_display_pose_teach()
        sd = self.estimate_sync_delay()
        if display_pose:
            tx, ty, tz = display_pose
            cprint('INFO', '--- 当前状态 ---')
            cprint('INFO', '末端位姿(示教器系): (%.3f, %.3f, %.3f) | 同步延迟: %.3fs' %
                   (tx, ty, tz, sd))

    def print_menu(self):
        cprint('INFO', '')
        cprint('INFO', '[1] 执行正方形轨迹 (20cm × 20cm, YZ 平面)')
        cprint('INFO', '[2] 输入自定义目标位姿 (x y z [roll pitch yaw])')
        cprint('INFO', '[3] 多路径点连续轨迹')
        cprint('INFO', '[4] 安全审查状态')
        cprint('INFO', '[5] 预设工件打磨测试 (3点笛卡尔轨迹)')
        cprint('INFO', '[6] 轨迹生成测试')
        cprint('INFO', '[7] 介绍 (README)')
        cprint('INFO', '[q] 退出')

    def run(self):
        if not self.wait_for_system_ready():
            return

        cprint('INFO', '')
        cprint('INFO', 'AUBO E5 正方形轨迹演示 -- 虚实同步控制')

        while not rospy.is_shutdown():
            self.print_status()
            self.print_menu()

            try:
                choice = safe_input()
            except (EOFError, KeyboardInterrupt):
                break

            if choice == '1':
                self.run_square_trajectory()

            elif choice == '2':
                cprint('INPUT', '输入目标位姿:')
                cprint('INPUT', '  仅位置:      x y z  (自动选择可达方向)')
                cprint('INPUT', '  位置 + RPY:  x y z roll pitch yaw (度)')
                try:
                    parts = safe_input().split()
                    x, y, z, orientation, err = parse_pose_input(parts)
                    if err:
                        cprint('ERROR', err)
                        continue
                except (ValueError, EOFError):
                    cprint('ERROR', '格式错误')
                    continue

                if not self.check_waypoint_safety(x, y, z):
                    cprint('WARN', '可能超出安全范围, 继续? (y/n)')
                    if safe_input().strip().lower() != 'y':
                        cprint('INFO', '已取消')
                        continue

                self.run_custom_waypoint(x, y, z, orientation)

            elif choice == '3':
                self.run_multi_waypoint()

            elif choice == '4':
                self.run_safety_watchdog_status()

            elif choice == '5':
                self.run_grinding_test()

            elif choice == '6':
                self.run_planning_algorithms_overview()

            elif choice == '7':
                self.run_show_readme()

            elif choice.lower() == 'q':
                cprint('INFO', '退出')
                break
            elif choice:
                cprint('WARN', '未知选项: %s' % choice)

        moveit_commander.roscpp_shutdown()


def main():
    parser = argparse.ArgumentParser(
        description='AUBO E5 正方形轨迹控制端')
    parser.add_argument('--action', choices=['1', '2', '3', '4', '5', '6', '7', 'q'],
                        help='执行一次菜单动作后退出')
    parser.add_argument('--pose',
                        help='动作 2 的目标位姿: "x y z [roll pitch yaw]"')
    parser.add_argument('--waypoint', action='append', default=[],
                        help='动作 3 的路径点, 可重复: "x y z [roll pitch yaw]"')
    parser.add_argument('--loops', type=int, default=1,
                        help='动作 3 的循环次数')
    args, ros_args = parser.parse_known_args()
    sys.argv = [sys.argv[0]] + ros_args

    try:
        ctrl = SquareDemoController()
        if args.action:
            if ctrl.wait_for_system_ready():
                ctrl.run_menu_action(
                    args.action,
                    pose_text=args.pose,
                    waypoint_texts=args.waypoint,
                    loops=args.loops)
            moveit_commander.roscpp_shutdown()
        else:
            ctrl.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print('')
        cprint('INFO', '用户中断')


if __name__ == '__main__':
    main()
