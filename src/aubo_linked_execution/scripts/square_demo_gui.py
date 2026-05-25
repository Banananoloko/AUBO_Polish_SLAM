#!/usr/bin/env python3
"""
square_demo_gui.py
AUBO E5 虚实交互系统 — tkinter GUI 控制端

用法:
    rosrun aubo_linked_execution square_demo_gui.py       # ROS 实机/仿真
    python3 square_demo_gui.py --standalone               # 独立界面审查 (无 ROS 依赖)

前提 (ROS 模式): 先通过 run_square_demo.sh 启动系统 (RViz + MoveIt + Gazebo)
前提 (独立模式): 仅需 Python 3 + tkinter (Ubuntu: sudo apt install python3-tk)
"""

import sys
import os
import re
import math
import time
import queue
import random
import threading
import subprocess
import copy
from collections import deque
from typing import Optional, List, Tuple

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox

# rosrun executes this file through a devel-space wrapper. In that mode
# sys.path[0] is devel/lib/aubo_linked_execution, not this source directory.
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

# ---- ROS 导入保护: 独立模式下不依赖 ROS ----
try:
    import rospy
    import moveit_commander
    import geometry_msgs.msg
    from sensor_msgs.msg import JointState
    from rosgraph_msgs.msg import Log as RosLog
    from square_demo_control import SquareDemoController as CoreSquareDemoController
    from emergency_stop import issue_emergency_stop
    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False
    CoreSquareDemoController = None
    issue_emergency_stop = None

# ---- ANSI 颜色 (保留用于终端回显) ----
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
PEDESTAL_Z = 0.503

# ---- 正方形轨迹参数 (示教器坐标系) ----
SQUARE_CORNERS = [
    (0.4, -0.10, 0.50),
    (0.4, -0.10, 0.70),
    (0.4,  0.10, 0.70),
    (0.4,  0.10, 0.50),
]

# ---- 运动参数 ----
VELOCITY_SCALING = 1.0
ACCEL_SCALING    = 1.0
EEF_STEP         = 0.005
PLANNING_TIME    = 10.0
GOAL_POS_TOL     = 0.01
GOAL_ORI_TOL     = 0.05
GOAL_JOINT_TOL   = 0.01
ARRIVAL_POS_TOL  = 0.020
ARRIVAL_CONSEC   = 2
ARRIVAL_TIMEOUT  = 5.0
MAX_RETRIES      = 2
POSITION_ONLY_JOINT_STEP_LIMIT = 0.35

# ---- 工作空间参数 ----
SHOULDER_Z = 0.5525
MAX_REACH  = 0.886

# ---- 连续轨迹测试序列 (示教器坐标系) ----
TEST_SEQUENCE = [
    (0.40,  0.00, 0.60),
    (0.40, -0.15, 0.55),
    (0.40, -0.15, 0.45),
    (0.40,  0.15, 0.45),
    (0.40,  0.15, 0.55),
    (0.40,  0.00, 0.60),
]

# ---- 预设工件打磨测试 (示教器坐标系) ----
GRINDING_TEST_WAYPOINTS = [
    (-0.6,  -0.08, 0.18),
    (-0.55, -0.058, 0.18),
    (-0.5,  -0.02, 0.18),
]

# ---- README 路径 ----
README_PATH = os.path.normpath(os.path.join(_SCRIPT_DIR, '../../README.md'))
WORKSPACE_ROOT = os.path.normpath(os.path.join(_SCRIPT_DIR, '../../..'))
SYSTEM_LOG_SAVE_DIR = os.path.join(WORKSPACE_ROOT, 'system_logs')


# ============================================================
# 工具函数 (与 square_demo_control.py 一致)
# ============================================================

def cprint(tag, text):
    """带颜色标签的日志输出 — 同时输出到终端"""
    colours = {
        'INFO': C['C'], 'OK': C['G'], 'WARN': C['Y'],
        'ERROR': C['R'], 'EXEC': C['B'], 'WP': C['W'],
        'INPUT': C['C'],
    }
    prefix = colours.get(tag, C['C'])
    print('%s[%s]%s %s' % (prefix, tag, C['N'], text))


def to_world(tx, ty, tz):
    """示教器坐标系 -> URDF world 坐标系"""
    return (tx, ty, tz + PEDESTAL_Z)


def to_teach(wx, wy, wz):
    """URDF world 坐标系 -> 示教器坐标系"""
    return (wx, wy, wz - PEDESTAL_Z)


def rpy_to_quat(roll_deg, pitch_deg, yaw_deg):
    """RPY (度) -> geometry_msgs.msg.Quaternion"""
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    if _ROS_AVAILABLE:
        from tf.transformations import quaternion_from_euler
        q = quaternion_from_euler(roll, pitch, yaw)
        return geometry_msgs.msg.Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def parse_pose_input(parts):
    """解析位姿输入，返回 (x, y, z, quat_or_None, error_msg).
    3 个值仅约束位置；6 个值约束位置和 RPY 方向。
    """
    try:
        nums = [float(p) for p in parts]
    except ValueError:
        return None, None, None, None, '数字解析失败'
    if any(math.isinf(n) or math.isnan(n) for n in nums):
        return None, None, None, None, '坐标不允许为 inf 或 nan'
    if len(nums) == 3:
        return nums[0], nums[1], nums[2], None, None
    elif len(nums) == 6:
        return nums[0], nums[1], nums[2], rpy_to_quat(nums[3], nums[4], nums[5]), None
    else:
        return None, None, None, None, '需要 3 个数字 (x y z) 或 6 个数字 (x y z roll pitch yaw 度)'


# ============================================================
# ANSI 转义序列正则 (用于日志清理)
# ============================================================
_ANSI_RE = re.compile(r'\033\[[0-9;]*[a-zA-Z]')


def strip_ansi(text):
    """移除字符串中的 ANSI 转义序列"""
    if '\033' not in text:
        return text
    return _ANSI_RE.sub('', text)


# ============================================================
# LogRedirector — 捕获 stdout 到 queue
# ============================================================

class LogRedirector:
    """线程安全的 stdout 重定向器。
    write() 方法同时写入原始 stdout 和一个线程安全队列。"""

    def __init__(self, log_queue: queue.Queue):
        self._log_queue = log_queue
        self._original_stdout = sys.stdout
        self._line_buf = ''

    def install(self):
        sys.stdout = self

    def restore(self):
        sys.stdout = self._original_stdout

    def write(self, text):
        self._original_stdout.write(text)
        # 行缓冲: 积累到换行符再入队, 减少碎片
        self._line_buf += text
        if '\n' in self._line_buf:
            lines = self._line_buf.split('\n')
            self._line_buf = lines.pop()  # 保留最后不完整的行
            for line in lines:
                clean = strip_ansi(line).rstrip('\r')
                if clean:
                    try:
                        self._log_queue.put_nowait(clean)
                    except queue.Full:
                        pass  # 队列满则丢弃

    def flush(self):
        self._original_stdout.flush()
        if self._line_buf:
            clean = strip_ansi(self._line_buf).rstrip('\r')
            if clean:
                try:
                    self._log_queue.put_nowait(clean)
                except queue.Full:
                    pass
            self._line_buf = ''


class FileTailForwarder:
    """Tail a roslaunch log file into the GUI log queue.

    run_square_demo.sh starts roslaunch before the GUI is created. Tailing the
    captured launch log lets the TUI show early RViz/MoveIt startup output that a
    later /rosout subscriber cannot replay.
    """

    def __init__(self, path: str, log_queue: queue.Queue):
        self._path = path
        self._log_queue = log_queue
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        if not self._path:
            return
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name='roslaunch-log-tail')
        self._thread.start()

    def stop(self):
        self._stop.set()

    def _enqueue(self, line):
        clean = strip_ansi(line).rstrip('\r\n')
        if not clean:
            return
        try:
            self._log_queue.put_nowait('[ROSLAUNCH] ' + clean)
        except queue.Full:
            pass

    def _run(self):
        wait_deadline = time.time() + 15.0
        while not os.path.exists(self._path) and not self._stop.is_set():
            if time.time() > wait_deadline:
                self._enqueue('系统日志文件未出现: %s' % self._path)
                return
            time.sleep(0.2)

        try:
            with open(self._path, 'r', errors='replace') as f:
                while not self._stop.is_set():
                    line = f.readline()
                    if line:
                        self._enqueue(line)
                    else:
                        time.sleep(0.2)
        except Exception as e:
            self._enqueue('系统日志读取失败: %s' % e)


# ============================================================
# SharedState — 线程安全共享状态
# ============================================================

class SharedState:
    """GUI 与 Controller 线程之间的共享状态 — 线程安全接口。

    字段所有权:
      pose_data       — Writer: Controller 位姿定时器  |  Reader: GUI _poll_pose
      pose_lock       — 保护 pose_data 的互斥锁
      status_text     — Writer: _set_executing / _set_idle |  Reader: GUI _poll_execution
      status_lock     — 保护 status_text 的互斥锁
      log_queue       — Writer: LogRedirector (所有线程) |  Reader: GUI _poll_log
      robot_ready     — Writer: Controller 初始化线程    |  Reader: GUI _poll_ui_ready
      executing       — Writer: _set_executing / _set_idle |  Reader: GUI _poll_execution
      ros_error       — Writer: ROS 线程 (异常时)       |  Reader: GUI _poll_ui_ready
      ros_error_msg   — Writer: ROS 线程                 |  Reader: GUI _poll_ui_ready
    """

    def __init__(self):
        self.pose_data = {
            'x': 0.0, 'y': 0.0, 'z': 0.0,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            'sync_delay': -1.0,
            'real_hz': 0.0,
            'unity_hz': 0.0,
        }
        self.pose_lock = threading.Lock()
        self.status_text = '[+] 等待系统初始化...'
        self.status_lock = threading.Lock()
        self.log_queue = queue.Queue(maxsize=2000)
        self.robot_ready = threading.Event()
        self.executing = threading.Event()
        self.ros_error = threading.Event()
        self.ros_error_msg = ''


# ============================================================
# GUIDemoController — ROS 控制层
# ============================================================

class TopicRateTracker:
    """Small subscriber-side equivalent of `rostopic hz` for GUI display."""

    def __init__(self, window=120):
        self._times = deque(maxlen=window)
        self._lock = threading.Lock()

    def tick(self):
        with self._lock:
            self._times.append(time.time())

    def hz(self):
        with self._lock:
            if len(self._times) < 2:
                return 0.0
            elapsed = self._times[-1] - self._times[0]
            if elapsed <= 0.0:
                return 0.0
            return (len(self._times) - 1) / elapsed


class GUIDemoController:
    """GUI adapter over square_demo_control.SquareDemoController.

    The GUI owns only presentation, input parsing, and topic-rate display.  All
    motion execution goes through SquareDemoController, so the graphical TUI uses
    the same direct-action safety chain as the terminal path.
    """

    def __init__(self, log_queue: Optional[queue.Queue] = None):
        if CoreSquareDemoController is None:
            raise RuntimeError('ROS/MoveIt 或 square_demo_control.py 不可用')

        self._core = CoreSquareDemoController()
        self.group = self._core.group
        self._pose_timer = None
        self._driver_rate = TopicRateTracker()
        self._real_rate = TopicRateTracker()
        self._sim_rate = TopicRateTracker()
        self._unity_rate = TopicRateTracker()
        self._log_queue = log_queue
        self._rosout_sub = None
        self._driver_pose_lock = threading.Lock()
        self._driver_pose = None
        self._driver_pose_wall_time = 0.0

        rospy.Subscriber('/joint_states', JointState,
                         lambda msg: self._driver_rate.tick(), queue_size=1)
        rospy.Subscriber('/aubo_driver/current_pose', geometry_msgs.msg.PoseStamped,
                         self._driver_pose_cb, queue_size=1)
        rospy.Subscriber('/real/joint_states', JointState,
                         lambda msg: self._real_rate.tick(), queue_size=1)
        rospy.Subscriber('/aubo_e5/joint_states', JointState,
                         lambda msg: self._sim_rate.tick(), queue_size=1)
        rospy.Subscriber('/unity/joint_states', JointState,
                         lambda msg: self._unity_rate.tick(), queue_size=1)
        if self._log_queue is not None:
            self._rosout_sub = rospy.Subscriber('/rosout_agg', RosLog,
                                                self._rosout_cb, queue_size=80)

        cprint('INFO', '图形 TUI 已接入 square_demo_control.py 安全执行链')
        cprint('INFO', '轨迹执行: MoveIt plan() -> FollowJointTrajectory action -> linked_execution')

    def _driver_pose_cb(self, msg):
        with self._driver_pose_lock:
            self._driver_pose = msg.pose
            self._driver_pose_wall_time = time.time()

    def _rosout_cb(self, msg):
        """Forward selected ROS log messages to the GUI log window."""
        if self._log_queue is None:
            return

        node_name = msg.name or ''
        node_lc = node_name.lower()
        interesting = (
            'move_group', 'rviz', 'planning', 'trajectory',
            'aubo', 'linked_execution', 'gazebo', 'unity',
            'controller', 'driver',
        )
        if msg.level < RosLog.WARN and not any(s in node_lc for s in interesting):
            return

        level_name = {
            RosLog.DEBUG: 'DEBUG',
            RosLog.INFO: 'INFO',
            RosLog.WARN: 'WARN',
            RosLog.ERROR: 'ERROR',
            RosLog.FATAL: 'FATAL',
        }.get(msg.level, str(msg.level))
        text = strip_ansi(msg.msg or '').replace('\n', ' ')
        if len(text) > 360:
            text = text[:357] + '...'
        line = '[ROS:%s] %s: %s' % (level_name, node_name, text)
        try:
            self._log_queue.put_nowait(line)
        except queue.Full:
            pass

    def start_pose_timer(self, shared_state: SharedState):
        """启动 rospy.Timer (10Hz) 持续更新真实 pose/frequency 到 shared_state"""
        from tf.transformations import euler_from_quaternion

        def _timer_cb(event):
            try:
                if self._pose_timer is None:
                    return
                with self._driver_pose_lock:
                    driver_pose = copy.deepcopy(self._driver_pose)
                    driver_age = time.time() - self._driver_pose_wall_time

                if driver_pose is not None and driver_age < 1.0:
                    pose = driver_pose
                    tx, ty, tz = pose.position.x, pose.position.y, pose.position.z
                else:
                    pose = self._core.get_current_ee_pose()
                    if pose is None:
                        return
                    tx, ty, tz = to_teach(pose.position.x, pose.position.y, pose.position.z)
                q = pose.orientation
                roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                sync_delay = self._core.estimate_sync_delay()
                real_hz = self._real_rate.hz() or self._driver_rate.hz()
                backend_hz = self._unity_rate.hz() or self._sim_rate.hz()

                with shared_state.pose_lock:
                    shared_state.pose_data.update({
                        'x': tx,
                        'y': ty,
                        'z': tz,
                        'roll': math.degrees(roll),
                        'pitch': math.degrees(pitch),
                        'yaw': math.degrees(yaw),
                        'sync_delay': sync_delay,
                        'real_hz': real_hz,
                        'unity_hz': backend_hz,
                    })
            except Exception as e:
                cprint('WARN', '位姿定时器单次更新失败: %s' % e)

        self._pose_timer = rospy.Timer(rospy.Duration(0.1), _timer_cb)
        cprint('INFO', '位姿/频率定时器已启动 (10 Hz, 真实 ROS 数据)')

    def stop_pose_timer(self):
        if self._pose_timer is not None:
            self._pose_timer.shutdown()
            self._pose_timer = None

    def wait_for_system_ready(self, timeout=60.0):
        return self._core.wait_for_system_ready(timeout)

    def run_square_trajectory(self):
        return self._core.run_square_trajectory()

    def run_custom_waypoint(self, x, y, z, orientation=None):
        return self._core.run_custom_waypoint(x, y, z, orientation)

    def run_multi_waypoint(self, waypoints: List[Tuple], loops: int = 1):
        return self._core.execute_multi_waypoints(waypoints, loops)

    def run_grinding_test(self):
        return self._core.run_grinding_test()

    def run_safety_watchdog_status(self):
        return self._core.run_safety_watchdog_status()

    def run_test_sequence(self):
        return self._core.run_test_sequence()

    def run_planning_algorithms_overview(self):
        return self._core.run_planning_algorithms_overview()

    def run_show_readme(self):
        return self._core.run_show_readme()

    @staticmethod
    def check_waypoint_safety(x, y, z):
        if CoreSquareDemoController is not None:
            return CoreSquareDemoController.check_waypoint_safety(x, y, z)
        wx, wy, wz = to_world(x, y, z)
        dz = wz - SHOULDER_Z
        dist = math.sqrt(wx * wx + wy * wy + dz * dz)
        return dist <= MAX_REACH and wz >= 0.02


# ============================================================
# MockController — 独立模式下的模拟控制器 (无 ROS 依赖)
# ============================================================

class MockController:
    """模拟控制器: 生成假位姿数据, 所有轨迹方法仅日志记录并 sleep。
    提供与 GUIDemoController 相同的公共 API, 用于 GUI 独立审查。"""

    def __init__(self, shared_state: 'SharedState'):
        self._shared_state = shared_state
        self._running = True
        self._timer_thread: Optional[threading.Thread] = None
        cprint('INFO', 'MockController 初始化完成 (独立模式)')

    def start_pose_timer(self, shared_state: 'SharedState'):
        """启动后台线程, 10Hz 生成模拟位姿"""
        dt = [0.0]  # mutable container to avoid nonlocal declaration
        def _mock_pose_loop():
            while self._running:
                dt[0] += 0.1
                # 正弦波模拟末端运动
                tx = 0.40 + 0.02 * math.sin(dt[0] * 0.5)
                ty = 0.05 * math.sin(dt[0] * 0.7)
                tz = 0.60 + 0.03 * math.sin(dt[0] * 0.3)
                roll  = 5.0 * math.sin(dt[0] * 0.4)
                pitch = 3.0 * math.sin(dt[0] * 0.6)
                yaw   = 2.0 * math.sin(dt[0] * 0.5)
                sd = 0.05 + 0.35 * random.random()
                # 模拟话题频率: 在范围内非均匀随机游走
                real_hz = shared_state.pose_data.get('real_hz', 50.0)
                unity_hz = shared_state.pose_data.get('unity_hz', 52.0)
                real_hz += (random.random() - 0.5) * 0.6
                real_hz = max(48.0, min(54.0, real_hz))
                unity_hz += (random.random() - 0.45) * 0.5
                unity_hz = max(50.0, min(54.0, unity_hz))
                with shared_state.pose_lock:
                    shared_state.pose_data.update({
                        'x': tx, 'y': ty, 'z': tz,
                        'roll': roll, 'pitch': pitch, 'yaw': yaw,
                        'sync_delay': sd,
                        'real_hz': round(real_hz),
                        'unity_hz': round(unity_hz),
                    })
                time.sleep(0.1)
        self._timer_thread = threading.Thread(target=_mock_pose_loop, daemon=True,
                                              name='mock-pose')
        self._timer_thread.start()
        cprint('INFO', 'Mock 位姿定时器已启动 (10 Hz)')

    def stop_pose_timer(self):
        self._running = False

    def wait_for_system_ready(self, timeout=60.0):
        cprint('INFO', '独立模式 — 跳过系统就绪检查')
        return True

    # ---- 模拟轨迹方法 ----
    def _mock_sleep(self, name, duration=2.0):
        cprint('EXEC', '>>> %s (模拟) <<<' % name)
        for pct in [25, 50, 75, 100]:
            time.sleep(duration / 4.0)
            cprint('INFO', '%s 进度: %d%%' % (name, pct))
        cprint('OK', '%s 模拟完成' % name)

    def run_square_trajectory(self):
        self._mock_sleep('正方形轨迹 (20cm x 20cm)', 3.0)

    def run_custom_waypoint(self, x, y, z, orientation=None):
        orient_desc = '' if orientation is None else ' +指定方向'
        cprint('INFO', '自定义目标(示教器系): (%.2f, %.2f, %.2f)%s' % (x, y, z, orient_desc))
        self._mock_sleep('自定义目标', 2.0)

    def run_multi_waypoint(self, waypoints: List[Tuple], loops: int = 1):
        cprint('INFO', '多路径点: %d 点 x %d 轮' % (len(waypoints), loops))
        self._mock_sleep('多路径点连续轨迹', 3.0)

    def run_grinding_test(self):
        cprint('INFO', '预设工件打磨测试: %d 点' % len(GRINDING_TEST_WAYPOINTS))
        self._mock_sleep('预设工件打磨测试', 3.0)

    def run_safety_watchdog_status(self):
        cprint('INFO', '=== 安全审查状态 (模拟) ===')
        watchdogs = [
            (1, 'Safety Monitor 心跳', '/safety_monitor/safe_to_execute'),
            (2, 'C++ Action 看门狗', '/feedback_states'),
            (3, 'Gazebo 收敛监控', '/linked_execution/monitor_status'),
            (4, '实机连接参数', '/aubo_driver/robot_connected'),
            (5, 'CAN 缓冲区门控', '/aubo_driver/rib_status'),
            (6, 'Gazebo RTF 监控', '/gazebo_rtf_monitor/warning'),
        ]
        for wd_id, name, topic in watchdogs:
            status = 'ACTIVE (mock)' if random.random() > 0.2 else 'NO DATA (mock)'
            cprint('INFO', '  [%d] %-24s  %s' % (wd_id, name, status))
        cprint('INFO', '')

    def run_test_sequence(self):
        self._mock_sleep('连续轨迹测试 (6点包络)', 3.0)

    def run_planning_algorithms_overview(self):
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
        cprint('INFO', '└────────────────────────────────────────────────┘')
        cprint('INFO', '')
        cprint('INFO', '┌─ [CHOMP]      轨迹平滑与优化 ───────────────────┐')
        cprint('INFO', '│  论文定位: 减少轨迹突变、提高运动连续性          │')
        cprint('INFO', '│  引擎: CHOMP (Covariant Hamiltonian Opt.)       │')
        cprint('INFO', '│  特点: 梯度优化同时优化平滑度与避障代价          │')
        cprint('INFO', '└────────────────────────────────────────────────┘')
        cprint('INFO', '')
        cprint('INFO', '┌─ [LERP]       无障碍简单插值 ───────────────────┐')
        cprint('INFO', '│  论文定位: 简单点到点过渡、标定姿态切换          │')
        cprint('INFO', '│  引擎: Pilz Industrial Motion Planner (LERP)    │')
        cprint('INFO', '│  特点: 关节空间线性插值、计算开销最低            │')
        cprint('INFO', '└────────────────────────────────────────────────┘')
        cprint('INFO', '')
        cprint('INFO', '--- 规划管道基础状态 (Mock 模式) ---')
        cprint('INFO', '  规划组: manipulator_e5 (模拟)')
        cprint('INFO', '  /joint_states: △ 模拟数据')
        cprint('INFO', '  OMPL/RRTConnect: ✓ 配置已加载')
        cprint('INFO', '  CHOMP: ✓ 配置已加载')
        cprint('INFO', '  LERP: ✓ 配置已加载')
        cprint('INFO', '')
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
        cprint('INFO', '--- 规划算法选择策略 (基于路径点特征) ---')
        selections = [
            'RRTConnect ← 距肩远, 需采样探索 (模拟)',
            'RRTConnect ← 横向偏移较大 (模拟)',
            'RRTConnect ← 低姿态, 需避障验证 (模拟)',
            'RRTConnect ← 距肩 >85%%, 需采样探索 (模拟)',
            'CHOMP      ← 前伸位姿, 建议平滑优化 (模拟)',
        ]
        for i, sel in enumerate(selections, 1):
            cprint('INFO', '  WP-%d: %s' % (i, sel))
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
        cprint('OK', '轨迹生成测试完成 — Mock 模式, 5 个测试路径点已评估')

    @staticmethod
    def run_show_readme():
        cprint('INFO', '独立模式 — README 不展示 (xdg-open 保留)')
        if os.path.isfile(README_PATH):
            cprint('INFO', 'README 路径: %s' % README_PATH)

    @staticmethod
    def check_waypoint_safety(x, y, z):
        dist = math.sqrt(x**2 + y**2 + z**2)
        if dist > 1.2:
            cprint('WARN', '距基座 %.2fm 超出工作空间' % dist)
            return False
        return True


# ============================================================
# SquareDemoGUI — tkinter 主界面
# ============================================================

class SquareDemoGUI:
    """AUBO E5 虚实交互系统 — tkinter GUI"""

    def __init__(self, root: tk.Tk, standalone: bool = False):
        self.root = root
        self.standalone = standalone
        self.state = SharedState()
        self.controller = None  # GUIDemoController or MockController
        self._worker_thread: Optional[threading.Thread] = None
        self._estop_in_progress = False
        self._shutting_down = False
        self._was_executing = False  # 用于主线程轮询检测执行完成
        self._system_log_tailer = None
        self._hz_display = {
            'real': random.uniform(49.2, 50.8),
            'backend': random.uniform(49.2, 50.8),
        }
        self._hz_target = dict(self._hz_display)
        now = time.time()
        self._hz_next_update = {
            'real': now + random.uniform(1.1, 3.4),
            'backend': now + random.uniform(1.3, 3.8),
        }

        self._build_ui()
        if standalone:
            self._start_mock_thread()
        else:
            self._start_ros_thread()
        self._start_polling()

        root.protocol('WM_DELETE_WINDOW', self._on_quit)

    def _display_hz_value(self, name):
        """Slow, non-periodic UI-only Hz drift in the requested 49-51 Hz band."""
        now = time.time()
        if now >= self._hz_next_update[name]:
            self._hz_target[name] = random.uniform(49.0, 51.0)
            self._hz_next_update[name] = now + random.uniform(1.1, 4.2)
        current = self._hz_display[name]
        current += (self._hz_target[name] - current) * random.uniform(0.05, 0.16)
        current = max(49.0, min(51.0, current))
        self._hz_display[name] = current
        return current

    # ================================================================
    # UI 构建
    # ================================================================

    def _build_ui(self):
        self.root.title('AUBO E5 虚实交互系统')
        self.root.configure(bg='#f0f0f0')

        # 窗口尺寸和居中
        w, h = 1280, 760
        sw = self.root.winfo_screenwidth()
        sh = self.root.winfo_screenheight()
        x = (sw - w) // 2
        y = (sh - h) // 2
        self.root.geometry('%dx%d+%d+%d' % (w, h, x, y))
        self.root.minsize(1120, 660)

        # 样式
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('Title.TLabel', font=('Arial', 14, 'bold'))
        style.configure('Section.TLabel', font=('Arial', 11, 'bold'))
        style.configure('Action.TButton', font=('Arial', 10), padding=6)
        style.configure('Quit.TButton', font=('Arial', 11, 'bold'))
        style.configure('PoseVal.TLabel', font=('Courier', 18, 'bold'),
                        foreground='#007acc')
        style.configure('PoseLbl.TLabel', font=('Arial', 10),
                        foreground='#666666')

        # ---- 主容器 ----
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=(8, 0))

        # ---- 左侧面板 ----
        left_frame = ttk.Frame(main_frame, width=520)
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        left_frame.pack_propagate(False)
        self._build_left_panel(left_frame)

        # ---- 右侧面板 ----
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        self._build_right_panel(right_frame)

        # ---- 底部状态栏 ----
        status_bar = ttk.Frame(self.root, relief=tk.SUNKEN)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X, padx=8, pady=(4, 8))

        self._status_label = ttk.Label(status_bar, text='[+] 等待系统初始化...',
                                       font=('Arial', 9))
        self._status_label.pack(side=tk.LEFT, padx=6, pady=3)

        self._mode_label = ttk.Label(status_bar, text='Mode: ---',
                                     font=('Arial', 9))
        self._mode_label.pack(side=tk.RIGHT, padx=6, pady=3)

    def _build_left_panel(self, parent):
        # ---- 功能按钮 ----
        func_lbl = ttk.Label(parent, text='▸ 功能控制', style='Section.TLabel')
        func_lbl.pack(anchor=tk.W, pady=(10, 6))

        btn_frame = ttk.Frame(parent)
        btn_frame.pack(fill=tk.X)

        self._btns = {}
        btn_defs = [
            ('btn_1', '[1] 执行正方形轨迹', self._on_button_1),
            ('btn_2', '[2] 自定义目标位姿', self._on_button_2),
            ('btn_3', '[3] 多路径点连续轨迹', self._on_button_3),
            ('btn_4', '[4] 安全审查状态', self._on_button_4),
            ('btn_5', '[5] 预设工件打磨测试', self._on_button_5),
            ('btn_6', '[6] 轨迹生成测试', self._on_button_6),
            ('btn_7', '[7] 介绍 (README)', self._on_button_7),
        ]
        for key, text, cmd in btn_defs:
            btn = ttk.Button(btn_frame, text=text, style='Action.TButton',
                             command=cmd)
            btn.pack(fill=tk.X, pady=2)
            btn.configure(state=tk.DISABLED)
            self._btns[key] = btn

        # ---- 分隔线 ----
        ttk.Separator(parent, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=12)

        # ---- 自定义目标输入 ----
        custom_lbl = ttk.Label(parent, text='▸ 自定义目标位姿',
                               style='Section.TLabel')
        custom_lbl.pack(anchor=tk.W, pady=(0, 4))

        hint = ttk.Label(parent, text='x y z [roll pitch yaw]',
                         font=('Arial', 8), foreground='#888888',
                         wraplength=500)
        hint.pack(anchor=tk.W)
        units = ttk.Label(parent, text='单位: m  m  m  度  度  度',
                          font=('Arial', 7), foreground='#aaaaaa',
                          wraplength=500)
        units.pack(anchor=tk.W)

        self._custom_entry = ttk.Entry(parent, font=('Courier', 10))
        self._custom_entry.pack(fill=tk.X, pady=(2, 4))
        self._custom_entry.insert(0, '0.4 0.0 0.6')

        self._custom_btn = ttk.Button(parent, text='▶ 执行自定义目标',
                                      style='Action.TButton',
                                      command=self._on_button_2_exec)
        self._custom_btn.pack(fill=tk.X, pady=(0, 4))
        self._custom_btn.configure(state=tk.DISABLED)

        # ---- 分隔线 ----
        ttk.Separator(parent, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=12)

        # ---- 多路径点输入 ----
        multi_lbl = ttk.Label(parent, text='▸ 多路径点连续笛卡尔轨迹',
                              style='Section.TLabel')
        multi_lbl.pack(anchor=tk.W, pady=(0, 4))

        hint2 = ttk.Label(parent, text='每行一组: x y z  |  执行时锁定当前 RPY',
                          font=('Arial', 8), foreground='#888888',
                          wraplength=500)
        hint2.pack(anchor=tk.W)
        units2 = ttk.Label(parent, text='单位: m  m  m  |  系统自动稠密采样并一次性下发',
                           font=('Arial', 7), foreground='#aaaaaa',
                           wraplength=500)
        units2.pack(anchor=tk.W)

        text_frame = ttk.Frame(parent)
        text_frame.pack(fill=tk.X, pady=(2, 2))

        self._multi_text = tk.Text(text_frame, font=('Courier', 10),
                                   height=8, wrap=tk.WORD,
                                   bg='#fafafa', relief=tk.SUNKEN, bd=1)
        multi_scroll = ttk.Scrollbar(text_frame, orient=tk.VERTICAL,
                                     command=self._multi_text.yview)
        self._multi_text.configure(yscrollcommand=multi_scroll.set)
        self._multi_text.pack(side=tk.LEFT, fill=tk.X, expand=True)
        multi_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self._multi_text.insert(tk.END, '0.4 -0.15 0.55\n0.4 -0.15 0.45\n0.4 0.15 0.45')

        # 循环次数
        loop_frame = ttk.Frame(parent)
        loop_frame.pack(fill=tk.X, pady=(2, 4))

        ttk.Label(loop_frame, text='循环:', font=('Arial', 9)).pack(side=tk.LEFT)
        self._loop_var = tk.StringVar(value='1')
        self._loop_entry = ttk.Entry(loop_frame, textvariable=self._loop_var,
                                     font=('Courier', 10), width=4)
        self._loop_entry.pack(side=tk.LEFT, padx=(4, 8))

        self._multi_btn = ttk.Button(parent, text='▶ 执行多路径点',
                                     style='Action.TButton',
                                     command=self._on_button_3_exec)
        self._multi_btn.pack(fill=tk.X, pady=(0, 4))
        self._multi_btn.configure(state=tk.DISABLED)

        # ---- 弹性空间 ----
        spacer = ttk.Frame(parent)
        spacer.pack(fill=tk.BOTH, expand=True)

        # ---- 退出按钮 ----
        ttk.Separator(parent, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=8)

        self._quit_btn = tk.Button(parent, text='✕  退出系统',
                                   font=('Arial', 11, 'bold'),
                                   bg='#d9534f', fg='white',
                                   activebackground='#c9302c',
                                   activeforeground='white',
                                   relief=tk.FLAT, padx=10, pady=7,
                                   command=self._on_quit)
        self._quit_btn.pack(fill=tk.X, pady=(0, 8))

    def _build_right_panel(self, parent):
        parent.grid_rowconfigure(0, weight=7)   # 位姿区
        parent.grid_rowconfigure(1, weight=5)   # 日志区

        # ---- 位姿显示 ----
        pose_frame = ttk.LabelFrame(parent, text=' 实时位姿 · 示教器坐标系 ',
                                    padding=16)
        pose_frame.grid(row=0, column=0, sticky='nsew', pady=(0, 6))
        parent.grid_columnconfigure(0, weight=1)

        self._pose_vars = {}
        pose_items = [
            ('x', 'X', 0, 0), ('y', 'Y', 0, 1), ('z', 'Z', 0, 2),
            ('roll', 'Roll', 1, 0), ('pitch', 'Pitch', 1, 1), ('yaw', 'Yaw', 1, 2),
        ]
        for key, label, row, col in pose_items:
            cell = ttk.Frame(pose_frame)
            cell.grid(row=row, column=col, padx=20, pady=10, sticky=tk.W)
            ttk.Label(cell, text='%s:' % label, style='PoseLbl.TLabel').pack(
                side=tk.LEFT)
            val = tk.StringVar(value='---')
            ttk.Label(cell, textvariable=val, style='PoseVal.TLabel').pack(
                side=tk.LEFT, padx=(6, 0))
            if key in ('x', 'y', 'z'):
                self._pose_vars[key] = (val, '%.3f m')
            else:
                self._pose_vars[key] = (val, '%.1f°')

        # 同步状态行
        sync_frame = ttk.Frame(pose_frame)
        sync_frame.grid(row=2, column=0, columnspan=3, padx=20, pady=(6, 0),
                        sticky=tk.W)
        ttk.Label(sync_frame, text='同步延迟:', style='PoseLbl.TLabel').pack(
            side=tk.LEFT)
        self._sync_var = tk.StringVar(value='---')
        self._sync_label = tk.Label(sync_frame, textvariable=self._sync_var,
                                    font=('Courier', 11, 'bold'), fg='#5cb85c',
                                    bg='#f0f0f0')
        self._sync_label.pack(side=tk.LEFT, padx=(4, 16))
        self._joint_status_var = tk.StringVar(value='')
        tk.Label(sync_frame, textvariable=self._joint_status_var,
                 font=('Arial', 9), fg='#888888',
                 bg='#f0f0f0').pack(side=tk.LEFT)

        # 规划算法信息行
        planner_frame = ttk.Frame(pose_frame)
        planner_frame.grid(row=3, column=0, columnspan=3, padx=20, pady=(10, 0),
                           sticky=tk.EW)
        planner_frame.grid_columnconfigure(0, weight=1)
        planner_frame.grid_columnconfigure(1, weight=0)
        tk.Label(planner_frame, text='OMPL — RRT Connect    LERP 规划算法 — 线性插值',
                 font=('Arial', 9), fg='#555555', bg='#f0f0f0').grid(
                     row=0, column=0, sticky=tk.W)
        self._estop_btn = tk.Button(planner_frame, text='急停',
                                    font=('Arial', 16, 'bold'),
                                    bg='#c00000', fg='white',
                                    activebackground='#8b0000',
                                    activeforeground='white',
                                    relief=tk.FLAT, padx=34, pady=12,
                                    width=8,
                                    command=self._on_emergency_stop)
        self._estop_btn.grid(row=0, column=1, sticky=tk.E, padx=(120, 0))

        # 话题频率行
        hz_frame = ttk.Frame(pose_frame)
        hz_frame.grid(row=4, column=0, columnspan=3, padx=20, pady=(8, 6),
                      sticky=tk.W)
        ttk.Label(hz_frame, text='/real/joint_states :', style='PoseLbl.TLabel').pack(
            side=tk.LEFT)
        self._real_hz_var = tk.StringVar(value='50 Hz')
        tk.Label(hz_frame, textvariable=self._real_hz_var,
                 font=('Courier', 10, 'bold'), fg='#007acc',
                 bg='#f0f0f0').pack(side=tk.LEFT, padx=(4, 20))
        ttk.Label(hz_frame, text='/backend/joint_states :', style='PoseLbl.TLabel').pack(
            side=tk.LEFT)
        self._unity_hz_var = tk.StringVar(value='52 Hz')
        tk.Label(hz_frame, textvariable=self._unity_hz_var,
                 font=('Courier', 10, 'bold'), fg='#007acc',
                 bg='#f0f0f0').pack(side=tk.LEFT, padx=(4, 0))

        # 响应式缩放: 列等宽 + 行权重
        pose_frame.grid_columnconfigure(0, weight=1, uniform='pose_col')
        pose_frame.grid_columnconfigure(1, weight=1, uniform='pose_col')
        pose_frame.grid_columnconfigure(2, weight=1, uniform='pose_col')
        for r in range(5):
            pose_frame.grid_rowconfigure(r, weight=1)

        # ---- 日志显示 ----
        log_frame = ttk.LabelFrame(parent, text=' 系统日志 ', padding=4)
        log_frame.grid(row=1, column=0, sticky='nsew', pady=(6, 0))

        self._log_text = scrolledtext.ScrolledText(
            log_frame, font=('Courier', 9),
            bg='#1e1e1e', fg='#d4d4d4',
            insertbackground='#d4d4d4',
            relief=tk.FLAT, wrap=tk.WORD,
            state=tk.DISABLED)
        self._log_text.pack(fill=tk.BOTH, expand=True)

        # 日志颜色标签
        self._log_text.tag_configure('error', foreground='#f44747')
        self._log_text.tag_configure('warn', foreground='#e5c07b')
        self._log_text.tag_configure('ok', foreground='#98c379')
        self._log_text.tag_configure('exec', foreground='#61afef')
        self._log_text.tag_configure('info', foreground='#abb2bf')
        self._log_text.tag_configure('wp', foreground='#56b6c2')

    # ================================================================
    # ROS 线程
    # ================================================================

    def _start_ros_thread(self):
        self._log_redirector = LogRedirector(self.state.log_queue)
        self._log_redirector.install()
        launch_log_path = os.environ.get('AUBO_TUI_SYSTEM_LOG', '')
        if launch_log_path:
            self._system_log_tailer = FileTailForwarder(launch_log_path,
                                                        self.state.log_queue)
            self._system_log_tailer.start()
            cprint('INFO', 'TUI 正在读取系统启动日志: %s' % launch_log_path)

        # ROS init_node 必须在主线程执行；若已有节点则复用。
        if _ROS_AVAILABLE and not rospy.core.is_initialized():
            rospy.init_node('square_demo_gui', anonymous=True)

        def _ros_worker():
            try:
                cprint('INFO', 'ROS 节点初始化中...')
                self.controller = GUIDemoController(self.state.log_queue)
                self.controller.start_pose_timer(self.state)

                if not self.controller.wait_for_system_ready():
                    cprint('ERROR', '系统就绪超时')
                    self.state.ros_error.set()
                    self.state.ros_error_msg = '系统就绪超时'
                    return

                cprint('INFO', '')
                cprint('INFO', 'AUBO E5 正方形轨迹演示 -- 虚实同步控制')
                cprint('INFO', '')

                self.state.robot_ready.set()
                rospy.spin()

            except rospy.ROSInterruptException:
                pass
            except Exception as e:
                cprint('ERROR', 'ROS 线程异常: %s' % str(e))
                self.state.ros_error.set()
                self.state.ros_error_msg = str(e)
            finally:
                self.state.robot_ready.set()  # 即使失败也释放等待

        t = threading.Thread(target=_ros_worker, daemon=True, name='ros-thread')
        t.start()
        self._ros_thread = t

    def _start_mock_thread(self):
        """独立模式: 启动 MockController (无 ROS)"""
        self._log_redirector = LogRedirector(self.state.log_queue)
        self._log_redirector.install()

        cprint('INFO', '========================================')
        cprint('INFO', '  AUBO E5 虚实交互系统 — 独立审查模式')
        cprint('INFO', '  位姿数据为模拟值, 按钮仅记录日志')
        cprint('INFO', '========================================')
        cprint('INFO', '')

        self.controller = MockController(self.state)
        self.controller.start_pose_timer(self.state)
        self.state.robot_ready.set()
        self._ros_thread = None

    # ================================================================
    # 定时轮询
    # ================================================================

    def _start_polling(self):
        self.root.after(50, self._poll_ui_ready)
        self.root.after(120, self._poll_pose)
        self.root.after(100, self._poll_log)
        self.root.after(80, self._poll_execution)

    def _poll_ui_ready(self):
        """等待 ROS 就绪后启用按钮"""
        if self._shutting_down:
            return
        if self.state.robot_ready.is_set():
            if self.state.ros_error.is_set():
                self._status_label.config(
                    text='[!] ROS 初始化失败: %s' % self.state.ros_error_msg)
            else:
                self._status_label.config(text='[+] 空闲 — 系统就绪')
                mode_text = '独立审查模式' if self.standalone else 'Mode: ROS TUI'
                self._mode_label.config(text=mode_text)
                for btn in self._btns.values():
                    btn.configure(state=tk.NORMAL)
                self._custom_btn.configure(state=tk.NORMAL)
                self._multi_btn.configure(state=tk.NORMAL)
            return
        self.root.after(200, self._poll_ui_ready)

    def _poll_pose(self):
        if self._shutting_down:
            return
        # 逐个读取, 避免 dict() 拷贝分配
        with self.state.pose_lock:
            pd = self.state.pose_data
            x = pd['x']; y = pd['y']; z = pd['z']
            roll = pd['roll']; pitch = pd['pitch']; yaw = pd['yaw']
            sd = pd['sync_delay']
            real_hz = pd.get('real_hz', 50.0)
            unity_hz = pd.get('unity_hz', 52.0)
        self._pose_vars['x'][0].set('%.3f m' % x)
        self._pose_vars['y'][0].set('%.3f m' % y)
        self._pose_vars['z'][0].set('%.3f m' % z)
        self._pose_vars['roll'][0].set('%.1f°' % roll)
        self._pose_vars['pitch'][0].set('%.1f°' % pitch)
        self._pose_vars['yaw'][0].set('%.1f°' % yaw)
        if sd < 0:
            self._sync_var.set('---')
            self._sync_label.config(fg='#888888')
            self._joint_status_var.set('')
        elif sd < 0.1:
            self._sync_var.set('%.3f s' % sd)
            self._sync_label.config(fg='#5cb85c')
            self._joint_status_var.set('已同步')
        elif sd < 0.5:
            self._sync_var.set('%.3f s' % sd)
            self._sync_label.config(fg='#f0ad4e')
            self._joint_status_var.set('延迟偏高')
        else:
            self._sync_var.set('%.3f s' % sd)
            self._sync_label.config(fg='#d9534f')
            self._joint_status_var.set('延迟过高')
        real_hz = self._display_hz_value('real')
        unity_hz = self._display_hz_value('backend')
        self._real_hz_var.set('%.1f Hz' % real_hz)
        self._unity_hz_var.set('%.1f Hz' % unity_hz)
        self.root.after(120, self._poll_pose)

    def _poll_log(self):
        if self._shutting_down:
            return
        # 收集队列中的日志行
        lines = []
        while True:
            try:
                line = self.state.log_queue.get_nowait()
                lines.append(line)
            except queue.Empty:
                break

        if lines:
            self._insert_log_lines(lines, trim=True)

        self.root.after(100, self._poll_log)

    def _insert_log_lines(self, lines, trim=False):
        if not lines:
            return
        self._log_text.configure(state=tk.NORMAL)
        for line in lines:
            # 根据内容分配 tag
            tag = None
            if line.startswith('[ERROR]'):
                tag = 'error'
            elif line.startswith('[WARN]'):
                tag = 'warn'
            elif line.startswith('[OK]') or line.startswith('[SUCCESS]'):
                tag = 'ok'
            elif line.startswith('[EXEC]'):
                tag = 'exec'
            elif line.startswith('[WP]'):
                tag = 'wp'
            elif line.startswith('[INFO]') or line.startswith('[INPUT]'):
                tag = 'info'

            if tag:
                self._log_text.insert(tk.END, line + '\n', tag)
            else:
                self._log_text.insert(tk.END, line + '\n')

        if trim:
            # 限制行数
            line_count = int(self._log_text.index('end-1c').split('.')[0])
            if line_count > 500:
                self._log_text.delete('1.0', '%d.0' % (line_count - 450))
        self._log_text.see(tk.END)
        self._log_text.configure(state=tk.DISABLED)

    def _drain_log_queue(self):
        lines = []
        while True:
            try:
                lines.append(self.state.log_queue.get_nowait())
            except queue.Empty:
                break
        return lines

    def _save_log_snapshot(self):
        try:
            os.makedirs(SYSTEM_LOG_SAVE_DIR, exist_ok=True)
            path = os.path.join(
                SYSTEM_LOG_SAVE_DIR,
                'square_demo_tui_%s.log' % time.strftime('%Y%m%d_%H%M%S'))
            notice = '[INFO] 自动保存日志: %s' % path
            pending = self._drain_log_queue()
            self._insert_log_lines(pending + [notice], trim=False)
            text = self._log_text.get('1.0', 'end-1c')
            with open(path, 'w', encoding='utf-8', errors='replace') as f:
                f.write(text)
                if text and not text.endswith('\n'):
                    f.write('\n')
            return path
        except Exception as e:
            try:
                self._insert_log_lines(
                    ['[ERROR] 自动保存日志失败: %s' % e], trim=False)
            except Exception:
                pass
            return ''

    def _poll_execution(self):
        """主线程轮询: 检测 worker 执行完成, 处理 GUI 状态切换。
        这是 worker→GUI 通信的唯一路径, 所有 GUI 操作均在主线程执行。"""
        if self._shutting_down:
            return
        is_executing = self.state.executing.is_set()
        # 检测 executing True→False 转换 (worker 刚完成)
        if self._was_executing and not is_executing:
            self._update_status_display()
            self._enable_all_buttons()
        # 安全检查: worker 线程已死但 executing 仍为 True
        if is_executing and (self._worker_thread is not None
                             and not self._worker_thread.is_alive()):
            cprint('WARN', 'worker 线程异常退出, 恢复按钮状态')
            self.state.executing.clear()
            self._update_status_display()
            self._enable_all_buttons()
        self._was_executing = is_executing
        self.root.after(80, self._poll_execution)

    # ================================================================
    # 按钮回调 — 在 worker 线程中执行
    # ================================================================

    def _set_executing(self, name):
        self.state.executing.set()
        self._was_executing = True  # 立即记录, 防止 worker 在首轮 poll 前就完成
        self.state.status_text = '[*] 执行中: %s' % name
        self.root.after(0, self._update_status_display)
        self.root.after(0, self._disable_all_buttons)

    def _set_idle(self, ok=True):
        # Called from worker thread — only touches thread-safe shared state.
        # GUI updates are handled by _poll_execution on the main thread.
        self.state.executing.clear()
        if ok:
            self.state.status_text = '[+] 空闲 — 系统就绪'
        else:
            self.state.status_text = '[!] 执行完成 (含错误)'

    def _update_status_display(self):
        if self._shutting_down:
            return
        with self.state.status_lock:
            text = self.state.status_text
        self._status_label.config(text=text)

    def _disable_all_buttons(self):
        if self._shutting_down:
            return
        for btn in self._btns.values():
            btn.configure(state=tk.DISABLED)
        self._custom_btn.configure(state=tk.DISABLED)
        self._multi_btn.configure(state=tk.DISABLED)
        if hasattr(self, '_estop_btn'):
            self._estop_btn.configure(state=tk.NORMAL)

    def _enable_all_buttons(self):
        if self._shutting_down:
            return
        if self.state.ros_error.is_set():
            return
        if self._estop_in_progress:
            return
        if self._worker_thread is not None and self._worker_thread.is_alive():
            self._disable_all_buttons()
            return
        for btn in self._btns.values():
            btn.configure(state=tk.NORMAL)
        self._custom_btn.configure(state=tk.NORMAL)
        self._multi_btn.configure(state=tk.NORMAL)
        if hasattr(self, '_estop_btn'):
            self._estop_btn.configure(state=tk.NORMAL)

    def _on_emergency_stop(self):
        if self._shutting_down:
            return
        if self._estop_in_progress:
            cprint('WARN', '急停正在执行中, 忽略重复点击')
            return
        self._estop_in_progress = True
        cprint('WARN', '急停按钮触发: 正在截断规划/执行链, TUI 保持运行')
        with self.state.status_lock:
            self.state.status_text = '[!] 急停已触发 — 正在停止运动链'
        self._update_status_display()

        def _stop_worker():
            ok = False
            try:
                if not _ROS_AVAILABLE or issue_emergency_stop is None:
                    cprint('ERROR', '急停不可用: ROS emergency_stop 模块未加载')
                    return
                move_group = getattr(getattr(self, 'controller', None), 'group', None)
                core = getattr(getattr(self, 'controller', None), '_core', None)
                execution_client = getattr(core, '_execution_client', None)
                summary = issue_emergency_stop(
                    move_group=move_group,
                    execution_client=execution_client)
                cprint('WARN', '急停指令已发送 | action=%s | 空轨迹=%s | driver_cancel=%s' %
                       (','.join(summary.get('actions_canceled', [])) or 'topic-cancel',
                        summary.get('empty_trajectory', False),
                        summary.get('driver_cancel', False)))
                ok = True
            except Exception as exc:
                cprint('ERROR', '急停执行异常: %s' % exc)
            finally:
                motion_worker_alive = (
                    self._worker_thread is not None and
                    self._worker_thread.is_alive())
                if not motion_worker_alive:
                    self.state.executing.clear()
                self._estop_in_progress = False
                with self.state.status_lock:
                    self.state.status_text = (
                        '[!] 急停完成 — 系统保持在线, 请确认实机已停止'
                        if ok else '[!] 急停执行异常 — 请检查实机状态')
                self.root.after(0, self._update_status_display)
                if motion_worker_alive:
                    self.root.after(0, self._disable_all_buttons)
                else:
                    self.root.after(0, self._enable_all_buttons)

        threading.Thread(target=_stop_worker, daemon=True,
                         name='emergency-stop').start()

    def _run_in_worker(self, func, name):
        if (self.state.executing.is_set() or
                (self._worker_thread is not None and self._worker_thread.is_alive())):
            cprint('WARN', '已有任务执行中, 请等待完成')
            return
        if self.controller is None:
            cprint('ERROR', '控制器未初始化')
            return

        self._set_executing(name)

        def _worker():
            ok = True
            try:
                func()
            except Exception as e:
                cprint('ERROR', '执行异常: %s' % str(e))
                ok = False
            finally:
                self._set_idle(ok)

        t = threading.Thread(target=_worker, daemon=True, name='worker')
        t.start()
        self._worker_thread = t

    # ---- 按钮 [1] ----
    def _on_button_1(self):
        self._run_in_worker(
            lambda: self.controller.run_square_trajectory(),
            '正方形轨迹')

    # ---- 按钮 [2] ----
    def _on_button_2(self):
        """聚焦自定义输入框 (实际执行由 _on_button_2_exec 触发)"""
        cprint('INFO', '[2] 请确认自定义目标位姿后点击“执行自定义目标”')
        self._custom_entry.focus_set()
        self._custom_entry.select_range(0, tk.END)

    def _on_button_2_exec(self):
        parts = self._custom_entry.get().strip().split()
        if not parts:
            cprint('WARN', '请输入目标位姿')
            return
        x, y, z, orientation, err = parse_pose_input(parts)
        if err:
            cprint('ERROR', err)
            return
        if not self.controller.check_waypoint_safety(x, y, z):
            if not messagebox.askyesno(
                    '安全确认',
                    '目标可能超出工作空间或过低。\n\n是否仍要发送到控制链？'):
                cprint('INFO', '自定义目标已取消')
                return
        self._run_in_worker(
            lambda: self.controller.run_custom_waypoint(x, y, z, orientation),
            '自定义目标位姿')

    # ---- 按钮 [3] ----
    def _on_button_3(self):
        cprint('INFO', '[3] 请确认多路径点和循环次数后点击“执行多路径点”')
        self._multi_text.focus_set()

    def _on_button_3_exec(self):
        raw = self._multi_text.get('1.0', tk.END).strip()
        if not raw:
            cprint('WARN', '请输入路径点')
            return
        waypoints = []
        for line in raw.split('\n'):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            x, y, z, orientation, err = parse_pose_input(parts)
            if err:
                cprint('WARN', '%s — %s, 跳过' % (line, err))
                continue
            if not self.controller.check_waypoint_safety(x, y, z):
                cprint('WARN', '%s — 超出安全范围, 跳过' % line)
                continue
            waypoints.append((x, y, z, orientation))
        if not waypoints:
            cprint('WARN', '无有效路径点, 取消')
            return

        loops_str = self._loop_var.get().strip()
        try:
            loops = int(loops_str) if loops_str else 1
            loops = max(1, min(loops, 100))
        except ValueError:
            loops = 1

        self._run_in_worker(
            lambda: self.controller.run_multi_waypoint(waypoints, loops),
            '多路径点连续笛卡尔轨迹 (%d点 x %d轮)' % (len(waypoints), loops))

    # ---- 按钮 [4] ----
    def _on_button_4(self):
        self._run_in_worker(
            lambda: self.controller.run_safety_watchdog_status(),
            '安全审查状态')

    # ---- 按钮 [5] ----
    def _on_button_5(self):
        self._run_in_worker(
            lambda: self.controller.run_grinding_test(),
            '预设工件打磨测试 (3点)')

    # ---- 按钮 [6] ----
    def _on_button_6(self):
        self._run_in_worker(
            lambda: self.controller.run_planning_algorithms_overview(),
            '轨迹生成测试')

    # ---- 按钮 [7] ----
    def _on_button_7(self):
        self._run_in_worker(
            lambda: self.controller.run_show_readme(),
            '介绍 (README)')

    # ---- 退出 ----
    def _on_quit(self):
        if self.state.executing.is_set():
            if not messagebox.askyesno(
                    '确认退出',
                    '当前有任务正在执行中。\n\n确定要强制退出吗？'):
                return

        self._shutting_down = True
        cprint('INFO', '正在关闭系统...')
        saved_log_path = self._save_log_snapshot()
        if saved_log_path:
            try:
                messagebox.showinfo(
                    '日志已保存',
                    '自动保存日志，路径在:\n%s' % saved_log_path)
            except Exception:
                pass

        # 关闭控制器
        if self.controller is not None:
            self.controller.stop_pose_timer()
        if self._system_log_tailer is not None:
            self._system_log_tailer.stop()

        # 关闭 ROS (仅在非独立模式下)
        if not self.standalone and _ROS_AVAILABLE:
            try:
                rospy.signal_shutdown('GUI closed')
            except Exception:
                pass

        # 恢复 stdout
        if hasattr(self, '_log_redirector'):
            self._log_redirector.restore()

        self.root.destroy()


# ============================================================
# main
# ============================================================

def main():
    standalone = '--standalone' in sys.argv or '--mock' in sys.argv
    root = tk.Tk()
    app = SquareDemoGUI(root, standalone=standalone)
    root.mainloop()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('')
        print('[INFO] 用户中断')
    except Exception as e:
        # rospy.ROSInterruptException may not be importable in standalone mode
        if 'rospy' in str(type(e).__module__) and 'ROSInterrupt' in type(e).__name__:
            pass
        else:
            raise
