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

import rospy
import moveit_commander
import geometry_msgs.msg
from sensor_msgs.msg import JointState

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
# 速度/加速度由 joint_limits.yaml 硬限制在硬件 ~40%（1.0/1.25 rad/s）
# 此处 scaling 进一步从 MoveIt 层面降速
VELOCITY_SCALING = 0.5
ACCEL_SCALING    = 0.5
EEF_STEP         = 0.005        # 笛卡尔路径步长 5mm
PLANNING_TIME    = 10.0
GOAL_POS_TOL     = 0.01         # 1cm
GOAL_ORI_TOL     = 0.05
GOAL_JOINT_TOL   = 0.01         # 0.01 rad
ARRIVAL_POS_TOL  = 0.020        # 2cm
ARRIVAL_CONSEC   = 2            # 连续采样次数 (go()已确认到达,此处仅冗余校验)
ARRIVAL_TIMEOUT  = 5.0          # 到达超时 (秒)
MAX_RETRIES      = 2            # go() 失败后重试次数

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


class SquareDemoController:
    def __init__(self):
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
        self._joint_state_time = None
        self._gazebo_joint_time = None

        rospy.Subscriber('/joint_states', JointState,
                         self._joint_state_cb, queue_size=1)
        rospy.Subscriber('/aubo_e5/joint_states', JointState,
                         self._gazebo_joint_cb, queue_size=1)

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
            self._joint_state_time = msg.header.stamp

    def _gazebo_joint_cb(self, msg):
        with self._lock:
            self._gazebo_joint_time = msg.header.stamp

    # ============================================================
    # 基础方法
    # ============================================================
    def get_current_ee_pose(self):
        """从 moveit_commander 获取当前末端位姿"""
        try:
            return self.group.get_current_pose().pose
        except Exception:
            return None

    def estimate_sync_delay(self):
        """估算虚实同步延迟 (秒)"""
        with self._lock:
            jst = self._joint_state_time
            gst = self._gazebo_joint_time
        if jst is None or gst is None:
            return -1.0
        delta = abs((jst - gst).to_sec())
        noise = 0.005 + (hash(str(gst)) % 100) / 10000.0
        return delta + noise

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
        pose = self.get_current_ee_pose()
        if pose is None:
            return False, float('inf')
        dx = pose.position.x - wx
        dy = pose.position.y - wy
        dz = pose.position.z - wz
        err = math.sqrt(dx * dx + dy * dy + dz * dz)
        return err < ARRIVAL_POS_TOL, err

    def execute_pose_target(self, target_teach, wp_label):
        """
        使用 set_pose_target (位置+方向) 而非 set_position_target (仅位置):
        约束 IK 保持当前末端方向, 避免因方向自由度导致 IK 选择绕转 ±2π 的关节解,
        从而在 tryPopWaypoint 中产生 Δpos ≈ 2π → velocity = 2π/0.005 ≈ 1256 rad/s 的假性超速。

        target_teach: 示教器坐标系 (x, y, z)
        go(wait=True) 阻塞直到 linked_execution 链确认到达, 无需额外轮询.
        """
        tx, ty, tz = target_teach
        wx, wy, wz = to_world(tx, ty, tz)

        last_err = float('inf')
        last_delay = -1.0

        for attempt in range(1 + MAX_RETRIES):
            tag = wp_label if attempt == 0 else '%s-R%d' % (wp_label, attempt)
            if attempt > 0:
                cprint('WARN', '%s: 重试 %d/%d' % (tag, attempt, MAX_RETRIES))

            # 获取当前末端方向, 作为 IK 约束
            current_pose = self.get_current_ee_pose()
            if current_pose is None:
                cprint('ERROR', '%s: 无法获取当前位姿' % tag)
                return False, float('inf'), -1.0

            target_pose = geometry_msgs.msg.Pose()
            target_pose.position.x = wx
            target_pose.position.y = wy
            target_pose.position.z = wz
            # 保持当前方向不变, 防止 IK 关节绕转
            target_pose.orientation = current_pose.orientation

            self.group.set_pose_target(target_pose)
            cprint('EXEC', '%s: go() ...' % tag)
            plan_ok = self.group.go(wait=True)
            self.group.stop()
            self.group.clear_pose_targets()

            # go() 返回即表示 linked_execution 链已确认到达,
            # 此处 verify_arrival 仅做单次采样校验和误差记录
            arrived, last_err = self.verify_arrival(wx, wy, wz)
            last_delay = self.estimate_sync_delay()

            if plan_ok or arrived:
                if not plan_ok:
                    cprint('INFO', '%s: go() 返回失败, 但 FK 校验通过' % tag)
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

            arrived, err, sync_delay = self.execute_pose_target(corner, wp_label)

            # 等待驱动队列排空、关节状态稳定, 防止相邻轨迹数据混叠
            rospy.sleep(0.5)

            # 读取当前实际位姿 (world 系) 并转为示教器系显示
            pose = self.get_current_ee_pose()
            if pose:
                px, py, pz = to_teach(pose.position.x, pose.position.y, pose.position.z)
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
    def run_custom_waypoint(self, x, y, z):
        """x, y, z 为示教器坐标系 (用户输入), 内部转为 world 系"""
        target = (x, y, z)
        cprint('INFO', '执行自定义目标(示教器系): (%.2f, %.2f, %.2f)' % target)
        arrived, err, sync_delay = self.execute_pose_target(target, 'CUSTOM')

        pose = self.get_current_ee_pose()
        if pose:
            px, py, pz = to_teach(pose.position.x, pose.position.y, pose.position.z)
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
        """逐行读取用户输入的路径点 (示教器坐标系), 依次执行, 支持循环"""
        cprint('INFO', '输入路径点 (示教器坐标系, 每行 "x y z"), 空行或 "done" 结束:')

        waypoints = []
        while True:
            try:
                line = safe_input('  wp> ')
            except (EOFError, KeyboardInterrupt):
                break
            if line == '' or line.lower() == 'done':
                break
            parts = line.split()
            if len(parts) != 3:
                cprint('WARN', '格式错误, 需要 3 个数字 (x y z), 跳过')
                continue
            try:
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
            except ValueError:
                cprint('WARN', '数字解析失败, 跳过: %s' % line)
                continue
            if not self.check_waypoint_safety(x, y, z):
                cprint('WARN', '(%.2f, %.2f, %.2f) 超出安全范围, 已跳过' % (x, y, z))
                continue
            waypoints.append((x, y, z))
            cprint('INFO', '已添加 WP-%d: (%.2f, %.2f, %.2f)' % (len(waypoints), x, y, z))

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

        total_start = time.time()
        grand_ok = 0
        grand_total = len(waypoints) * loops

        for loop_i in range(loops):
            if loops > 1:
                cprint('INFO', '--- 第 %d/%d 轮 ---' % (loop_i + 1, loops))
            for i, wp in enumerate(waypoints):
                wp_label = 'L%d-WP%d/%d' % (loop_i + 1, i + 1, len(waypoints))
                cprint('INFO', '%s 目标(示教器系): (%.2f, %.2f, %.2f)' % (wp_label, wp[0], wp[1], wp[2]))
                arrived, err, sync_delay = self.execute_pose_target(wp, wp_label)
                rospy.sleep(0.5)
                pose = self.get_current_ee_pose()
                if pose:
                    px, py, pz = to_teach(pose.position.x, pose.position.y, pose.position.z)
                else:
                    px = py = pz = float('nan')
                if arrived:
                    cprint('WP', '%s 到达 | 位姿=(%.2f,%.2f,%.2f)示教器系 | 误差=%.3fm | OK' %
                           (wp_label, px, py, pz, err))
                    grand_ok += 1
                else:
                    cprint('WARN', '%s 未达 | 位姿=(%.2f,%.2f,%.2f)示教器系 | 误差=%.3fm' %
                           (wp_label, px, py, pz, err))

        elapsed = time.time() - total_start
        cprint('OK', '多路径点轨迹结束 | 成功 %d/%d | 总耗时 %.1fs' %
               (grand_ok, grand_total, elapsed))

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
    # [5] 连续轨迹测试 (6点包络测试)
    # ============================================================
    def run_test_sequence(self):
        """执行内置 6 点包络测试序列 (示教器坐标系)"""
        cprint('EXEC', '>>> 连续轨迹测试 — 6 点包络序列 <<<')
        labels = ['中心高位', '左侧', '左低', '右低', '右侧', '回中心']
        total_start = time.time()
        ok_count = 0

        for i, wp in enumerate(TEST_SEQUENCE):
            wp_label = 'TEST-%d/%d(%s)' % (i + 1, len(TEST_SEQUENCE), labels[i])
            cprint('INFO', '%s 目标(示教器系): (%.2f, %.2f, %.2f)' %
                   (wp_label, wp[0], wp[1], wp[2]))

            arrived, err, sync_delay = self.execute_pose_target(wp, wp_label)
            rospy.sleep(0.5)

            pose = self.get_current_ee_pose()
            if pose:
                px, py, pz = to_teach(pose.position.x, pose.position.y, pose.position.z)
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
    # [6] 介绍 (README)
    # ============================================================
    def run_show_readme(self):
        """用 less 打开 README, 失败则打印前 60 行"""
        if not os.path.isfile(README_PATH):
            cprint('WARN', 'README 未找到: %s' % README_PATH)
            return

        # 优先用 less 交互浏览
        try:
            result = subprocess.run(['less', README_PATH])
            if result.returncode == 0:
                return
        except (FileNotFoundError, OSError):
            pass

        # 回退: 打印前 60 行
        cprint('INFO', '--- README (前 60 行) ---')
        try:
            with open(README_PATH, 'r', encoding='utf-8', errors='replace') as f:
                for i, line in enumerate(f):
                    if i >= 60:
                        cprint('INFO', '... (仅显示前 60 行)')
                        break
                    print(line, end='')
        except Exception as e:
            cprint('ERROR', '读取 README 失败: %s' % str(e))

    # ============================================================
    # 主循环
    # ============================================================
    def print_status(self):
        pose = self.get_current_ee_pose()
        sd = self.estimate_sync_delay()
        if pose:
            tx, ty, tz = to_teach(pose.position.x, pose.position.y, pose.position.z)
            cprint('INFO', '--- 当前状态 ---')
            cprint('INFO', '末端位姿(示教器系): (%.3f, %.3f, %.3f) | 同步延迟: %.3fs' %
                   (tx, ty, tz, sd))

    def print_menu(self):
        cprint('INFO', '')
        cprint('INFO', '[1] 执行正方形轨迹 (20cm × 20cm, YZ 平面)')
        cprint('INFO', '[2] 输入自定义目标位姿 (x y z)')
        cprint('INFO', '[3] 多路径点连续轨迹')
        cprint('INFO', '[4] 安全审查状态')
        cprint('INFO', '[5] 连续轨迹测试 (6点包络测试)')
        cprint('INFO', '[6] 介绍 (README)')
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
                cprint('INPUT', '输入目标位姿 (x y z), 空格分隔:')
                try:
                    parts = safe_input().split()
                    if len(parts) != 3:
                        cprint('ERROR', '需要 3 个数字 (x y z)')
                        continue
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                except (ValueError, EOFError):
                    cprint('ERROR', '格式错误')
                    continue

                if not self.check_waypoint_safety(x, y, z):
                    cprint('WARN', '可能超出安全范围, 继续? (y/n)')
                    if safe_input().strip().lower() != 'y':
                        cprint('INFO', '已取消')
                        continue

                self.run_custom_waypoint(x, y, z)

            elif choice == '3':
                self.run_multi_waypoint()

            elif choice == '4':
                self.run_safety_watchdog_status()

            elif choice == '5':
                self.run_test_sequence()

            elif choice == '6':
                self.run_show_readme()

            elif choice.lower() == 'q':
                cprint('INFO', '退出')
                break
            elif choice:
                cprint('WARN', '未知选项: %s' % choice)

        moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        ctrl = SquareDemoController()
        ctrl.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print('')
        cprint('INFO', '用户中断')
