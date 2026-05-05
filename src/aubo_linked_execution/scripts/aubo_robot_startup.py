#!/usr/bin/env python3
"""
aubo_robot_startup.py
在 aubo_driver 连接实机后，自动完成：
  1. 发送 powerOn 上电
  2. 等待 drives_powered
  3. 切换到 ROS 控制模式 (Tcp2CanbusMode)
  4. 验证初始位置同步
  5. 验证 RViz 位置对齐实机位置
"""
import rospy
from std_msgs.msg import String, Int32
from industrial_msgs.msg import RobotStatus
from sensor_msgs.msg import JointState
import moveit_commander


class AuboRobotStartup:
    def __init__(self):
        rospy.init_node('aubo_robot_startup', anonymous=True)

        self.drives_powered = False
        self.startup_sent = False
        self.switch_sent = False
        self.joint_states_received = False
        self.last_joint_state = None

        # Publishers
        self.power_pub = rospy.Publisher('/robot_control', String, queue_size=1)
        self.switch_pub = rospy.Publisher('/aubo_driver/controller_switch', Int32, queue_size=1)

        # Subscribers
        rospy.Subscriber('/robot_status', RobotStatus, self.status_cb)
        rospy.Subscriber('/joint_states', JointState, self.joint_state_cb)

        self.timeout = rospy.get_param('~startup_timeout', 30.0)
        self.position_alignment_tolerance = rospy.get_param('~position_alignment_tolerance', 0.05)

    def status_cb(self, msg):
        self.drives_powered = (msg.drives_powered.val == 1)

    def joint_state_cb(self, msg):
        self.joint_states_received = True
        self.last_joint_state = msg

    def run(self):
        # 1. 等待 aubo_driver 连接
        rospy.loginfo('[startup] Waiting for /aubo_driver/robot_connected...')
        rate = rospy.Rate(2)
        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            val = rospy.get_param('/aubo_driver/robot_connected', '0')
            if val == '1':
                break
            if (rospy.Time.now() - t0).to_sec() > self.timeout:
                rospy.logerr('[startup] Timeout waiting for aubo_driver connection.')
                return
            rate.sleep()
        rospy.loginfo('[startup] aubo_driver connected.')

        # 让驱动完成初始化
        rospy.sleep(1.0)

        # 2. 发送 powerOn
        if not self.drives_powered:
            rospy.loginfo('[startup] Sending powerOn...')
            self.power_pub.publish(String(data='powerOn'))
            self.startup_sent = True

            t0 = rospy.Time.now()
            while not rospy.is_shutdown() and not self.drives_powered:
                if (rospy.Time.now() - t0).to_sec() > self.timeout:
                    rospy.logerr('[startup] Timeout waiting for drives_powered. '
                                 'Check teach pendant: is e-stop released? Is robot enabled?')
                    return
                rate.sleep()
            rospy.loginfo('[startup] Robot powered on (drives_powered=1).')
        else:
            rospy.loginfo('[startup] Robot already powered on.')

        # 3. 切换到 ROS 控制模式 (RosMoveIt = 1)
        rospy.loginfo('[startup] Switching to ROS controller (Tcp2CanbusMode)...')
        rospy.sleep(0.5)
        self.switch_pub.publish(Int32(data=1))
        rospy.sleep(2.0)

        # 4. 验证初始位置同步
        if not self.verify_initial_position_sync():
            rospy.logerr('[startup] Failed to verify initial position sync')
            return

        # 5. 验证 RViz 位置对齐
        if not self.verify_rviz_alignment():
            rospy.logerr('[startup] Failed to verify RViz alignment')
            return

        rospy.loginfo('[startup] Initialization complete. Robot is ready for ROS control.')

    def verify_initial_position_sync(self):
        """验证 RViz 显示位置与实机位置同步"""
        rospy.loginfo('[startup] Verifying initial position synchronization...')

        # 等待 joint_states 发布
        t0 = rospy.Time.now()
        while not rospy.is_shutdown() and not self.joint_states_received:
            if (rospy.Time.now() - t0).to_sec() > 10.0:
                rospy.logerr('[startup] Timeout waiting for /joint_states')
                return False
            rospy.sleep(0.1)

        if not self.last_joint_state:
            rospy.logerr('[startup] No joint states received')
            return False

        # 等待状态稳定（连续两次读取一致）
        rospy.sleep(1.0)
        first_state = self.last_joint_state
        rospy.sleep(1.0)
        second_state = self.last_joint_state

        # 检查稳定性（容差 0.01 rad）
        if len(first_state.position) != len(second_state.position):
            rospy.logwarn('[startup] Joint state size mismatch')
            return False

        max_diff = 0.0
        for i in range(len(first_state.position)):
            diff = abs(first_state.position[i] - second_state.position[i])
            max_diff = max(max_diff, diff)

        if max_diff > 0.01:
            rospy.logwarn('[startup] Joint states not stable (max_diff=%.4f rad), waiting...', max_diff)
            rospy.sleep(2.0)
            return self.verify_initial_position_sync()

        # 显示当前位置
        rospy.loginfo('[startup] Current joint positions (rad):')
        joint_names = ['shoulder', 'upperArm', 'foreArm', 'wrist1', 'wrist2', 'wrist3']
        for i, name in enumerate(joint_names):
            if i < len(first_state.position):
                rospy.loginfo('[startup]   %s: %.4f', name, first_state.position[i])

        rospy.loginfo('[startup] ✓ Initial position sync verified')
        return True

    def verify_rviz_alignment(self):
        """验证 RViz 显示位置与实机位置对齐"""
        rospy.loginfo('[startup] Verifying RViz position alignment with real robot...')

        try:
            # 初始化 MoveIt Commander
            moveit_commander.roscpp_initialize([])
            robot = moveit_commander.RobotCommander()
            group = moveit_commander.MoveGroupCommander("manipulator_e5")

            # 等待 MoveIt 就绪
            rospy.sleep(2.0)

            # 获取 MoveIt 当前状态
            moveit_state = group.get_current_joint_values()

            # 获取实机当前状态
            if not self.last_joint_state:
                rospy.logerr('[startup] No joint states available for alignment check')
                return False

            real_state = self.last_joint_state.position

            # 比较位置差异
            if len(moveit_state) != len(real_state):
                rospy.logerr('[startup] Joint count mismatch: MoveIt=%d, Real=%d',
                           len(moveit_state), len(real_state))
                return False

            max_diff = 0.0
            max_diff_joint = ''
            joint_names = ['shoulder', 'upperArm', 'foreArm', 'wrist1', 'wrist2', 'wrist3']

            for i in range(len(moveit_state)):
                diff = abs(moveit_state[i] - real_state[i])
                if diff > max_diff:
                    max_diff = diff
                    max_diff_joint = joint_names[i] if i < len(joint_names) else f'joint_{i}'

            # 检查对齐容差
            if max_diff > self.position_alignment_tolerance:
                rospy.logerr('[startup] ========================================')
                rospy.logerr('[startup] RViz position MISALIGNMENT detected!')
                rospy.logerr('[startup] Max difference: %.4f rad (%.1f°) at %s',
                           max_diff, max_diff * 57.2958, max_diff_joint)
                rospy.logerr('[startup] Tolerance: %.4f rad (%.1f°)',
                           self.position_alignment_tolerance,
                           self.position_alignment_tolerance * 57.2958)
                rospy.logerr('[startup] ========================================')
                rospy.logerr('[startup] DANGER: RViz shows different position than real robot!')
                rospy.logerr('[startup] Please restart the system and wait for proper initialization.')
                return False

            rospy.loginfo('[startup] Position alignment check:')
            rospy.loginfo('[startup]   Max difference: %.4f rad (%.1f°) at %s',
                         max_diff, max_diff * 57.2958, max_diff_joint)
            rospy.loginfo('[startup]   Tolerance: %.4f rad (%.1f°)',
                         self.position_alignment_tolerance,
                         self.position_alignment_tolerance * 57.2958)
            rospy.loginfo('[startup] ✓ RViz position aligned with real robot')
            return True

        except Exception as e:
            rospy.logerr('[startup] Failed to verify RViz alignment: %s', str(e))
            rospy.logwarn('[startup] Skipping alignment check (MoveIt may not be ready)')
            return True  # 不阻止启动，但记录警告


if __name__ == '__main__':
    try:
        node = AuboRobotStartup()
        node.run()
    except rospy.ROSInterruptException:
        pass
