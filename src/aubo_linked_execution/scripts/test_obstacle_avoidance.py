#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
避障测试脚本
自动化测试 MoveIt 避障规划功能
"""

import rospy
import random
import moveit_commander
import geometry_msgs.msg
from std_srvs.srv import Trigger


class ObstacleAvoidanceTest:
    """避障测试类"""

    def __init__(self):
        """初始化"""
        # 初始化 moveit_commander
        moveit_commander.roscpp_initialize([])

        # 初始化节点
        rospy.init_node('obstacle_avoidance_test', anonymous=True)

        # 创建 MoveGroupCommander
        self.group = moveit_commander.MoveGroupCommander("manipulator_e5")

        # 等待服务
        rospy.loginfo("Waiting for obstacle spawner services...")
        rospy.wait_for_service('/obstacle_spawner/spawn_obstacles')
        rospy.wait_for_service('/obstacle_spawner/clear_obstacles')

        # 创建服务代理
        self.spawn_srv = rospy.ServiceProxy('/obstacle_spawner/spawn_obstacles', Trigger)
        self.clear_srv = rospy.ServiceProxy('/obstacle_spawner/clear_obstacles', Trigger)

        rospy.loginfo("Obstacle Avoidance Test initialized")

    def run_single_test(self, target_pose):
        """
        运行单次避障测试

        Args:
            target_pose: 目标位姿

        Returns:
            (success, planning_time): 是否成功和规划时间
        """
        # 1. 清除旧障碍物
        self.clear_srv()
        rospy.sleep(1.0)

        # 2. 生成新障碍物
        resp = self.spawn_srv()
        rospy.loginfo("Spawned obstacles: %s", resp.message)
        rospy.sleep(2.0)  # 等待 Planning Scene 更新

        # 3. 规划路径
        self.group.set_pose_target(target_pose)

        start_time = rospy.Time.now()
        plan = self.group.plan()
        planning_time = (rospy.Time.now() - start_time).to_sec()

        # 检查规划结果（ROS Noetic 返回元组）
        if isinstance(plan, tuple):
            success = plan[0]
            trajectory = plan[1]
        else:
            success = bool(plan.joint_trajectory.points)
            trajectory = plan

        if success:
            rospy.loginfo("Planning succeeded in %.2fs", planning_time)

            # 4. 执行路径
            if isinstance(plan, tuple):
                self.group.execute(trajectory, wait=True)
            else:
                self.group.execute(plan, wait=True)

            return True, planning_time
        else:
            rospy.logwarn("Planning failed")
            return False, planning_time

    def run_batch_tests(self, num_tests=5):
        """
        批量测试

        Args:
            num_tests: 测试次数

        Returns:
            dict: 测试结果统计
        """
        results = []

        # 定义测试目标位置
        test_poses = [
            self.create_pose(0.4, 0.2, 0.3),
            self.create_pose(0.4, -0.2, 0.3),
            self.create_pose(0.5, 0.0, 0.4),
            self.create_pose(0.3, 0.3, 0.35),
            self.create_pose(0.3, -0.3, 0.35),
        ]

        rospy.loginfo("Starting batch test with %d iterations", num_tests)

        for i in range(num_tests):
            rospy.loginfo("=== Test %d/%d ===", i+1, num_tests)

            # 随机选择目标
            target = random.choice(test_poses)

            # 运行测试
            success, time = self.run_single_test(target)

            results.append({
                'test_id': i + 1,
                'success': success,
                'planning_time': time
            })

            rospy.sleep(2.0)

        # 统计结果
        success_count = sum(r['success'] for r in results)
        success_rate = success_count / len(results) if results else 0
        avg_time = sum(r['planning_time'] for r in results) / len(results) if results else 0

        stats = {
            'total': len(results),
            'success': success_count,
            'failed': len(results) - success_count,
            'success_rate': success_rate,
            'avg_planning_time': avg_time
        }

        rospy.loginfo("=== Test Results ===")
        rospy.loginfo("Total tests: %d", stats['total'])
        rospy.loginfo("Success: %d", stats['success'])
        rospy.loginfo("Failed: %d", stats['failed'])
        rospy.loginfo("Success rate: %.1f%%", stats['success_rate'] * 100)
        rospy.loginfo("Average planning time: %.2fs", stats['avg_planning_time'])

        return stats

    def create_pose(self, x, y, z):
        """
        创建位姿

        Args:
            x, y, z: 位置坐标

        Returns:
            geometry_msgs.msg.Pose
        """
        pose = geometry_msgs.msg.Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        return pose

    def shutdown(self):
        """关闭"""
        # 清除所有障碍物
        self.clear_srv()
        moveit_commander.roscpp_shutdown()


def main():
    """主函数"""
    try:
        # 创建测试对象
        test = ObstacleAvoidanceTest()

        # 等待系统稳定
        rospy.sleep(3.0)

        # 运行批量测试
        test.run_batch_tests(num_tests=5)

        # 关闭
        test.shutdown()
        rospy.loginfo("Test completed")

    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupted by user")
    except Exception as e:
        rospy.logerr("Error: %s", str(e))
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
