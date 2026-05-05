#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
障碍物生成节点
在 Gazebo 中动态生成随机障碍物，并同步到 MoveIt Planning Scene
"""

import rospy
import random
import geometry_msgs.msg
from gazebo_msgs.srv import SpawnModel, DeleteModel
from moveit_commander import PlanningSceneInterface
from std_srvs.srv import Trigger, TriggerResponse


class ObstacleSpawner:
    """障碍物生成器"""

    def __init__(self):
        """初始化"""
        rospy.init_node('obstacle_spawner')

        # 等待 Gazebo 服务
        rospy.loginfo("Waiting for Gazebo services...")
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        rospy.wait_for_service('/gazebo/delete_model')

        # 创建服务代理
        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

        # 初始化 Planning Scene
        rospy.loginfo("Initializing Planning Scene...")
        self.scene = PlanningSceneInterface()
        rospy.sleep(2.0)  # 等待 Planning Scene 初始化

        # 障碍物计数和名称列表
        self.obstacle_count = 0
        self.obstacle_names = []

        # 工作空间参数（从参数服务器读取）
        self.workspace_x_min = rospy.get_param('~workspace_x_min', 0.2)
        self.workspace_x_max = rospy.get_param('~workspace_x_max', 0.6)
        self.workspace_y_min = rospy.get_param('~workspace_y_min', -0.4)
        self.workspace_y_max = rospy.get_param('~workspace_y_max', 0.4)
        self.workspace_z_min = rospy.get_param('~workspace_z_min', 0.1)
        self.workspace_z_max = rospy.get_param('~workspace_z_max', 0.5)

        # 创建 ROS 服务
        self.spawn_srv = rospy.Service('~spawn_obstacles', Trigger, self.handle_spawn_obstacles)
        self.clear_srv = rospy.Service('~clear_obstacles', Trigger, self.handle_clear_obstacles)

        rospy.loginfo("Obstacle Spawner initialized")
        rospy.loginfo("Workspace: x[%.2f, %.2f], y[%.2f, %.2f], z[%.2f, %.2f]",
                     self.workspace_x_min, self.workspace_x_max,
                     self.workspace_y_min, self.workspace_y_max,
                     self.workspace_z_min, self.workspace_z_max)

    def generate_box_sdf(self, size):
        """
        生成 box 的 SDF 描述

        Args:
            size: [width, depth, height] 列表

        Returns:
            SDF XML 字符串
        """
        sdf = f"""<?xml version='1.0'?>
<sdf version='1.5'>
  <model name='obstacle'>
    <static>true</static>
    <link name='link'>
      <collision name='collision'>
        <geometry>
          <box>
            <size>{size[0]} {size[1]} {size[2]}</size>
          </box>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <box>
            <size>{size[0]} {size[1]} {size[2]}</size>
          </box>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
        return sdf

    def spawn_random_obstacle(self):
        """
        生成一个随机障碍物

        Returns:
            bool: 是否成功
        """
        # 生成随机位置
        x = random.uniform(self.workspace_x_min, self.workspace_x_max)
        y = random.uniform(self.workspace_y_min, self.workspace_y_max)
        z = random.uniform(self.workspace_z_min, self.workspace_z_max)

        # 生成随机尺寸
        size = [
            random.uniform(0.05, 0.15),  # width
            random.uniform(0.05, 0.15),  # depth
            random.uniform(0.1, 0.3)     # height
        ]

        # 生成名称
        name = f"obstacle_{self.obstacle_count}"
        self.obstacle_count += 1

        # 创建位姿
        pose = geometry_msgs.msg.Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0

        try:
            # 1. 在 Gazebo 中生成
            sdf = self.generate_box_sdf(size)
            self.spawn_model(name, sdf, "", pose, "world")
            rospy.loginfo("Spawned %s in Gazebo at (%.2f, %.2f, %.2f), size: (%.2f, %.2f, %.2f)",
                         name, x, y, z, size[0], size[1], size[2])

            # 等待 Gazebo 生成完成
            rospy.sleep(0.5)

            # 2. 添加到 MoveIt Planning Scene
            self.add_box_to_planning_scene(name, pose, size)

            # 记录名称
            self.obstacle_names.append(name)

            return True

        except Exception as e:
            rospy.logerr("Failed to spawn obstacle: %s", str(e))
            return False

    def add_box_to_planning_scene(self, name, pose, size):
        """
        添加 box 到 Planning Scene

        Args:
            name: 障碍物名称
            pose: geometry_msgs.msg.Pose
            size: [width, depth, height] 列表
        """
        # 创建 PoseStamped
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.header.stamp = rospy.Time.now()
        box_pose.pose = pose

        # 添加到 Planning Scene
        self.scene.add_box(name, box_pose, size=tuple(size))

        rospy.loginfo("Added %s to Planning Scene", name)

    def clear_all_obstacles(self):
        """清除所有障碍物"""
        cleared_count = 0

        for name in self.obstacle_names:
            try:
                # 从 Gazebo 删除
                self.delete_model(name)
                rospy.loginfo("Deleted %s from Gazebo", name)
            except Exception as e:
                rospy.logwarn("Failed to delete %s from Gazebo: %s", name, str(e))

            try:
                # 从 Planning Scene 删除
                self.scene.remove_world_object(name)
                rospy.loginfo("Removed %s from Planning Scene", name)
                cleared_count += 1
            except Exception as e:
                rospy.logwarn("Failed to remove %s from Planning Scene: %s", name, str(e))

        # 清空列表
        self.obstacle_names = []

        rospy.loginfo("Cleared %d obstacles", cleared_count)
        return cleared_count

    def handle_spawn_obstacles(self, req):
        """
        处理生成障碍物服务请求

        Args:
            req: Trigger 请求

        Returns:
            TriggerResponse
        """
        # 默认生成 3 个障碍物
        count = 3
        success_count = 0

        rospy.loginfo("Spawning %d obstacles...", count)

        for i in range(count):
            if self.spawn_random_obstacle():
                success_count += 1

        rospy.loginfo("Successfully spawned %d/%d obstacles", success_count, count)

        return TriggerResponse(
            success=(success_count > 0),
            message=f"Spawned {success_count}/{count} obstacles"
        )

    def handle_clear_obstacles(self, req):
        """
        处理清除障碍物服务请求

        Args:
            req: Trigger 请求

        Returns:
            TriggerResponse
        """
        rospy.loginfo("Clearing all obstacles...")
        cleared_count = self.clear_all_obstacles()

        return TriggerResponse(
            success=True,
            message=f"Cleared {cleared_count} obstacles"
        )

    def run(self):
        """运行节点"""
        rospy.loginfo("Obstacle Spawner ready. Use services:")
        rospy.loginfo("  - rosservice call /obstacle_spawner/spawn_obstacles")
        rospy.loginfo("  - rosservice call /obstacle_spawner/clear_obstacles")
        rospy.spin()


def main():
    """主函数"""
    try:
        spawner = ObstacleSpawner()
        spawner.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupted by user")
    except Exception as e:
        rospy.logerr("Error: %s", str(e))
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
