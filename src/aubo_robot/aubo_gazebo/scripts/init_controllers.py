#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
初始化 Gazebo 控制器：读取当前关节位置并设置为控制器目标
防止控制器启动时因目标为0导致的振荡
"""
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class ControllerInitializer:
    def __init__(self):
        rospy.init_node('controller_initializer', anonymous=True)

        # 关节名称列表
        self.joint_names = [
            'shoulder_joint',
            'upperArm_joint',
            'foreArm_joint',
            'wrist1_joint',
            'wrist2_joint',
            'wrist3_joint'
        ]

        # 创建发布器
        self.publishers = {}
        for joint in self.joint_names:
            topic = '/aubo_e5/{}_position_controller/command'.format(joint)
            self.publishers[joint] = rospy.Publisher(topic, Float64, queue_size=1)

        # 订阅关节状态
        self.joint_positions = {}
        self.received_state = False
        rospy.Subscriber('/aubo_e5/joint_states', JointState, self.joint_state_callback)

        rospy.loginfo("等待关节状态...")

    def joint_state_callback(self, msg):
        """接收关节状态并记录位置"""
        if not self.received_state:
            for i, name in enumerate(msg.name):
                if name in self.joint_names:
                    self.joint_positions[name] = msg.position[i]

            if len(self.joint_positions) == len(self.joint_names):
                self.received_state = True
                rospy.loginfo("收到初始关节状态")

    def initialize_controllers(self):
        """等待关节状态并初始化控制器"""
        rate = rospy.Rate(10)
        timeout = rospy.Time.now() + rospy.Duration(5.0)

        while not self.received_state and rospy.Time.now() < timeout:
            rate.sleep()

        if not self.received_state:
            rospy.logerr("超时：未收到关节状态")
            return False

        rospy.loginfo("初始化控制器目标位置...")
        rospy.sleep(0.5)  # 等待发布器建立连接

        for joint in self.joint_names:
            pos = self.joint_positions[joint]
            self.publishers[joint].publish(Float64(pos))
            rospy.loginfo("  {} -> {:.4f} rad".format(joint, pos))

        rospy.loginfo("控制器初始化完成")
        return True

if __name__ == '__main__':
    try:
        initializer = ControllerInitializer()
        if initializer.initialize_controllers():
            rospy.loginfo("控制器已稳定，节点退出")
        else:
            rospy.logerr("控制器初始化失败")
    except rospy.ROSInterruptException:
        pass
