#!/usr/bin/env python3
"""
Fake real robot joint state publisher for testing Shadow mode.
Publishes sinusoidal joint trajectories to /real/joint_states.
"""
import rospy
from sensor_msgs.msg import JointState
import math

def main():
    rospy.init_node('fake_real_robot')
    pub = rospy.Publisher('/real/joint_states', JointState, queue_size=10)

    joint_names = [
        'shoulder_joint', 'upperArm_joint', 'foreArm_joint',
        'wrist1_joint', 'wrist2_joint', 'wrist3_joint'
    ]

    rate = rospy.Rate(50)  # 50Hz (实机是500Hz，这里简化)
    start_time = rospy.Time.now()

    rospy.loginfo("[fake_real_robot] Publishing to /real/joint_states at 50Hz")
    rospy.loginfo("Simulating sinusoidal motion on all joints")

    while not rospy.is_shutdown():
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = joint_names

        # 生成正弦波运动（周期10秒，幅度0.5弧度）
        t = (rospy.Time.now() - start_time).to_sec()
        amplitude = 0.5
        period = 10.0

        msg.position = [
            amplitude * math.sin(2 * math.pi * t / period),           # shoulder
            amplitude * math.sin(2 * math.pi * t / period + 1.0),     # upperArm
            amplitude * math.sin(2 * math.pi * t / period + 2.0),     # foreArm
            amplitude * math.sin(2 * math.pi * t / period + 3.0),     # wrist1
            amplitude * math.sin(2 * math.pi * t / period + 4.0),     # wrist2
            amplitude * math.sin(2 * math.pi * t / period + 5.0)      # wrist3
        ]
        msg.velocity = [0.0] * 6
        msg.effort = [0.0] * 6

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
