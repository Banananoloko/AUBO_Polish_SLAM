#!/usr/bin/env python3
"""
gazebo_rtf_monitor.py
监控 Gazebo 实时因子 (Real-Time Factor)，检测仿真性能异常
"""
import rospy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


class GazeboRTFMonitor:
    def __init__(self):
        rospy.init_node('gazebo_rtf_monitor')

        self.last_sim_time = None
        self.last_real_time = None
        self.rtf_threshold_low = rospy.get_param('~rtf_threshold_low', 0.8)
        self.rtf_threshold_high = rospy.get_param('~rtf_threshold_high', 1.2)
        self.check_interval = rospy.get_param('~check_interval', 2.0)  # 检查间隔（秒）

        # Publisher
        self.warning_pub = rospy.Publisher('/gazebo_rtf_monitor/warning', String, queue_size=1)

        # Subscriber
        rospy.Subscriber('/clock', Clock, self.clock_callback, queue_size=1)

        # 定时检查
        rospy.Timer(rospy.Duration(self.check_interval), self.check_rtf)

        rospy.loginfo('[gazebo_rtf_monitor] Gazebo RTF monitor started')
        rospy.loginfo('[gazebo_rtf_monitor] RTF threshold: %.1f - %.1f',
                      self.rtf_threshold_low, self.rtf_threshold_high)
        rospy.loginfo('[gazebo_rtf_monitor] Check interval: %.1f seconds', self.check_interval)

    def clock_callback(self, msg):
        """接收 Gazebo 仿真时间"""
        current_sim_time = msg.clock.to_sec()
        current_real_time = rospy.Time.now().to_sec()

        # 保存时间戳
        if self.last_sim_time is not None and self.last_real_time is not None:
            sim_dt = current_sim_time - self.last_sim_time
            real_dt = current_real_time - self.last_real_time

            # 计算 RTF
            if real_dt > 0.1:  # 避免除零和噪声
                rtf = sim_dt / real_dt

                # 检查是否异常
                if rtf < self.rtf_threshold_low or rtf > self.rtf_threshold_high:
                    warning_msg = 'Gazebo RTF abnormal: %.2f (expected %.1f-%.1f)' % (
                        rtf, self.rtf_threshold_low, self.rtf_threshold_high)
                    rospy.logwarn('[gazebo_rtf_monitor] %s', warning_msg)
                    self.warning_pub.publish(String(data=warning_msg))

        self.last_sim_time = current_sim_time
        self.last_real_time = current_real_time

    def check_rtf(self, event):
        """定时检查 RTF（用于检测 /clock 停止发布）"""
        if self.last_real_time is not None:
            elapsed = rospy.Time.now().to_sec() - self.last_real_time
            if elapsed > self.check_interval * 2:
                warning_msg = 'Gazebo /clock topic not updating (%.1fs since last message)' % elapsed
                rospy.logwarn('[gazebo_rtf_monitor] %s', warning_msg)
                self.warning_pub.publish(String(data=warning_msg))

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        monitor = GazeboRTFMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
