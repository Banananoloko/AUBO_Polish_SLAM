#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
unity_sync_monitor.py - Unity Shadow Mode Synchronization Monitor

Monitors and records all metrics for Unity shadow mode validation:
- Joint state frequencies (/real/joint_states, /unity/joint_states)
- Command age and feedback loop latency
- Joint synchronization errors (MAE, RMS, P95)
- TCP synchronization errors
- Unity frame rate
"""

import rospy
import numpy as np
from collections import deque
from datetime import datetime
import json
import os

from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped, PoseStamped
from std_msgs.msg import Float32


class UnitySyncMonitor:
    """Monitor Unity shadow mode synchronization metrics"""

    def __init__(self, buffer_size=50000):  # Increased for longer tests (~100s at 500Hz)
        rospy.init_node('unity_sync_monitor', anonymous=True)

        self.buffer_size = buffer_size
        self.sync_lock = threading.Lock()  # Thread safety for callbacks

        # Data buffers
        self.real_joint_data = deque(maxlen=buffer_size)
        self.unity_joint_data = deque(maxlen=buffer_size)
        self.wrench_data = deque(maxlen=buffer_size)
        self.command_age_data = deque(maxlen=buffer_size)
        self.feedback_loop_data = deque(maxlen=buffer_size)
        self.unity_fps_data = deque(maxlen=buffer_size)

        # Synchronization error buffers
        self.joint_errors = deque(maxlen=buffer_size)
        self.tcp_position_errors = deque(maxlen=buffer_size)
        self.tcp_orientation_errors = deque(maxlen=buffer_size)

        # Frequency tracking
        self.real_joint_times = deque(maxlen=100)
        self.unity_joint_times = deque(maxlen=100)

        # Latest states
        self.latest_real_joints = None
        self.latest_unity_joints = None

        # Monitoring flag
        self.is_monitoring = False
        self.start_time = None

        # Subscribers
        self.real_joint_sub = rospy.Subscriber(
            '/real/joint_states', JointState, self.real_joint_callback, queue_size=10
        )
        self.unity_joint_sub = rospy.Subscriber(
            '/unity/joint_states', JointState, self.unity_joint_callback, queue_size=10
        )
        self.wrench_sub = rospy.Subscriber(
            '/wrench', WrenchStamped, self.wrench_callback, queue_size=10
        )
        self.unity_fps_sub = rospy.Subscriber(
            '/unity/fps', Float32, self.unity_fps_callback, queue_size=10
        )

        rospy.loginfo("[UnitySyncMonitor] Monitor initialized")

    def start_monitoring(self):
        """Start data collection"""
        self.is_monitoring = True
        self.start_time = rospy.Time.now()

        # Clear buffers
        self.real_joint_data.clear()
        self.unity_joint_data.clear()
        self.wrench_data.clear()
        self.command_age_data.clear()
        self.feedback_loop_data.clear()
        self.unity_fps_data.clear()
        self.joint_errors.clear()
        self.tcp_position_errors.clear()
        self.tcp_orientation_errors.clear()

        rospy.loginfo("[UnitySyncMonitor] Started monitoring")

    def stop_monitoring(self):
        """Stop data collection"""
        self.is_monitoring = False
        rospy.loginfo("[UnitySyncMonitor] Stopped monitoring")

    def real_joint_callback(self, msg):
        """Callback for real robot joint states"""
        if not self.is_monitoring:
            return

        current_time = rospy.Time.now()
        self.real_joint_times.append(current_time.to_sec())

        data = {
            'timestamp': current_time.to_sec(),
            'positions': list(msg.position),
            'velocities': list(msg.velocity) if msg.velocity else [],
            'efforts': list(msg.effort) if msg.effort else []
        }
        self.real_joint_data.append(data)
        self.latest_real_joints = data

        # Calculate synchronization error if Unity data available
        with self.sync_lock:
            self.latest_real_joints = data
            if self.latest_unity_joints is not None:
                self._calculate_sync_error()

    def unity_joint_callback(self, msg):
        """Callback for Unity joint states"""
        if not self.is_monitoring:
            return

        current_time = rospy.Time.now()
        self.unity_joint_times.append(current_time.to_sec())

        # Calculate command age (time since message was sent)
        if msg.header.stamp.to_sec() > 0:
            command_age = (current_time - msg.header.stamp).to_sec() * 1000  # ms
            self.command_age_data.append(command_age)

        data = {
            'timestamp': current_time.to_sec(),
            'positions': list(msg.position),
            'velocities': list(msg.velocity) if msg.velocity else [],
            'header_stamp': msg.header.stamp.to_sec()
        }
        self.unity_joint_data.append(data)
        with self.sync_lock:
            self.latest_unity_joints = data

        # Calculate feedback loop time
        if len(self.real_joint_data) > 0:
            latest_real_time = self.real_joint_data[-1]['timestamp']
            feedback_loop = (current_time.to_sec() - latest_real_time) * 1000  # ms
            self.feedback_loop_data.append(feedback_loop)

    def wrench_callback(self, msg):
        """Callback for force/torque sensor"""
        if not self.is_monitoring:
            return

        data = {
            'timestamp': rospy.Time.now().to_sec(),
            'force': [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z],
            'torque': [msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
        }
        self.wrench_data.append(data)

    def unity_fps_callback(self, msg):
        """Callback for Unity frame rate"""
        if not self.is_monitoring:
            return

        self.unity_fps_data.append(msg.data)

    def _calculate_sync_error(self):
        """Calculate synchronization error between real and Unity"""
        with self.sync_lock:
            if self.latest_real_joints is None or self.latest_unity_joints is None:
                return

            real_pos = np.array(self.latest_real_joints['positions'])
            unity_pos = np.array(self.latest_unity_joints['positions'])

        # Ensure same length
        min_len = min(len(real_pos), len(unity_pos))
        real_pos = real_pos[:min_len]
        unity_pos = unity_pos[:min_len]

        # Calculate joint error
        joint_error = np.abs(real_pos - unity_pos)
        self.joint_errors.append(joint_error)

    def calculate_frequency(self, times):
        """Calculate frequency from timestamps"""
        if len(times) < 2:
            return 0.0

        times_array = np.array(list(times))
        diffs = np.diff(times_array)
        if len(diffs) == 0:
            return 0.0

        avg_period = np.mean(diffs)
        if avg_period == 0:
            return 0.0

        return 1.0 / avg_period

    def calculate_statistics(self):
        """Calculate all statistics from collected data"""
        stats = {}

        # Frequencies
        stats['real_joint_frequency'] = self.calculate_frequency(self.real_joint_times)
        stats['unity_joint_frequency'] = self.calculate_frequency(self.unity_joint_times)

        # Command age statistics
        if len(self.command_age_data) > 0:
            command_ages = np.array(list(self.command_age_data))
            stats['command_age_median'] = np.median(command_ages)
            stats['command_age_p95'] = np.percentile(command_ages, 95)
            stats['command_age_mean'] = np.mean(command_ages)
        else:
            stats['command_age_median'] = 0
            stats['command_age_p95'] = 0
            stats['command_age_mean'] = 0

        # Feedback loop statistics
        if len(self.feedback_loop_data) > 0:
            feedback_loops = np.array(list(self.feedback_loop_data))
            stats['feedback_loop_median'] = np.median(feedback_loops)
            stats['feedback_loop_p95'] = np.percentile(feedback_loops, 95)
            stats['feedback_loop_mean'] = np.mean(feedback_loops)
        else:
            stats['feedback_loop_median'] = 0
            stats['feedback_loop_p95'] = 0
            stats['feedback_loop_mean'] = 0

        # Unity FPS statistics
        if len(self.unity_fps_data) > 0:
            fps_array = np.array(list(self.unity_fps_data))
            stats['unity_fps_mean'] = np.mean(fps_array)
            stats['unity_fps_min'] = np.min(fps_array)
            stats['unity_fps_max'] = np.max(fps_array)
        else:
            stats['unity_fps_mean'] = 0
            stats['unity_fps_min'] = 0
            stats['unity_fps_max'] = 0

        # Joint synchronization errors
        if len(self.joint_errors) > 0:
            joint_errors_array = np.array(list(self.joint_errors))

            # MAE (Mean Absolute Error)
            stats['joint_mae'] = np.mean(joint_errors_array)
            stats['joint_mae_deg'] = np.degrees(stats['joint_mae'])

            # RMS (Root Mean Square)
            stats['joint_rms'] = np.sqrt(np.mean(joint_errors_array ** 2))
            stats['joint_rms_deg'] = np.degrees(stats['joint_rms'])

            # P95
            stats['joint_p95'] = np.percentile(joint_errors_array.flatten(), 95)
            stats['joint_p95_deg'] = np.degrees(stats['joint_p95'])

            # Synchronization accuracy (< 0.03 rad threshold)
            sync_threshold = 0.03  # rad
            max_errors = np.max(joint_errors_array, axis=1)
            sync_accurate = np.sum(max_errors < sync_threshold)
            stats['joint_sync_accuracy'] = (sync_accurate / len(max_errors)) * 100
        else:
            stats['joint_mae'] = 0
            stats['joint_mae_deg'] = 0
            stats['joint_rms'] = 0
            stats['joint_rms_deg'] = 0
            stats['joint_p95'] = 0
            stats['joint_p95_deg'] = 0
            stats['joint_sync_accuracy'] = 0

        # Visible sync delay estimate
        if stats['feedback_loop_median'] > 0:
            stats['visible_sync_delay_min'] = stats['feedback_loop_median']
            stats['visible_sync_delay_max'] = stats['feedback_loop_p95']
        else:
            stats['visible_sync_delay_min'] = 0
            stats['visible_sync_delay_max'] = 0

        # Data collection info
        stats['duration'] = (rospy.Time.now() - self.start_time).to_sec() if self.start_time else 0
        stats['real_joint_samples'] = len(self.real_joint_data)
        stats['unity_joint_samples'] = len(self.unity_joint_data)

        return stats

    def generate_report(self, output_dir=None):
        """Generate detailed test report"""
        if output_dir is None:
            output_dir = os.path.expanduser("~/aubo_polish/test_reports")

        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_file = os.path.join(output_dir, f"unity_sync_report_{timestamp}.json")

        stats = self.calculate_statistics()

        report = {
            'timestamp': timestamp,
            'duration_seconds': stats['duration'],
            'metrics': stats,
            'raw_data_samples': {
                'real_joint_states': stats['real_joint_samples'],
                'unity_joint_states': stats['unity_joint_samples'],
                'command_age': len(self.command_age_data),
                'feedback_loop': len(self.feedback_loop_data),
                'unity_fps': len(self.unity_fps_data),
                'joint_errors': len(self.joint_errors)
            }
        }

        with open(report_file, 'w') as f:
            json.dump(report, f, indent=2)

        rospy.loginfo(f"[UnitySyncMonitor] Report saved to: {report_file}")

        return report_file, stats

    def print_summary(self, stats=None):
        """Print summary to console"""
        if stats is None:
            stats = self.calculate_statistics()

        print("\n" + "="*70)
        print("Unity Shadow Mode Synchronization Test Results")
        print("="*70)
        print()

        print("Frequency Metrics:")
        print(f"  /real/joint_states frequency:    {stats['real_joint_frequency']:.1f} Hz")
        print(f"  /unity/joint_states frequency:   {stats['unity_joint_frequency']:.1f} Hz")
        print()

        print("Unity Performance:")
        print(f"  Unity Game View frame rate:      {stats['unity_fps_min']:.0f}–{stats['unity_fps_max']:.0f} fps")
        print(f"  Average FPS:                     {stats['unity_fps_mean']:.1f} fps")
        print()

        print("Latency Metrics:")
        print(f"  Command age median / P95:        {stats['command_age_median']:.0f} ms / {stats['command_age_p95']:.0f} ms")
        print(f"  Feedback loop median / P95:      {stats['feedback_loop_median']:.0f} ms / {stats['feedback_loop_p95']:.0f} ms")
        print(f"  Visible sync delay:              ~{stats['visible_sync_delay_min']:.0f}–{stats['visible_sync_delay_max']:.0f} ms")
        print()

        print("Joint Synchronization Errors:")
        print(f"  Joint MAE:                       {stats['joint_mae']:.4f} rad, ~{stats['joint_mae_deg']:.2f}°")
        print(f"  Joint RMS error:                 {stats['joint_rms']:.4f} rad, ~{stats['joint_rms_deg']:.2f}°")
        print(f"  Joint P95 error:                 {stats['joint_p95']:.4f} rad, ~{stats['joint_p95_deg']:.2f}°")
        print(f"  Joint sync accuracy:             {stats['joint_sync_accuracy']:.1f}% (threshold < 0.03 rad)")
        print()

        print("Data Collection:")
        print(f"  Test duration:                   {stats['duration']:.1f} seconds")
        print(f"  Real joint samples:              {stats['real_joint_samples']}")
        print(f"  Unity joint samples:             {stats['unity_joint_samples']}")
        print()

        print("="*70)
        print()


if __name__ == '__main__':
    try:
        monitor = UnitySyncMonitor()

        print("\nUnity Sync Monitor Ready")
        print("Commands:")
        print("  start  - Start monitoring")
        print("  stop   - Stop monitoring and show results")
        print("  quit   - Exit")
        print()

        while not rospy.is_shutdown():
            cmd = input("> ").strip().lower()

            if cmd == 'start':
                monitor.start_monitoring()
                print("✓ Monitoring started")
            elif cmd == 'stop':
                monitor.stop_monitoring()
                report_file, stats = monitor.generate_report()
                monitor.print_summary(stats)
                print(f"✓ Report saved to: {report_file}")
            elif cmd == 'quit' or cmd == 'q':
                break
            else:
                print("Unknown command")

    except rospy.ROSInterruptException:
        pass
