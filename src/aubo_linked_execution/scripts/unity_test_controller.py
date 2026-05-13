#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
unity_test_controller.py - Unity Linked Execution Test Controller

Complete test framework with:
- Interactive pose control (goto_pose_enhanced_v2.py)
- Real-time synchronization monitoring
- Automatic metrics collection
- Detailed test report generation
"""

import rospy
import subprocess
import threading
import time
import os
import sys
from datetime import datetime

# Add path for monitor
sys.path.insert(0, os.path.dirname(__file__))
from unity_sync_monitor import UnitySyncMonitor


class UnityTestController:
    """Main test controller"""

    def __init__(self):
        self.monitor = None
        self.test_process = None
        self.is_running = False

    def start_monitor(self):
        """Start synchronization monitor"""
        print("\n" + "="*70)
        print("Starting Unity Synchronization Monitor...")
        print("="*70)

        self.monitor = UnitySyncMonitor()
        self.monitor.start_monitoring()

        print("✓ Monitor started - collecting data in background")
        print()

    def start_test_script(self):
        """Start interactive test script"""
        print("\n" + "="*70)
        print("Starting Interactive Test Script...")
        print("="*70)
        print()
        print("Test Script: goto_pose_enhanced_v2.py")
        print("Features:")
        print("  • Colorful UI with real-time status")
        print("  • Joint states and Cartesian pose display")
        print("  • Preset positions (home, test1, test2)")
        print("  • Relative motion support (+x 0.1, +y -0.05)")
        print("  • Execution statistics")
        print()
        print("Commands:")
        print("  x y z                  - Move to position")
        print("  x y z roll pitch yaw   - Move with orientation")
        print("  +x 0.1                 - Relative motion")
        print("  home / test1 / test2   - Preset positions")
        print("  stats                  - Show statistics")
        print("  q / quit               - Exit test")
        print()
        print("="*70)
        print()

        # Wait for user to be ready
        input("Press Enter to start the test script...")
        print()

        # Start test script in subprocess
        try:
            self.test_process = subprocess.Popen(
                ['rosrun', 'aubo_planner', 'goto_pose_enhanced_v2.py'],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,  # Merge stderr into stdout
                universal_newlines=True
            )

            self.is_running = True

            # Monitor test process
            self._monitor_test_process()

        except Exception as e:
            print(f"✗ Failed to start test script: {e}")
            return False

        return True

    def _monitor_test_process(self):
        """Monitor test process output"""
        def read_output():
            while self.is_running:
                if self.test_process.poll() is not None:
                    break

                line = self.test_process.stdout.readline()
                if line:
                    print(line, end='')

        output_thread = threading.Thread(target=read_output, daemon=True)
        output_thread.start()

    def stop_test(self):
        """Stop test and generate report"""
        print("\n" + "="*70)
        print("Stopping Test and Generating Report...")
        print("="*70)
        print()

        # Stop test process
        if self.test_process and self.test_process.poll() is None:
            self.test_process.terminate()
            self.test_process.wait(timeout=5)

        self.is_running = False

        # Stop monitor and generate report
        if self.monitor:
            self.monitor.stop_monitoring()
            report_file, stats = self.monitor.generate_report()

            # Print summary
            self.monitor.print_summary(stats)

            # Print report location
            print(f"✓ Detailed report saved to: {report_file}")
            print()

            return report_file, stats

        return None, None

    def run_complete_test(self):
        """Run complete test flow"""
        print("\n" + "╔" + "="*68 + "╗")
        print("║" + " "*68 + "║")
        print("║" + "  Unity Linked Execution Complete Test Suite".center(68) + "║")
        print("║" + " "*68 + "║")
        print("╚" + "="*68 + "╝")
        print()

        # Step 1: Start monitor
        self.start_monitor()

        # Step 2: Start test script
        if not self.start_test_script():
            return

        # Step 3: Wait for test to complete
        print("\n" + "="*70)
        print("Test Running...")
        print("="*70)
        print()
        print("Instructions:")
        print("  • Perform your test movements in the test script")
        print("  • Type 'q' in the test script when done")
        print("  • Or press Ctrl+C here to stop")
        print()

        try:
            # Wait for test process to finish
            if self.test_process:
                self.test_process.wait()

        except KeyboardInterrupt:
            print("\n\n✓ Test interrupted by user")

        # Step 4: Generate report
        self.stop_test()


def main():
    """Main entry point"""
    try:
        # Initialize ROS node
        rospy.init_node('unity_test_controller', anonymous=True)

        # Create controller
        controller = UnityTestController()

        # Run complete test
        controller.run_complete_test()

        print("\n" + "="*70)
        print("Test Complete!")
        print("="*70)
        print()
        print("Thank you for using Unity Linked Execution Test Suite")
        print()

    except rospy.ROSInterruptException:
        print("\n✗ ROS interrupted")
    except KeyboardInterrupt:
        print("\n✓ Test stopped by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
