cmake_minimum_required(VERSION 3.0.2)
project(aubo_linked_execution)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/joint_state_mirror_adapter.py
  scripts/linked_execution_monitor.py
  scripts/linked_execution_action_server.py
  scripts/aubo_robot_startup.py
  scripts/continuous_motion_demo.py
  scripts/obstacle_spawner.py
  scripts/test_obstacle_avoidance.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
