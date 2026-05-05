cmake_minimum_required(VERSION 3.0.2)
project(aubo_unity_bridge)

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  rospy
  sensor_msgs
  std_msgs
  trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS
    control_msgs
    rospy
    sensor_msgs
    std_msgs
    trajectory_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/fake_aubo_joint_states.py
  scripts/unity_joint_states_publisher.py
  scripts/unity_command_forwarder.py
  scripts/unity_execution_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install config files
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
