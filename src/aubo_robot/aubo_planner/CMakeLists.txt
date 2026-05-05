cmake_minimum_required(VERSION 2.8.3)
project(aubo_planner)

find_package(catkin REQUIRED COMPONENTS aubo_msgs  moveit_core moveit_ros_planning moveit_ros_planning_interface pluginlib cmake_modules)

find_package(Boost REQUIRED system filesystem date_time thread)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs
  control_msgs trajectory_msgs actionlib_msgs actionlib
  urdf industrial_msgs industrial_utils)

if("$ENV{ROS_DISTRO}" STREQUAL "kinetic")
    find_package(Eigen3 REQUIRED)
    Message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note: Find Eigen3 Package in aubo_planner for kinetic")
endif()

if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
    find_package(Eigen REQUIRED)
    Message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note: Find Eigen Package in aubo_planner for  for indigo")
endif()

# Noetic uses Eigen3 but it's found via catkin/moveit, add include path explicitly
if("$ENV{ROS_DISTRO}" STREQUAL "noetic")
    find_package(Eigen3 REQUIRED)
endif()

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    control_msgs
    actionlib
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    trajectory_msgs
    industrial_msgs
    actionlib
    actionlib_msgs
    industrial_utils
    urdf
    LIBRARIES ${PROJECT_NAME}_dummy
)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif()

#set(CMAKE_CXX_COMPILER "clang++")
#set(CMAKE_C_COMPILER "clang")

include_directories(include  ${catkin_INCLUDE_DIRS})

if("$ENV{ROS_DISTRO}" STREQUAL "kinetic")
    include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
    Message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note: Find EIGEN3_INCLUDE_DIRS Package in aubo_planner for kinetic")
endif()

if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
    include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
    Message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note: Find EIGEN_INCLUDE_DIRS Package in aubo_planner for  for indigo")
endif()

if("$ENV{ROS_DISTRO}" STREQUAL "noetic")
    include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
endif()

link_directories(${catkin_LIBRARY_DIRS} )

ADD_EXECUTABLE(aubo_planner_test src/aubo_planner_test.cpp)
TARGET_LINK_LIBRARIES(aubo_planner_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#ADD_EXECUTABLE(aubo_ros_plan src/aubo_ros_plan.cpp)
#TARGET_LINK_LIBRARIES(aubo_ros_plan ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#install(TARGETS aubo_ros_plan DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS script/aubo_planner/aubo_ros_plan
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Install Python modules for motion planning interface
catkin_install_python(PROGRAMS
  scripts/motion_planning_interface.py
  scripts/planner_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install Python packages
install(DIRECTORY scripts/goals scripts/planners scripts/validators scripts/execution
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.py"
)

# Install examples
catkin_install_python(PROGRAMS
  examples/simple_motion_example.py
  examples/advanced_motion_example.py
  examples/goto_pose.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)
