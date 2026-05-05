cmake_minimum_required(VERSION 3.0.2)
project(PointCLoud_SLAM)

add_compile_options(-std=c++14 -O2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  pcl_ros
  pcl_conversions
  message_filters
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    std_msgs
    geometry_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_eigen
    pcl_ros
    pcl_conversions
    message_filters
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(tabletop_segmentation_node
  src/tabletop_segmentation_node.cpp
)

target_link_libraries(tabletop_segmentation_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(tabletop_segmentation_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(poisson_reconstruction_node
  src/poisson_reconstruction_node.cpp
)

target_link_libraries(poisson_reconstruction_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(poisson_reconstruction_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(cloud_saver_node
  src/cloud_saver_node.cpp
)

target_link_libraries(cloud_saver_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(cloud_saver_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
