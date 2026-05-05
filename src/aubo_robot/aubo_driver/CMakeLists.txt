cmake_minimum_required(VERSION 2.8.3)
project(aubo_driver)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  std_msgs
  trajectory_msgs
  aubo_msgs
  industrial_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS sensor_msgs roscpp std_msgs trajectory_msgs aubo_msgs industrial_msgs
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

# Use local protobuf 2.6.1 instead of system protobuf 3.x
set(LOCAL_PROTOBUF_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../opt")
include_directories(
    include
    include/otg
    ${catkin_INCLUDE_DIRS}
    ${LOCAL_PROTOBUF_DIR}/include
)

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        link_directories(
            ${catkin_LIB_DIRS}
            ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib64
            ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib64/log4cplus
            ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib64/config
            ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib64/aubocontroller
            ${LOCAL_PROTOBUF_DIR}/lib
        )
else()
        link_directories(${catkin_LIB_DIRS}  lib/lib32)
endif()

## Declare a C++ executable
set(${PROJECT_NAME}_SOURCES1
    src/driver_node.cpp
)
set(${PROJECT_NAME}_SOURCES2
    src/testIO.cpp
)
set(${PROJECT_NAME}_SOURCES3
    src/testAuboAPI.cpp
)

# aubo_driver and test tools require libauborobotcontroller (needs protobuf 2.x / libprotobuf.so.9)
set(AUBO_CONTROLLER_LIB "${CMAKE_CURRENT_SOURCE_DIR}/lib/lib64/aubocontroller/libauborobotcontroller.so")

if(EXISTS "${AUBO_CONTROLLER_LIB}")
  message(STATUS "Found AUBO controller library: ${AUBO_CONTROLLER_LIB}")
else()
  set(AUBO_CONTROLLER_LIB FALSE)
  message(WARNING "libauborobotcontroller not found at ${AUBO_CONTROLLER_LIB}")
endif()

if(AUBO_CONTROLLER_LIB)
  # Set RPATH to find local protobuf 2.6.1 and other libs at runtime (use absolute paths)
  get_filename_component(PROTOBUF_LIB_ABS "${LOCAL_PROTOBUF_DIR}/lib" ABSOLUTE)
  get_filename_component(LIB64_ABS "${CMAKE_CURRENT_SOURCE_DIR}/lib/lib64" ABSOLUTE)
  set(CMAKE_INSTALL_RPATH "${PROTOBUF_LIB_ABS}:${LIB64_ABS}:${LIB64_ABS}/aubocontroller:${LIB64_ABS}/config:${LIB64_ABS}/log4cplus")
  set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

  add_executable(aubo_driver ${${PROJECT_NAME}_SOURCES1})
  add_executable(testIO ${${PROJECT_NAME}_SOURCES2})
  add_executable(testAuboAPI ${${PROJECT_NAME}_SOURCES3})

  # Link against local protobuf 2.6.1 explicitly
  target_link_libraries(aubo_driver pthread ${catkin_LIBRARIES} auborobotcontroller otgLib ${LOCAL_PROTOBUF_DIR}/lib/libprotobuf.so config log4cplus)
  target_link_libraries(testAuboAPI ${catkin_LIBRARIES} auborobotcontroller otgLib ${LOCAL_PROTOBUF_DIR}/lib/libprotobuf.so config log4cplus)
  target_link_libraries(testIO ${catkin_LIBRARIES} otgLib)

  add_dependencies(aubo_driver aubo_msgs_gencpp)
  add_dependencies(testIO aubo_msgs_gencpp)
  add_dependencies(testAuboAPI aubo_msgs_gencpp)
else()
  message(WARNING "libauborobotcontroller not found, skipping aubo_driver/testIO/testAuboAPI")
endif()

add_executable(aubo_gazebo_driver src/aubo_gazebo_driver.cpp)
target_link_libraries(aubo_gazebo_driver ${catkin_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
 )


