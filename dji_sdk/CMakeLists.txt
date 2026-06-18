cmake_minimum_required(VERSION 3.10)
project(dji_sdk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake-modules")
find_package(djiosdk-core QUIET)
if(NOT djiosdk-core_FOUND)
  MESSAGE(STATUS "DO NOT FIND THE djiosdk-core LIB, WILL TRY TO DOWNLOAD FORM GITHUB, COMPILE, INSTALL")
  execute_process(COMMAND mkdir ${CMAKE_CURRENT_SOURCE_DIR}/../cmake-modules/build)
  execute_process(
          COMMAND cmake ..
          WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../cmake-modules/build)

  execute_process(
          COMMAND make
          WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../cmake-modules/build)
endif()

find_package(DJIOSDK REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FCTimeInUTC.msg"
  "msg/GPSUTC.msg"
  "msg/Gimbal.msg"
  "msg/Waypoint.msg"
  "msg/WaypointList.msg"
  "msg/MobileData.msg"
  "msg/PayloadData.msg"
  "msg/MissionWaypointAction.msg"
  "msg/MissionWaypoint.msg"
  "msg/MissionWaypointTask.msg"
  "msg/MissionHotpointTask.msg"
  "msg/FlightAnomaly.msg"
  "msg/VOPosition.msg"
  "srv/Activation.srv"
  "srv/CameraAction.srv"
  "srv/DroneTaskControl.srv"
  "srv/SDKControlAuthority.srv"
  "srv/SetLocalPosRef.srv"
  "srv/MFIOConfig.srv"
  "srv/MFIOSetValue.srv"
  "srv/DroneArmControl.srv"
  "srv/MissionStatus.srv"
  "srv/MissionWpAction.srv"
  "srv/MissionHpAction.srv"
  "srv/MissionWpUpload.srv"
  "srv/MissionWpSetSpeed.srv"
  "srv/MissionWpGetSpeed.srv"
  "srv/MissionWpGetInfo.srv"
  "srv/MissionHpUpload.srv"
  "srv/MissionHpGetInfo.srv"
  "srv/MissionHpUpdateYawRate.srv"
  "srv/MissionHpUpdateRadius.srv"
  "srv/MissionHpResetYaw.srv"
  "srv/SendMobileData.srv"
  "srv/SendPayloadData.srv"
  "srv/SetHardSync.srv"
  "srv/QueryDroneVersion.srv"
  "srv/Stereo240pSubscription.srv"
  "srv/StereoVGASubscription.srv"
  "srv/StereoDepthSubscription.srv"
  "srv/SetupCameraStream.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${DJIOSDK_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(dji_sdk
#   src/${PROJECT_NAME}/dji_sdk.cpp
# )

## Declare a cpp executable
add_executable(dji_sdk_node
  src/main.cpp
  src/modules/dji_sdk_node_control.cpp
  src/modules/dji_sdk_node_services.cpp
  src/modules/dji_sdk_node.cpp
  src/modules/dji_sdk_node_mission_services.cpp
  src/modules/dji_sdk_node_subscriber.cpp
  src/modules/dji_sdk_node_publisher.cpp
  src/modules/dji_sdk_node_mobile_comm.cpp
  src/modules/dji_sdk_node_payload_comm.cpp
  src/modules/dji_sdk_node_time_sync.cpp)

target_link_libraries(dji_sdk_node
  PUBLIC rclcpp::rclcpp geometry_msgs::geometry_msgs sensor_msgs::sensor_msgs std_msgs::std_msgs nmea_msgs::nmea_msgs tf2::tf2 ${cpp_typesupport_target} ${DJIOSDK_LIBRARIES}
)

#############
## Install ##
#############
# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS dji_sdk_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


ament_package()
