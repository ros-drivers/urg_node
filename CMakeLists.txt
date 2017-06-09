cmake_minimum_required(VERSION 2.8.3)
project(urg_node)

#find_package(catkin REQUIRED COMPONENTS
#  diagnostic_updater dynamic_reconfigure laser_proc message_generation
#  nodelet rosconsole roscpp sensor_msgs std_msgs std_srvs tf urg_c
#)
find_package(ament_cmake REQUIRED)
find_package(urg_node_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
#find_package(dynamic_reconfigure REQUIRED)
find_package(laser_proc REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(urg_c REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ros2_time REQUIRED)
find_package(Boost REQUIRED thread)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11")

# catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES urg_c_wrapper urg_node_driver
#  CATKIN_DEPENDS dynamic_reconfigure laser_proc message_runtime nodelet rosconsole
#  roscpp sensor_msgs std_msgs std_srvs urg_c
#  DEPENDS
#)

set(INCLUDE_DIRS
  include
  ${urg_node_msgs_INCLUDE_DIRS}
  ${diagnostic_updater_INCLUDE_DIRS}
  #${dynamic_reconfigure_INCLUDE_DIRS}
  ${laser_proc_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${urg_c_INCLUDE_DIRS}
  ${ros2_time_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(LIBS
  ${urg_node_msgs_LIBRARIES}
  ${diagnostic_updater_LIBRARIES}
  #${dynamic_reconfigure_LIBRARIES}
  ${laser_proc_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${std_srvs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${urg_c_LIBRARIES}
  ${ros2_time_LIBRARIES}
  ${Boost_LIBRARIES}
)

include_directories(include ${INCLUDE_DIRS})

## Declare a cpp library
add_library(urg_c_wrapper src/urg_c_wrapper.cpp)
target_link_libraries(urg_c_wrapper ${LIBS})

# Declare the cpp as a lib also
add_library(urg_node_driver src/urg_node_driver.cpp)
add_dependencies(urg_node_driver ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
target_link_libraries(urg_node_driver urg_c_wrapper ${LIBS})

## Declare a cpp executable
add_executable(urg_node src/urg_node.cpp)
target_link_libraries(urg_node urg_node_driver ${LIBS})

## Declare getID tool
add_executable(getID src/getID.cpp)
target_link_libraries(getID urg_c_wrapper ${LIBS})

## Mark executables and/or libraries for installation
install(TARGETS urg_c_wrapper urg_node urg_node_driver getID
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
   RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION include/${PROJECT_NAME}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

## Install script to set IP addresses
install(PROGRAMS
   scripts/set_urg_ip.py
   DESTINATION bin
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslint)
  # Ignore long -> int32_t warning, urg_c driver uses
  # long, etc. Wrapper must also use these
  set(ROSLINT_CPP_OPTS "--filter=-runtime/references,-runtime/int")
  roslint_cpp()
  roslint_add_test()
endif()

ament_export_dependencies(urg_node_msgs)
ament_export_dependencies(diagnostic_updater)
ament_export_dependencies(dynamic_reconfigure)
ament_export_dependencies(laser_proc)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(std_srvs)
ament_export_dependencies(tf2)
ament_export_dependencies(urg_c)
ament_export_dependencies(builtin_interfaces)
ament_export_dependencies(rosidl_default_generators)
ament_export_dependencies(ros2_time)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(urg_c_wrapper urg_node_driver)

ament_package()
