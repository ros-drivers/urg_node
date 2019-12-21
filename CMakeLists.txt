cmake_minimum_required(VERSION 3.5)
project(urg_node)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(boost COMPONENTS crc REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(laser_proc REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(urg_c REQUIRED)
find_package(urg_node_msgs REQUIRED)

add_library(urg_c_wrapper src/urg_c_wrapper.cpp)
target_include_directories(urg_c_wrapper
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIR}
)
target_link_libraries(urg_c_wrapper ${Boost_LIBRARIES})
ament_target_dependencies(urg_c_wrapper
  rclcpp
  sensor_msgs
  urg_c
)

add_library(urg_node_driver SHARED src/urg_node_driver.cpp)
target_include_directories(urg_node_driver
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(urg_node_driver urg_c_wrapper)
ament_target_dependencies(urg_node_driver
  diagnostic_updater
  laser_proc
  rclcpp
  std_srvs
  urg_node_msgs
)

add_executable(urg_node src/urg_node.cpp)
target_link_libraries(urg_node urg_node_driver)

add_executable(getID src/get_id.cpp)
target_link_libraries(getID urg_node_driver)

install(TARGETS urg_node urg_node_driver getID
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS scripts/set_urg_ip.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(urg_node_msgs)
ament_export_dependencies(
  diagnostic_updater
  laser_proc
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  urg_c
)
ament_export_include_directories(include)
ament_export_libraries(urg_node_driver)
ament_package()
