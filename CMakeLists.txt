cmake_minimum_required(VERSION 3.5)
project(urg_node)

# find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# we currently still rely on boost for its `crc` component
find_package(Boost REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(laser_proc REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(urg_c REQUIRED)
find_package(urg_node_msgs REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED 
  src/urg_c_wrapper.cpp
  src/urg_node.cpp
)

rclcpp_components_register_node(urg_node
  PLUGIN "urg_node::UrgNode"
  EXECUTABLE urg_node_driver)

ament_auto_add_executable(getID src/get_id.cpp)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
