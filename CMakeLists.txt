cmake_minimum_required(VERSION 3.0.2)
project(urg_node)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater dynamic_reconfigure laser_proc message_generation
  nodelet rosconsole roscpp sensor_msgs std_msgs std_srvs tf urg_c
)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11")

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/URG.cfg)

add_message_files(FILES
  Status.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES urg_c_wrapper urg_node_driver
  CATKIN_DEPENDS dynamic_reconfigure laser_proc message_runtime nodelet rosconsole
  roscpp sensor_msgs std_msgs std_srvs urg_c
  DEPENDS
)


include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(urg_c_wrapper src/urg_c_wrapper.cpp)
target_link_libraries(urg_c_wrapper ${catkin_LIBRARIES})

# Declare the cpp as a lib also
add_library(urg_node_driver src/urg_node_driver.cpp)
add_dependencies(urg_node_driver ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
target_link_libraries(urg_node_driver urg_c_wrapper ${catkin_LIBRARIES})

## Declare a cpp executable
add_executable(urg_node src/urg_node.cpp)
target_link_libraries(urg_node urg_node_driver ${catkin_LIBRARIES})

## Declare getID tool
add_executable(getID src/getID.cpp)
target_link_libraries(getID urg_c_wrapper ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS urg_c_wrapper urg_node urg_node_driver getID
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

## Install script to set IP addresses
install(PROGRAMS
   scripts/set_urg_ip.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY urdf meshes launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslint)
  # Ignore long -> int32_t warning, urg_c driver uses
  # long, etc. Wrapper must also use these
  set(ROSLINT_CPP_OPTS "--filter=-runtime/references,-runtime/int")
  roslint_cpp(
    include/urg_node/urg_c_wrapper.h
    include/urg_node/urg_node_driver.h
    src/getID.cpp
    src/urg_c_wrapper.cpp
    src/urg_node_driver.cpp
    src/urg_node.cpp
  )
  roslint_add_test()

  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/urg_lidar.launch)
endif()
