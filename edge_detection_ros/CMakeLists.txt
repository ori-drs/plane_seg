cmake_minimum_required(VERSION 2.8.3)
project(edge_detection_ros)

add_compile_options(-std=c++11)
set(DEFAULT_BUILD "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${DEFAULT_BUILD}' as none was specified.")
  set(CMAKE_BUILD_TYPE ${DEFAULT_BUILD} CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug"
    "Release"
    "MinSizeRel"
    "RelWithDebInfo")
endif()

add_definitions(-DMELO_FUNCTION_PRINTS)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  std_msgs
  geometry_msgs
  visualization_msgs
  grid_map_core
  grid_map_filters
  grid_map_cv
  message_generation
  edge_detection
  )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
catkin_package(
  INCLUDE_DIRS include
     LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
  grid_map_core
  grid_map_filters
  grid_map_cv
  message_runtime
  std_msgs
  geometry_msgs
  edge_detection
  )

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_executable(${PROJECT_NAME}
        src/edge_detection_ros_node.cpp
        src/edge_detection_ros.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

add_executable(fake_edges_publisher
        src/fake_edges_publisher.cpp
        src/edge_detection_ros.cpp
        )

target_link_libraries(fake_edges_publisher
        ${catkin_LIBRARIES}
        )

#############
## Install ##
#############
# Mark library for installation
install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

# Mark header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  )

# Mark other files for installation
install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
