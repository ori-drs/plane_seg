cmake_minimum_required(VERSION 2.8.3)
project(terrain_simplification_ros)

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
  grid_map_filters_drs
  message_generation
  terrain_simplification
  tf_conversions
  )

add_service_files(
  FILES
  GetValueAtPosition.srv
)

generate_messages()


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
  grid_map_filters_drs
  message_runtime
  std_msgs
  geometry_msgs
  terrain_simplification
  )

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(${PROJECT_NAME}
  src/terrain_simplification_ros.cpp
  )

add_dependencies(${PROJECT_NAME} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )

add_executable(${PROJECT_NAME}_node
  src/terrain_simplification_ros_node.cpp
  )

add_dependencies(${PROJECT_NAME}_node  
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )

add_executable(${PROJECT_NAME}_service_caller
  src/terrain_simplification_ros_service_caller.cpp
  )

target_link_libraries(${PROJECT_NAME}_service_caller
  ${PROJECT_NAME}
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
