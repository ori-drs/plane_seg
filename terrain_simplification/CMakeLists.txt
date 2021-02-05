cmake_minimum_required(VERSION 2.8.3)
project(terrain_simplification)

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
  std_msgs
  geometry_msgs
  visualization_msgs
  grid_map_core
  grid_map_filters
  grid_map_cv
  message_generation
  )

find_package(OpenCV REQUIRED)
# Generate messages in the 'msg' folder
#add_message_files(
#        FILES
#        XXX.msg
#)
#
#generate_messages(
#        DEPENDENCIES
#        std_msgs
#        geometry_msgs
#)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
catkin_package(
  INCLUDE_DIRS include
     LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  grid_map_core
  grid_map_filters
  grid_map_cv
  message_runtime
  std_msgs
  geometry_msgs
  )

###########
## Build ##
###########
add_library(${PROJECT_NAME}
  src/terrain_simplification.cpp
  )

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )

add_dependencies(${PROJECT_NAME} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${OpenCV_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})


target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
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
  FILES_MATCHING PATTERN "*.hpp"
  )

# Mark other files for installation
install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
