cmake_minimum_required(VERSION 2.6.0)

project(plane_seg_ros)

add_compile_options(-std=c++1y -Wall)
#add_definitions(-std=c++1y -Wall)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  pcl_conversions
  tf_conversions
  pcl_ros
  grid_map_core
  grid_map_ros
  grid_map_msgs
  plane_seg
)

find_package(OpenCV 3.0 QUIET)

catkin_package(
  INCLUDE_DIRS
    #include
  CATKIN_DEPENDS eigen_conversions pcl_conversions tf_conversions pcl_ros plane_seg
)


include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
)


set(APP_NAME plane_seg_ros)
add_executable(${APP_NAME} src/${APP_NAME}.cpp)
target_link_libraries(${APP_NAME} boost_system ${catkin_LIBRARIES})
