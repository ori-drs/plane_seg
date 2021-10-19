cmake_minimum_required(VERSION 3.0.2)

project(plane_seg_ros)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  pcl_conversions
  tf_conversions
  tf2_eigen
  tf2_ros
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

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})

# TODO: Install
