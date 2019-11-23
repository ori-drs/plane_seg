cmake_minimum_required(VERSION 2.6.0)

project(plane_seg)

add_compile_options(-std=c++1y -Wall)
#add_definitions(-std=c++1y -Wall)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
)

find_package(OpenCV 3.0 QUIET)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES plane_seg
  CATKIN_DEPENDS  pcl_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


# create library
set(LIB_NAME plane_seg)
add_library(${LIB_NAME} SHARED
  src/PlaneFitter.cpp
  src/RobustNormalEstimator.cpp
  src/IncrementalPlaneEstimator.cpp
  src/PlaneSegmenter.cpp
  src/RectangleFitter.cpp
  src/BlockFitter.cpp
)
add_dependencies(plane_seg ${catkin_EXPORTED_TARGETS})
target_link_libraries(plane_seg ${catkin_LIBRARIES})


# standalone lcm-based block fitter
set(APP_NAME block_fitter)
add_executable(${APP_NAME} src/block-fitter.cpp)
target_link_libraries(${APP_NAME} boost_system ${catkin_LIBRARIES})


# test program
set(APP_NAME plane_seg_test)
add_executable(${APP_NAME} src/${APP_NAME}.cpp)
target_link_libraries(${APP_NAME} boost_system ${catkin_LIBRARIES} plane_seg)


# fit single block
set(APP_NAME plane_seg_test2)
add_executable(${APP_NAME} src/plane_seg_test2.cpp)
target_link_libraries(${APP_NAME} boost_system ${catkin_LIBRARIES} plane_seg ${OpenCV_LIBS})


# install
install(TARGETS plane_seg
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})