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


find_package(OpenCV 3.0 REQUIRED COMPONENTS core highgui imgproc imgcodecs)

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
               CATKIN_DEPENDS eigen_conversions
                              pcl_conversions
                              tf_conversions
                              pcl_ros
                              plane_seg
                              grid_map_msgs)




include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}.cpp
                                   src/geometry_utils.cpp
                                   src/Visualizer.cpp
                                   )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})


set(APP_NAME ${PROJECT_NAME}_node)
add_executable(${APP_NAME} src/${APP_NAME}.cpp)
target_link_libraries(${APP_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${APP_NAME} ${PROJECT_NAME} ${OpenCV_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS ${APP_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY data/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
