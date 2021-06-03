cmake_minimum_required(VERSION 2.8.3)
project(tiago_planar_pose)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  visp_bridge
  std_srvs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  image_transport
  visp_ros
  
)

## System dependencies are found with CMake's conventions
find_package(OpenCV COMPONENTS opencv_highgui opencv_features2d opencv_xfeatures2d opencv_calib3d)
if(NOT OPENCV_FOUND)
	find_package(OpenCV REQUIRED COMPONENTS opencv_highgui) 
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

catkin_package(
  LIBRARIES tiago_planar_pose
  CATKIN_DEPENDS roscpp image_transport cv_bridge sensor_msgs  std_msgs geometry_msgs visp_bridge
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${ViSP_INCLUDE_DIRS}
)

add_executable(planar_pose src/tiago_planar_pose.cpp)
target_link_libraries(planar_pose ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


install(TARGETS planar_pose
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)