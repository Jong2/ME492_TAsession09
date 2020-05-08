cmake_minimum_required(VERSION 2.8.3)
project(lidar_pcl_tutorial)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

find_package( OpenCV REQUIRED )

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  #include
)
add_executable(lidar_pcl_tutorial src/lidar_pcl_tutorial.cpp)

target_link_libraries(lidar_pcl_tutorial
  ${catkin_LIBRARIES}
)
