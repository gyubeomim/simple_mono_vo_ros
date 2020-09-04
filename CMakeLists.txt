cmake_minimum_required(VERSION 3.0.2)
project(mono_vo_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  cv_bridge
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  src/
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(mono_vo_node
  src/mono_vo_node.cpp
  src/mono_vo.cpp
  src/util.cpp
  )

target_link_libraries(mono_vo_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  )
