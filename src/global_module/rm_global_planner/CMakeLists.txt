cmake_minimum_required(VERSION 3.0.2)
project(rm_global_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rm_global_planner
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(path2waypoint src/path2waypoint.cpp)

target_link_libraries(path2waypoint
  ${catkin_LIBRARIES}
)