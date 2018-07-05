cmake_minimum_required(VERSION 3.5)
project(nxt_slam)

find_package(catkin REQUIRED COMPONENTS std_msgs)
find_package(gmapping REQUIRED)

catkin_package(
  CATKIN_DEPENDS std_msgs
)

