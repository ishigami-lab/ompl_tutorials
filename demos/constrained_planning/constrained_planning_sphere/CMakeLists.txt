cmake_minimum_required(VERSION 2.8)
project(constrained_planning_sphere)

find_package(ompl REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)

include_directories(
  /usr/include/eigen3
  ${OMPL_INCLUDE_DIRS}
)

add_executable (Main main.cpp)
target_link_libraries (Main ${OMPL_LIBRARIES})