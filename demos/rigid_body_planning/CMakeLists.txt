cmake_minimum_required(VERSION 2.8)
project(rigid_body_planning)

find_package(ompl REQUIRED)

include_directories(
  /usr/include/eigen3
  ${OMPL_INCLUDE_DIRS}
)

add_executable (Main main.cpp)
target_link_libraries (Main ${OMPL_LIBRARIES})