cmake_minimum_required(VERSION 3.0.2)
project(project_rp)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(. /usr/include/eigen3)

add_executable(make_manipulator src_cpp/make_manipulator.cpp)
target_link_libraries (make_manipulator Eigen3::Eigen)
