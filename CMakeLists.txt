cmake_minimum_required(VERSION 3.5)
project(lidar2d_preprocess)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(
#  include ${catkin_INCLUDE_DIRS}
#)

## Declare C++ executables
add_executable(distance_histogram src/distance_histogram.cpp)
ament_target_dependencies(distance_histogram rclcpp sensor_msgs)

add_executable(lidar2d_preprocess src/lidar2d_preprocess.cpp)
ament_target_dependencies(lidar2d_preprocess rclcpp sensor_msgs)

## Add cmake target dependencies of the executable
#add_dependencies(distance_histogram ${PROJECT_NAME}_gencfg)
#add_dependencies(lidar2d_preprocess ${PROJECT_NAME}_gencfg)


#############
## Install ##
#############

install(
    TARGETS
        distance_histogram
        lidar2d_preprocess
    DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}/
# )

ament_package()
