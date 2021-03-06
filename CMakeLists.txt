cmake_minimum_required(VERSION 2.8.3)
project(rtt_ros_control_embedded)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    hardware_interface
    roscpp
    rtt_ros
    rtt_rosclock
    urdf
    cmake_modules
)

find_package(Eigen REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${USE_OROCOS_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_definitions(-DRTT_COMPONENT)

orocos_library(rtt_hw_interface
  src/rtt_hw_interface.cpp
)
orocos_component(${PROJECT_NAME} src/rtt_ros_control_embedded.cpp)

target_link_libraries(rtt_hw_interface
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    rtt_hw_interface
    ${USE_OROCOS_LIBRARIES}
)

orocos_generate_package(DEPENDS rtt_ros)
