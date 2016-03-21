cmake_minimum_required(VERSION 2.8.3)
project(rtt_ros_control_embedded)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    hardware_interface
    roscpp
    rtt_ros
    rtt_rosclock
    cmake_modules
)
find_package(orocos_kdl REQUIRED)
find_package(Eigen REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${USE_OROCOS_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_definitions(-DRTT_COMPONENT)

## Declare a cpp library
add_library(rtt_hw_interface
  src/rtt_hw_interface.cpp
)
orocos_component(${PROJECT_NAME} src/rtt_ros_control_embedded.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(rtt_hw_interface
  ${catkin_LIBRARIES}
  ${USE_OROCOS_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    rtt_hw_interface
    ${USE_OROCOS_LIBRARIES}
)

orocos_generate_package(DEPENDS rtt_ros)