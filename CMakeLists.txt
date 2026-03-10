cmake_minimum_required(VERSION 3.10)

project(imu_udp_plugin)

find_package(gazebo REQUIRED)

include_directories(
    ${GAZEBO_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/msgpack/include
    )

add_library(ImuUDPPlugin SHARED ImuUDPPlugin.cc)

target_link_libraries(ImuUDPPlugin
  ${GAZEBO_LIBRARIES}
)