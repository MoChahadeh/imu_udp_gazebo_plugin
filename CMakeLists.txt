cmake_minimum_required(VERSION 3.22)

project(imu_udp_plugin CXX)

find_package(gz-cmake4 REQUIRED)

gz_find_package(gz-plugin REQUIRED COMPONENTS register)
gz_find_package(gz-sim REQUIRED)
gz_find_package(gz-transport REQUIRED)
gz_find_package(gz-msgs REQUIRED)

add_library(ImuUDPPlugin SHARED src/ImuUDPPlugin.cc)
set_property(TARGET ImuUDPPlugin PROPERTY CXX_STANDARD 17)

target_include_directories(ImuUDPPlugin
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/msgpack/include)

target_link_libraries(ImuUDPPlugin
  PRIVATE
    gz-plugin::gz-plugin
    gz-sim::gz-sim
    gz-transport::gz-transport
    gz-msgs::gz-msgs)