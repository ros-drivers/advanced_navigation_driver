cmake_minimum_required(VERSION 2.8.3)
project(advanced_navigation_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(advanced_navigation_driver src/an_driver.cpp src/spatial_packets.c src/an_packet_protocol.c src/rs232/rs232.c)
target_link_libraries(advanced_navigation_driver ${catkin_LIBRARIES})
