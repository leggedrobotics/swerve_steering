cmake_minimum_required(VERSION 3.0.2)
project(strafe_joy)

find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  swerve_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(strafe_joy src/strafe_joy.cpp)

target_link_libraries(strafe_joy ${catkin_LIBRARIES})