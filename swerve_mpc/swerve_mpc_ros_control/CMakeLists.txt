cmake_minimum_required(VERSION 3.0.2)
project(swerve_mpc_ros_control)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  swerve_mpc
  swerve_msgs
  ddynamic_reconfigure
)

find_package(Eigen3 REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
  INCLUDE_DIRS 
    include
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    controller_interface
    hardware_interface
    swerve_msgs
    ddynamic_reconfigure
  DEPENDS 

)

include_directories(
  include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/swerve_mpc_ros_control.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)



install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES swerve_mpc_ros_control_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

