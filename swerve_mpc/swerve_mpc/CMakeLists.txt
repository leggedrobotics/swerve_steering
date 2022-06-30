cmake_minimum_required(VERSION 3.0.2)
project(swerve_mpc)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


set(CATKIN_PACKAGE_DEPENDENCIES
  tf
  tf2_ros
  tf2_geometry_msgs
  urdf
  kdl_parser
  robot_state_publisher
  visualization_msgs
  geometry_msgs
  nav_msgs
  ocs2_ros_interfaces
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_msgs
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  trapezoidal_tracker
  manipulation_msgs
  swerve_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    Boost
    pinocchio
)

###########
## Build ##
###########

set(FLAGS ${OCS2_CXX_FLAGS} ${pinocchio_CFLAGS_OTHER} -Wno-ignored-attributes -Wno-invalid-partial-specialization -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR)


# Add directories for all targets
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}

)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

# swerve interface library
add_library(${PROJECT_NAME}
  src/constraint/EndEffectorConstraint.cpp
  src/constraint/JointVelocityLimits.cpp
  src/constraint/JointLimitsConstraint.cpp
  src/constraint/LegsInputConstraints.cpp
  src/constraint/WheelRollingConstraintCppAd.cpp
  src/constraint/LegsConstraintCppAd.cpp
  src/dynamics/SwerveDynamics.cpp
  src/reference_manager/SwerveReferenceManager.cpp
  src/cost/NominalSteerStateCppAd.cpp
  src/cost/NominalArmStateCppAd.cpp
  src/SwervePinocchioMapping.cpp
  src/SwerveInterface.cpp
  src/SwerveTarget.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${pinocchio_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

# MPC node
add_executable(swerve_mpc_node
  src/SwerveMpcNode.cpp
)
add_dependencies(swerve_mpc_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(swerve_mpc_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)
target_compile_options(swerve_mpc_node PUBLIC ${FLAGS})

# Dummy node
add_executable(swerve_dummy_mrt_node
  src/dummy_tools/SwerveDummyMRT.cpp
  src/dummy_tools/SwerveDummyVisualization.cpp
)
add_dependencies(swerve_dummy_mrt_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(swerve_dummy_mrt_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)
target_compile_options(swerve_dummy_mrt_node PUBLIC ${FLAGS})


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config launch rviz 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS)
