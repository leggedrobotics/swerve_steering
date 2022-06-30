#include <swerve_mpc/SwerveInterface.hpp>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <swerve_mpc/dummy_tools/SwerveDummyVisualization.hpp>
#include "swerve_mpc/SwerveModelInfo.hpp"
#include "swerve_mpc/SwerveTarget.hpp"

#include <nav_msgs/Path.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesInteractiveMarker.h>
#include "swerve_msgs/BrakesService.h"

#include <ros/init.h>
#include <ros/package.h>

using namespace ocs2;
using namespace swerve;

int main(int argc, char** argv) {
  const std::string robotName = "swerve";

  // task files
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  std::string taskFileFolderName = std::string(programArgs[1]);

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mrt");
  ros::NodeHandle nodeHandle;

  // Get node parameters
  std::string taskFile, libFolder, urdfString;
  nodeHandle.getParam("/taskFile", taskFile);
  nodeHandle.getParam("/libFolder", libFolder);
  nodeHandle.getParam("/swerve_mpc/robot_description", urdfString);

  // Robot interface
  SwerveInterface interface(taskFile, libFolder, urdfString);

  // MRT
  MRT_ROS_Interface mrt(robotName);
  mrt.initRollout(&interface.getRollout());
  mrt.launchNodes(nodeHandle);

  SwerveModelInfo swerveModelInfo = interface.getSwerveModelInfo();

  // Visualization
  std::shared_ptr<SwerveDummyVisualization> dummyVisualization(new SwerveDummyVisualization(nodeHandle, interface, swerveModelInfo));
  // Dummy MRT
  MRT_ROS_Dummy_Loop dummy(mrt, interface.mpcSettings().mrtDesiredFrequency_, interface.mpcSettings().mpcDesiredFrequency_);
  dummy.subscribeObservers({dummyVisualization});

  SwerveTarget swerveTarget_(nodeHandle, swerveModelInfo, "world");
  swerveTarget_.initDummyVisualizationTarget();
  if (swerveModelInfo.armPresent) {
    swerveTarget_.initInterectiveMarker();
  } else {
    swerveTarget_.initSetGoalPose();
  }
  // initial state
  SystemObservation initObservation;
  initObservation.state = interface.getInitialState();
  initObservation.input.setZero(swerveModelInfo.inputDim);
  initObservation.time = 0.0;

  // initial command
  vector_t initTarget(11);
  initTarget.head(3) << 0, 0, 0;
  initTarget.tail(8).head(4) << Eigen::Quaternion<scalar_t>(1, 0, 0, 0).coeffs();
  initTarget.tail(4) << 0, 0, 0, 0;
  const vector_t zeroInput = vector_t::Zero(swerveModelInfo.inputDim);
  const TargetTrajectories initTargetTrajectories({initObservation.time}, {initTarget}, {zeroInput});

  // Run dummy (loops while ros is ok)
  dummy.run(initObservation, initTargetTrajectories);

  // Successful exit
  return 0;
}