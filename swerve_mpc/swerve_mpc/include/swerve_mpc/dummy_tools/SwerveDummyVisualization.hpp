#pragma once

#include <robot_state_publisher/robot_state_publisher.h>
#include <tf/transform_broadcaster.h>

#include <ocs2_ros_interfaces/mrt/DummyObserver.h>

#include "swerve_mpc/SwerveInterface.hpp"
#include "swerve_mpc/SwerveModelInfo.hpp"


namespace ocs2 {
namespace swerve {

class SwerveDummyVisualization final : public DummyObserver {
  public:
    SwerveDummyVisualization(ros::NodeHandle& nodeHandle, const SwerveInterface& interface, SwerveModelInfo swerveModelInfo)
      : pinocchioInterface_(interface.getPinocchioInterface()), swerveModelInfo_(swerveModelInfo) {
    launchVisualizerNode(nodeHandle);
  }
  ~SwerveDummyVisualization() override = default;

  void update(const SystemObservation& observation, const PrimalSolution& policy, const CommandData& command) override;

  private:
    void launchVisualizerNode(ros::NodeHandle& nodeHandle);

    void publishObservation(const ros::Time& timeStamp, const SystemObservation& observation);
    void publishTargetTrajectories(const ros::Time& timeStamp, const TargetTrajectories& targetTrajectories);
    void publishOptimizedTrajectory(const ros::Time& timeStamp, const PrimalSolution& policy);

    PinocchioInterface pinocchioInterface_;

    std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;
    tf::TransformBroadcaster tfBroadcaster_;

    ros::Publisher stateOptimizedPublisher_;
    ros::Publisher stateOptimizedPosePublisher_;

    SwerveModelInfo swerveModelInfo_;
};

}  // namespace swerve
}  // namespace ocs2