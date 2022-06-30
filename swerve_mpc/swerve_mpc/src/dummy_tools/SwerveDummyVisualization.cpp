#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ros/package.h>
#include <tf/tf.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_ros_interfaces/common/RosMsgHelpers.h>

#include <swerve_mpc/SwerveInterface.hpp>

#include <swerve_mpc/dummy_tools/SwerveDummyVisualization.hpp>

namespace ocs2 {
namespace swerve {

Eigen::Vector3d getBasePosition(Eigen::VectorXd state) {
  return Eigen::Vector3d(state.head<3>());
}

Eigen::Quaterniond getBaseOrientation(Eigen::VectorXd state) {
  return Eigen::Quaterniond(state.segment<4>(3));
}

template <typename It>
void assignHeader(It firstIt, It lastIt, const std_msgs::Header& header) {
  for (; firstIt != lastIt; ++firstIt) {
    firstIt->header = header;
  }
}

template <typename It>
void assignIncreasingId(It firstIt, It lastIt, int startId = 0) {
  for (; firstIt != lastIt; ++firstIt) {
    firstIt->id = startId++;
  }
}

void SwerveDummyVisualization::launchVisualizerNode(ros::NodeHandle& nodeHandle) {
  // load a kdl-tree from the urdf robot description and initialize the robot state publisher
  const std::string urdfParam = "/swerve_mpc/robot_description";
  urdf::Model model;
  if (!model.initParam(urdfParam)) {
    ROS_ERROR("URDF model load was NOT successful");
  }
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
  }

  std::string taskFile;
  nodeHandle.getParam("/taskFile", taskFile);

  robotStatePublisherPtr_.reset(new robot_state_publisher::RobotStatePublisher(tree));
  robotStatePublisherPtr_->publishFixedTransforms(true);

  stateOptimizedPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/swerve/optimizedStateTrajectory", 1);
  stateOptimizedPosePublisher_ = nodeHandle.advertise<geometry_msgs::PoseArray>("/swerve/optimizedPoseTrajectory", 1);
}

void SwerveDummyVisualization::update(const SystemObservation& observation, const PrimalSolution& policy, const CommandData& command) {
  const ros::Time timeStamp = ros::Time::now();

  publishObservation(timeStamp, observation);
  // publishTargetTrajectories(timeStamp, command.mpcTargetTrajectories_);
  // publishOptimizedTrajectory(timeStamp, policy);
}

void SwerveDummyVisualization::publishObservation(const ros::Time& timeStamp, const SystemObservation& observation) {
  // publish world -> base transform
  const auto position = getBasePosition(observation.state);
  const auto orientation = getBaseOrientation(observation.state);

  geometry_msgs::TransformStamped base_tf;
  base_tf.header.stamp = timeStamp;
  base_tf.header.frame_id = "world";
  base_tf.child_frame_id = "base_link";
  base_tf.transform.translation = ros_msg_helpers::getVectorMsg(position);
  base_tf.transform.rotation = ros_msg_helpers::getOrientationMsg(orientation);
  tfBroadcaster_.sendTransform(base_tf);

  // publish joints transforms
  const auto jointsState = observation.state.tail(swerveModelInfo_.inputDim - 3);
  std::map<std::string, scalar_t> jointPositions{
      {"left_back_brake_joint", jointsState(0)},    {"left_back_steer_joint", jointsState(1)},
      {"left_back_wheel_joint", jointsState(2)},    {"left_front_brake_joint", jointsState(3)},
      {"left_front_steer_joint", jointsState(4)},   {"left_front_wheel_joint", jointsState(5)},
      {"right_back_brake_joint", jointsState(6)},   {"right_back_steer_joint", jointsState(7)},
      {"right_back_wheel_joint", jointsState(8)},   {"right_front_brake_joint", jointsState(9)},
      {"right_front_steer_joint", jointsState(10)}, {"right_front_wheel_joint", jointsState(11)}};
  if (swerveModelInfo_.armPresent) {
    jointPositions.insert({{"SH_ROT", jointsState(12)},
                           {"SH_FLE", jointsState(13)},
                           {"EL_FLE", jointsState(14)},
                           {"EL_ROT", jointsState(15)},
                           {"WR_FLE", jointsState(16)},
                           {"WR_ROT", jointsState(17)}});
  }
  robotStatePublisherPtr_->publishTransforms(jointPositions, timeStamp);
}

void SwerveDummyVisualization::publishTargetTrajectories(const ros::Time& timeStamp, const TargetTrajectories& targetTrajectories) {
  // publish command transform
  const Eigen::Vector3d eeDesiredPosition = targetTrajectories.stateTrajectory.back().head(3);
  Eigen::Quaterniond eeDesiredOrientation;
  eeDesiredOrientation.coeffs() = targetTrajectories.stateTrajectory.back().tail(8).head(4);
  geometry_msgs::TransformStamped command_tf;
  command_tf.header.stamp = timeStamp;
  command_tf.header.frame_id = "world";
  command_tf.child_frame_id = "command";
  command_tf.transform.translation = ros_msg_helpers::getVectorMsg(eeDesiredPosition);
  command_tf.transform.rotation = ros_msg_helpers::getOrientationMsg(eeDesiredOrientation);
  tfBroadcaster_.sendTransform(command_tf);
}

void SwerveDummyVisualization::publishOptimizedTrajectory(const ros::Time& timeStamp, const PrimalSolution& policy) {
  const scalar_t TRAJECTORYLINEWIDTH = 0.005;
  const std::array<scalar_t, 3> red{0.6350, 0.0780, 0.1840};
  const std::array<scalar_t, 3> blue{0, 0.4470, 0.7410};
  const auto& mpcStateTrajectory = policy.stateTrajectory_;

  visualization_msgs::MarkerArray markerArray;

  // Base trajectory
  std::vector<geometry_msgs::Point> baseTrajectory;
  baseTrajectory.reserve(mpcStateTrajectory.size());
  geometry_msgs::PoseArray poseArray;
  poseArray.poses.reserve(mpcStateTrajectory.size());

  // End effector trajectory
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();

  std::vector<geometry_msgs::Point> endEffectorTrajectory;
  endEffectorTrajectory.reserve(mpcStateTrajectory.size());
  std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const Eigen::VectorXd& state) {
    pinocchio::forwardKinematics(model, data, state);
    pinocchio::updateFramePlacements(model, data);
    const auto eeIndex = model.getBodyId("base");
    const vector_t eePosition = data.oMf[eeIndex].translation();
    endEffectorTrajectory.push_back(ros_msg_helpers::getPointMsg(eePosition));
  });

  markerArray.markers.emplace_back(ros_msg_helpers::getLineMsg(std::move(endEffectorTrajectory), blue, TRAJECTORYLINEWIDTH));
  markerArray.markers.back().ns = "EE Trajectory";

  // Extract base pose from state
  std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const vector_t& state) {
    geometry_msgs::Pose pose;
    pose.position = ros_msg_helpers::getPointMsg(getBasePosition(state));
    pose.orientation = ros_msg_helpers::getOrientationMsg(getBaseOrientation(state));
    baseTrajectory.push_back(pose.position);
    poseArray.poses.push_back(std::move(pose));
  });

  markerArray.markers.emplace_back(ros_msg_helpers::getLineMsg(std::move(baseTrajectory), red, TRAJECTORYLINEWIDTH));
  markerArray.markers.back().ns = "Base Trajectory";

  assignHeader(markerArray.markers.begin(), markerArray.markers.end(), ros_msg_helpers::getHeaderMsg("world", timeStamp));
  assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());
  poseArray.header = ros_msg_helpers::getHeaderMsg("world", timeStamp);

  stateOptimizedPublisher_.publish(markerArray);
  stateOptimizedPosePublisher_.publish(poseArray);
}

}  // namespace swerve
}  // namespace ocs2
