#pragma once

#include <controller_interface/controller.h>
#include "swerve_mpc/SwerveTarget.hpp"

#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include "nav_msgs/Odometry.h"
#include "swerve_mpc/SwerveInterface.hpp"
#include "swerve_mpc/SwerveModelInfo.hpp"

#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/package.h>

#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <nav_msgs/Path.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesKeyboardPublisher.h>

#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include "swerve_mpc/reference_manager/SwerveReferenceManager.hpp"

#include <tf/transform_listener.h>
#include <memory>
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "tf/transform_datatypes.h"

#include <ros/ros.h>

using namespace ocs2;
using namespace swerve;

namespace swerve_mpc_ros_control {

class SwerveMpcRosControl : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
 public:
  SwerveMpcRosControl(){};

  /**
   * \brief Initialize controller
   * \param hw            Velocity joint interface for the wheels
   * \param root_nh       Node handle at root namespace
   * \param controller_nh Node handle inside the controller namespace
   */
  bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

  /**
   * \brief Updates controller and sets the new velocity commands
   * \param time   Current time
   * \param period Time since the last called to update
   */
  void update(const ros::Time& time, const ros::Duration& period);

 protected:
  std::unique_ptr<SwerveInterface> swerveInterface_;
  std::shared_ptr<MRT_ROS_Interface> mpcInterface_;
  ros::Timer mpcUpdateTimer_;

 private:
  void mrtUpdate();
  void mapInputs(vector_t optimalInput);
  void odomCallback(const nav_msgs::OdometryPtr& msg);
  void missionPathCallback(const nav_msgs::PathPtr& msg);
  void adjustPathTimeStamps(nav_msgs::Path& path);

  SystemObservation observation_;
  bool firstRun_;
  bool realtimeLoop_ = false;
  unsigned int loopCounter_ = 0;
  int frequencyRatio_;
  bool policyUpdated = false;
  bool init_ = false;
  scalar_t timeStep_ = 0;
  SystemObservation currObservation_;
  ros::Subscriber odometrySub_;
  vector_t initTarget_;

  std::shared_ptr<SwerveReferenceManager> referenceManagerPtr_;

  std::unique_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr_;
  double brakesPGain_ = 100.0;

  std::mutex updatePolicyMutex_;
  std::mutex updateOdomMutex_;

  /// Hardware handles
  std::vector<hardware_interface::JointHandle> steerJoints_;
  std::vector<hardware_interface::JointHandle> wheelJoints_;
  std::vector<hardware_interface::JointHandle> armJoints_;
  std::vector<hardware_interface::JointHandle> brakeJoints_;


  geometry_msgs::Pose currentObservedPose_;
  struct RobotInputs {
    Eigen::Vector4d wheelsInput;
    Eigen::Vector4d steersInput;
    Eigen::VectorXd armInputs;
    Eigen::Vector4d brakesInput;
    ros::Time stamp;

    RobotInputs()
        : wheelsInput(Eigen::Vector4d::Zero()),
          steersInput(Eigen::Vector4d::Zero()),
          armInputs(Eigen::VectorXd::Zero(6)),
          brakesInput(Eigen::Vector4d::Zero()),
          stamp(0.0) {}
  };
  realtime_tools::RealtimeBuffer<RobotInputs> robotInputs_;

  ModelSettings modelSettings_;

  Eigen::Vector4d currBrakePos_;
  Eigen::Vector4d desiredBrakePos_;
  std::unique_ptr<tf::TransformListener> listener_;
  tf::StampedTransform ee_transform_;

  std::unique_ptr<SwerveTarget> swerveTarget_;
  SwerveModelInfo swerveModelInfo_;

  ros::Publisher optimalBasePathPublisher_;
  ros::Timer rosDebugInfoTimer_;
};

}  // namespace swerve_mpc_ros_control
