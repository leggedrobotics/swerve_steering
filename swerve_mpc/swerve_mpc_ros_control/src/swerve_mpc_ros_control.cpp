#include "swerve_mpc_ros_control/swerve_mpc_ros_control.hpp"

namespace swerve_mpc_ros_control {

bool SwerveMpcRosControl::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
  // Controller initialization
  typedef hardware_interface::VelocityJointInterface VelIface;

  const std::string complete_ns = controller_nh.getNamespace();

  std::size_t id = complete_ns.find_last_of("/");
  std::string name_ = complete_ns.substr(id + 1);

  // MPC Initialization
  firstRun_ = true;

  // Get node parameters
  std::string taskFile, libFolder, urdfString;
  controller_nh.getParam("/taskFile", taskFile);
  controller_nh.getParam("/libFolder", libFolder);
  controller_nh.getParam("/swerve_mpc/robot_description", urdfString);

  // Robot interface
  swerveInterface_.reset(new SwerveInterface(taskFile, libFolder, urdfString, true));

  swerveModelInfo_ = swerveInterface_->getSwerveModelInfo();

  mpcInterface_.reset(new MRT_ROS_Interface("swerve"));
  mpcInterface_->initRollout(&swerveInterface_->getRollout());
  mpcInterface_->launchNodes(root_nh);

  // initial state
  observation_.state = swerveInterface_->getInitialState();
  observation_.input.setZero(swerveModelInfo_.inputDim);
  observation_.time = ros::Time().now().toSec();

  currObservation_ = observation_;

  realtimeLoop_ = swerveInterface_->mpcSettings().mpcDesiredFrequency_ <= 0;
  frequencyRatio_ =
      static_cast<size_t>(swerveInterface_->mpcSettings().mrtDesiredFrequency_ / swerveInterface_->mpcSettings().mpcDesiredFrequency_);
  timeStep_ = 1.0 / swerveInterface_->mpcSettings().mrtDesiredFrequency_;

  mpcUpdateTimer_.stop();

  ddr_.reset(new ddynamic_reconfigure::DDynamicReconfigure(controller_nh));
  ddr_->registerVariable("brakes_p_gain", &brakesPGain_, "P gain for velocity controlled brake joint", 0.0, 1000.0);
  ddr_->publishServicesTopics();

  // Initialize SwerveTarget
  swerveTarget_.reset(new SwerveTarget(root_nh, swerveModelInfo_, "odom"));
  if (swerveModelInfo_.armPresent) {
    swerveTarget_->initInterectiveMarker();
  } else {
    swerveTarget_->initSetGoalPose();
  }

  std::string odomTopic = controller_nh.param<std::string>("odom_topic", "odometry_controller/odom");
  ROS_INFO_STREAM("MPC odom topic: " << odomTopic);
  odometrySub_ = root_nh.subscribe(odomTopic, 10, &SwerveMpcRosControl::odomCallback, this);

  // Initialize tf listener
  listener_.reset(new tf::TransformListener);

  // Set brakes variables to zero
  currBrakePos_ = Eigen::Vector4d::Zero();
  desiredBrakePos_ = Eigen::Vector4d::Zero();

  // Load joints names
  std::vector<std::string> steerJointNames = controller_nh.param("steer_joint_names", std::vector<std::string>());
  std::vector<std::string> wheelJointNames = controller_nh.param("wheel_joint_names", std::vector<std::string>());
  std::vector<std::string> brakeJointNames = controller_nh.param("brake_joint_names", std::vector<std::string>());
  std::vector<std::string> armJointNames = controller_nh.param("arm_joint_names", std::vector<std::string>());

  for (const auto& jointName : steerJointNames) {
    steerJoints_.push_back(hw->getHandle(jointName));
    ROS_INFO_STREAM_NAMED(name_, "Adding steering velocity joint: " << jointName);
  }

  for (const auto& jointName : wheelJointNames) {
    wheelJoints_.push_back(hw->getHandle(jointName));
    ROS_INFO_STREAM_NAMED(name_, "Adding wheel velocity joint: " << jointName);
  }

  for (const auto& jointName : brakeJointNames) {
    brakeJoints_.push_back(hw->getHandle(jointName));
    ROS_INFO_STREAM_NAMED(name_, "Adding brake velocity joint: " << jointName);
  }

  if (swerveModelInfo_.armPresent) {
    for (const auto& jointName : armJointNames) {
      armJoints_.push_back(hw->getHandle(jointName));
      ROS_INFO_STREAM_NAMED(name_, "Adding arm velocity joint: " << jointName);
    }
  }

  optimalBasePathPublisher_ = controller_nh.advertise<nav_msgs::Path>("optimal_base_path", 1, false);
  rosDebugInfoTimer_ = controller_nh.createTimer(ros::Duration(0.1), [this](const auto &){
    if (mpcInterface_->initialPolicyReceived()){
      auto policy = mpcInterface_->getPolicy();
      nav_msgs::Path path;
      path.header.stamp = ros::Time::now();
      path.header.frame_id = "odom"; // TODO: do not hardcode this!

      for (size_t i = 0; i < policy.timeTrajectory_.size(); ++i){
        geometry_msgs::PoseStamped ps;
        ps.header.stamp = ros::Time(policy.timeTrajectory_[i]);
        ps.header.frame_id = path.header.frame_id;
        Eigen::VectorXd mpcState = policy.stateTrajectory_[i];

        ps.pose.position.x = mpcState[x_state_ind];
        ps.pose.position.y = mpcState[y_state_ind];
        ps.pose.position.z = mpcState[z_state_ind];

        ps.pose.orientation.x = mpcState[x_quat_state_ind];
        ps.pose.orientation.y = mpcState[y_quat_state_ind];
        ps.pose.orientation.z = mpcState[z_quat_state_ind];
        ps.pose.orientation.w = mpcState[w_quat_state_ind];

        path.poses.push_back(ps);
      }

      optimalBasePathPublisher_.publish(path);
    }
        
  });

  ROS_INFO_STREAM_NAMED(name_, "Finished controller initialization");

  return true;
}

void SwerveMpcRosControl::update(const ros::Time& time, const ros::Duration& period) {
  // Update the current observation
  currObservation_.time = ros::Time().now().toSec();
  {
    std::lock_guard<std::mutex> lockGuard(updateOdomMutex_);

    // Write current odometry
    currObservation_.state(x_state_ind) = currentObservedPose_.position.x;
    currObservation_.state(y_state_ind) = currentObservedPose_.position.y;
    currObservation_.state(z_state_ind) = currentObservedPose_.position.z;

    currObservation_.state(x_quat_state_ind) = currentObservedPose_.orientation.x;
    currObservation_.state(y_quat_state_ind) = currentObservedPose_.orientation.y;
    currObservation_.state(z_quat_state_ind) = currentObservedPose_.orientation.z;
    currObservation_.state(w_quat_state_ind) = currentObservedPose_.orientation.w;
  }

  // Set observations
  for (size_t i = 0; i < steerJoints_.size(); ++i) {
    currObservation_.state(modelSettings_.steersStateIndex[i]) = steerJoints_[i].getPosition();
  }
  for (size_t i = 0; i < wheelJoints_.size(); ++i) {
    currObservation_.state(modelSettings_.wheelsStateIndex[i]) = wheelJoints_[i].getPosition();
  }
  for (size_t i = 0; i < brakeJoints_.size(); ++i) {
    currObservation_.state(modelSettings_.brakesStateIndex[i]) = brakeJoints_[i].getPosition();
    currBrakePos_[i] = brakeJoints_[i].getPosition();
  }

  if (swerveModelInfo_.armPresent) {
    for (size_t i = 0; i < armJoints_.size(); ++i) {
      currObservation_.state(modelSettings_.armJointsStateIndex[i]) = armJoints_[i].getPosition();
    }
  }

  // Update MRT
  mrtUpdate();

  RobotInputs currInputs = *(robotInputs_.readFromRT());

  // Set the speed commands
  for (size_t i = 0; i < steerJoints_.size(); ++i) {
    steerJoints_[i].setCommand(currInputs.steersInput(i));
  }

  // Set the speed commands
  for (size_t i = 0; i < wheelJoints_.size(); ++i) {
    wheelJoints_[i].setCommand(currInputs.wheelsInput(i));
  }

  desiredBrakePos_ = swerveTarget_->getDesiredBrakesPosition();

  // Calculate velocity command for brakes when fixed (This is only useful in GAZEBO)
  Eigen::Vector4d velocityCommandsBrakes = (desiredBrakePos_ - currBrakePos_) * brakesPGain_;

  // Set the speed commands
  for (size_t i = 0; i < brakeJoints_.size(); ++i) {
    if (swerveTarget_->getBrakeState(i)) {
      brakeJoints_[i].setCommand(0.0);  // For GAZEBO
    } else {
      brakeJoints_[i].setCommand(velocityCommandsBrakes[i]);  // For GAZEBO
    }
  }

  if (swerveModelInfo_.armPresent) {
    for (size_t i = 0; i < armJoints_.size(); ++i) {
      armJoints_[i].setCommand(currInputs.armInputs(i));
    }
  }
}

void SwerveMpcRosControl::mrtUpdate() {
  if (firstRun_) {
    try {
      listener_->lookupTransform("odom", swerveModelInfo_.eeFrame, ros::Time(0), ee_transform_);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
    firstRun_ = false;
    // initial command
    vector_t initTarget(11);
    initTarget.head(3) << ee_transform_.getOrigin().x(), ee_transform_.getOrigin().y(), ee_transform_.getOrigin().z();
    initTarget.tail(8).head(4) << ee_transform_.getRotation().x(), ee_transform_.getRotation().y(), ee_transform_.getRotation().z(),
        ee_transform_.getRotation().w();

    initTarget.tail(4) << vector_t::Zero(4);
    const vector_t zeroInput = vector_t::Zero(swerveModelInfo_.inputDim);
    const TargetTrajectories initTargetTrajectories({observation_.time}, {initTarget}, {zeroInput});
    loopCounter_ = 0;

    mpcInterface_->resetMpcNode(initTargetTrajectories);
  }

  if (mpcInterface_->initialPolicyReceived()) {
    // this should be called before updatePolicy()
    mpcInterface_->spinMRT();

    // Checks for new policy and updates the policy

    policyUpdated = mpcInterface_->updatePolicy();
    vector_t optimalState = vector_t().Zero(swerveModelInfo_.stateDim);
    vector_t optimalInput = vector_t().Zero(swerveModelInfo_.inputDim);

    {
      std::lock_guard<std::mutex> lockGuard(updatePolicyMutex_);
      mpcInterface_->evaluatePolicy(ros::Time().now().toSec(), currObservation_.state, optimalState, optimalInput, observation_.mode);
    }

    // time and loop counter increment
    loopCounter_++;
    observation_.time = ros::Time().now().toSec();

    // publish observation
    {
      std::lock_guard<std::mutex> lockGuard(updatePolicyMutex_);
      mpcInterface_->setCurrentObservation(currObservation_);
    }

    SwerveMpcRosControl::mapInputs(optimalInput);

  } else {
    ROS_INFO_STREAM("Waiting for the initial policy ...");
    mpcInterface_->spinMRT();
    // for initial plan
    {
      std::lock_guard<std::mutex> lockGuard(updatePolicyMutex_);
      mpcInterface_->setCurrentObservation(currObservation_);
    }
  }
}

void SwerveMpcRosControl::mapInputs(vector_t optimalInput) {
  RobotInputs tempInputs;
  // Write inputs
  for (int i = 0; i < tempInputs.steersInput.size(); ++i) {
    tempInputs.steersInput(i) = optimalInput(modelSettings_.steersInputIndex[i]);
  }

  for (int i = 0; i < tempInputs.wheelsInput.size(); ++i) {
    tempInputs.wheelsInput(i) = optimalInput(modelSettings_.wheelsInputIndex[i]);
  }

  for (int i = 0; i < tempInputs.brakesInput.size(); ++i) {
    tempInputs.brakesInput(i) = optimalInput(modelSettings_.brakesInputIndex[i]);
  }

  if (swerveModelInfo_.armPresent) {
    for (int i = 0; i < tempInputs.wheelsInput.size(); ++i) {
      tempInputs.armInputs(i) = optimalInput(modelSettings_.armJointsInputIndex[i]);
    }
  }

  robotInputs_.writeFromNonRT(tempInputs);
}

void SwerveMpcRosControl::odomCallback(const nav_msgs::OdometryPtr& msg) {
  std::lock_guard<std::mutex> lockGuard(updateOdomMutex_);
  currentObservedPose_ = msg->pose.pose;
}

PLUGINLIB_EXPORT_CLASS(swerve_mpc_ros_control::SwerveMpcRosControl, controller_interface::ControllerBase)

}  // namespace swerve_mpc_ros_control
