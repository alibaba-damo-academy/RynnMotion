#include "ros_mj_interface.hpp"

#include <thread>
#include "scene_manager.hpp"

namespace mujoco {

RosMujocoInterface::RosMujocoInterface(const YAML::Node &mujocoYaml, const YAML::Node &motionYaml, int robotType, int sceneNumber) :
    MujocoInterface(mujocoYaml, motionYaml, robotType, sceneNumber) {
  initROS();
  timingManager->addChannel("control", 500.0);
}

RosMujocoInterface::~RosMujocoInterface() {
  rclcpp::shutdown();
}

void RosMujocoInterface::initROS() {
  rclcpp::init(0, nullptr);
  rosNode_ = std::make_shared<rclcpp::Node>("ros_mujoco");
  rosNode_->set_parameter(rclcpp::Parameter("use_sim_time", true));
  rosClockPublisher_ = rosNode_->create_publisher<rosgraph_msgs::msg::Clock>("/mjRosClock", 10);
  rosRate_ = std::make_shared<rclcpp::Rate>(1000);
  auto qos = rclcpp::QoS(10).reliable();

  rynn::RobotType robotType = robotManager->getRobotType();

  switch (robotType) {
  case rynn::RobotType::dual_rm75: {
    leftArmPublisher_ = rosNode_->create_publisher<sensor_msgs::msg::JointState>(
        "/left_arm_mujoco/joint_states", qos);
    rightArmPublisher_ = rosNode_->create_publisher<sensor_msgs::msg::JointState>(
        "/right_arm_mujoco/joint_states", qos);
#ifdef USE_RM_INTERFACES
    leftArmSubscriber_ = rosNode_->create_subscription<rm_ros_interfaces::msg::Jointpos>(
        "/left_arm/rm_driver/movej_canfd_cmd", qos,
        std::bind(&RosMujocoInterface::leftArmCallback, this, std::placeholders::_1));
    rightArmSubscriber_ = rosNode_->create_subscription<rm_ros_interfaces::msg::Jointpos>(
        "/right_arm/rm_driver/movej_canfd_cmd", qos,
        std::bind(&RosMujocoInterface::rightArmCallback, this, std::placeholders::_1));
#endif
    leftArmState_.name.resize(7);
    leftArmState_.position.resize(7);
    rightArmState_.name.resize(7);
    rightArmState_.position.resize(7);
    std::vector<std::string> jointNames = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
    leftArmState_.name = jointNames;
    rightArmState_.name = jointNames;
    // RCLCPP_INFO(rosNode_->get_logger(), "Initialized ROS interface for dual_rm75 robot.");
    break;
  }

  case rynn::RobotType::diffmobile:
  case rynn::RobotType::mobile_fr3:
  case rynn::RobotType::diffcar: {
    lidarScanPublisher_ = rosNode_->create_publisher<sensor_msgs::msg::LaserScan>("scan", qos);
    odometryPublisher_ = rosNode_->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
    cmdVelSubscriber_ = rosNode_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", qos,
        std::bind(&RosMujocoInterface::chasisVelCallback, this, std::placeholders::_1));
    // RCLCPP_INFO(rosNode_->get_logger(), "Initialized ROS interface for diffMobile robot.");
    break;
  }

  case rynn::RobotType::rizon4s: {
#ifdef USE_USSCAN_MSGS
    // Only initialize usscan topics if scene is kUsscan
    if (sceneManager && sceneManager->getSceneType() == rynn::SceneType::kUsscan) {
      initUsscanROS();
    }
#endif
    break;
  }

  default:
    // RCLCPP_WARN(rosNode_->get_logger(), "Robot type not recognized. No ROS topics initialized.");
    break;
  }
}

#ifdef USE_RM_INTERFACES
// All rm_ros_interfaces related code grouped here
void RosMujocoInterface::leftArmCallback(const rm_ros_interfaces::msg::Jointpos::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mjMutex_);

  // Get current commands using batch API
  Eigen::VectorXd qCmd, qdCmd, qtauCmd;
  runtimeData_.getJointsCommand(qCmd, qdCmd, qtauCmd);

  // Update left arm commands (first 7 DOF)
  for (size_t i = 0; i < msg->joint.size() && i < qCmd.size(); ++i) {
    qCmd(i) = msg->joint[i];
  }

  // Set commands back using batch API
  runtimeData_.setJointsCommand(qCmd, qdCmd, qtauCmd);
  // _publishLeftArmStates();
}

void RosMujocoInterface::rightArmCallback(const rm_ros_interfaces::msg::Jointpos::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mjMutex_);

  // Get current commands using batch API
  Eigen::VectorXd qCmd, qdCmd, qtauCmd;
  runtimeData_.getJointsCommand(qCmd, qdCmd, qtauCmd);

  // Update right arm commands (DOF 7-13)
  for (size_t i = 0; i < msg->joint.size() && (i + 7) < qCmd.size(); ++i) {
    qCmd(i + 7) = msg->joint[i];
  }

  // Set commands back using batch API
  runtimeData_.setJointsCommand(qCmd, qdCmd, qtauCmd);
  // _publishRightArmStates();
}
#endif

void RosMujocoInterface::publishLeftArmStates() {
  leftArmState_.header.stamp = rosNode_->now();

  Eigen::VectorXd qFb, qdFb, qtauFb;
  runtimeData_.getJointsFeedback(qFb, qdFb, qtauFb);

  for (int i = 0; i < 7 && i < qFb.size(); ++i) {
    leftArmState_.position[i] = qFb(i);
  }
  leftArmPublisher_->publish(leftArmState_);
}

void RosMujocoInterface::publishRightArmStates() {
  rightArmState_.header.stamp = rosNode_->now();

  Eigen::VectorXd qFb, qdFb, qtauFb;
  runtimeData_.getJointsFeedback(qFb, qdFb, qtauFb);

  for (int i = 0; i < 7 && (i + 7) < qFb.size(); ++i) {
    rightArmState_.position[i] = qFb(i + 7);
  }
  rightArmPublisher_->publish(rightArmState_);
}

void RosMujocoInterface::publishLidarScan() {
  const std::vector<double> &ranges = multiRayDistances_;

  sensor_msgs::msg::LaserScan lidarScan;
  lidarScan.header.stamp = rosNode_->now();
  lidarScan.header.frame_id = "base_link";
  lidarScan.angle_min = -2.3562;
  lidarScan.angle_max = 2.3562;
  lidarScan.angle_increment = 0.017453;
  lidarScan.range_min = 0.1;
  lidarScan.range_max = 20;
  lidarScan.ranges.reserve(ranges.size());

  for (const auto &range : ranges) {
    if (range >= lidarScan.range_min && range <= lidarScan.range_max) {
      lidarScan.ranges.push_back(range);
    } else {
      lidarScan.ranges.push_back(1000.0);
    }
  }

  lidarScanPublisher_->publish(lidarScan);
}

void RosMujocoInterface::publishOdom() {
  nav_msgs::msg::Odometry odometry;
  odometry.header.frame_id = "map";

  int chasisIndex = 0;
  const auto &chassis = runtimeData_.frameSensor(chasisIndex);

  const Eigen::Vector3d &pos = chassis.pos;
  odometry.pose.pose.position.x = pos[0];
  odometry.pose.pose.position.y = pos[1];
  odometry.pose.pose.position.z = pos[2];

  odometry.pose.pose.orientation.x = chassis.quat.x();
  odometry.pose.pose.orientation.y = chassis.quat.y();
  odometry.pose.pose.orientation.z = chassis.quat.z();
  odometry.pose.pose.orientation.w = chassis.quat.w();

  const Eigen::Vector3d linVel = chassis.velocity.head<3>();
  const Eigen::Vector3d angVel = chassis.velocity.tail<3>();

  odometry.twist.twist.linear.x = linVel[0];
  odometry.twist.twist.linear.y = linVel[1];
  odometry.twist.twist.linear.z = linVel[2];

  odometry.twist.twist.angular.x = angVel[0];
  odometry.twist.twist.angular.y = angVel[1];
  odometry.twist.twist.angular.z = angVel[2];

  odometryPublisher_->publish(odometry);
}

void RosMujocoInterface::chasisVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mjMutex_);
  mjData_->ctrl[0] = msg->linear.x;
  mjData_->ctrl[1] = msg->linear.y;
}

void RosMujocoInterface::callController() {
  if (pauseSim_) return;
  getFeedbacks();

#ifdef USE_USSCAN_MSGS
  // Tick usscan FSM for rizon4s with usscan scene
  if (sceneManager && sceneManager->getSceneType() == rynn::SceneType::kUsscan) {
    float dt = 0.001f;  // 1kHz control loop
    tickUsscanFsm(dt);
  }
#endif

  if (timingManager->shouldTrigger("control", mjData_->time)) {
    setActuatorCommands();
  }
}

void RosMujocoInterface::resetController() {
  getFeedbacks();
  timingManager->reset("control", mjData_->time);
}

void RosMujocoInterface::keyboard(GLFWwindow *window, int key, int scancode, int action, int mods) {
  // Call base class implementation first
  MujocoInterface::keyboard(window, key, scancode, action, mods);

#ifdef USE_USSCAN_MSGS
  // Handle usscan-specific keys
  if (action == GLFW_PRESS && sceneManager && sceneManager->getSceneType() == rynn::SceneType::kUsscan) {
    handleUsscanKeyEvent(key);
  }
#endif
}

void RosMujocoInterface::runApplication() {
  std::thread physicsRosThread([this]() {
    rclcpp::Rate rate(1000);

    while (rclcpp::ok() && !exitRequest_.load() && !glfwWindowShouldClose(glfwWindow)) {
      if (pauseSim_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        rate.sleep();
        continue;
      }

      if (resetSim_) {
        resetRoutine();
        resetSim_ = false;
        continue;
      }

      {
        std::lock_guard<std::mutex> lock(mjMutex_);
        callController();
        stepPhysics();

        rosgraph_msgs::msg::Clock clockMsg;
        clockMsg.clock = rclcpp::Time(mjData_->time * 1e9);
        rosClockPublisher_->publish(clockMsg);

        publishTopics();
      }

      rclcpp::spin_some(rosNode_);

      rate.sleep();
    }
  });

  renderLoop();

  exitRequest_.store(true);
  physicsRosThread.join();
}

void RosMujocoInterface::publishTopics() {
  rynn::RobotType robotType = robotManager->getRobotType();

  switch (robotType) {
  case rynn::RobotType::dual_rm75:
    publishLeftArmStates();
    publishRightArmStates();
    break;
  case rynn::RobotType::diffmobile:
  case rynn::RobotType::mobile_fr3:
  case rynn::RobotType::diffcar:
    publishLidarScan();
    publishOdom();
    break;
  case rynn::RobotType::rizon4s:
#ifdef USE_USSCAN_MSGS
    if (sceneManager && sceneManager->getSceneType() == rynn::SceneType::kUsscan) {
      publishUsscanState();
      publishUsscanFeedback();
    }
#endif
    break;
  default:
    break;
  }
}

#ifdef USE_USSCAN_MSGS
void RosMujocoInterface::initUsscanROS() {
  auto qos = rclcpp::QoS(10).reliable();

  // Publishers: motion state and feedback to nav_node
  usscanStatePublisher_ = rosNode_->create_publisher<usscan_msgs::msg::UsscanState>(
      "/usscan/motion_state", qos);
  usscanFeedbackPublisher_ = rosNode_->create_publisher<usscan_msgs::msg::UsscanFeedback>(
      "/usscan/motion_feedback", qos);

  // Subscribers: receive nav state and trajectory from nav_node
  navStateSubscriber_ = rosNode_->create_subscription<usscan_msgs::msg::UsscanState>(
      "/usscan/nav_state", qos,
      std::bind(&RosMujocoInterface::navStateCallback, this, std::placeholders::_1));
  trajectorySubscriber_ = rosNode_->create_subscription<usscan_msgs::msg::ProbeTrajectoryProposal>(
      "/usscan/probe_trajectory", qos,
      std::bind(&RosMujocoInterface::trajectoryCallback, this, std::placeholders::_1));

  // Initialize FSM
  usscan::UsscanMotionFsm::start();
  usscan::UsscanMotionFsm::reset();

  // Initialize trajectory generators (1kHz control loop)
  usscanTrajGen_ = std::make_unique<EEPoseTrajGen>(0.001);
  usscanTrajGen_->setTcpLimits(0.1, 0.5);  // 0.1 m/s max speed, 0.5 m/s² max acc

  usscanJointTrajGen_ = std::make_unique<JointTrajGen<7>>(0.001);
  Eigen::VectorXd qdMax = Eigen::VectorXd::Constant(7, 1.0);  // 1 rad/s max joint speed
  Eigen::VectorXd qddMax = Eigen::VectorXd::Constant(7, 2.0); // 2 rad/s² max joint acc
  usscanJointTrajGen_->setJointMotionLimits(qdMax, qddMax);

  // Set home joint position for rizon4s (typical standby pose)
  homeJointPos_ = Eigen::VectorXd::Zero(7);
  homeJointPos_ << 0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854;  // Similar to Franka home

  usscanInitialized_ = true;

  RCLCPP_INFO(rosNode_->get_logger(), "Initialized usscan ROS interface for rizon4s with FSM");
}

void RosMujocoInterface::publishUsscanState() {
  usscan_msgs::msg::UsscanState stateMsg;
  stateMsg.stamp = rosNode_->now();
  stateMsg.state_id = motionStateId_;
  stateMsg.state_time = motionStateTime_;
  stateMsg.total_time = motionTotalTime_;
  usscanStatePublisher_->publish(stateMsg);
}

void RosMujocoInterface::publishUsscanFeedback() {
  usscan_msgs::msg::UsscanFeedback feedbackMsg;
  feedbackMsg.stamp = rosNode_->now();

  // Get joint feedback from dataStream
  Eigen::VectorXd qFb, qdFb, qtauFb;
  runtimeData_.getJointsFeedback(qFb, qdFb, qtauFb);

  // Fill joint data (7 DOF for rizon4s) - fixed size arrays
  for (int i = 0; i < 7 && i < qFb.size(); ++i) {
    feedbackMsg.joint_position[i] = qFb(i);
    feedbackMsg.joint_velocity[i] = qdFb(i);
    feedbackMsg.joint_torque[i] = qtauFb(i);
  }

  // Get end-effector pose from frame sensor (index 0)
  const auto &eePose = runtimeData_.frameSensor(0);

  // EE position (array[3])
  feedbackMsg.ee_position[0] = eePose.pos[0];
  feedbackMsg.ee_position[1] = eePose.pos[1];
  feedbackMsg.ee_position[2] = eePose.pos[2];

  // EE orientation as quaternion (array[4]: x, y, z, w)
  feedbackMsg.ee_orientation[0] = eePose.quat.x();
  feedbackMsg.ee_orientation[1] = eePose.quat.y();
  feedbackMsg.ee_orientation[2] = eePose.quat.z();
  feedbackMsg.ee_orientation[3] = eePose.quat.w();

  // EE velocity (array[6]: linear xyz + angular xyz)
  if (eePose.velocity.size() >= 6) {
    for (int i = 0; i < 6; ++i) {
      feedbackMsg.ee_velocity[i] = eePose.velocity[i];
    }
  }

  // F/T sensor data
  if (!runtimeData_.ftSensors.empty()) {
    const auto &ft = runtimeData_.ftSensors[0];

    // Force (array[3])
    feedbackMsg.ft_force[0] = ft.force[0];
    feedbackMsg.ft_force[1] = ft.force[1];
    feedbackMsg.ft_force[2] = ft.force[2];

    // Torque (array[3])
    feedbackMsg.ft_torque[0] = ft.torque[0];
    feedbackMsg.ft_torque[1] = ft.torque[1];
    feedbackMsg.ft_torque[2] = ft.torque[2];

    // Contact status
    double forceNorm = ft.force.norm();
    feedbackMsg.in_contact = (forceNorm > 5.0);  // 5N threshold
    feedbackMsg.contact_force = forceNorm;
  } else {
    // No F/T sensor, set defaults
    feedbackMsg.ft_force.fill(0.0);
    feedbackMsg.ft_torque.fill(0.0);
    feedbackMsg.in_contact = false;
    feedbackMsg.contact_force = 0.0;
  }

  usscanFeedbackPublisher_->publish(feedbackMsg);
}

void RosMujocoInterface::navStateCallback(const usscan_msgs::msg::UsscanState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mjMutex_);
  int prevNavState = navStateId_;
  navStateId_ = msg->state_id;

  // Process nav state changes and dispatch corresponding FSM events
  processNavStateEvent(msg->state_id);

  if (prevNavState != navStateId_) {
    RCLCPP_INFO(rosNode_->get_logger(), "Nav state changed: %d -> %d", prevNavState, navStateId_);
  }
}

void RosMujocoInterface::processNavStateEvent(int navStateId) {
  // Map nav state IDs to FSM events
  // Nav states: S0=Init, S1=Ready, S2=Contact, S3=IqSearch, S4=Bestview, S5=Scanning, S6=Revisit, S7=Done
  // Nav-triggered events:
  //   S3→S4: EventIqConverged (nav finished IQ optimization)
  //   S4→S5: EventAtBestview (nav ready to start scanning)
  //   S5→S6: EventScanDoneWithLesions
  //   S5→S7: EventScanDoneNoLesions
  //   S6→S7: EventQueueEmpty

  auto currentMotionState = usscan::UsscanMotionFsm::currentStateID;

  switch (navStateId) {
  case 4: // Nav reached S4 (Bestview ready) - motion should be in M3
    if (currentMotionState == usscan::MotionStateID::M3_IQ_SEARCH) {
      RCLCPP_INFO(rosNode_->get_logger(), "Nav signaled IQ converged, dispatching EventIqConverged");
      usscan::UsscanMotionFsm::dispatch(usscan::EventIqConverged{});
    }
    break;

  case 5: // Nav reached S5 (Scanning) - motion should be in M4
    if (currentMotionState == usscan::MotionStateID::M4_MOVETO_BESTVIEW) {
      RCLCPP_INFO(rosNode_->get_logger(), "Nav signaled at bestview, dispatching EventAtBestview");
      usscan::UsscanMotionFsm::dispatch(usscan::EventAtBestview{});
    }
    break;

  case 6: // Nav reached S6 (Lesion revisit) - scan done with lesions
    if (currentMotionState == usscan::MotionStateID::M5_STANDARD_PLANE_SCAN) {
      RCLCPP_INFO(rosNode_->get_logger(), "Nav signaled scan done with lesions, dispatching EventScanDoneWithLesions");
      usscan::UsscanMotionFsm::dispatch(usscan::EventScanDoneWithLesions{});
    }
    break;

  case 7: // Nav reached S7 (Done)
    if (currentMotionState == usscan::MotionStateID::M5_STANDARD_PLANE_SCAN) {
      RCLCPP_INFO(rosNode_->get_logger(), "Nav signaled scan done no lesions, dispatching EventScanDoneNoLesions");
      usscan::UsscanMotionFsm::dispatch(usscan::EventScanDoneNoLesions{});
    } else if (currentMotionState == usscan::MotionStateID::M6_LESION_REVISIT_FIFO) {
      RCLCPP_INFO(rosNode_->get_logger(), "Nav signaled queue empty, dispatching EventQueueEmpty");
      usscan::UsscanMotionFsm::dispatch(usscan::EventQueueEmpty{});
    }
    break;

  default:
    break;
  }
}

void RosMujocoInterface::trajectoryCallback(const usscan_msgs::msg::ProbeTrajectoryProposal::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(trajectoryQueueMutex_);
  trajectoryQueue_.push(*msg);
  RCLCPP_INFO(rosNode_->get_logger(), "Queued trajectory with %zu points (queue size: %zu)",
              msg->points.size(), trajectoryQueue_.size());
}

void RosMujocoInterface::tickUsscanFsm(float dt) {
  if (!usscanInitialized_) return;

  // Tick FSM (updates state time)
  usscan::UsscanMotionFsm::dispatch(usscan::EventTick{dt});

  // Execute state-specific motion
  executeCurrentState();

  // Update state for publishing
  motionStateId_ = static_cast<uint8_t>(usscan::UsscanMotionFsm::currentStateID);
  motionStateTime_ = usscan::UsscanMotionFsm::stateTime;
  motionTotalTime_ = usscan::UsscanMotionFsm::totalTime;
}

void RosMujocoInterface::executeCurrentState() {
  auto currentState = usscan::UsscanMotionFsm::currentStateID;
  bool onEntry = usscan::UsscanMotionFsm::onEntry;

  // Get current joint feedback
  Eigen::VectorXd qFb, qdFb, qtauFb;
  runtimeData_.getJointsFeedback(qFb, qdFb, qtauFb);

  // Get current EE pose from frame sensor
  const auto &eePose = runtimeData_.frameSensor(0);
  Eigen::Vector3d eePos = eePose.pos;
  Eigen::Quaterniond eeQuat = eePose.quat;

  switch (currentState) {
  case usscan::MotionStateID::M0_INIT_STANDBY: {
    // Move to home joint position using joint trajectory generator
    if (onEntry) {
      // Initialize trajectory from current position to home
      usscanJointTrajGen_->setStartState(qFb, qdFb);
      usscanJointTrajGen_->setTargetState(homeJointPos_);
      RCLCPP_INFO(rosNode_->get_logger(), "M0: Moving to home position");
    }

    auto result = usscanJointTrajGen_->update();
    usscanJointTrajGen_->passToInput();

    Eigen::VectorXd qCmd = usscanJointTrajGen_->getCurrPos();
    Eigen::VectorXd qdCmd = usscanJointTrajGen_->getCurrVel();
    runtimeData_.setJointsCommand(qCmd, qdCmd, Eigen::VectorXd::Zero(7));

    // Check if we reached home position
    if (result == ruckig::Result::Finished) {
      double posError = (qFb - homeJointPos_).norm();
      if (posError < 0.01) {  // 0.01 rad tolerance
        RCLCPP_INFO(rosNode_->get_logger(), "M0: Reached home, dispatching EventAtStandby");
        usscan::UsscanMotionFsm::dispatch(usscan::EventAtStandby{});
      }
    }
    break;
  }

  case usscan::MotionStateID::M1_HAND_GUIDE: {
    // Low-impedance floating mode: track current position (no active motion)
    // Robot stays at current position, user can manually guide
    if (onEntry) {
      RCLCPP_INFO(rosNode_->get_logger(), "M1: Hand-guide mode - press 's' to start approach");
    }
    // Hold current position
    runtimeData_.setJointsCommand(qFb, Eigen::VectorXd::Zero(7), Eigen::VectorXd::Zero(7));
    break;
  }

  case usscan::MotionStateID::M2_APPROACH: {
    // Slow descent in -Z direction until contact detected
    if (onEntry) {
      approachStartPos_ = eePos;
      usscanTrajGen_->setStartState(eePos, eeQuat);
      RCLCPP_INFO(rosNode_->get_logger(), "M2: Starting approach descent");
    }

    // Target: descend continuously
    Eigen::Vector3d targetPos = approachStartPos_;
    targetPos.z() -= approachSpeed_ * usscan::UsscanMotionFsm::stateTime;

    usscanTrajGen_->setTargetState(targetPos, eeQuat);
    usscanTrajGen_->update();
    usscanTrajGen_->passToInput();

    auto [cmdPos, cmdQuat] = usscanTrajGen_->getCurrPose();

    // For MuJoCo simulation, we need to convert EE pose to joint commands
    // For now, use Cartesian impedance-like behavior by holding the target
    // The actual IK would be handled by a motion controller
    runtimeData_.setJointsCommand(qFb, Eigen::VectorXd::Zero(7), Eigen::VectorXd::Zero(7));

    // Check for contact
    if (checkContactDetected()) {
      RCLCPP_INFO(rosNode_->get_logger(), "M2: Contact detected, dispatching EventContactDetected");
      usscan::UsscanMotionFsm::dispatch(usscan::EventContactDetected{});
    }
    break;
  }

  case usscan::MotionStateID::M3_IQ_SEARCH:
  case usscan::MotionStateID::M4_MOVETO_BESTVIEW:
  case usscan::MotionStateID::M5_STANDARD_PLANE_SCAN:
  case usscan::MotionStateID::M6_LESION_REVISIT_FIFO: {
    // Execute trajectories from queue
    if (onEntry) {
      trajectoryActive_ = false;
      currentWaypointIndex_ = 0;
      RCLCPP_INFO(rosNode_->get_logger(), "State %d: Ready to execute trajectories",
                  static_cast<int>(currentState));
    }

    // Check if we have a trajectory to execute
    if (!trajectoryActive_) {
      std::lock_guard<std::mutex> lock(trajectoryQueueMutex_);
      if (!trajectoryQueue_.empty()) {
        // Start new trajectory
        trajectoryActive_ = true;
        currentWaypointIndex_ = 0;
        RCLCPP_INFO(rosNode_->get_logger(), "Starting trajectory execution");
      }
    }

    if (trajectoryActive_) {
      std::lock_guard<std::mutex> lock(trajectoryQueueMutex_);
      if (!trajectoryQueue_.empty()) {
        auto &traj = trajectoryQueue_.front();
        if (currentWaypointIndex_ < traj.points.size()) {
          auto &wp = traj.points[currentWaypointIndex_];
          Eigen::Vector3d targetPos(wp.pose.position.x, wp.pose.position.y, wp.pose.position.z);
          Eigen::Quaterniond targetQuat(wp.pose.orientation.w, wp.pose.orientation.x,
                                        wp.pose.orientation.y, wp.pose.orientation.z);

          usscanTrajGen_->setStartState(eePos, eeQuat);
          usscanTrajGen_->setTargetState(targetPos, targetQuat);
          auto result = usscanTrajGen_->update();
          usscanTrajGen_->passToInput();

          if (result == ruckig::Result::Finished) {
            currentWaypointIndex_++;
            RCLCPP_DEBUG(rosNode_->get_logger(), "Reached waypoint %zu/%zu",
                        currentWaypointIndex_, traj.points.size());
          }
        } else {
          // Trajectory complete
          trajectoryQueue_.pop();
          trajectoryActive_ = false;
          RCLCPP_INFO(rosNode_->get_logger(), "Trajectory complete, queue size: %zu",
                      trajectoryQueue_.size());
        }
      }
    }

    // Hold current position if no trajectory
    runtimeData_.setJointsCommand(qFb, Eigen::VectorXd::Zero(7), Eigen::VectorXd::Zero(7));
    break;
  }

  case usscan::MotionStateID::M7_DISENGAGE_AND_RETURN: {
    // Return to home position
    if (onEntry) {
      usscanJointTrajGen_->setStartState(qFb, qdFb);
      usscanJointTrajGen_->setTargetState(homeJointPos_);
      RCLCPP_INFO(rosNode_->get_logger(), "M7: Returning to home position");
    }

    auto result = usscanJointTrajGen_->update();
    usscanJointTrajGen_->passToInput();

    Eigen::VectorXd qCmd = usscanJointTrajGen_->getCurrPos();
    Eigen::VectorXd qdCmd = usscanJointTrajGen_->getCurrVel();
    runtimeData_.setJointsCommand(qCmd, qdCmd, Eigen::VectorXd::Zero(7));

    // Check if we reached home position
    if (result == ruckig::Result::Finished) {
      double posError = (qFb - homeJointPos_).norm();
      if (posError < 0.01) {
        RCLCPP_INFO(rosNode_->get_logger(), "M7: Reached home, dispatching EventReachStandby");
        usscan::UsscanMotionFsm::dispatch(usscan::EventReachStandby{});
      }
    }
    break;
  }

  case usscan::MotionStateID::M8_FAULT_OR_RECOVERY: {
    // Stop all motion, hold current position
    if (onEntry) {
      RCLCPP_WARN(rosNode_->get_logger(), "M8: Fault/Recovery state - motion stopped");
    }
    runtimeData_.setJointsCommand(qFb, Eigen::VectorXd::Zero(7), Eigen::VectorXd::Zero(7));
    break;
  }

  default:
    break;
  }
}

bool RosMujocoInterface::checkContactDetected() {
  if (runtimeData_.ftSensors.empty()) return false;

  const auto &ft = runtimeData_.ftSensors[0];
  double forceNorm = ft.force.norm();

  // Contact threshold: 5N
  return forceNorm > 5.0;
}

void RosMujocoInterface::handleUsscanKeyEvent(int key) {
  if (!usscanInitialized_) return;

  auto currentState = usscan::UsscanMotionFsm::currentStateID;

  switch (key) {
  case GLFW_KEY_S:
    // Operator confirm: M1 -> M2
    if (currentState == usscan::MotionStateID::M1_HAND_GUIDE) {
      RCLCPP_INFO(rosNode_->get_logger(), "Key 's': Operator confirm, dispatching EventOperatorConfirm");
      usscan::UsscanMotionFsm::dispatch(usscan::EventOperatorConfirm{});
    }
    break;

  case GLFW_KEY_R:
    // Go to standby from any state
    RCLCPP_INFO(rosNode_->get_logger(), "Key 'r': Go to standby, dispatching EventGoToStandby");
    usscan::UsscanMotionFsm::dispatch(usscan::EventGoToStandby{});
    break;

  case GLFW_KEY_E:
    // Fault/Emergency stop
    RCLCPP_WARN(rosNode_->get_logger(), "Key 'e': Fault triggered, dispatching EventFault");
    usscan::UsscanMotionFsm::dispatch(usscan::EventFault{});
    break;

  default:
    break;
  }
}
#endif

} // namespace mujoco
