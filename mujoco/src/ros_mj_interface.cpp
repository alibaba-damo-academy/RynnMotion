#include "ros_mj_interface.hpp"

#include <thread>

namespace mujoco {

RosMujocoInterface::RosMujocoInterface(const YAML::Node &mujocoYaml, const YAML::Node &motionYaml, int robotType) :
    MujocoInterface(mujocoYaml, motionYaml, robotType) {
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
  dataStream->getJointsCommand(qCmd, qdCmd, qtauCmd);

  // Update left arm commands (first 7 DOF)
  for (size_t i = 0; i < msg->joint.size() && i < qCmd.size(); ++i) {
    qCmd(i) = msg->joint[i];
  }

  // Set commands back using batch API
  dataStream->setJointsCommand(qCmd, qdCmd, qtauCmd);
  // _publishLeftArmStates();
}

void RosMujocoInterface::rightArmCallback(const rm_ros_interfaces::msg::Jointpos::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mjMutex_);

  // Get current commands using batch API
  Eigen::VectorXd qCmd, qdCmd, qtauCmd;
  dataStream->getJointsCommand(qCmd, qdCmd, qtauCmd);

  // Update right arm commands (DOF 7-13)
  for (size_t i = 0; i < msg->joint.size() && (i + 7) < qCmd.size(); ++i) {
    qCmd(i + 7) = msg->joint[i];
  }

  // Set commands back using batch API
  dataStream->setJointsCommand(qCmd, qdCmd, qtauCmd);
  // _publishRightArmStates();
}
#endif

void RosMujocoInterface::publishLeftArmStates() {
  leftArmState_.header.stamp = rosNode_->now();

  Eigen::VectorXd qFb, qdFb, qtauFb;
  dataStream->getJointsFeedback(qFb, qdFb, qtauFb);

  for (int i = 0; i < 7 && i < qFb.size(); ++i) {
    leftArmState_.position[i] = qFb(i);
  }
  leftArmPublisher_->publish(leftArmState_);
}

void RosMujocoInterface::publishRightArmStates() {
  rightArmState_.header.stamp = rosNode_->now();

  Eigen::VectorXd qFb, qdFb, qtauFb;
  dataStream->getJointsFeedback(qFb, qdFb, qtauFb);

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
  const auto &chassis = dataStream->frameSensor(chasisIndex);

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
  if (timingManager->shouldTrigger("control", mjData_->time)) {
    setActuatorCommands();
  }
}

void RosMujocoInterface::resetController() {
  getFeedbacks();
  timingManager->reset("control", mjData_->time);
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
  default:
    break;
  }
}

} // namespace mujoco
