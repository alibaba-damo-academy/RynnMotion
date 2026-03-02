#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <thread>

#include "mj_interface.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#ifdef USE_RM_INTERFACES
#include <rm_ros_interfaces/msg/jointpos.hpp>
#endif

#ifdef USE_USSCAN_MSGS
#include <usscan_msgs/msg/usscan_state.hpp>
#include <usscan_msgs/msg/usscan_feedback.hpp>
#include <usscan_msgs/msg/probe_trajectory_proposal.hpp>
#include "fsm/usscan_fsm.hpp"
#include "trajgen.hpp"
#include <queue>
#endif
namespace mujoco {

class RosMujocoInterface : public MujocoInterface {
public:
  RosMujocoInterface(const YAML::Node &mujocoYaml, const YAML::Node &motionYaml, int robotType, int sceneNumber = 1);
  ~RosMujocoInterface();

  void runApplication() override;

  // Method to expose ROS node for thread-safe operation
  rclcpp::Node::SharedPtr getRosNode() {
    return rosNode_;
  }

  // Make publishTopics public for thread-safe operation
  void publishTopics();

protected:
  void resetController() override;
  void callController() override;
  void keyboard(GLFWwindow *window, int key, int scancode, int action, int mods) override;
  void initROS();

private:
  void publishLeftArmStates();
  void publishRightArmStates();
  void publishLidarScan();
  void publishOdom();
  void chasisVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

#ifdef USE_RM_INTERFACES
  void leftArmCallback(const rm_ros_interfaces::msg::Jointpos::SharedPtr msg);
  void rightArmCallback(const rm_ros_interfaces::msg::Jointpos::SharedPtr msg);
  rclcpp::Subscription<rm_ros_interfaces::msg::Jointpos>::SharedPtr leftArmSubscriber_;
  rclcpp::Subscription<rm_ros_interfaces::msg::Jointpos>::SharedPtr rightArmSubscriber_;
#endif

#ifdef USE_USSCAN_MSGS
  void initUsscanROS();
  void publishUsscanState();
  void publishUsscanFeedback();
  void navStateCallback(const usscan_msgs::msg::UsscanState::SharedPtr msg);
  void trajectoryCallback(const usscan_msgs::msg::ProbeTrajectoryProposal::SharedPtr msg);

  // FSM tick and motion execution
  void tickUsscanFsm(float dt);
  void processNavStateEvent(int navStateId);
  void executeCurrentState();
  bool checkContactDetected();

  // Keyboard event handlers for usscan
  void handleUsscanKeyEvent(int key);

  rclcpp::Publisher<usscan_msgs::msg::UsscanState>::SharedPtr usscanStatePublisher_;
  rclcpp::Publisher<usscan_msgs::msg::UsscanFeedback>::SharedPtr usscanFeedbackPublisher_;
  rclcpp::Subscription<usscan_msgs::msg::UsscanState>::SharedPtr navStateSubscriber_;
  rclcpp::Subscription<usscan_msgs::msg::ProbeTrajectoryProposal>::SharedPtr trajectorySubscriber_;

  // Trajectory queue for probe motion
  std::queue<usscan_msgs::msg::ProbeTrajectoryProposal> trajectoryQueue_;
  std::mutex trajectoryQueueMutex_;

  // Trajectory generators
  std::unique_ptr<EEPoseTrajGen> usscanTrajGen_;
  std::unique_ptr<JointTrajGen<7>> usscanJointTrajGen_;

  // FSM state tracking
  int navStateId_ = -1;
  uint8_t motionStateId_ = 0;
  float motionStateTime_ = 0.0f;
  float motionTotalTime_ = 0.0f;
  bool usscanInitialized_ = false;

  // Home position for rizon
  Eigen::VectorXd homeJointPos_;

  // M2 approach state
  Eigen::Vector3d approachStartPos_;
  double approachSpeed_ = 0.01;  // 1cm/s descent speed

  // Current trajectory tracking
  size_t currentWaypointIndex_ = 0;
  bool trajectoryActive_ = false;
#endif

  rclcpp::Node::SharedPtr rosNode_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr rosClockPublisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr leftArmPublisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr rightArmPublisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidarScanPublisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPublisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSubscriber_;
  std::shared_ptr<rclcpp::Rate> rosRate_;
  sensor_msgs::msg::JointState leftArmState_, rightArmState_;
  std::mutex mjMutex_;
};

} // namespace mujoco
