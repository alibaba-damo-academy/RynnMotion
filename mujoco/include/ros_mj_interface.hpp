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
namespace mujoco {

class RosMujocoInterface : public MujocoInterface {
public:
  RosMujocoInterface(const YAML::Node &mujocoYaml, const YAML::Node &motionYaml, int robotType);
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
