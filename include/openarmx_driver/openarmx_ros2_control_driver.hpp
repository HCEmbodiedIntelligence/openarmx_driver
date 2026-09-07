// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#ifndef OPENARMX_DRIVER__OPENARMX_ROS2_CONTROL_DRIVER_HPP_
#define OPENARMX_DRIVER__OPENARMX_ROS2_CONTROL_DRIVER_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "humanoid_driver_interface/ros2_driver_plugin.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace openarmx_driver
{

class OpenArmXRos2ControlDriver final :
  public humanoid_driver_interface::Ros2DriverPlugin
{
public:
  OpenArmXRos2ControlDriver() = default;
  ~OpenArmXRos2ControlDriver() override = default;

  humanoid_driver_interface::DriverResult attachRosNode(rclcpp::Node & node) override;
  humanoid_driver_interface::DriverResult configure(
    const humanoid_driver_interface::DriverConfiguration & configuration) override;
  humanoid_driver_interface::DriverResult connect() override;
  humanoid_driver_interface::DriverResult disconnect() override;
  humanoid_driver_interface::DriverResult activate() override;
  humanoid_driver_interface::DriverResult deactivate() override;
  humanoid_driver_interface::DriverResult readJointState(
    humanoid_driver_interface::JointState & state) override;
  humanoid_driver_interface::DriverResult startJointStream() override;
  humanoid_driver_interface::DriverResult writeJointCommand(
    const humanoid_driver_interface::JointCommand & command) override;
  humanoid_driver_interface::DriverResult stopJointStream() override;
  humanoid_driver_interface::DriverResult stopAll() override;
  humanoid_driver_interface::DriverHealth health() override;

private:
  using Clock = std::chrono::steady_clock;
  using Duration = std::chrono::duration<double>;
  using Mapping = humanoid_driver_interface::JointMapping;
  using Result = humanoid_driver_interface::DriverResult;

  static constexpr std::size_t kArmJointCount = 7U;

  void stateCallback(const sensor_msgs::msg::JointState::SharedPtr message);
  bool stateFreshLocked(Clock::time_point now) const;
  Result rejectCommandLocked(humanoid_driver_interface::DriverError error, std::string message);
  Result publishTargetsLocked();
  Result publishHoldLocked(Clock::time_point now);

  mutable std::mutex mutex_;
  rclcpp::Node * node_{nullptr};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_command_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_command_publisher_;

  std::vector<Mapping> mappings_;
  std::unordered_map<std::string, std::size_t> mapping_by_logical_name_;
  std::array<std::size_t, kArmJointCount> left_mapping_indices_{};
  std::array<std::size_t, kArmJointCount> right_mapping_indices_{};
  humanoid_driver_interface::JointState latest_state_;
  std::array<double, kArmJointCount> latest_left_vendor_positions_{};
  std::array<double, kArmJointCount> latest_right_vendor_positions_{};
  std::array<double, kArmJointCount> left_targets_{};
  std::array<double, kArmJointCount> right_targets_{};
  double latest_left_gripper_position_{0.0};
  double latest_right_gripper_position_{0.0};

  Clock::time_point configured_at_{Clock::now()};
  Clock::time_point last_state_received_{Clock::now()};
  Duration state_timeout_{0.25};
  Duration startup_grace_{15.0};
  std::string state_topic_;
  std::string left_command_topic_;
  std::string right_command_topic_;
  std::string left_group_{"left_arm"};
  std::string right_group_{"right_arm"};
  std::string left_gripper_joint_{"openarmx_left_finger_joint1"};
  std::string right_gripper_joint_{"openarmx_right_finger_joint1"};
  std::string last_feedback_error_;
  std::string last_command_error_;

  bool include_gripper_{true};
  bool configured_{false};
  bool connected_{false};
  bool active_{false};
  bool streaming_{false};
  bool holding_{true};
  bool have_state_{false};
  bool targets_initialized_{false};
};

}  // namespace openarmx_driver

#endif  // OPENARMX_DRIVER__OPENARMX_ROS2_CONTROL_DRIVER_HPP_
