// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "openarmx_driver/openarmx_ros2_control_driver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "pluginlib/class_list_macros.hpp"

namespace hdi = humanoid_driver_interface;

namespace
{

constexpr std::array<const char *, 7> kLeftVendorJoints = {
  "openarmx_left_joint1", "openarmx_left_joint2", "openarmx_left_joint3",
  "openarmx_left_joint4", "openarmx_left_joint5", "openarmx_left_joint6",
  "openarmx_left_joint7"};
constexpr std::array<const char *, 7> kRightVendorJoints = {
  "openarmx_right_joint1", "openarmx_right_joint2", "openarmx_right_joint3",
  "openarmx_right_joint4", "openarmx_right_joint5", "openarmx_right_joint6",
  "openarmx_right_joint7"};

double parsePositiveSeconds(const std::string & key, const std::string & value)
{
  std::size_t consumed = 0U;
  const double result = std::stod(value, &consumed);
  if (consumed != value.size() || !std::isfinite(result) || result <= 0.0) {
    throw std::invalid_argument(key + " must be a positive finite number of seconds");
  }
  return result;
}

bool allFinite(const std::vector<double> & values)
{
  return std::all_of(
    values.begin(), values.end(), [](const double value) {return std::isfinite(value);});
}

std::string booleanText(const bool value)
{
  return value ? "true" : "false";
}

}  // namespace

namespace openarmx_driver
{

OpenArmXRos2ControlDriver::Result OpenArmXRos2ControlDriver::attachRosNode(rclcpp::Node & node)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (configured_) {
    return Result::failure(hdi::DriverError::kInvalidState, "attach ROS node before configure");
  }
  if (node_ != nullptr && node_ != &node) {
    return Result::failure(hdi::DriverError::kInvalidState, "ROS node is already attached");
  }
  node_ = &node;
  return Result::success("runtime ROS node attached");
}

OpenArmXRos2ControlDriver::Result OpenArmXRos2ControlDriver::configure(
  const hdi::DriverConfiguration & configuration)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (node_ == nullptr) {
    return Result::failure(
      hdi::DriverError::kInvalidState, "attach the runtime ROS node before configure");
  }
  if (connected_ || active_) {
    return Result::failure(
      hdi::DriverError::kInvalidState, "cannot configure a connected OpenArmX driver");
  }

  std::string state_topic;
  std::string left_command_topic;
  std::string right_command_topic;
  std::string left_group = "left_arm";
  std::string right_group = "right_arm";
  double state_timeout_seconds = 0.25;
  double startup_grace_seconds = 15.0;

  try {
    for (const auto & [key, value] : configuration.parameters) {
      if (key == "state_topic") {
        state_topic = value;
      } else if (key == "left_command_topic") {
        left_command_topic = value;
      } else if (key == "right_command_topic") {
        right_command_topic = value;
      } else if (key == "left_group") {
        left_group = value;
      } else if (key == "right_group") {
        right_group = value;
      } else if (key == "state_timeout_s") {
        state_timeout_seconds = parsePositiveSeconds(key, value);
      } else if (key == "startup_grace_s") {
        startup_grace_seconds = parsePositiveSeconds(key, value);
      } else {
        throw std::invalid_argument("unknown OpenArmX driver parameter '" + key + "'");
      }
    }
  } catch (const std::exception & error) {
    return Result::failure(hdi::DriverError::kInvalidConfiguration, error.what());
  }

  if (state_topic.empty() || left_command_topic.empty() || right_command_topic.empty()) {
    return Result::failure(
      hdi::DriverError::kInvalidConfiguration,
      "state_topic, left_command_topic, and right_command_topic must be explicitly configured");
  }
  if (left_group.empty() || right_group.empty() || left_group == right_group) {
    return Result::failure(
      hdi::DriverError::kInvalidConfiguration,
      "left_group and right_group must be non-empty and distinct");
  }
  if (configuration.joints.size() != 2U * kArmJointCount) {
    return Result::failure(
      hdi::DriverError::kInvalidConfiguration,
      "OpenArmX bimanual configuration requires exactly 14 arm joint mappings");
  }

  std::unordered_set<std::string> logical_names;
  std::unordered_set<std::string> vendor_names;
  std::unordered_map<std::string, std::size_t> indices_by_vendor_name;
  for (std::size_t index = 0U; index < configuration.joints.size(); ++index) {
    const auto & mapping = configuration.joints[index];
    if (mapping.logical_name.empty() || mapping.vendor_name.empty() || mapping.vendor_group.empty() ||
      !std::isfinite(mapping.vendor_to_logical_scale) ||
      std::abs(mapping.vendor_to_logical_scale) < std::numeric_limits<double>::epsilon() ||
      !std::isfinite(mapping.vendor_to_logical_offset_rad))
    {
      return Result::failure(
        hdi::DriverError::kInvalidConfiguration,
        "joint mappings require names, a non-zero finite scale, and a finite offset");
    }
    if (!logical_names.insert(mapping.logical_name).second ||
      !vendor_names.insert(mapping.vendor_name).second)
    {
      return Result::failure(
        hdi::DriverError::kInvalidConfiguration,
        "logical and vendor arm joint names must be unique");
    }
    indices_by_vendor_name.emplace(mapping.vendor_name, index);
  }
  std::array<std::size_t, kArmJointCount> left_indices{};
  std::array<std::size_t, kArmJointCount> right_indices{};
  for (std::size_t index = 0U; index < kArmJointCount; ++index) {
    const auto left = indices_by_vendor_name.find(kLeftVendorJoints[index]);
    const auto right = indices_by_vendor_name.find(kRightVendorJoints[index]);
    if (left == indices_by_vendor_name.end() || right == indices_by_vendor_name.end()) {
      return Result::failure(
        hdi::DriverError::kInvalidConfiguration,
        "arm mappings must contain every official OpenArmX left/right joint1..joint7 name");
    }
    if (configuration.joints[left->second].vendor_group != left_group ||
      configuration.joints[right->second].vendor_group != right_group)
    {
      return Result::failure(
        hdi::DriverError::kInvalidConfiguration,
        "OpenArmX arm joint mapping uses the wrong configured vendor group");
    }
    left_indices[index] = left->second;
    right_indices[index] = right->second;
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_publisher;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_publisher;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscription;
  try {
    left_publisher = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      left_command_topic, rclcpp::QoS(10).reliable());
    right_publisher = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      right_command_topic, rclcpp::QoS(10).reliable());
    state_subscription = node_->create_subscription<sensor_msgs::msg::JointState>(
      state_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {stateCallback(message);});
  } catch (const std::exception & error) {
    return Result::failure(
      hdi::DriverError::kInternal,
      "failed to create OpenArmX ROS endpoints: " + std::string(error.what()));
  }

  mappings_ = configuration.joints;
  mapping_by_logical_name_.clear();
  for (std::size_t index = 0U; index < mappings_.size(); ++index) {
    mapping_by_logical_name_.emplace(mappings_[index].logical_name, index);
  }
  left_mapping_indices_ = left_indices;
  right_mapping_indices_ = right_indices;
  state_subscription_ = std::move(state_subscription);
  left_command_publisher_ = std::move(left_publisher);
  right_command_publisher_ = std::move(right_publisher);
  state_topic_ = std::move(state_topic);
  left_command_topic_ = std::move(left_command_topic);
  right_command_topic_ = std::move(right_command_topic);
  left_group_ = std::move(left_group);
  right_group_ = std::move(right_group);
  state_timeout_ = Duration(state_timeout_seconds);
  startup_grace_ = Duration(startup_grace_seconds);
  configured_at_ = Clock::now();
  last_state_received_ = configured_at_;
  latest_state_ = {};
  latest_left_vendor_positions_.fill(0.0);
  latest_right_vendor_positions_.fill(0.0);
  left_targets_.fill(0.0);
  right_targets_.fill(0.0);
  last_feedback_error_.clear();
  last_command_error_.clear();
  configured_ = true;
  connected_ = false;
  active_ = false;
  streaming_ = false;
  holding_ = true;
  have_state_ = false;
  targets_initialized_ = false;
  return Result::success("OpenArmX ros2_control driver configured");
}

OpenArmXRos2ControlDriver::Result OpenArmXRos2ControlDriver::connect()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!configured_ || connected_ || active_) {
    return Result::failure(
      hdi::DriverError::kInvalidState,
      "configure a disconnected OpenArmX driver before connect");
  }
  if (!state_subscription_ || !left_command_publisher_ || !right_command_publisher_) {
    return Result::failure(hdi::DriverError::kInternal, "OpenArmX ROS endpoints are unavailable");
  }
  connected_ = true;
  return Result::success("OpenArmX ROS endpoints connected without blocking for feedback");
}

OpenArmXRos2ControlDriver::Result OpenArmXRos2ControlDriver::disconnect()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_) {
    return Result::failure(
      hdi::DriverError::kInvalidState, "deactivate OpenArmX driver before disconnect");
  }
  connected_ = false;
  streaming_ = false;
  holding_ = true;
  return Result::success("OpenArmX ROS endpoints disconnected");
}

OpenArmXRos2ControlDriver::Result OpenArmXRos2ControlDriver::activate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!connected_ || active_) {
    return Result::failure(
      hdi::DriverError::kInvalidState, "connect an inactive OpenArmX driver before activate");
  }
  active_ = true;
  holding_ = true;
  return Result::success(
    targets_initialized_ ? "OpenArmX driver active with measured targets synchronized" :
    "OpenArmX driver active and awaiting measured targets");
}

OpenArmXRos2ControlDriver::Result OpenArmXRos2ControlDriver::deactivate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_) {
    return Result::success("OpenArmX driver already inactive");
  }
  active_ = false;
  streaming_ = false;
  holding_ = true;
  return Result::success("OpenArmX driver inactive");
}

OpenArmXRos2ControlDriver::Result OpenArmXRos2ControlDriver::readJointState(
  hdi::JointState & state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || !streaming_) {
    return Result::failure(hdi::DriverError::kNotActive, "OpenArmX joint stream is not active");
  }
  const auto now = Clock::now();
  if (!have_state_) {
    if (now - configured_at_ < startup_grace_) {
      return Result::failure(
        hdi::DriverError::kNoFeedback,
        "awaiting first complete OpenArmX joint state on " + state_topic_);
    }
    return Result::failure(
      hdi::DriverError::kCommunication,
      "startup grace expired without a complete OpenArmX joint state on " + state_topic_);
  }
  if (!stateFreshLocked(now)) {
    return Result::failure(
      hdi::DriverError::kCommunication, "OpenArmX joint state is stale on " + state_topic_);
  }
  state = latest_state_;
  return Result::success();
}

OpenArmXRos2ControlDriver::Result OpenArmXRos2ControlDriver::startJointStream()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_) {
    return Result::failure(hdi::DriverError::kNotActive, "OpenArmX driver is not active");
  }
  streaming_ = true;
  return Result::success("OpenArmX joint stream started");
}

OpenArmXRos2ControlDriver::Result OpenArmXRos2ControlDriver::writeJointCommand(
  const hdi::JointCommand & command)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || !streaming_) {
    return rejectCommandLocked(
      hdi::DriverError::kNotActive, "OpenArmX joint stream is not active");
  }
  if (!have_state_ || !targets_initialized_) {
    return rejectCommandLocked(
      hdi::DriverError::kNoFeedback,
      "cannot command OpenArmX before the first complete measured state");
  }
  if (!stateFreshLocked(Clock::now())) {
    return rejectCommandLocked(
      hdi::DriverError::kCommunication, "cannot command OpenArmX with stale feedback");
  }

  const auto count = command.joint_names.size();
  if (count == 0U || command.positions.size() != count ||
    (!command.velocities.empty() && command.velocities.size() != count) ||
    (!command.accelerations.empty() && command.accelerations.size() != count) ||
    (!command.efforts.empty() && command.efforts.size() != count) ||
    !allFinite(command.positions) || !allFinite(command.velocities) ||
    !allFinite(command.accelerations) || !allFinite(command.efforts))
  {
    return rejectCommandLocked(
      hdi::DriverError::kRejectedCommand,
      "joint command has empty/mismatched fields or contains NaN/Inf");
  }

  auto next_left_targets = left_targets_;
  auto next_right_targets = right_targets_;
  std::unordered_set<std::string> logical_names;
  for (std::size_t command_index = 0U; command_index < count; ++command_index) {
    const auto & logical_name = command.joint_names[command_index];
    const auto found = mapping_by_logical_name_.find(logical_name);
    if (found == mapping_by_logical_name_.end()) {
      return rejectCommandLocked(
        hdi::DriverError::kRejectedCommand,
        "joint command contains unknown logical joint '" + logical_name + "'");
    }
    if (!logical_names.insert(logical_name).second) {
      return rejectCommandLocked(
        hdi::DriverError::kRejectedCommand,
        "joint command repeats logical joint '" + logical_name + "'");
    }

    const auto mapping_index = found->second;
    const auto & mapping = mappings_[mapping_index];
    const double vendor_position =
      (command.positions[command_index] - mapping.vendor_to_logical_offset_rad) /
      mapping.vendor_to_logical_scale;
    const auto left = std::find(
      left_mapping_indices_.begin(), left_mapping_indices_.end(), mapping_index);
    const auto right = std::find(
      right_mapping_indices_.begin(), right_mapping_indices_.end(), mapping_index);
    if (left != left_mapping_indices_.end()) {
      next_left_targets[static_cast<std::size_t>(left - left_mapping_indices_.begin())] =
        vendor_position;
    } else if (right != right_mapping_indices_.end()) {
      next_right_targets[static_cast<std::size_t>(right - right_mapping_indices_.begin())] =
        vendor_position;
    } else {
      return rejectCommandLocked(
        hdi::DriverError::kInternal, "configured joint is not assigned to an OpenArmX arm");
    }
  }

  left_targets_ = next_left_targets;
  right_targets_ = next_right_targets;
  const auto result = publishTargetsLocked();
  if (!result) {
    return result;
  }
  holding_ = false;
  last_command_error_.clear();
  return Result::success("published complete left/right OpenArmX position targets");
}

OpenArmXRos2ControlDriver::Result OpenArmXRos2ControlDriver::stopJointStream()
{
  std::lock_guard<std::mutex> lock(mutex_);
  streaming_ = false;
  return Result::success("OpenArmX joint stream stopped");
}

OpenArmXRos2ControlDriver::Result OpenArmXRos2ControlDriver::stopAll()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!configured_) {
    return Result::failure(hdi::DriverError::kInvalidState, "OpenArmX driver is not configured");
  }
  return publishHoldLocked(Clock::now());
}

hdi::DriverHealth OpenArmXRos2ControlDriver::health()
{
  std::lock_guard<std::mutex> lock(mutex_);
  hdi::DriverHealth result;
  result.connected = connected_;
  result.active = active_;
  result.details["configured"] = booleanText(configured_);
  result.details["connected"] = booleanText(connected_);
  result.details["active"] = booleanText(active_);
  result.details["streaming"] = booleanText(streaming_);
  result.details["have_complete_feedback"] = booleanText(have_state_);
  result.details["holding"] = booleanText(holding_);
  result.details["mode"] = holding_ ? "holding" : "operational";
  result.details["state_topic"] = state_topic_;
  result.details["left_command_topic"] = left_command_topic_;
  result.details["right_command_topic"] = right_command_topic_;
  result.details["last_feedback_error"] = last_feedback_error_;
  result.details["last_command_error"] = last_command_error_;
  const auto left_subscribers = left_command_publisher_ ?
    left_command_publisher_->get_subscription_count() : 0U;
  const auto right_subscribers = right_command_publisher_ ?
    right_command_publisher_->get_subscription_count() : 0U;
  result.details["left_publisher_subscription_count"] = std::to_string(left_subscribers);
  result.details["right_publisher_subscription_count"] = std::to_string(right_subscribers);

  if (!configured_) {
    result.level = hdi::HealthLevel::kStale;
    result.message = "OpenArmX driver is not configured";
    return result;
  }

  const auto now = Clock::now();
  const bool fresh = stateFreshLocked(now);
  result.details["feedback_fresh"] = booleanText(fresh);
  if (!have_state_) {
    if (now - configured_at_ < startup_grace_) {
      result.level = hdi::HealthLevel::kWarning;
      result.communication_ok = true;
      result.message = "awaiting first complete OpenArmX feedback during startup grace";
    } else {
      result.level = hdi::HealthLevel::kStale;
      result.message = "no complete OpenArmX feedback before startup grace expired";
    }
    return result;
  }
  if (!fresh) {
    result.level = hdi::HealthLevel::kStale;
    result.message = holding_ ?
      "OpenArmX feedback is stale; holding last measured position" :
      "OpenArmX feedback is stale";
    return result;
  }

  result.communication_ok = true;
  if (!last_feedback_error_.empty()) {
    result.level = hdi::HealthLevel::kWarning;
    result.message = "latest OpenArmX feedback was rejected; prior complete sample remains fresh";
  } else if (!last_command_error_.empty()) {
    result.level = hdi::HealthLevel::kWarning;
    result.message = "OpenArmX command path reports: " + last_command_error_;
  } else if (left_subscribers == 0U || right_subscribers == 0U) {
    result.level = hdi::HealthLevel::kWarning;
    result.message = "one or both OpenArmX position publishers have no controller subscriber";
  } else if (!active_ || !streaming_) {
    result.level = hdi::HealthLevel::kWarning;
    result.message = "OpenArmX driver is inactive or its joint stream is stopped";
  } else if (holding_) {
    result.level = hdi::HealthLevel::kWarning;
    result.message = "OpenArmX driver is holding position";
  } else {
    result.level = hdi::HealthLevel::kOk;
    result.message = "OpenArmX position driver operational";
  }
  return result;
}

void OpenArmXRos2ControlDriver::stateCallback(
  const sensor_msgs::msg::JointState::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!configured_) {
    return;
  }
  if (message->name.size() != message->position.size() ||
    (!message->velocity.empty() && message->velocity.size() != message->name.size()) ||
    (!message->effort.empty() && message->effort.size() != message->name.size()))
  {
    last_feedback_error_ = "OpenArmX JointState fields have inconsistent lengths";
    return;
  }
  if (!allFinite(message->position) || !allFinite(message->velocity) ||
    !allFinite(message->effort))
  {
    last_feedback_error_ = "OpenArmX JointState contains NaN or Inf";
    return;
  }

  std::unordered_map<std::string, std::size_t> indices;
  for (std::size_t index = 0U; index < message->name.size(); ++index) {
    if (!indices.emplace(message->name[index], index).second) {
      last_feedback_error_ = "OpenArmX JointState repeats joint '" + message->name[index] + "'";
      return;
    }
  }

  hdi::JointState next_state;
  next_state.joint_names.reserve(mappings_.size());
  next_state.positions.reserve(mappings_.size());
  if (!message->velocity.empty()) {
    next_state.velocities.reserve(mappings_.size());
  }
  if (!message->effort.empty()) {
    next_state.efforts.reserve(mappings_.size());
  }
  for (const auto & mapping : mappings_) {
    const auto found = indices.find(mapping.vendor_name);
    if (found == indices.end()) {
      last_feedback_error_ =
        "OpenArmX JointState is missing arm joint '" + mapping.vendor_name + "'";
      return;
    }
    const auto index = found->second;
    next_state.joint_names.push_back(mapping.logical_name);
    next_state.positions.push_back(
      mapping.vendor_to_logical_scale * message->position[index] +
      mapping.vendor_to_logical_offset_rad);
    if (!message->velocity.empty()) {
      next_state.velocities.push_back(mapping.vendor_to_logical_scale * message->velocity[index]);
    }
    if (!message->effort.empty()) {
      next_state.efforts.push_back(message->effort[index] / mapping.vendor_to_logical_scale);
    }
  }

  std::array<double, kArmJointCount> left_vendor_positions{};
  std::array<double, kArmJointCount> right_vendor_positions{};
  for (std::size_t index = 0U; index < kArmJointCount; ++index) {
    left_vendor_positions[index] =
      message->position[indices.at(mappings_[left_mapping_indices_[index]].vendor_name)];
    right_vendor_positions[index] =
      message->position[indices.at(mappings_[right_mapping_indices_[index]].vendor_name)];
  }

  const auto received_at = Clock::now();
  next_state.sample_time = received_at;
  latest_state_ = std::move(next_state);
  latest_left_vendor_positions_ = left_vendor_positions;
  latest_right_vendor_positions_ = right_vendor_positions;
  last_state_received_ = received_at;
  last_feedback_error_.clear();
  have_state_ = true;
  if (!targets_initialized_) {
    left_targets_ = latest_left_vendor_positions_;
    right_targets_ = latest_right_vendor_positions_;
    targets_initialized_ = true;
  }
}

bool OpenArmXRos2ControlDriver::stateFreshLocked(const Clock::time_point now) const
{
  return have_state_ && now >= last_state_received_ && now - last_state_received_ <= state_timeout_;
}

OpenArmXRos2ControlDriver::Result OpenArmXRos2ControlDriver::rejectCommandLocked(
  const hdi::DriverError error, std::string message)
{
  last_command_error_ = message;
  return Result::failure(error, std::move(message));
}

OpenArmXRos2ControlDriver::Result OpenArmXRos2ControlDriver::publishTargetsLocked()
{
  if (!left_command_publisher_ || !right_command_publisher_) {
    return rejectCommandLocked(
      hdi::DriverError::kCommunication, "OpenArmX command publishers are unavailable");
  }

  std_msgs::msg::Float64MultiArray left_message;
  std_msgs::msg::Float64MultiArray right_message;
  left_message.data.assign(left_targets_.begin(), left_targets_.end());
  right_message.data.assign(right_targets_.begin(), right_targets_.end());
  std::vector<std::string> errors;
  try {
    left_command_publisher_->publish(left_message);
  } catch (const std::exception & error) {
    errors.push_back("left publish failed: " + std::string(error.what()));
  }
  try {
    right_command_publisher_->publish(right_message);
  } catch (const std::exception & error) {
    errors.push_back("right publish failed: " + std::string(error.what()));
  }
  if (!errors.empty()) {
    std::ostringstream message;
    for (std::size_t index = 0U; index < errors.size(); ++index) {
      if (index != 0U) {
        message << "; ";
      }
      message << errors[index];
    }
    return rejectCommandLocked(hdi::DriverError::kCommunication, message.str());
  }
  return Result::success();
}

OpenArmXRos2ControlDriver::Result OpenArmXRos2ControlDriver::publishHoldLocked(
  const Clock::time_point now)
{
  if (!have_state_) {
    holding_ = true;
    return rejectCommandLocked(
      hdi::DriverError::kNoFeedback,
      "cannot hold OpenArmX position before the first complete measured state");
  }

  left_targets_ = latest_left_vendor_positions_;
  right_targets_ = latest_right_vendor_positions_;
  targets_initialized_ = true;
  const bool stale = !stateFreshLocked(now);
  const auto published = publishTargetsLocked();
  if (!published) {
    return published;
  }
  holding_ = true;
  if (stale) {
    last_command_error_ = "hold published from the last measured position, but feedback is stale";
    return Result::success(last_command_error_);
  }
  last_command_error_.clear();
  return Result::success("holding latest measured OpenArmX arm positions");
}

}  // namespace openarmx_driver

PLUGINLIB_EXPORT_CLASS(
  openarmx_driver::OpenArmXRos2ControlDriver,
  humanoid_driver_interface::RobotDriverPlugin)
