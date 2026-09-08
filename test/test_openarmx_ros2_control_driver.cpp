// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "openarmx_driver/openarmx_ros2_control_driver.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace hdi = humanoid_driver_interface;
namespace oad = openarmx_driver;
using namespace std::chrono_literals;

namespace
{

std::atomic<unsigned int> g_test_id{0U};

std::string leftVendorName(const std::size_t index)
{
  return "openarmx_left_joint" + std::to_string(index + 1U);
}

std::string rightVendorName(const std::size_t index)
{
  return "openarmx_right_joint" + std::to_string(index + 1U);
}

std::string leftLogicalName(const std::size_t index)
{
  return "logical_left_" + std::to_string(index + 1U);
}

std::string rightLogicalName(const std::size_t index)
{
  return "logical_right_" + std::to_string(index + 1U);
}

class OpenArmXDriverTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    const auto id = std::to_string(g_test_id.fetch_add(1U));
    prefix_ = "/openarmx_driver_test_" + id;
    node_ = std::make_shared<rclcpp::Node>("openarmx_driver_test_" + id);
    executor_.add_node(node_);
    state_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      prefix_ + "/joint_states", rclcpp::SensorDataQoS());
    left_subscription_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      prefix_ + "/left_commands", rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr message) {
        left_commands_.push_back(*message);
      });
    right_subscription_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      prefix_ + "/right_commands", rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr message) {
        right_commands_.push_back(*message);
      });
  }

  void TearDown() override
  {
    executor_.remove_node(node_);
    right_subscription_.reset();
    left_subscription_.reset();
    state_publisher_.reset();
    node_.reset();
  }

  hdi::DriverConfiguration configuration(
    const std::string & state_timeout = "0.25",
    const std::string & startup_grace = "1.0") const
  {
    hdi::DriverConfiguration result;
    for (std::size_t index = 0U; index < 7U; ++index) {
      result.joints.push_back(
        {leftLogicalName(index), leftVendorName(index), "left_arm", 1.0, 0.0});
      result.joints.push_back(
        {rightLogicalName(index), rightVendorName(index), "right_arm", 1.0, 0.0});
    }
    result.parameters = {
      {"state_topic", prefix_ + "/joint_states"},
      {"left_command_topic", prefix_ + "/left_commands"},
      {"right_command_topic", prefix_ + "/right_commands"},
      {"left_group", "left_arm"},
      {"right_group", "right_arm"},
      {"state_timeout_s", state_timeout},
      {"startup_grace_s", startup_grace},
    };
    return result;
  }

  sensor_msgs::msg::JointState stateMessage(const double position_shift = 0.0) const
  {
    sensor_msgs::msg::JointState message;
    for (std::size_t index = 0U; index < 7U; ++index) {
      message.name.push_back(leftVendorName(index));
      message.position.push_back(position_shift + 0.1 * static_cast<double>(index + 1U));
      message.velocity.push_back(0.01 * static_cast<double>(index + 1U));
      message.effort.push_back(1.0 + static_cast<double>(index));
      message.name.push_back(rightVendorName(index));
      message.position.push_back(position_shift - 0.2 * static_cast<double>(index + 1U));
      message.velocity.push_back(-0.02 * static_cast<double>(index + 1U));
      message.effort.push_back(-1.0 - static_cast<double>(index));
    }
    return message;
  }

  static void reverseMessage(sensor_msgs::msg::JointState & message)
  {
    std::reverse(message.name.begin(), message.name.end());
    std::reverse(message.position.begin(), message.position.end());
    std::reverse(message.velocity.begin(), message.velocity.end());
    std::reverse(message.effort.begin(), message.effort.end());
  }

  void startDriver(
    oad::OpenArmXRos2ControlDriver & driver,
    const hdi::DriverConfiguration & config)
  {
    ASSERT_TRUE(driver.attachRosNode(*node_));
    ASSERT_TRUE(driver.configure(config));
    ASSERT_TRUE(driver.connect());
    ASSERT_TRUE(driver.activate());
    ASSERT_TRUE(driver.startJointStream());
  }

  bool spinUntil(const std::function<bool()> & predicate, const std::chrono::milliseconds timeout = 1s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    executor_.spin_some();
    return predicate();
  }

  bool publishUntilReadable(
    oad::OpenArmXRos2ControlDriver & driver,
    const sensor_msgs::msg::JointState & message,
    hdi::JointState & output)
  {
    return spinUntil(
      [this, &driver, &message, &output]() {
        state_publisher_->publish(message);
        executor_.spin_some();
        return static_cast<bool>(driver.readJointState(output));
      });
  }

  bool publishUntilFeedbackError(
    oad::OpenArmXRos2ControlDriver & driver,
    const sensor_msgs::msg::JointState & message,
    const std::string & fragment)
  {
    return spinUntil(
      [this, &driver, &message, &fragment]() {
        state_publisher_->publish(message);
        executor_.spin_some();
        const auto health = driver.health();
        const auto found = health.details.find("last_feedback_error");
        return found != health.details.end() && found->second.find(fragment) != std::string::npos;
      });
  }

  bool waitForCommandCount(const std::size_t count)
  {
    return spinUntil(
      [this, count]() {
        return left_commands_.size() >= count && right_commands_.size() >= count;
      });
  }

  std::string prefix_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr left_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr right_subscription_;
  std::vector<std_msgs::msg::Float64MultiArray> left_commands_;
  std::vector<std_msgs::msg::Float64MultiArray> right_commands_;
};

TEST_F(OpenArmXDriverTest, RejectsInvalidParametersAndIncompleteMappings)
{
  const auto expectInvalid = [this](hdi::DriverConfiguration config) {
      oad::OpenArmXRos2ControlDriver driver;
      EXPECT_TRUE(driver.attachRosNode(*node_));
      const auto result = driver.configure(config);
      EXPECT_FALSE(result);
      EXPECT_EQ(result.error, hdi::DriverError::kInvalidConfiguration);
    };

  auto config = configuration();
  config.parameters.erase("state_topic");
  expectInvalid(config);

  config = configuration();
  config.parameters["unknown_parameter"] = "value";
  expectInvalid(config);

  config = configuration();
  config.parameters["state_timeout_s"] = "0";
  expectInvalid(config);

  config = configuration();
  config.joints.pop_back();
  expectInvalid(config);

  config = configuration();
  config.joints.back().vendor_name = config.joints.front().vendor_name;
  expectInvalid(config);
}

TEST_F(OpenArmXDriverTest, RejectsIllegalLifecycleOrderWithoutPublishing)
{
  oad::OpenArmXRos2ControlDriver driver;
  EXPECT_FALSE(driver.configure(configuration()));
  ASSERT_TRUE(driver.attachRosNode(*node_));
  ASSERT_TRUE(driver.configure(configuration()));
  EXPECT_FALSE(driver.activate());
  EXPECT_FALSE(driver.startJointStream());
  ASSERT_TRUE(driver.connect());
  ASSERT_TRUE(driver.activate());
  EXPECT_FALSE(driver.activate());
  ASSERT_TRUE(driver.startJointStream());

  hdi::JointCommand command;
  command.joint_names = {leftLogicalName(0U)};
  command.positions = {1.0};
  const auto result = driver.writeJointCommand(command);
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error, hdi::DriverError::kNoFeedback);
  executor_.spin_some();
  EXPECT_TRUE(left_commands_.empty());
  EXPECT_TRUE(right_commands_.empty());

  EXPECT_FALSE(driver.disconnect());
  EXPECT_TRUE(driver.deactivate());
  EXPECT_TRUE(driver.disconnect());
}

TEST_F(OpenArmXDriverTest, MapsRandomFeedbackOrderWithScaleOffsetVelocityAndEffort)
{
  auto config = configuration();
  config.joints[0].vendor_to_logical_scale = -2.0;
  config.joints[0].vendor_to_logical_offset_rad = 0.1;
  config.joints[1].vendor_to_logical_scale = 0.5;
  config.joints[1].vendor_to_logical_offset_rad = -0.2;

  oad::OpenArmXRos2ControlDriver driver;
  startDriver(driver, config);
  auto message = stateMessage();
  reverseMessage(message);
  hdi::JointState state;
  ASSERT_TRUE(publishUntilReadable(driver, message, state));

  ASSERT_EQ(state.joint_names.size(), 14U);
  EXPECT_EQ(state.joint_names[0], leftLogicalName(0U));
  EXPECT_EQ(state.joint_names[1], rightLogicalName(0U));
  EXPECT_NEAR(state.positions[0], -0.1, 1e-12);
  EXPECT_NEAR(state.positions[1], -0.3, 1e-12);
  EXPECT_NEAR(state.velocities[0], -0.02, 1e-12);
  EXPECT_NEAR(state.velocities[1], -0.01, 1e-12);
  EXPECT_NEAR(state.efforts[0], -0.5, 1e-12);
  EXPECT_NEAR(state.efforts[1], -2.0, 1e-12);

  hdi::JointCommand command;
  command.joint_names = {rightLogicalName(0U), leftLogicalName(0U)};
  command.positions = {0.3, -0.5};
  ASSERT_TRUE(driver.writeJointCommand(command));
  ASSERT_TRUE(waitForCommandCount(1U));
  EXPECT_NEAR(left_commands_.back().data[0], 0.3, 1e-12);
  EXPECT_NEAR(right_commands_.back().data[0], 1.0, 1e-12);
  EXPECT_EQ(driver.health().details.at("mode"), "operational");
}

TEST_F(OpenArmXDriverTest, RejectsMalformedDuplicateMissingAndNonfiniteFeedback)
{
  oad::OpenArmXRos2ControlDriver driver;
  startDriver(driver, configuration());

  auto message = stateMessage();
  message.position.pop_back();
  ASSERT_TRUE(publishUntilFeedbackError(driver, message, "inconsistent lengths"));

  message = stateMessage();
  message.name.back() = message.name.front();
  ASSERT_TRUE(publishUntilFeedbackError(driver, message, "repeats joint"));

  message = stateMessage();
  message.name.erase(message.name.begin());
  message.position.erase(message.position.begin());
  message.velocity.erase(message.velocity.begin());
  message.effort.erase(message.effort.begin());
  ASSERT_TRUE(publishUntilFeedbackError(driver, message, "missing arm joint"));

  message = stateMessage();
  message.position[0] = std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(publishUntilFeedbackError(driver, message, "NaN or Inf"));

  message = stateMessage();
  message.velocity[0] = std::numeric_limits<double>::infinity();
  ASSERT_TRUE(publishUntilFeedbackError(driver, message, "NaN or Inf"));

  hdi::JointState state;
  const auto result = driver.readJointState(state);
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error, hdi::DriverError::kNoFeedback);
}

TEST_F(OpenArmXDriverTest, SplitsShuffledFourteenAxisCommand)
{
  oad::OpenArmXRos2ControlDriver driver;
  startDriver(driver, configuration());
  auto message = stateMessage();
  reverseMessage(message);
  hdi::JointState state;
  ASSERT_TRUE(publishUntilReadable(driver, message, state));

  hdi::JointCommand command;
  for (std::size_t reverse_index = 14U; reverse_index > 0U; --reverse_index) {
    const std::size_t index = reverse_index - 1U;
    if (index % 2U == 0U) {
      command.joint_names.push_back(leftLogicalName(index / 2U));
      command.positions.push_back(10.0 + static_cast<double>(index / 2U));
    } else {
      command.joint_names.push_back(rightLogicalName(index / 2U));
      command.positions.push_back(-10.0 - static_cast<double>(index / 2U));
    }
  }
  command.velocities.assign(14U, 0.0);
  command.accelerations.assign(14U, 0.0);
  ASSERT_TRUE(driver.writeJointCommand(command));
  ASSERT_TRUE(waitForCommandCount(1U));

  ASSERT_EQ(left_commands_.back().data.size(), 7U);
  ASSERT_EQ(right_commands_.back().data.size(), 7U);
  for (std::size_t index = 0U; index < 7U; ++index) {
    EXPECT_DOUBLE_EQ(left_commands_.back().data[index], 10.0 + static_cast<double>(index));
    EXPECT_DOUBLE_EQ(right_commands_.back().data[index], -10.0 - static_cast<double>(index));
  }
}

TEST_F(OpenArmXDriverTest, PartialCommandsMergeWithPreviousSafeTargets)
{
  oad::OpenArmXRos2ControlDriver driver;
  startDriver(driver, configuration());
  hdi::JointState state;
  ASSERT_TRUE(publishUntilReadable(driver, stateMessage(), state));

  hdi::JointCommand first;
  first.joint_names = {leftLogicalName(2U)};
  first.positions = {9.0};
  ASSERT_TRUE(driver.writeJointCommand(first));
  ASSERT_TRUE(waitForCommandCount(1U));
  EXPECT_DOUBLE_EQ(left_commands_.back().data[2], 9.0);
  EXPECT_DOUBLE_EQ(left_commands_.back().data[1], 0.2);
  EXPECT_DOUBLE_EQ(right_commands_.back().data[3], -0.8);

  hdi::JointCommand second;
  second.joint_names = {rightLogicalName(3U)};
  second.positions = {-8.0};
  ASSERT_TRUE(driver.writeJointCommand(second));
  ASSERT_TRUE(waitForCommandCount(2U));
  EXPECT_DOUBLE_EQ(left_commands_.back().data[2], 9.0);
  EXPECT_DOUBLE_EQ(left_commands_.back().data[1], 0.2);
  EXPECT_DOUBLE_EQ(right_commands_.back().data[3], -8.0);
  EXPECT_DOUBLE_EQ(right_commands_.back().data[4], -1.0);
}

TEST_F(OpenArmXDriverTest, RejectsMalformedUnknownDuplicateAndNonfiniteCommands)
{
  oad::OpenArmXRos2ControlDriver driver;
  startDriver(driver, configuration());
  hdi::JointState state;
  ASSERT_TRUE(publishUntilReadable(driver, stateMessage(), state));

  hdi::JointCommand command;
  command.joint_names = {leftLogicalName(0U), leftLogicalName(1U)};
  command.positions = {1.0};
  EXPECT_EQ(driver.writeJointCommand(command).error, hdi::DriverError::kRejectedCommand);

  command.joint_names = {"unknown"};
  command.positions = {1.0};
  EXPECT_EQ(driver.writeJointCommand(command).error, hdi::DriverError::kRejectedCommand);

  command.joint_names = {leftLogicalName(0U), leftLogicalName(0U)};
  command.positions = {1.0, 2.0};
  EXPECT_EQ(driver.writeJointCommand(command).error, hdi::DriverError::kRejectedCommand);

  command.joint_names = {leftLogicalName(0U)};
  command.positions = {std::numeric_limits<double>::quiet_NaN()};
  EXPECT_EQ(driver.writeJointCommand(command).error, hdi::DriverError::kRejectedCommand);

  command.positions = {1.0};
  command.efforts = {std::numeric_limits<double>::infinity()};
  EXPECT_EQ(driver.writeJointCommand(command).error, hdi::DriverError::kRejectedCommand);
  executor_.spin_some();
  EXPECT_TRUE(left_commands_.empty());
  EXPECT_TRUE(right_commands_.empty());
}

TEST_F(OpenArmXDriverTest, ReportsStartupGraceThenCommunicationTimeoutWithoutFeedback)
{
  oad::OpenArmXRos2ControlDriver driver;
  startDriver(driver, configuration("0.25", "0.02"));
  hdi::JointState state;
  auto result = driver.readJointState(state);
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error, hdi::DriverError::kNoFeedback);

  std::this_thread::sleep_for(30ms);
  result = driver.readJointState(state);
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error, hdi::DriverError::kCommunication);
}

TEST_F(OpenArmXDriverTest, RejectsCommandsAndReadsAfterStateTimeout)
{
  oad::OpenArmXRos2ControlDriver driver;
  startDriver(driver, configuration("0.02", "1.0"));
  hdi::JointState state;
  ASSERT_TRUE(publishUntilReadable(driver, stateMessage(), state));
  std::this_thread::sleep_for(30ms);

  EXPECT_EQ(driver.readJointState(state).error, hdi::DriverError::kCommunication);
  hdi::JointCommand command;
  command.joint_names = {leftLogicalName(0U)};
  command.positions = {1.0};
  EXPECT_EQ(driver.writeJointCommand(command).error, hdi::DriverError::kCommunication);
}

TEST_F(OpenArmXDriverTest, StopAllIsIdempotentAndHoldsLatestMeasuredPositions)
{
  oad::OpenArmXRos2ControlDriver driver;
  startDriver(driver, configuration());
  hdi::JointState state;
  ASSERT_TRUE(publishUntilReadable(driver, stateMessage(), state));

  hdi::JointCommand command;
  command.joint_names = {leftLogicalName(0U), rightLogicalName(0U)};
  command.positions = {4.0, -4.0};
  ASSERT_TRUE(driver.writeJointCommand(command));
  ASSERT_TRUE(waitForCommandCount(1U));

  ASSERT_TRUE(publishUntilReadable(driver, stateMessage(0.5), state));
  ASSERT_TRUE(driver.stopAll());
  ASSERT_TRUE(driver.stopAll());
  ASSERT_TRUE(waitForCommandCount(3U));
  for (std::size_t index = 0U; index < 7U; ++index) {
    EXPECT_DOUBLE_EQ(
      left_commands_[left_commands_.size() - 1U].data[index],
      0.5 + 0.1 * static_cast<double>(index + 1U));
    EXPECT_DOUBLE_EQ(
      right_commands_[right_commands_.size() - 1U].data[index],
      0.5 - 0.2 * static_cast<double>(index + 1U));
  }
  EXPECT_EQ(driver.health().details.at("mode"), "holding");
}

TEST_F(OpenArmXDriverTest, StopAllPublishesLastMeasurementEvenWhenFeedbackIsStale)
{
  oad::OpenArmXRos2ControlDriver driver;
  startDriver(driver, configuration("0.02", "1.0"));
  hdi::JointState state;
  ASSERT_TRUE(publishUntilReadable(driver, stateMessage(), state));
  std::this_thread::sleep_for(30ms);

  const auto result = driver.stopAll();
  EXPECT_TRUE(result);
  EXPECT_NE(result.message.find("stale"), std::string::npos);
  ASSERT_TRUE(waitForCommandCount(1U));
  EXPECT_DOUBLE_EQ(left_commands_.back().data[0], 0.1);
  EXPECT_DOUBLE_EQ(right_commands_.back().data[0], -0.2);
  EXPECT_EQ(driver.health().level, hdi::HealthLevel::kStale);
}

TEST_F(OpenArmXDriverTest, StopAllWithoutAnyFeedbackReturnsNoFeedback)
{
  oad::OpenArmXRos2ControlDriver driver;
  startDriver(driver, configuration());
  const auto first = driver.stopAll();
  const auto second = driver.stopAll();
  EXPECT_FALSE(first);
  EXPECT_FALSE(second);
  EXPECT_EQ(first.error, hdi::DriverError::kNoFeedback);
  EXPECT_EQ(second.error, hdi::DriverError::kNoFeedback);
  EXPECT_TRUE(left_commands_.empty());
  EXPECT_TRUE(right_commands_.empty());
}

TEST_F(OpenArmXDriverTest, HealthReportsLifecycleFreshnessHoldingTopicsErrorsAndSubscribers)
{
  left_subscription_.reset();
  right_subscription_.reset();
  oad::OpenArmXRos2ControlDriver driver;
  startDriver(driver, configuration("0.02", "1.0"));
  hdi::JointState state;
  ASSERT_TRUE(publishUntilReadable(driver, stateMessage(), state));

  ASSERT_TRUE(spinUntil(
      [&driver]() {
        const auto health = driver.health();
        return health.details.at("left_publisher_subscription_count") == "0" &&
               health.details.at("right_publisher_subscription_count") == "0";
      }));
  auto health = driver.health();
  EXPECT_EQ(health.level, hdi::HealthLevel::kWarning);
  EXPECT_EQ(health.details.at("configured"), "true");
  EXPECT_EQ(health.details.at("connected"), "true");
  EXPECT_EQ(health.details.at("active"), "true");
  EXPECT_EQ(health.details.at("streaming"), "true");
  EXPECT_EQ(health.details.at("have_complete_feedback"), "true");
  EXPECT_EQ(health.details.at("feedback_fresh"), "true");
  EXPECT_EQ(health.details.at("mode"), "holding");
  EXPECT_EQ(health.details.at("state_topic"), prefix_ + "/joint_states");
  EXPECT_FALSE(health.details.count("last_feedback_error") == 0U);
  EXPECT_FALSE(health.details.count("last_command_error") == 0U);

  std::this_thread::sleep_for(30ms);
  health = driver.health();
  EXPECT_EQ(health.level, hdi::HealthLevel::kStale);
  EXPECT_EQ(health.details.at("feedback_fresh"), "false");
  EXPECT_EQ(health.details.at("mode"), "holding");
}

}  // namespace
