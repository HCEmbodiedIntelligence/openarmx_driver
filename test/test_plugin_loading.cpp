// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <memory>

#include <gtest/gtest.h>

#include "humanoid_driver_interface/robot_driver_plugin.hpp"
#include "humanoid_driver_interface/ros2_driver_plugin.hpp"
#include "pluginlib/class_loader.hpp"

namespace hdi = humanoid_driver_interface;

TEST(PluginLoading, LoadsOpenArmXDriverByDeclaredClassName)
{
  pluginlib::ClassLoader<hdi::RobotDriverPlugin> loader(
    "humanoid_driver_interface", "humanoid_driver_interface::RobotDriverPlugin");
  auto plugin = loader.createSharedInstance("openarmx_driver/OpenArmXRos2ControlDriver");
  ASSERT_NE(plugin, nullptr);
  EXPECT_NE(dynamic_cast<hdi::Ros2DriverPlugin *>(plugin.get()), nullptr);
}
