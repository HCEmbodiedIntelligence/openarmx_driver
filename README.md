# OpenArmX driver plugin

统一下载、构建及完整管理器导入包由 `robot_bringup/workspace.sh bundle` 提供，见
[工作区说明](https://github.com/HCEmbodiedIntelligence/robot_bringup/blob/main/docs/workspace.md)。
OpenArmX 模型部署模板随本仓库的 `deployment/openarmx_v10_bimanual` 管理；
`tools/create_model_bundle.py` 从这些模板和官方 `openarmx_description` 的 URDF 生成模型插件。
无需修改官方模型仓库，也不依赖该仓库的未跟踪文件。

从原来的“两条 launch”迁移到统一入口，见 [真机日常启动](docs/start_on_robot.md)。
该示例使用左 `can0`、右 `can1`，由入口管理厂商驱动、夹爪初始化、算法和已配置的相机。

`openarmx_driver` is a device-layer plugin for `humanoid_driver_runtime`. It reuses the runtime's
`rclcpp::Node` and bridges the platform joint API to the official OpenArmX v10 bimanual
`ros2_control` topics. The package builds only the shared library
`openarmx_ros2_control_driver`; it does not create a node, executor, CAN connection, or hardware
SDK client.

Plugin class:

```text
openarmx_driver/OpenArmXRos2ControlDriver
```

## ROS interfaces

The three topic parameters are required. A root-topic real-hardware configuration normally uses:

| Direction | Parameter | Default deployment topic | Type |
| --- | --- | --- | --- |
| input | `state_topic` | `/joint_states` | `sensor_msgs/msg/JointState` |
| output | `left_command_topic` | `/left_forward_position_controller/commands` | `std_msgs/msg/Float64MultiArray` |
| output | `right_command_topic` | `/right_forward_position_controller/commands` | `std_msgs/msg/Float64MultiArray` |

Each output contains exactly the seven arm joints. Grippers are deliberately excluded from this
package and are configured as independent `humanoid_gripper` plugins and ros2_control controllers.

The plugin never publishes to the forward effort controllers. OpenArmX gravity compensation owns
those topics and the MIT `tau_ff` path. Real deployments are expected to use OpenArmX's
`control_mode:=mit`, `robot_controller:=forward_position_controller`, and
`enable_forward_effort:=true`; KP/KD remain owned by the OpenArmX hardware parameter node.

## Plugin parameters

| Parameter | Required/default | Meaning |
| --- | --- | --- |
| `state_topic` | required | Fully resolved OpenArmX joint-state topic |
| `left_command_topic` | required | Fully resolved left position-controller command topic |
| `right_command_topic` | required | Fully resolved right position-controller command topic |
| `left_group` | `left_arm` | Expected vendor group for left mappings |
| `right_group` | `right_arm` | Expected vendor group for right mappings |
| `state_timeout_s` | `0.25` | Positive feedback freshness timeout |
| `startup_grace_s` | `15.0` | Positive grace period for the first executor-delivered state |

Unknown parameters and invalid mappings are rejected. The platform configuration must contain
exactly the 14 official arm vendor names, split across the configured left/right groups. Logical
names may differ, and every state/command conversion applies the configured scale and zero offset.

## Safety behavior

No command is published by `configure()`, `connect()`, or `activate()`. The first complete feedback
sample seeds every arm target. Commands may update any subset of platform joints; all other targets
retain their previous safe values. `stopAll()` is idempotent and publishes the latest measured arm
positions. If the last sample is stale it still attempts that
hold and reports the stale condition; without any valid sample it returns `kNoFeedback`.

This software hold is not a replacement for a physical emergency stop or motor-side safety system.

Real-hardware topology and acceptance checks are documented in
[`docs/openarmx_v10_real_hardware.md`](docs/openarmx_v10_real_hardware.md).

## No-build deployment bundle

Compile this package on the matching ROS 2 Humble architecture on a development machine or in CI,
then stage its installed library, driver parameters, and plugin metadata as an upload bundle:

```bash
python3 src/openarmx_driver/tools/create_deployment_bundle.py \
  "$(ros2 pkg prefix openarmx_driver)" openarmx-driver.zip
```

The target deploys the resulting ZIP through
`ros2 run humanoid_manager humanoid_pluginctl.py`; deploying the same plugin ID overwrites
its directory. The target does not install the `openarmx_driver` source tree and does not run a
compiler.
