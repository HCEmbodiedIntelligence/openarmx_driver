# OpenArmX v10 双臂真机接入说明

本文说明如何把 HC 通用运动栈接到 OpenArmX v10 双臂真机。
目标是保持上层运动、VR 和 Action 接口不变；切换机器人时只更换 URDF、机器人 profile、
驱动参数和必要的厂商插件。

> 文档基于 2026-08-26 检查的 OpenArmX 官方 `6.0_basic` 分支。真机上电前应再次确认
> 官方仓库当前版本的控制器名称、关节顺序和启动参数。

## 1. 结论

OpenArmX 已经通过 `ros2_control` 提供关节反馈和位置命令接口。HC 不需要直接发送 CAN 帧，
应接在官方 `ros2_control` 控制器上层：

```text
VR / MoveJ / MoveL / MoveP
            |
            v
humanoid_motion_server
            |
            | /hc_teleop/joint_cmd       sensor_msgs/msg/JointState
            v
openarmx_driver/OpenArmXRos2ControlDriver
       |                         ^
       | 左右臂 Float64MultiArray | /joint_states
       v                         |
OpenArmX forward position controllers
            |
            v
OpenArmX ros2_control hardware plugin
            |
            v
        CAN / 电机
```

官方真机接口已经存在，但当前工作区的通用 `RosTopicRobotDriver` 不能直接作为真机适配器：

- OpenArmX 状态端使用 `sensor_msgs/msg/JointState`，与现有插件兼容；
- OpenArmX 命令端使用两个 `std_msgs/msg/Float64MultiArray` Topic；
- 当前 `RosTopicRobotDriver` 要求状态端和命令端都使用 `JointState`；
- 因此真机必须使用 `openarmx_driver/OpenArmXRos2ControlDriver`。

工作区现在通过独立包 `openarmx_driver` 提供这个很薄的 OpenArmX `RobotDriverPlugin`。控制算法、
Motion Action、VR 前端和平台统一 Topic 都不包含 OpenArmX 特殊逻辑。

## 2. 官方已经提供的接口

官方源码：

- [OpenArmX ROS 2 仓库](https://github.com/openarmx/openarmx_ros2/tree/6.0_basic)
- [v10 双臂控制器配置](https://github.com/openarmx/openarmx_ros2/blob/6.0_basic/openarmx_bringup/config/v10_controllers/openarmx_v10_bimanual_controllers.yaml)
- [v10 ros2_control 硬件实现](https://github.com/openarmx/openarmx_ros2/blob/6.0_basic/openarmx_hardware/src/v10_simple_hardware.cpp)
- [双臂启动文件](https://github.com/openarmx/openarmx_ros2/blob/6.0_basic/openarmx_bringup/launch/openarmx.bimanual.launch.py)
- [官方 VR 控制程序](https://github.com/openarmx/openarmx_teleop_vr/blob/6.0_basic/openarmx_teleop_vr/openarmx_teleop_vr/openarmx_teleop_vr_node.py)

默认不使用 ROS namespace 时，接口如下。

| 方向 | 接口 | 类型 | 说明 |
| --- | --- | --- | --- |
| 真机 → HC | `/joint_states` | `sensor_msgs/msg/JointState` | 左右臂及启用的夹爪反馈 |
| HC → 左臂 | `/left_forward_position_controller/commands` | `std_msgs/msg/Float64MultiArray` | 连续位置设定值 |
| HC → 右臂 | `/right_forward_position_controller/commands` | `std_msgs/msg/Float64MultiArray` | 连续位置设定值 |
| 轨迹控制 | `/left_joint_trajectory_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 左臂 7 轴轨迹 Action |
| 轨迹控制 | `/right_joint_trajectory_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 右臂 7 轴轨迹 Action |
| 夹爪控制 | `/left_gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | 轨迹控制器模式下使用 |
| 夹爪控制 | `/right_gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | 轨迹控制器模式下使用 |

这些接口只有在官方 `ros2_control_node`、`joint_state_broadcaster` 和对应控制器成功启动后才会出现。

官方还配置了速度和力矩 forward controller，但当前 HC 栈已经在 100 Hz 生成限速、限加速度的
位置目标，真机首版应只接 `forward_position_controller`。不要同时激活 forward position 和
joint trajectory 控制器，它们会争用相同的 position command interface。

### 2.1 位置命令数组顺序

官方标准夹爪配置中，每侧 forward position controller 接收 8 个值。消息没有关节名，
只能依靠固定数组下标。该默认配置不能与已经解耦的 HC 手臂/夹爪运行时直接共用。

左侧顺序：

```text
0  openarmx_left_joint1
1  openarmx_left_joint2
2  openarmx_left_joint3
3  openarmx_left_joint4
4  openarmx_left_joint5
5  openarmx_left_joint6
6  openarmx_left_joint7
7  openarmx_left_finger_joint1
```

右侧顺序：

```text
0  openarmx_right_joint1
1  openarmx_right_joint2
2  openarmx_right_joint3
3  openarmx_right_joint4
4  openarmx_right_joint5
5  openarmx_right_joint6
6  openarmx_right_joint7
7  openarmx_right_finger_joint1
```

当前 HC 的 `openarmx_driver` 严格只控制 14 个手臂关节，每侧只发布 7 个值。必须使用
`humanoid_gripper/config/v10_controllers/openarmx_v10_split_controllers.yaml`：左右手臂 controller
各声明 7 个手臂关节，左右夹爪 controller 各自只声明一个 `finger_joint1`。不能向官方 8 轴
controller 发送 7 个值，也不能让两个 controller 同时声明同一 position command interface。

### 2.2 Namespace

官方启动文件中的 Topic 使用相对名称。设置 `arm_prefix:=openarmx1` 后，接口会解析为：

```text
/openarmx1/joint_states
/openarmx1/left_forward_position_controller/commands
/openarmx1/right_forward_position_controller/commands
/openarmx1/left_gripper_controller/commands
/openarmx1/right_gripper_controller/commands
/openarmx1/controller_manager
```

手臂和夹爪插件配置都必须填写解析后的完整 Topic。使用 `arm_prefix:=openarmx1` 时，应在
manager 的“驱动参数”和“夹爪驱动”页把模板中的根话题改成上述 `/openarmx1/...` 话题。
不要假设所有部署都位于根 namespace；启动后以 `ros2 topic list` 和
`ros2 topic info -v` 的实际结果为准。

## 3. HC 侧 OpenArmX 插件

独立 ROS 2 包 `openarmx_driver` 导出：

```text
openarmx_driver/OpenArmXRos2ControlDriver
```

插件继承 `humanoid_driver_interface::RobotDriverPlugin`，通用 runtime 继续只按 `plugin_class`
加载它。不要在 `humanoid_driver_runtime`、`humanoid_motion_server` 或 VR 节点中增加机器人型号分支。

### 3.1 插件职责

`configure()`：

- 读取 state、左右手臂命令 Topic和反馈超时等参数；
- 建立逻辑关节名到 OpenArmX 官方关节名的映射；
- 校验左右臂各 7 个关节且没有重复、缺失或未知名称；
- 创建 `/joint_states` subscription 和左右 `Float64MultiArray` publisher。

`connect()`：

- 检查 ROS subscription 和左右 publisher 已创建；
- 不阻塞等待第一条反馈，因为 runtime 此时尚未把所属节点加入 executor；
- 第一条完整反馈通过回调完成 14 个手臂关节的校验；夹爪反馈由独立插件读取；
- `startup_grace_s` 内无反馈返回 `kNoFeedback`，超期后转为通信故障。

`activate()`：

- 如果已有反馈，将第一帧待发布命令初始化为测量位置；否则继续等待回调同步；
- 未完成“反馈同步 → 命令初始化”前不得发布位置命令；
- 禁止使用全零数组初始化，避免控制器激活瞬间跳到零位。

`writeJointCommand()`：

- 按消息中的逻辑关节名更新对应设定值，不依赖输入数组下标；
- 合并部分关节命令，未更新关节保持上一次安全命令；
- 最终按官方固定顺序拆成左右两个完整数组；
- 拒绝 NaN、Inf、重复关节、未知关节和错误长度；
- 使用 rad 和 rad/s，方向与零位转换留在插件 mapping 配置中；
- 两侧命令应在同一控制周期内发布。

`readJointState()`：

- 以 `JointState.name` 查找关节，不能依赖官方反馈数组顺序；
- 输出平台逻辑关节名和 SI 单位；
- 超过 `state_timeout_s` 没有新的有效反馈时返回通信故障。

`stopAll()`：

- 必须幂等；
- 有有效反馈时向两侧发送完整的当前测量位置，形成位置保持；
- 没有有效反馈时执行经真机验证的等价安全停机策略；
- 不能只停止 publisher，因为 forward controller 可能继续保持最后一个目标；
- 软件保持不能替代物理急停或电机侧安全回路。

`health()`：

- 至少报告首次反馈、反馈新鲜度、命令新鲜度、左右 publisher 连接数和最后一次错误；
- 最好同时纳入 OpenArmX 底层 CAN/电机故障状态，而不是只判断 ROS Topic 是否仍在发布。

### 3.2 目标真机配置

下面是插件当前支持的配置形态。工作区的根 Topic 真机配置见
`openarmx_driver/config/openarmx_v10_bimanual.yaml`；本节用 `/openarmx1` 展示显式
namespace 配置：

```yaml
humanoid_driver_runtime:
  ros__parameters:
    plugin_class: openarmx_driver/OpenArmXRos2ControlDriver

    platform_joint_state_topic: /hc_teleop/joint_states
    platform_joint_command_topic: /hc_teleop/joint_cmd
    diagnostics_topic: /diagnostics

    control_frequency_hz: 100.0
    command_watchdog_ms: 100.0
    diagnostic_frequency_hz: 10.0

    joint_names:
      - openarmx_left_joint1
      - openarmx_left_joint2
      - openarmx_left_joint3
      - openarmx_left_joint4
      - openarmx_left_joint5
      - openarmx_left_joint6
      - openarmx_left_joint7
      - openarmx_right_joint1
      - openarmx_right_joint2
      - openarmx_right_joint3
      - openarmx_right_joint4
      - openarmx_right_joint5
      - openarmx_right_joint6
      - openarmx_right_joint7

    vendor_joint_names:
      - openarmx_left_joint1
      - openarmx_left_joint2
      - openarmx_left_joint3
      - openarmx_left_joint4
      - openarmx_left_joint5
      - openarmx_left_joint6
      - openarmx_left_joint7
      - openarmx_right_joint1
      - openarmx_right_joint2
      - openarmx_right_joint3
      - openarmx_right_joint4
      - openarmx_right_joint5
      - openarmx_right_joint6
      - openarmx_right_joint7

    vendor_joint_groups:
      - left_arm
      - left_arm
      - left_arm
      - left_arm
      - left_arm
      - left_arm
      - left_arm
      - right_arm
      - right_arm
      - right_arm
      - right_arm
      - right_arm
      - right_arm
      - right_arm

    vendor_to_logical_scales:
      [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
       1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    vendor_to_logical_offsets_rad:
      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    plugin_parameters:
      - state_topic=/openarmx1/joint_states
      - left_command_topic=/openarmx1/left_forward_position_controller/commands
      - right_command_topic=/openarmx1/right_forward_position_controller/commands
      - state_timeout_s=0.25
      - startup_grace_s=15.0
```

真机驱动配置保存在 `openarmx_driver` 插件中，不安装到目标机 ROS 工作区。目标机只接收 manager
校验后的插件产物，避免通过手工修改 launch 或 YAML 切换机器人。

### 3.3 插件单元测试

至少覆盖以下情况：

1. `JointState` 输入顺序随机变化时，输出逻辑状态仍正确；
2. 缺少关节、重复关节、数组长度不一致、NaN 或 Inf 时拒绝反馈；
3. 14 轴整机命令正确拆分为左右两个 7 元素数组；
4. 单臂或部分关节命令只更新目标关节，其余位置保持；
5. 夹爪不出现在手臂命令数组中，夹爪反馈缺失不阻塞手臂反馈；
6. 反馈超时和启动阶段无反馈时报告通信故障；
7. `stopAll()` 重复调用安全，发布的是测量位置而不是零位；
8. namespace 改变后不需要修改插件源码；
9. 左右命令 Topic 没有 subscriber 时禁止激活或明确报告 degraded；
10. command watchdog 到期后进入保持状态且不会继续追踪旧的运动轨迹。

## 4. 真机启动方式

日常使用网页统一管理厂商驱动、HC、夹爪及相机时，使用
[真机日常启动](start_on_robot.md) 的单入口命令。
下文保留先单独运行厂商控制器、再启动 HC 的分层联调方式；两种方式不能同时启动同一套厂商驱动。

### 4.1 启动 OpenArmX 官方硬件和控制器

CAN 速率、CAN-FD 选择和网卡初始化必须按实际硬件版本及官方文档设置。不要仅根据下面的接口名
猜测总线参数。

确认硬件急停可用、机器人处于支撑状态且工作空间无人后，启动官方双臂控制器：

```bash
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
  runtime_config_package:=humanoid_gripper \
  controllers_file:=openarmx_v10_split_controllers.yaml \
  use_fake_hardware:=false \
  robot_controller:=forward_position_controller \
  control_mode:=mit \
  right_can_interface:=can0 \
  left_can_interface:=can1 \
  arm_prefix:=openarmx1
```

官方启动文件在 forward-position 模式下不会自动启动独立夹爪 controller。
更新后的 HC 夹爪插件通过清单 `startup` 声明所需控制器；第二步启动 HC 时会自动等待
controller_manager、加载并激活左右夹爪，不再需要手动运行 spawner。已激活的控制器会复用。
需要更新管理器、重新打包导入夹爪插件，并在机器人配置中选择它、保存。
本节使用 `/openarmx1`，打包前将 `humanoid_gripper/config/openarmx_v10_bimanual.startup.yaml`
中的 controller-manager 参数同步设为 `/openarmx1/controller_manager`；根命名空间使用默认值。

如果不设置 `arm_prefix`，后续检查命令中的 `/openarmx1` 前缀也应删除。

先不要启动 HC 命令链，完成只读检查：

```bash
ros2 control list_controllers -c /openarmx1/controller_manager
ros2 topic info -v /openarmx1/joint_states
ros2 topic hz /openarmx1/joint_states
ros2 topic echo --once /openarmx1/joint_states
ros2 topic info -v /openarmx1/left_forward_position_controller/commands
ros2 topic info -v /openarmx1/right_forward_position_controller/commands
```

期望结果：

- `joint_state_broadcaster`、左右 `forward_position_controller` 均为 `active`；
- `/joint_states` 类型为 `sensor_msgs/msg/JointState`；
- 状态包含左右各 7 个手臂关节以及左右 `finger_joint1`，但两类命令由不同 controller 接收；
- 状态位置均为有限值，缓慢人工移动或低功率测试时方向与 URDF 一致；
- 左右命令 Topic 各有且只有预期的控制器 subscriber。

### 4.2 启动 HC 栈

先在开发机生成并验证手臂驱动、模型和夹爪插件，再导入 `humanoid_manager`。随后在网页
“机器人配置”中选择这三个插件，创建 `openarmx_v10_bimanual`。组合清单由管理器内部生成。
首次联调必须关闭 VR：

```bash
PLUGIN_ROOT="$HOME/.local/share/humanoid-plugins"
ros2 run humanoid_manager humanoid_pluginctl.py --root "$PLUGIN_ROOT" deploy openarmx-driver.zip
ros2 run humanoid_manager humanoid_pluginctl.py --root "$PLUGIN_ROOT" deploy openarmx-v10-model.zip
ros2 run humanoid_manager humanoid_pluginctl.py --root "$PLUGIN_ROOT" deploy openarmx-gripper.zip
ros2 launch humanoid_manager managed_robot.launch.py \
  plugin_root:="$PLUGIN_ROOT" \
  robot_id:=openarmx_v10_bimanual \
  start_teleop:=false
```

`humanoid_manager` 只解析已部署的 `robot_id`，不包含 OpenArmX 名称、真机/仿真分支或厂商
上电逻辑。OpenArmX 官方控制器应先由整机 supervisor 启动并完成只读检查。

### 4.3 首次运动

不要用手工 `ros2 topic pub` 向真机发布全零或任意角度数组。首次运动应走完整 HC 安全链：

1. 先验证平台 `/hc_teleop/joint_states` 与官方 `/joint_states` 一致；
2. 确认插件激活后的首个命令等于当前测量位置，机器人不应跳动；
3. 保持 VR 和自动使能关闭；
4. 通过 `MoveJ` 只移动一个关节，增量不超过经现场批准的测试值；
5. 检查实际方向、零位、反馈、速度限制和停止行为；
6. 逐关节完成左右臂验证后，再测试单臂轨迹和双臂同步；
7. 最后才启用 VR，并重新校准手柄到机器人 base/TCP 的坐标映射。

## 5. 上真机前的阻断项

以下任一项未完成，都不应进入正常真机遥操作。

### 5.1 CAN 反馈新鲜度

截至本文检查的官方 `v10_simple_hardware.cpp`，`read()` 调用 `refresh_all()` 和 `recv_all()` 后
直接返回 `hardware_interface::return_type::OK`。仅观察 `/joint_states` 的 ROS 到达时间，可能无法
区分“新的电机反馈”和“控制循环重新发布的旧值”。

必须通过以下方式之一解决，并实际做拔线测试：

- 在官方 hardware plugin 中检查每个电机最后接收时间，超时后从 `read()` 返回 `ERROR`；或
- 由官方底层发布包含逐电机反馈时间和 CAN 故障的 diagnostics，HC 插件把它纳入 `health()`；或
- 使用厂商已经提供、且能证明基于真实 CAN 帧时间戳的等价故障接口。

拔掉任意一侧 CAN 或停止电机反馈后，系统必须在规定时间内退出运动状态并执行安全保持/停机。
如果 `/joint_states` 仍以正常频率发布旧数据并被 HC 判断为健康，则验收失败。

### 5.2 命令中断

必须分别验证：

- 停止 `/hc_teleop/joint_cmd`；
- 杀掉 `humanoid_motion_server`；
- 杀掉 `humanoid_driver_runtime`；
- VR 网络中断；
- OpenArmX controller subscriber 消失；
- 左右 CAN 任意一侧中断；
- 软件急停；
- 硬件急停。

每种情况下都要记录触发时间、停止时间、最终关节行为以及恢复条件。进程退出、Topic 消失和停止
publisher 本身不等于安全停止。

### 5.3 限位和坐标系

- 以实际机器人标定结果核对 URDF 关节零位、正方向、机械限位和软限位；
- 当前 motion profile 的限速是保守初始值，不是厂商额定值证明；
- 逐轴核对 `vendor_to_logical_scales` 和 `vendor_to_logical_offsets_rad`；
- 重新标定 VR `axis_mapping`、左右 base frame、TCP 和工具偏置；
- 检查自碰撞、工作台碰撞和线缆限制；当前运动栈的软件限位不能替代机械防护。

## 6. 真机验收清单

### 接口

- [ ] 官方 controller 名称、Topic 和消息类型与本文一致
- [ ] `/joint_states` 包含全部预期关节且没有重复名称
- [ ] 左右命令数组长度及顺序与当前 controller YAML 一致
- [ ] 插件可以通过 namespace 参数切换机器人实例
- [ ] 目标机没有安装或编译 OpenArmX driver/description 源码包

### 启动

- [ ] 默认不自动使能运动和 VR
- [ ] 未收到完整反馈时插件可以处于启动等待状态，但不能发布任何位置命令
- [ ] 激活后的首命令等于测量位置，没有回零跳变
- [ ] 只有一个节点拥有 `/hc_teleop/joint_cmd` 的权威发布权
- [ ] 只有预期插件向左右 OpenArmX command Topic 发布

### 运动

- [ ] 14 个手臂关节逐轴方向正确
- [ ] 左右臂零位和软限位正确
- [ ] 夹爪保持/开合方向和范围正确
- [ ] MoveJ、MoveL、MoveP 分别完成低速测试
- [ ] 双臂同步命令不会错位或交换左右臂
- [ ] VR base/TCP 坐标映射完成现场标定

### 故障与停止

- [ ] command watchdog 触发后安全保持
- [ ] `/joint_states` 超时后停止
- [ ] CAN 旧数据不会被当成新反馈
- [ ] 左 CAN、右 CAN 分别断线时行为符合设计
- [ ] 软件急停和硬件急停均通过测试
- [ ] 故障清除后必须显式重新使能，不会自动恢复运动

## 7. 多机器人复用边界

如果“多机器人共用一套”是指同一套软件在不同机器人型号之间切换，当前设计方向正确。每个型号只需
提供：

```text
URDF/Xacro
motion/profile YAML
RobotDriverPlugin 及其驱动参数
关节映射、限位、运动学和工具配置
robot_model 插件与 robot_composition 组合清单
```

以下上层接口保持不变：

```text
/hc_teleop/joint_states
/hc_teleop/joint_cmd
/motion/<group>/move_j
/motion/<group>/move_l
/motion/<group>/move_p
/teleop/<group>/servo_p
```

如果目标是在同一个 ROS domain 中同时控制多台机器人，当前绝对名称 `/hc_teleop/*` 会冲突，TF frame
名称也可能冲突。此场景需要进一步把平台 endpoint、Action、diagnostics 和 TF frame 全部实例化到
机器人 namespace，或者每台机器人使用独立 `ROS_DOMAIN_ID`。仅给 OpenArmX 官方 Topic 加 namespace
不足以完成多机并发隔离。

## 8. 与当前工作区文件的对应关系

- OpenArmX 真机驱动配置：
  [`openarmx_v10_bimanual.yaml`](../config/openarmx_v10_bimanual.yaml)
- OpenArmX motion 参数：
  [`motion.yaml`](../../openarmx_description/deployment/openarmx_v10_bimanual/model/motion.yaml)
- OpenArmX channel 注册：
  [`channels.yaml`](../../openarmx_description/deployment/openarmx_v10_bimanual/model/channels.yaml)
- OpenArmX robot_model 打包器：
  [`create_deployment_bundle.py`](../../openarmx_description/tools/create_deployment_bundle.py)
- 通用 ROS Topic 机器人说明：
  [`adding_ros_topic_robot.md`](../../humanoid_driver_runtime/docs/adding_ros_topic_robot.md)
- 独立驱动插件说明：
  [`adding_driver_plugin.md`](../../humanoid_driver_runtime/docs/adding_driver_plugin.md)

下一步实施顺序应为：部署两个插件及组合清单 → 补齐 CAN 反馈新鲜度 → 完成台架停止测试 → 逐轴低速
验收 → 最后启用 VR。
