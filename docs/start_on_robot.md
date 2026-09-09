# OpenArmX v10 双臂真机日常启动

适用现场参数：左臂 `can0`、右臂 `can1`，经典 CAN 1 Mbit/s，MIT 模式，
每臂 7 轴位置控制器，夹爪独立控制器。工作区已更新并编译，机器人配置已部署。
接口名、比特率和控制模式来自该机器现有启动参数；其他机器按其实际配置填写。

## 当前自动启动能力

- `registered_robot.launch.py` 支持通过插件 `startup` 或 `vendor_*` 参数启动厂商节点。
- 当前 OpenArmX 手臂插件打包器没有默认写入厂商 launch；只传 `robot_id` 不能替代原来的第一条 launch。
- 新版 OpenArmX 夹爪插件会等待 `controller_manager`，加载并激活左右夹爪控制器。
  这依赖已重新打包导入的夹爪插件，并在机器人配置中选择、保存该实例。
- 相机按机器人保存的配置启动；手动曝光/时间验收脚本由用户按需运行。

以下使用已有的 `vendor_*` 接口把厂商 launch 交给机器人子进程管理。
厂商节点参数保留在 OpenArmX 的集成配置中，通用平台不根据机器人名称添加启动项。

## 1. 初始化 CAN

先停止上一轮机器人进程，再执行现有 CAN 初始化命令：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
```

CAN 初始化仍属于主机网络配置；当前统一 launch 不执行 sudo 网络操作。

## 2. 用一条 launch 启动

不再另外执行 `ros2 launch openarmx_bringup ...`。
如果插件启动配置中已经添加了相同厂商 launch，则使用该插件配置，并省略下面的三个 `vendor_*` 参数，避免重复。

本例统一使用 ROS domain 0。若现场使用其他 domain，所有相关终端与网页设置必须使用同一个值。
网页未配置时默认 domain 为 14，因此不能只依赖另一个终端默认的 domain 0。

```bash
source /home/hc_op/workspace/teleop_ws/install/setup.bash
export ROS_DOMAIN_ID=0

ros2 launch robot_bringup registered_robot.launch.py \
  robot_id:=openarmx_v10_bimanual \
  plugin_root:=/home/hc_op/workspace/teleop_ws/deployed_plugins \
  domain_id:="$ROS_DOMAIN_ID" \
  start_teleop:=false \
  start_cameras:=true \
  vendor_package:=openarmx_bringup \
  vendor_launch_file:=openarmx.bimanual.launch.py \
  vendor_arguments:='{
    "runtime_config_package": "humanoid_gripper",
    "controllers_file": "openarmx_v10_split_controllers.yaml",
    "use_fake_hardware": "false",
    "robot_controller": "forward_position_controller",
    "control_mode": "mit",
    "right_can_interface": "can1",
    "left_can_interface": "can0",
    "can_fd": "false",
    "enable_forward_effort": "true"
  }'
```

这里显式传入 `robot_id`，网页服务就绪后会尝试启动该已保存机器人。
打开 `http://机器人IP:7876/dashboard/#robots` 查看启动结果、日志及手动测试。
厂商控制器、HC、夹爪运行时和已配置相机均属于被管理的机器人子进程，随网页“关闭/重启机器人”一起操作。
初始化失败会显示失败原因；不代表硬件已通过验收。

上面未设置厂商 namespace，机械臂插件的话题应为根路径 `/joint_states`、
`/left_forward_position_controller/commands` 和 `/right_forward_position_controller/commands`；
夹爪插件的 `instance_parameters.namespace` 使用空字符串。

## 3. 更新旧夹爪插件（需要时执行一次）

源码同步不会替换 `deployed_plugins` 中已有的插件副本。
在相同架构、已编译的工作区生成新版插件：

```bash
python3 src/humanoid_gripper/tools/create_deployment_bundle.py \
  "$(ros2 pkg prefix humanoid_gripper)" \
  /tmp/openarmx-v10-gripper.zip \
  --config openarmx_v10_bimanual.yaml \
  --plugin-id openarmx_v10_bimanual_gripper
```

在页面停止机器人，导入该 ZIP，在现有机器人配置中选择新版夹爪实例并保存，再启动机器人。
首次配置时应先完成本节；如果启动时出现夹爪控制器缺失或没有夹爪实例，检查导入及选择是否完成。

## 4. 日常修改和测试

- 修改模型参数（例如 `base_frame`）后，在页面保存配置，再点击“重启机器人”。
  使用统一入口时，这会重启整套被管理的机器人子进程，网页保持运行。
- 夹爪开合、相机拍照使用页面对应设备的测试按钮。
- `/hc_teleop/joint_cmd` 及 `sensor_msgs/msg/JointState` 接口没有改变；逻辑关节名来自所选模型。
  手动发关节目标时必须避免算法或其他节点同时发布目标；原文的 `目标位置` 仍是需要替换的占位符，单位为弧度。
- 相机曝光、补偿响应和时间戳检查使用
  [手动验收脚本](https://github.com/HCEmbodiedIntelligence/humanoid_camera/blob/main/docs/manual_acceptance.md)。
  测试终端同样设置 `ROS_DOMAIN_ID=0`。

## 保留原来的分层联调方式

需要分别调试厂商层与 HC 时，可以保留原来的两条 launch：第一条的手臂/夹爪拆分配置继续使用；
第二条添加 `domain_id:=0` 并保留 `robot_id`、`plugin_root`、`start_teleop:=false`。
所有终端设置 `ROS_DOMAIN_ID=0`，并确保插件没有重复声明第一条厂商 launch。
这种方式下网页只管理 HC 子进程及插件启动项，单独启动的厂商终端需要手动退出。
完全不用网页时，第二条可加 `web:=false`，ROS domain 由该终端环境决定。
