# DAC_SFC

DAC-SFC 在 EGO-Planner-v2 框架中集成路径搜索、稀疏路径生成、CSGN、Active-Witness 安全飞行走廊和 GCOPTER 轨迹优化，并通过 PX4 与 Gazebo 完成闭环飞行仿真。

## Demo

https://github.com/user-attachments/assets/2d2e5cf1-5618-469b-aa0f-b5d681f7ddbc

## Simulation Tutorial

本教程使用 [feat/dac-sfc-realflight 分支](https://github.com/wangyuxuan125/DAC_SFC_real/tree/feat/dac-sfc-realflight)的单机仿真入口，按环境配置、编译、启动和交互的顺序介绍复现步骤。`main` 分支用于展示说明，仿真代码位于上述功能分支。

### 1. 环境准备

**运行本教程中的仿真需要 Gazebo 环境。** 还需安装 ROS 1、MAVROS，并提前配置和编译兼容本项目的 PX4 SITL / XTDrone 环境。

下面的安装命令以 **Ubuntu 20.04 + ROS Noetic + Gazebo Classic 11** 为例；PX4 和 XTDrone 版本应与所用模型、插件及启动文件配套。环境安装可参考 [XTDrone 使用文档](https://www.yuque.com/xtdrone/manual_cn)和 [PX4 的 ROS / Gazebo 配置说明](https://docs.px4.io/v1.13/en/simulation/ros_interface)。本分支采用 ROS 1 的 `gazebo_ros` 接口。

| 组件 | 用途及要求 |
| --- | --- |
| ROS 1 / catkin | 编译工作空间并运行规划、控制和可视化节点 |
| Gazebo Classic / gazebo_ros | 物理仿真、场景和深度相机数据 |
| PX4 SITL / XTDrone | 飞控仿真；须提供 `single_vehicle_spawn_xtd.launch` |
| MAVROS | PX4 与 ROS 通信；须安装 GeographicLib 数据集 |
| QGroundControl | 本教程用于起飞、进入 Hold 模式和降落 |
| Eigen3、PCL、OpenCV、Armadillo、Qt5 | 规划、地图、控制和 RViz 插件的编译依赖 |

在已安装 ROS Noetic 并配置软件源的环境中，安装本项目使用的依赖：

```bash
source /opt/ros/noetic/setup.bash
sudo apt update
sudo apt install build-essential cmake git wget \
  libeigen3-dev libpcl-dev libopencv-dev libarmadillo-dev \
  qtbase5-dev qtdeclarative5-dev \
  ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
  ros-noetic-mavros ros-noetic-mavros-extras \
  ros-noetic-rviz ros-noetic-rqt-reconfigure \
  ros-noetic-cv-bridge ros-noetic-pcl-ros ros-noetic-pcl-conversions \
  ros-noetic-image-transport ros-noetic-tf ros-noetic-tf2-ros \
  ros-noetic-cmake-modules ros-noetic-roslint
```

按 [MAVROS 安装说明](https://docs.px4.io/v1.14/en/ros/mavros_installation)安装 GeographicLib 数据集；已安装时可跳过：

```bash
mkdir -p ~/dac_sfc_deps
cd ~/dac_sfc_deps
wget -O install_geographiclib_datasets.sh \
  https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash install_geographiclib_datasets.sh
```

### 2. 获取代码并编译

```bash
cd ~
git clone --branch feat/dac-sfc-realflight --single-branch \
  https://github.com/wangyuxuan125/DAC_SFC_real.git
cd ~/DAC_SFC_real/src/main_ws
```

**实际 catkin 工作空间是 `DAC_SFC_real/src/main_ws`。** 后续在该目录执行编译。

当前分支引用了仓库外部的 `decomp_ros_msgs`。若现有工作空间尚未提供它，可从 [DecompROS](https://github.com/sikang/DecompROS)获取消息包，并补充其 `catkin_simple` 依赖：

```bash
cd ~/DAC_SFC_real/src/main_ws
git clone --depth 1 https://github.com/catkin/catkin_simple.git src/catkin_simple
git clone --depth 1 --filter=blob:none --sparse \
  https://github.com/sikang/DecompROS.git src/DecompROS
git -C src/DecompROS sparse-checkout set decomp_ros_msgs
```

如果这些包已在其他工作空间编译，先加载该工作空间的 `devel/setup.bash`，避免重复引入同名包。

当前分支的 `rviz_plugins` 还保留了 `multi_map_server` 的旧编译依赖，而仓库未包含该包。对于独立编译本教程的工作空间，先将其替换为当前 ROS 消息依赖，并移除对应的旧包声明：

```bash
cd ~/DAC_SFC_real/src/main_ws
sed -i \
  's/add_dependencies(${PROJECT_NAME} multi_map_server_generate_messages_cpp)/add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})/' \
  src/Utils/rviz_plugins/CMakeLists.txt
sed -i '/multi_map_server/d' src/Utils/rviz_plugins/package.xml

catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
rospack find ego_planner
rospack find px4ctrl
```

### 3. 加载 PX4 / Gazebo 环境

以下以已经配置好 XTDrone 的 `~/PX4_Firmware` 为例，请将路径改为本机实际位置。先完成该 PX4 环境的 SITL 和 Gazebo 插件编译，再加载环境：

```bash
source /opt/ros/noetic/setup.bash
source ~/DAC_SFC_real/src/main_ws/devel/setup.bash
export DAC_SFC_PX4_DIR=~/PX4_Firmware
source "$DAC_SFC_PX4_DIR/Tools/setup_gazebo.bash" \
  "$DAC_SFC_PX4_DIR" "$DAC_SFC_PX4_DIR/build/px4_sitl_default"
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$DAC_SFC_PX4_DIR:$DAC_SFC_PX4_DIR/Tools/sitl_gazebo"
```

上述路径对应本项目采用的 PX4 / XTDrone 目录结构。仿真入口还会使用以下外部资源，请在启动前确认它们存在：

```bash
gazebo --version
rospack find px4
rospack find mavlink_sitl_gazebo
test -f "$(rospack find px4)/launch/single_vehicle_spawn_xtd.launch"
test -f "$(rospack find mavlink_sitl_gazebo)/worlds/ego.world"
test -d "$(rospack find mavlink_sitl_gazebo)/models/iris_realsense_camera"
```

`ego.world`、`iris_realsense_camera` 及 `single_vehicle_spawn_xtd.launch` 由外部 PX4 / XTDrone 环境提供，需要提前配置到对应目录。仅克隆本仓库和安装 Gazebo，尚不能补齐这些资源。

### 4. 启动单机仿真

在已加载上述环境的终端中运行：

```bash
roslaunch ego_planner dac_sfc_single_sim.launch planning_horizon:=5.0
```

该命令会启动 Gazebo、PX4 SITL、带深度相机的 Iris 模型、MAVROS、`px4ctrl`、规划器、轨迹服务器和 RViz。完整入口已包含飞控及仿真节点，一次运行该入口即可。

这里显式设置 `planning_horizon:=5.0`，使局部目标位于当前配置的 5.5 m 局部地图更新范围内。需要更换场景时，可传入 `world:=/absolute/path/to/your.world`。

默认启用 **shadow 模式**：DAC-SFC 执行完整规划流程并记录结果，飞行控制使用 EGO 轨迹。

### 5. 起飞并发送目标点

新开一个终端，加载 ROS 和本项目环境：

```bash
source /opt/ros/noetic/setup.bash
source ~/DAC_SFC_real/src/main_ws/devel/setup.bash
rostopic echo -n 1 /iris_0/mavros/state
rostopic hz /iris_0/mavros/local_position/odom
rostopic hz /iris_0/realsense/depth_camera/depth/image_raw
```

确认 MAVROS 的 `connected` 为 `True`，里程计和深度图持续发布；各个 `hz` 检查用 `Ctrl+C` 结束。

本分支的仿真控制器配置启用了 `no_RC`，通过动态参数模拟控制开关。按[控制器说明](https://github.com/wangyuxuan125/DAC_SFC_real/blob/feat/dac-sfc-realflight/src/main_ws/src/realflight_modules/px4ctrl/README.md)中的 Gazebo 操作流程起飞并开启轨迹接收：

1. 打开 QGroundControl 并连接仿真中的 PX4，在设置中启用虚拟摇杆。保持控制器动态参数 `mode_bool=false`、`cmd_bool=false`，使用 QGroundControl 的 **Takeoff** 按钮起飞，等待进入 **Hold** 模式并稳定悬停。

2. 运行下面的命令，并在 `/iris_0/px4ctrl` 中依次勾选 `mode_bool`、`cmd_bool`：

   ```bash
   rosrun rqt_reconfigure rqt_reconfigure
   ```

   `mode_bool` 开启悬停 / Offboard 控制，`cmd_bool` 允许接收轨迹指令。这两个开关须在起飞完成后开启。

3. 在 RViz 中使用 **3D Nav Goal**（快捷键 `g`）：按住左键选择平面位置和方向，保持左键并按住右键、上下移动鼠标调整高度，松开左键发送。先选择附近空旷区域、高度约 1 m 的目标。

也可以直接发送带无人机编号的三维目标。坐标单位为米，使用规划器的 `world` 坐标；下列数值仅展示消息格式，运行时应改为地图内可达的空闲位置：

```bash
rostopic pub -1 /goal_with_id quadrotor_msgs/GoalSet \
  '{drone_id: 0, goal: [2.0, 0.0, 1.0]}'
```

本入口使用 `/goal_with_id` 接收 `quadrotor_msgs/GoalSet`；RViz 的普通 **2D Nav Goal** 发布 `PoseStamped`，需通过其他节点转换后才能接入此接口。可检查终端中的 `Received goal` 和 `[DAC-SFC]` 输出，确认目标已触发规划。

### 6. 执行 DAC-SFC 轨迹

完成 shadow 验证、确认规划成功且碰撞检查通过后，先结束本次仿真，再以较低速度重新启动：

```bash
roslaunch ego_planner dac_sfc_single_sim.launch \
  dac_sfc_shadow_only:=false \
  dac_sfc_fallback_to_ego:=true \
  max_vel:=0.5 max_acc:=1.0 planning_horizon:=5.0
```

重复上一节的起飞、模式切换和目标点发送步骤。此时，成功且通过检查的 DAC-SFC 轨迹会交给控制器执行；规划失败时保留 EGO 回退。

| 参数 | 作用 |
| --- | --- |
| `dac_sfc_shadow_only` | 默认 `true`；设为 `false` 后允许执行 DAC-SFC 轨迹 |
| `dac_sfc_fallback_to_ego` | 默认 `true`；DAC-SFC 失败时回退至 EGO |
| `max_vel` / `max_acc` | 速度和加速度上限，单位分别为 m/s、m/s² |
| `planning_horizon` | 局部规划距离；本教程显式使用 5.0 m |
| `world` / `gui` | Gazebo 场景路径 / 是否显示 Gazebo 窗口 |

### 7. 查看结果与结束仿真

RViz 中可观察地图、A* 路径和执行轨迹。当前代码按会话记录 CSV 日志，终端会打印 `[DAC-SFC] Session CSV:` 及文件路径。查看最新会话：

```bash
cat /tmp/dac_sfc_deployment/latest_csv_path.txt
tail -f "$(cat /tmp/dac_sfc_deployment/latest_csv_path.txt)"
```

汇总最新会话的规划成功率、轨迹激活次数、EGO 回退次数和规划耗时：

```bash
cd ~/DAC_SFC_real
python3 src/main_ws/src/planner/dac_sfc/tools/summarize_deployment.py
```

重点关注 `pipeline_success`、`sampled_collision_free`、`trajectory_activated`、`ego_fallback_used` 和 `failure_stage`。shadow 模式中 `trajectory_activated=0` 符合预期；active 模式下应结合 `trajectory_activated=1` 判断是否实际采用 DAC-SFC 轨迹。

结束时，先在动态参数中取消 `cmd_bool`，等待控制器回到 `AUTO_HOVER`；再取消 `mode_bool`，通过 QGroundControl 切换到 **Land** 模式降落。等待着陆并上锁后，在启动仿真的终端按 `Ctrl+C` 退出。

### 8. 常见问题

| 现象 | 处理方法 |
| --- | --- |
| 找不到 `px4`、`mavlink_sitl_gazebo` 或 `setup_gazebo.bash` | 检查 PX4 / XTDrone 安装目录、版本及第 3 节的环境加载步骤 |
| 缺少 `single_vehicle_spawn_xtd.launch`、`ego.world` 或模型 | 补齐外部 XTDrone 启动文件、场景及模型，并重新加载 Gazebo 环境 |
| 找不到 `decomp_ros_msgs`，或提示 `multi_map_server_generate_messages_cpp` 不存在 | 完成第 2 节的消息包安装和旧 RViz 依赖处理 |
| MAVROS 因 GeographicLib 数据缺失而退出 | 安装第 1 节的 GeographicLib 数据集 |
| `connected=False` 或里程计无数据 | 检查 PX4 SITL 启动输出和 MAVROS 连接；本入口使用 `udp://:24540@localhost:34580` |
| 无深度图或地图不更新 | 检查 `iris_realsense_camera` 模型、相机插件和第 5 节的深度图话题 |
| 飞机已悬停但不跟踪轨迹 | 检查 `mode_bool`、`cmd_bool`、`/iris_0/position_cmd` 和目标的 `drone_id=0` |
| 默认启动后仍执行 EGO 轨迹 | 默认是 shadow 模式；按第 6 节显式切换到 active 模式，并核对 CSV 的激活及回退字段 |
| 找不到旧文档中的 `dac_sfc_deployment.csv` | 使用 `latest_csv_path.txt` 指向的当前会话文件 |

更多算法流程、部署参数和验证要求见功能分支的 [部署验证文档](https://github.com/wangyuxuan125/DAC_SFC_real/blob/feat/dac-sfc-realflight/docs/dac_sfc_deployment_validation.md)。

## Acknowledgements

本项目使用或参考了 [EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2)、[GCOPTER](https://github.com/wangyuxuan125/GCOPTER)、[XTDrone](https://github.com/robin-shaun/XTDrone)、[PX4](https://github.com/PX4/PX4-Autopilot)和 [MAVROS](https://github.com/mavlink/mavros)。
