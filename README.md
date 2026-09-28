# 2027 赛季 RoboCon 导航与仿真系统 (Nav27)

**全国大学生机器人大赛（ABU Robocon 2027 女娲补天 / The Pursuit of Mustika Nusantara）仿真与自主导航工作空间**

本项目深度融合了：
1. **仿真物理环境**（源自北极熊战队 `at26_rc_nav_test02/sim`）：Ignition Gazebo 仿真世界、`ros_gz_bridge` 传感器/控制桥接、`MecanumDrive2` 全向力控底盘插件、`SimpleStairClimber` 爬台阶插件及全套机器人描述。
2. **核心导航全栈**（源自 A&T 战队 `wulin_r2`，并融合 `at26_rc_nav_test02/nav` 仿真适配）：全向 MPPI 轨迹优化控制器（`motion_model: Omni`）、`PreciseGoalChecker`、Point-LIO 激光惯性里程计、基于 2027 赛场先验点云（`rc2027_field.pcd`）的 `small_gicp` 毫米级全局重定位、局部/全局地形感知（`terrain_analysis`）以及行为树架构。
3. **2027 赛场官方标准**（源自 `rc2027_field`）：高精度 3D 网格模型、25 处精确碰撞基元（地面、双层台阶、坡道、外围围栏、中央柱与五色石基座）、24.8 万点高精先验点云与 2D 占用栅格地图。

---

## 一、工作空间组织架构

为保持模块解耦和工程清晰，本项目分为两个独立的 ROS 2 工作空间：

```text
nav27/
├── sim/                              # 1. 仿真物理工作空间 (Ignition Gazebo)
│   ├── src/
│   │   ├── rmu_gazebo_simulator/     # 2027 赛场世界配置 (rc_2027_world.sdf)、模型与参数桥接
│   │   ├── pb2025_robot_description/ # AT_R2 全向麦轮底盘模型、Livox Mid360 雷达与 IMU 描述
│   │   ├── rmoss_gazebo/             # 仿真动力学插件 (libMecanumDrive2, SimpleStairClimber)
│   │   ├── rmoss_core/               # 基础核心工具库
│   │   ├── rmoss_gz_resources/       # 麦克纳姆轮网格与仿真资源
│   │   ├── rmoss_interfaces/         # 仿真通信接口定义
│   │   ├── sdformat_tools/           # SDF / URDF 解析与生成工具
│   │   └── at_r2_control/            # 仿真键盘遥控节点
│   └── run_sim.sh                    # 仿真一键启动脚本
│
├── nav/                              # 2. 导航定位工作空间 (Nav2 + LIO + GICP + RViz2)
│   ├── src/
│   │   ├── action/
│   │   │   ├── at_r2_nav_bringup/    # 导航总入口：2027 赛场 launch、2D地图、先验PCD、Nav2参数
│   │   │   ├── at_r2_bt/             # 行为树任务框架 (BehaviorTree.CPP)
│   │   │   ├── virtual_serial_port/  # 虚拟串口模拟 (仿真解耦)
│   │   │   └── at_r2_serial_bridge/  # 实车硬件串口通信
│   │   ├── location/
│   │   │   ├── point_lio/            # 激光惯性里程计 (高频实时里程计)
│   │   │   ├── small_gicp/           # small_gicp 点云配准核心库
│   │   │   ├── small_gicp_relocalization/ # 基于 2027 PCD 的快速全局重定位 (发布 map->odom)
│   │   │   ├── loam_interface/       # 里程计坐标系适配
│   │   │   ├── sensor_scan_generation/ # 点云系转换与底盘 TF 发布 (odom->chassis)
│   │   │   ├── pointcloud_to_laserscan/ # 地形点云转 2D LaserScan 供代价地图使用
│   │   │   ├── ign_sim_pointcloud_tool/ # 仿真点云适配工具 (为 GZ Mid360 点云添加 ring/time)
│   │   │   ├── costmap_converter/    # 代价地图多边形转换器
│   │   │   └── livox_ros_driver2/    # 实车 Livox 驱动 (保留实车移植兼容)
│   │   ├── navigation/
│   │   │   ├── at_nav2_plugins/      # 自定义 Nav2 插件 (PreciseGoalChecker 等)
│   │   │   ├── pb_omni_pid_pursuit_controller/ # 全向 PID 跟踪控制器
│   │   │   ├── teb_local_planner/    # TEB 全向局部规划器
│   │   │   ├── terrain_analysis/     # 4m 局部地形障碍分析
│   │   │   └── terrain_analysis_ext/ # 全局地形障碍分析
│   │   ├── robot/
│   │   │   ├── at_r2_robot_description/ # 机器人本体 TF 与 URDF 描述
│   │   │   ├── at_r2_control/        # 底盘控制工具
│   │   │   ├── robot_resources/      # 机器人模型资源
│   │   │   └── sdformat_tools/       # SDF 格式工具
│   │   └── third_party/
│   │       └── BehaviorTree.CPP/     # BT 库
│   ├── run_navigation.sh             # 导航一键启动脚本
│   ├── run_navigation_red.sh         # 红区专用启动脚本
│   └── run_navigation_blue.sh        # 蓝区专用启动脚本
│
├── run_sim.sh                        # 根目录一键启动仿真快捷方式 -> sim/run_sim.sh
├── run_nav.sh                        # 根目录一键启动导航快捷方式 -> nav/run_navigation.sh
└── README.md                         # 本说明文档
```

---

## 二、赛场规格与坐标系

- **世界坐标系**：场地中心为坐标原点 `(0, 0, 0)`，单位为米，Z 轴垂直向上。
- **场地尺寸**：`11.1m × 11.1m`（含外围围栏）。
- **红方出发区（Red Start Zone）**：位于西南侧，默认位姿 `(-4.75, -5.15, 0.15, yaw=1.5708)`（朝向北侧）。
- **蓝方出发区（Blue Start Zone）**：位于东南侧，默认位姿 `(4.75, -5.15, 0.15, yaw=1.5708)`（朝向北侧）。
- **关键地标**：
  - 中央基座：`(0.0, 0.0)`，高 0.8m（顶面 z=1.725m）。
  - 五色石（Mustika）基座：`(0.0, 4.25)`，高 0.5m，顶部有直径 180mm 凹槽。
  - 第一层平台（L1）：`6m × 6m`，高 0.6m，西侧（红）与东侧（蓝）各有过渡带（南侧为 3 级台阶，北侧为 9.74° 坡道）。

---

## 三、快速开始

### 1. 编译工程

在两个工作空间分别执行编译（推荐加上 `--symlink-install`）：

```bash
# 终端 1：编译仿真工作空间
cd /home/xjh/Desktop/nav27/sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 终端 2：编译导航工作空间
cd /home/xjh/Desktop/nav27/nav
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> **提示**：启动脚本自带 `-b` 选项，例如 `./run_sim.sh -b` 会自动先完成编译再启动。

---

### 2. 运行仿真与导航

#### 方式一：红区（默认）
打开两个独立终端：

**终端 1（启动 2027 仿真环境）：**
```bash
cd /home/xjh/Desktop/nav27
./run_sim.sh --red
```
*Gazebo 将加载 2027 赛场，并在红方出发点 `(-4.75, -5.15)` 生成 AT_R2 全向机器人，同时启动雷达、IMU 和速度桥接。*

**终端 2（启动 2027 导航系统与 RViz2）：**
```bash
cd /home/xjh/Desktop/nav27
./run_nav.sh --red
```
*RViz2 将自动打开，加载 2027 赛场 3D 点云与 2D 栅格地图。`small_gicp_relocalization` 自动完成与出发位姿的点云匹配，发布稳定的 `map -> odom` TF，Nav2 全节点激活完毕。*

#### 方式二：蓝区
**终端 1（仿真）：**
```bash
./run_sim.sh --blue
```

**终端 2（导航）：**
```bash
./run_nav.sh --blue
```

---

## 四、交互导航操作

1. 当导航与仿真均成功启动后，在 RViz2 工具栏中选择 **`Nav2 Goal`**（快捷键 `g`）。
2. 在 2027 赛场地面的任意位置（例如五色石基座前方 `(0.0, 3.5)` 或过渡坡道入口处 `(-3.5, 0.0)`）点击并拖拽设定目标点与期望航向。
3. 全局规划器将生成绿色无碰撞路径，全向 MPPI 控制器实时下发速度指令，小车将在 Gazebo 中平稳移动到达目标点，并在 RViz2 中显示实时轨迹与激光雷达避障点云。

---

## 五、核心数据流与原理说明

1. **时钟同步**：Gazebo 仿真物理时钟通过 `ros_gz_bridge` 转发至 `/clock` 话题，全栈节点均以 `use_sim_time:=true` 严格同步。
2. **点云流转换**：Gazebo 仿真 Mid360 输出至 `/AT_R2/mid_360/lidar`，由 `ign_sim_pointcloud_tool` 实时计算时间戳与线束 ID，转为标准 `velodyne_points`。
3. **里程计与 TF**：`point_lio` 融合雷达与 `/AT_R2/mid_360/imu` 生成 `aft_mapped_to_init`，经 `loam_interface` 和 `sensor_scan_generation` 输出连续平滑的 `odom -> chassis` 变换。
4. **全局重定位**：`small_gicp_relocalization` 将当前扫描与 `rc_2027_field.pcd` 配准，广播全局 `map -> odom`，消除长时间漂移。
5. **运动控制闭环**：Nav2 MPPI 控制器输出 `/AT_R2/cmd_vel_nav2_result`，经 `ros_gz_bridge` 传给 Gazebo `/<robot_name>/cmd_vel`，由 `MecanumDrive2` 力控插件作用于四个麦克纳姆轮。
