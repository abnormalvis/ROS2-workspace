# 🤖 ROS2-Ignition Gazebo 仿真教程指南

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Sim-blue.svg)](https://gazebosim.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## 📋 目录

- [🤖 ROS2-Ignition Gazebo 仿真教程指南](#-ros2-ignition-gazebo-仿真教程指南)
  - [📋 目录](#-目录)
  - [🎯 项目概述](#-项目概述)
  - [🏗️ 项目结构](#️-项目结构)
  - [🔧 环境配置](#-环境配置)
  - [🚀 快速开始](#-快速开始)
  - [📦 仿真包详解](#-仿真包详解)
  - [🌉 ROS2-Ignition桥接配置](#-ros2-ignition桥接配置)
  - [🌍 SDF世界文件创建](#-sdf世界文件创建)
  - [🎮 仿真运行与测试](#-仿真运行与测试)
  - [🔍 调试与故障排除](#-调试与故障排除)
  - [📚 进阶教程](#-进阶教程)
  - [🤝 贡献指南](#-贡献指南)

## 🎯 项目概述

本教程项目旨在帮助开发者学习和掌握ROS2与Ignition Gazebo（前Gazebo）的集成仿真技术。通过一系列完整的示例和实践指导，您将学会如何构建复杂的机器人仿真环境，涵盖从2D到3D仿真、传感器集成、多机器人系统等核心技能。

### 🌟 核心特性

- ✅ **完整的2D和3D仿真环境**
- ✅ **ROS2与Ignition无缝集成**
- ✅ **多种传感器模型支持**
- ✅ **多机器人系统仿真**
- ✅ **RViz可视化集成**
- ✅ **参数化配置系统**
- ✅ **实战示例和最佳实践**

### 🎯 学习目标

完成本教程后，您将能够：

1. 搭建完整的ROS2-Ignition仿真开发环境
2. 创建和配置SDF仿真世界文件
3. 建立ROS2与Ignition之间的数据桥接
4. 实现传感器数据在ROS2中的读取和处理
5. 运行和调试复杂的机器人仿真场景
6. 开发自定义的仿真控制器和算法
7. 集成多机器人协作仿真
8. 优化仿真性能和资源使用

## 🏗️ 项目结构

```
ws08_application/
├── src/                            # 源码目录
│   ├── ignition_sim_pkg/          # Ignition Gazebo仿真包
│   │   ├── launch/                # 启动文件
│   │   │   └── ignition.launch.py # Ignition仿真器启动文件
│   │   ├── config/                # 配置文件夹
│   │   │   └── bridge.conf        # ROS2-Ignition桥接配置
│   │   ├── world/                 # 仿真世界文件
│   │   │   └── demo.sdf           # SDF仿真世界
│   │   └── package.xml            # ROS2包描述
│   │
│   ├── Stage/                     # 2D机器人仿真器
│   │   ├── worlds/                # 2D仿真环境(50+场景)
│   │   ├── libstage/              # Stage核心库
│   │   ├── examples/              # 示例程序
│   │   └── assets/                # 资源文件
│   │
│   ├── stage_ros2/                # Stage的ROS2接口
│   │   ├── launch/                # 2D仿真启动文件
│   │   │   ├── my_house.launch.py # 单机器人仿真
│   │   │   └── my_house_multi.launch.py # 多机器人仿真
│   │   └── config/                # 配置文件
│   │
│   └── stage_ros_sim_demo/       # 演示包
│       ├── world/                # 自定义世界文件
│       └── launch/               # 演示启动文件
│
├── build/                         # 构建输出目录
├── install/                       # 安装目录
├── log/                          # 日志目录
└── README.md                     # 本文档
```

## 🔧 环境配置

### 📋 系统要求

- **操作系统**: Ubuntu 20.04/22.04 LTS (推荐)
- **ROS版本**: ROS2 Humble/Hawksbill/Foxy
- **Python**: 3.8+
- **CMake**: 3.16+
- **GCC**: 9.0+

### 🛠️ ROS2安装

```bash
# 更新系统包
sudo apt update && sudo apt upgrade -y

# 安装ROS2 (以Humble为例)
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装ROS2桌面版
sudo apt update
sudo apt install ros-humble-desktop -y

# 设置环境变量
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 🎮 Ignition Gazebo安装

```bash
# 安装Ignition Gazebo (以Citadel版本为例)
sudo apt install ignition-citadel -y

# 验证安装
ign gazebo --versions

# 安装ROS2-Ignition桥接包
sudo apt install ros-humble-ros-ign-bridge -y
sudo apt install ros-humble-ros-ign-gazebo -y
sudo apt install ros-humble-ros-ign-image -y

# 其他有用工具
sudo apt install ros-humble-rqt-robot-steering -y
sudo apt install ros-humble-teleop-twist-keyboard -y
```

### 🔧 开发环境搭建

```bash
# 创建工作空间
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 克隆本仓库
git clone <repository-url> .

# 返回工作空间根目录
cd ~/ros2_ws

# 安装依赖
sudo apt install python3-colcon-common-extensions -y
rosdep install -i --from-path src --rosdistro humble -y

# 构建项目
colcon build --symlink-install

# 设置环境
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 🚀 快速开始

### 💻 基本使用

1. **启动Ignition Gazebo仿真**:
```bash
# 进入工作空间
cd ~/ros2_ws

# 构建项目 (首次运行)
colcon build --symlink-install

# 启动Ignition仿真
ros2 launch ignition_sim_pkg ignition.launch.py
```

2. **启动2D Stage仿真** (替代方案):
```bash
# 启动单机器人仿真
ros2 launch stage_ros2 my_house.launch.py

# 启动多机器人仿真
ros2 launch stage_ros2 my_house_multi.launch.py
```

3. **验证ROS2节点**:
```bash
# 查看运行中的节点
ros2 node list

# 查看主题列表
ros2 topic list

# 查看参
ros2 param list
```

### 📊 数据监控

```bash
# 查看仿真数据流
ros2 topic echo /chatter

# 通过RViz可视化
rviz2

# 使用rqt工具监控
rqt_graph
```

### 🕹️ 手动控制测试

```bash
# 键盘控制机器人移动
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 或者使用rqt机器人方向控制
rqt-robot-steering
```

## 📦 仿真包详解

### 🔥 ignition_sim_pkg - Ignition Gazebo仿真包

这是最核心的3D仿真包，提供了完整的Ignition Gazebo集成。

#### 🎯 主要功能

- **Ignition Gazebo启动器**: 自动启动Ignition仿真器
- **ROS2桥接配置**: 管理ROS2与Ignition主题/服务的数据流
- **SDF世界文件**: 加载和管理仿真环境
- **参数化配置**: 支持运行时参数调整

#### 📁 文件结构

```
ignition_sim_pkg/
├── package.xml              # 包描述和依赖声明
├── CMakeLists.txt           # CMake构建脚本
├── launch/
│   └── ignition.launch.py   # 主要启动文件
├── config/
│   └── bridge.conf          # 数据桥接配置
├── worlds/
│   └── demo.sdf             # 示例Gazebo仿真世界
└── README.md               # 包级文档
```

#### 🔍 Launch文件详解

[launch/ignition.launch.py](src/ignition_sim_pkg/launch/ignition.launch.py:1)文件结构：

```python
# 核心组件:
# 1. 启动Ignition Gazebo仿真器
# 2. 配置ROS2-Ignition桥接
# 3. 加载SDF世界文件
# 4. 设置数据流量映射
```

#### ⚙️ 桥接配置

[config/bridge.conf](src/ignition_sim_pkg/config/bridge.conf:1)定义了ROS2和Ignition之间的数据传输：

```yaml
# 主题映射规则:
# publish: ROS2主题 -> Ignition主题
# subscribe: Ignition主题 -> ROS2主题
```

### 🎮 Stage - 2D机器人仿真器

经典的2D/2.5D机器人仿真器，适用于快速原型开发和算法验证。

#### 🌟 主要特性

- **高性能**: 优化的2D渲染和物理引擎
- **多传感器**: 激光雷达、声纳、摄像头、里程计等
- **模块化**: 可作为库集成到自定义应用中
- **丰富示例**: 50+预定义仿真环境
- **多机器人**: 原生支持多机器人仿真

#### 🗺️ 世界文件

Stage提供丰富的2D仿真环境：

| 环境类型 | 文件示例 | 描述 |
|---------|---------|------|
| 室内环境 | `my_house.world` | 家庭室内环境 |
| 户外环境 | `cave.world` | 洞穴探索环境 |
| 竞赛场景 | `competition.world` | 机器人竞赛场地 |
| 办公环境 | `office.world` | 办公楼层布局 |

#### 📡 传感器模型

```bash
# 激光雷达
laser_samples: 180    # 扫描样本数
laser_res: 1          # 分辨率(度)
laser_range: 8.0      # 最大范围(米)

# 声纳
sonar_samples: 16     # 声纳数量
sonar_range: 2.0      # 探测范围

# 视觉传感器
camera_width: 320     # 图像宽度
camera_height: 240    # 图像高度
```

### 🔗 stage_ros2 - ROS2接口包

将Stage仿真器与ROS2生态系统完美集成。

#### 🚀 主要功能

- **ROS2节点封装**: 将Stage封装为ROS2节点
- **Topic发布**: 发布传感器数据和机器人状态
- **Service支持**: 提供仿真控制服务
- **RViz集成**: 完美支持RViz可视化
- **TF变换**: 发布坐标变换信息

#### 📋 Topic映射

| ROS2主题 | Stage数据 | 消息类型 | 频率 |
|---------|----------|---------|------|
| `/scan` | 激光雷达数据 | `sensor_msgs/LaserScan` | 10Hz |
| `/odom` | 里程计数据 | `nav_msgs/Odometry` | 20Hz |
| `/base_scan` | 原始激光数据 | `sensor_msgs/LaserScan` | 10Hz |
| `/tf` | 坐标变换 | `tf/tfMessage` | 20Hz |

## 🌉 ROS2-Ignition桥接配置

ROS2和Ignition Gazebo之间的数据桥接是整个系统的核心组件。

### 🔧 桥接原理

```
ROS2 Topics <---> 桥接节点 <---> Ignition Topics/Services
 ↓                      ↓
ROS2节点              Ignition仿真器
```

### 📄 配置文件详解

[config/bridge.conf](src/ignition_sim_pkg/config/bridge.conf:1)详细说明：

```yaml
# 主题桥接配置格式:
# <方向> <ROS2主题名> <消息类型> <Ignition主题名>

# 示例配置:
publish /chatter std_msgs/msg/String /chatter        # ROS2发布到Ignition
subscribe /clock rosgraph_msgs/msg/Clock /clock      # Ignition时间同步
publish /cmd_vel geometry_msgs/msg/Twist /cmd_vel    # 机器人控制指令
```

### 🎯 常用配置模板

#### 📡 移动机器人配置

```yaml
# 传感器数据 (Ignition → ROS2)
subscribe /laser_scan sensor_msgs/msg/LaserScan /laser_scan
subscribe /odom nav_msgs/msg/Odometry /odom
subscribe /imu sensor_msgs/msg/Imu /imu
subscribe /camera/image_raw sensor_msgs/msg/Image /camera/image_raw

# 控制指令 (ROS2 → Ignition)
publish /cmd_vel geometry_msgs/msg/Twist /cmd_vel
publish /cmd_pos geometry_msgs/msg/Pose /cmd_pos
```

#### 🤖 机械臂配置

```yaml
# 关节状态
subscribe /joint_states sensor_msgs/msg/JointState /joint_states

# 关节控制
publish /joint_command trajectory_msgs/msg/JointTrajectory /joint_cmd

# 末端执行器
publish /gripper_cmd std_msgs/msg/Bool /gripper_cmd
```

#### 🌐 系统级配置

```yaml
# 时间同步
subscribe /clock rosgraph_msgs/msg/Clock /clock

# 参数设置
publish /set_parameters rcl_interfaces/msg/ParameterEvent /set_params
subscribe /parameter_events rcl_interfaces/msg/ParameterEvent /param_events
```

### 🔍 动态桥接配置

```bash
# 运行时动态配置桥接
ros2 run ros_ign_bridge parameter_bridge

# 示例：添加新的桥接
ros2 run ros_ign_bridge parameter_bridge /laser_scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan]
```

## 🌍 SDF世界文件创建

### 📚 SDF基础结构

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="demo">
    <!-- 物理引擎配置 -->
    <physics type="ode">
      <real_time_update_rate>1000</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
    </physics>

    <!-- 光照配置 -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
    </light>

    <!-- 模型定义 -->
    <model name="robot">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="base_link">
        <!-- 机器人基座定义 -->
      </link>
    </model>

    <!-- 环境障碍物 -->
    <model name="wall">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>2 0.1 2</size></box>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

### 🚀 常用模型库

#### 🤖 机器人模型

```xml
<!-- 差速驱动机器人 -->
<include>
  <uri>model://differential_drive</uri>
  <pose>0 0 0.5 0 0 0</pose>
</include>

<!-- 四足机器人 -->
<include>
  <uri>model://quadcopter</uri>
  <pose>2 0 0.5 0 0 0</pose>
</include>
```

#### 🏠 环境模型

```xml
<!-- 建筑物 -->
<include>
  <uri>model://house</uri>
  <pose>5 5 0 0 0 0</pose>
</include>

<!-- 地形 -->
<include>
  <uri>model://rough_terrain</uri>
  <pose>0 0 0 0 0 0</pose>
</include>
```

#### 📡 传感器模型

```xml
<!-- 激光雷达 -->
<include>
  <uri>model://lidar_sensor</uri>
  <pose>0 0 0.3 0 0 0</pose>
</include>

<!-- 摄像头 -->
<include>
  <uri>model://camera_sensor</uri>
  <pose>0.1 0 0.2 0 0 0</pose>
</include>
```

### 🎯 高级配置

#### 🎮 物理参数调优

```xml
<physics type="ode">
  <real_time_update_rate>1000</real_time_update_rate>
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <ode>
    <solver>
      <type>world</type>
      <min_step_size>1e-6</min_step_size>
      <iterations>50</iterations>
      <precon_iters>0</precon_iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
    </constraints>
  </ode>
</physics>
```

#### 🌙 环境光照

```xml
<light name="sun" type="directional">
  <pose>0 0 10 0 0 0</pose>
  <diffuse>1 1 1 1</diffuse>
  <specular>0.1 0.1 0.1 1</specular>
  <attenuation>
    <range>1000</range>
    <constant>0.9</constant>
    <linear>0.01</linear>
    <quadratic>0.001</quadratic>
  </attenuation>
  <direction>0.5 0.5 -1</direction>
</light>
```

## 🎮 仿真运行与测试

### 🚀 启动流程

#### 📝 标准启动程序

```bash
# 1. 构建项目
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 2. 启动Ignition仿真
ros2 launch ignition_sim_pkg ignition.launch.py

# 3. 在另一个终端，监控节点状态
ros2 node list
ros2 topic list

# 4. 可视化监控
rviz2
# 或者使用Ignition自带可视化界面

# 5. 手动控制测试
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### 🔍 启动参数说明

```bash
# Ignition仿真启动选项
ros2 launch ignition_sim_pkg ignition.launch.py \
  world:=demo.sdf              # 指定世界文件

# Stage 2D仿真启动选项
ros2 launch stage_ros2 my_house.launch.py \
  world:=my_house.world        # 指定2D世界文件

# 多机器人仿真
ros2 launch stage_ros2 my_house_multi.launch.py \
  robots:=2                    # 机器人数量
```

### 📊 性能监控

#### ⚡ 实时性能指标

```bash
# 监控仿真性能
ign topic -e -t /stats

# 查看帧率信息
ign gazebo -s --record_period 1

# 内存使用监控
htop 或 top

# 网络流量监控
ros2 topic hz /laser_scan
```

#### 🔧 性能优化建议

1. **CPU优化**: 减小`real_time_factor`降低仿真精度换取速度
2. **GPU优化**: 禁用不必要的视觉特效，使用简化的渲染
3. **内存优化**: 合理配置缓存大小，及时清理存储
4. **I/O优化**: 减少日志级别，禁用不必要的topic发布

### 🧪 测试用例

#### ✅ 单元测试

```bash
# 运行所有测试
colcon test

# 显示测试结果
colcon test-result --verbose

# 运行特定包的测试
colcon test --packages-select ignition_sim_pkg
```

#### 🎯 集成测试

```bash
# 自动化测试脚本
./test/integration_test.sh

# SSH远程测试
ros2 launch test/multi_robot_test.launch.py

# 性能压力测试
./test/performance_test.py
```

## 🔍 调试与故障排除

### 🚨 常见问题

#### Q1: Ignition启动失败

**症状**: `ign gazebo`命令无响应或报错

**原因分析**:
- Ignition库未正确安装
- 环境变量未设置
- 显卡驱动不支持

**解决方案**:
```bash
# 检查Ignition安装
ign gazebo --versions

# 重新安装Ignition
sudo apt install ignition-citadel --reinstall

# 检查环境变量
echo $IGN_GAZEBO_RESOURCE_PATH
```

#### Q2: ROS2桥接失败

**症状**: ROS2节点无法获取到Ignition数据

**调试步骤**:
```bash
# 验证桥接运行状态
ros2 node list | grep bridge

# 检查主题映射
ros2 topic info /laser_scan

# 手动测试桥接
ros2 run ros_ign_bridge parameter_bridge /laser_scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan]
```

#### Q3: 仿真性能低下

**性能诊断**:
```bash
# 查看系统资源使用
top -p $(pgrep gz)

# 监控磁盘I/O
iotop

# 检查GPU使用率
nvidia-smi  # NVIDIA显卡
watch -n1 cat /proc/driver/*/gpu_busy_percent  # Intel集成显卡
```

#### Q4: 模型加载失败

**检查点列表**:
```bash
# 验证模型文件路径
ls -la ~/ros2_ws/src/ignition_sim_pkg/world/

# 检查XML/SDF格式
xmllint --noout demo.sdf

# 验证模型资源
ign sdf --check demo.sdf
```

### 🔧 调试工具

#### 📊 日志分析

```bash
# 启用详细日志
export IGNITION_TRANSPORT_LOG_LEVEL=3
export RUST_LOG=debug

# 查看日志文件
tail -f ~/.ignition/gazebo/*.log
```

#### 🔍 网络诊断

```bash
# 检查Ignition传输
ign topic -l

# 监控ROS2通信
ros2 topic echo --flow-style /statistics

# 发现服务
ros2 service list
```

#### 🐛 内存调试

```bash
# Valgrind内存检查
valgrind --leak-check=full ign gazebo demo.sdf

# GDB调试
gdb ign gazebo
(gdb) run demo.sdf
```

### 📞 社区支持

- **ROS社区**: [https://discourse.ros.org/](https://discourse.ros.org/)
- **Ignition仿真**: [https://community.gazebosim.org/](https://community.gazebosim.org/)
- **GitHub Issue**: 本项目的问题报告
- **Stack Overflow**: 使用 `ros2-ignition` 标签

## 📚 进阶教程

### 🎨 自定义传感器开发

#### 📡 激光雷达插件示例

```cpp
// laser_plugin.cc
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class LaserPlugin : public ignition::gazebo::System,
                   public ignition::gazebo::ISystemPostUpdate
{
public:
  void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                  const ignition::gazebo::EntityComponentManager &_ecm) override
  {
    // 获取激光数据
    auto laserData = this->GetLaserData(_ecm);

    // 转换为ROS消息
    sensor_msgs::LaserScan rosScan;
    rosScan.header.stamp = ros::Time::now();
    rosScan.header.frame_id = "laser_link";
    rosScan.angle_min = -PI/2;
    rosScan.angle_max = PI/2;
    rosScan.angle_increment = PI/180;
    rosScan.time_increment = 0.0;
    rosScan.scan_time = 0.1;
    rosScan.range_min = 0.1;
    rosScan.range_max = 8.0;
    rosScan.ranges = laserData.ranges;

    // 发布ROS主题
    publisher.publish(rosScan);
  }
private:
  ros::Publisher publisher;
};
```

### 🤖 多机器人协调

#### 🌐 编队控制示例

```python
# formation_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist
import numpy as np

class FormationController(Node):
    def __init__(self):
        super().__init__('formation_controller')

        # 订阅多机器人位置
        self.pose_sub = self.create_subscription(
            PoseArray, '/multi_robot_poses', self.pose_callback, 10)

        # 发布控制指令
        self.cmd_pubs = []
        for i in range(N_ROBOTS):
            pub = self.create_publisher(Twist, f'/robot{i}/cmd_vel', 10)
            self.cmd_pubs.append(pub)

        # 编队参数
        self.formation_type = "triangle"  # 三角形编队
        self.formation_radius = 2.0       # 编队半径

    def pose_callback(self, msg):
        """处理多机器人位置信息"""
        poses = msg.poses

        # 计算期望编队位置
        desired_positions = self.calculate_formation(poses[0])

        # 计算控制指令
        for i, (pose, desired) in enumerate(zip(poses, desired_positions)):
            control = self.calculate_control(pose, desired)
            twist = Twist()
            twist.linear.x = control[0]
            twist.angular.z = control[1]

            self.cmd_pubs[i].publish(twist)
```

### 🔬 物理引擎调优

#### ⚙️ 接触力模型配置

```xml
<physics type="ode">
  <ode>
    <constraints>
      <!-- 接触模型参数 -->
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <cfm>0.0</cfm>  <!-- 约束力混合参数 -->
      <erp>0.2</erp>  <!-- 误差减少参数 -->
    </constraints>
    <solver>
      <!-- 求解器参数 -->
      <type>world</type>
      <min_step_size>1e-6</min_step_size>
      <iterations>50</iterations>
      <sor>1.3</sor>  <!-- 连续过度松弛 -->
    </solver>
  </ode>
</physics>
```

### 🧠 AI/ML集成

#### 🎯 强化学习环境

```python
# reinforcement_learning_env.py
from gym import Env, spaces
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class RobotEnv(Env):
    def __init__(self):
        super(RobotEnv, self).__init__()

        # ROS2初始化
        rclpy.init()
        self.node = rclpy.create_node('rl_env')

        # 定义动作空间 (线速度, 角速度)
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0]),
            high=np.array([1.0, 1.0]),
            dtype=np.float32
        )

        # 定义观测空间 (激光雷达数据)
        self.observation_space = spaces.Box(
            low=0.0, high=10.0,
            shape=(360,), dtype=np.float32
        )

        # ROS2发布者和订阅者
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.node.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.current_scan = None

    def reset(self):
        """重置环境状态"""
        self.current_scan = None
        # 重置仿真器和机器人位置
        # ... 实现重置逻辑
        return self.get_observation()

    def step(self, action):
        """执行动作并返回新状态"""
        # 发送控制指令
        twist = Twist()
        twist.linear.x = action[0]
        twist.angular.z = action[1]
        self.cmd_pub.publish(twist)

        # 获取新观测
        obs = self.get_observation()

        # 计算奖励
        reward = self.calculate_reward(obs, action)

        # 检查是否结束
        done = self.check_done(obs)

        return obs, reward, done, {}

    def get_observation(self):
        """获取当前观测"""
        return self.current_scan if self.current_scan is not None \
               else np.zeros(360, dtype=np.float32)
```

## 🤝 贡献指南

### 🌟 如何贡献

我们欢迎所有形式的贡献，包括代码改进、文档完善、问题报告和功能建议！

#### 📝 贡献流程

1. **Fork项目**到您的GitHub帐号
2. **创建特性分支**: `git checkout -b feature/amazing-feature`
3. **提交更改**: `git commit -m 'Add amazing feature'`
4. **推送到分支**: `git push origin feature/amazing-feature`
5. **发起Pull Request**

#### 🎯 代码规范

- 遵循**ROS2编码规范**
- 使用**Google C++ Style Guide** (C++)
- 遵循**PEP 8** (Python)
- 添加详细的代码注释
- 包含完整的单元测试

#### 📋 Pull Request模板

```markdown
## 📝 修改说明
简要描述您所做的修改

## 🎯 修改类型
- [ ] Bug修复 (非破坏性问题修复)
- [ ] 新功能 (非破坏性新功能)
- [ ] 破坏性变更 (会导致现有功能失效的修复或功能)
- [ ] 文档更新
- [ ] 代码重构
- [ ] 性能优化
- [ ] 测试用例
- [ ] 其他 (请描述):

## 🔗 相关Issue
关联的issue链接 (如果有)

## 🧪 测试
描述您测试的方法

## 📋 检查清单
- [ ] 我的代码遵循项目的代码风格
- [ ] 我已经执行了自测
- [ ] 我已经添加了相应的文档
- [ ] 我的变更不会引入新的警告
```

### 🐛 问题报告

如发现bug或有功能建议，请在GitHub Issues中报告。

#### 📋 问题报告模板

```markdown
## 🐛 Bug描述
清晰简洁地描述bug

## 🎯 期望行为
描述您期望的行为

## 🪜 复现步骤
1. 第一步: '...'
2. 第二步: '...'
3. ...) 第三步: '
...

## 📊 环境信息
- 操作系统: [e.g. Ubuntu 20.04]
- ROS版本: [e.g. ROS2 Humble]
- Ignition版本: [e.g. Citadel]
- Python版本: [e.g. 3.8]

## 📝 错误日志
添加相关的错误日志或截图
```

### 📞 联系我们

- 📧 **邮件**: your-email@example.com
- 💬 **讨论**: GitHub Discussions
- 🚀 **特性请求**: GitHub Issues (Feature Request标签)

---

## 📄 许可证

本项目采用**Apache License 2.0**许可证 - 详情请参见[LICENSE](LICENSE)文件。

## 🙏 致谢

- **ROS团队**: 提供强大的机器人操作系统
- **Ignition Gazebo团队**: 提供优秀的仿真平台
- **开源社区**: 所有贡献者和使用者的支持

---

<div align="center">

⭐ 如果觉得这个项目有用，请给我们一个Star! ⭐

**Made with ❤️ by the Robotics Community**

</div>