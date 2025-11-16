# SLAM Demo Package

这是一个用于演示ROS2中两种主要SLAM算法的演示包：**slam_toolbox** 和 **cartographer**。

## 功能简介

该包提供了完整的launch文件和配置文件，可以轻松地在ROS2环境中运行SLAM算法。

### 包含内容

- **slam_toolbox**演示launch文件
- **cartographer**演示launch文件
- 对应的参数配置文件
- RViz可视化配置文件

## 依赖要求

已自动配置以下依赖项：
- `slam_toolbox`
- `cartographer_ros`
- `cartographer_ros_msgs`
- `geometry_msgs`
- `sensor_msgs`
- `nav_msgs`
- `tf2`
- `tf2_ros`
- `tf2_geometry_msgs`
- `rclcpp`

## 使用方法

### 1. 编译工作空间

```bash
cd ~/ROS2_Learning/ws09_slam
colcon build --packages-select slam_demo
source install/setup.bash
```

### 2. 运行SLAM Toolbox演示

```bash
ros2 launch slam_demo slam_toolbox_demo.launch.py
```

可以修改参数：
- `use_sim_time`: 使用模拟时间 (默认: true)
- `slam_params_file`: 参数文件路径

示例：
```bash
ros2 launch slam_demo slam_toolbox_demo.launch.py use_sim_time:=true
```

### 3. 运行Cartographer演示

```bash
ros2 launch slam_demo cartographer_demo.launch.py
```

可以修改参数：
- `use_sim_time`: 使用模拟时间 (默认: true)
- `resolution`: 地图分辨率 (默认: 0.05)
- `publish_period_sec`: 地图发布周期 (默认: 1.0)

示例：
```bash
ros2 launch slam_demo cartographer_demo.launch.py use_sim_time:=true resolution:=0.05
```

## 配置说明

### slam_toolbox参数配置

配置文件路径：`config/slam_toolbox_params.yaml`

主要配置项：
- `scan_topic`: 激光雷达数据topic，默认为`/scan`
- `mode`: 运行模式，可选`mapping`, `localization`, `lifelong`
- `map_frame`, `odom_frame`, `base_frame`: TF坐标系名称
- `resolution`: 地图分辨率

### cartographer参数配置

配置文件路径：`config/cartographer_config.lua`

主要配置项：
- `map_frame`, `tracking_frame`, `published_frame`: TF坐标系配置
- `trajectory_builder_2d.min_range`, `max_range`: 激光雷达测量范围
- `submaps.resolution`: 子图分辨率

## 与其他包配合使用

这个演示包设计为与ROS2的仿真环境配合使用，如：
- Gazebo/Ignition仿真
- TurtleBot3仿真
- 实际的激光雷达硬件

### 典型的topic连接

期望的输入topic：
- `/scan`: 激光雷达扫描数据
- `/odom`: 里程计数据 (可选)
- `/tf`: 坐标变换

输出topic：
- `/map`: 栅格地图
- `/map_metadata`: 地图元数据
- `/slam_toolbox/scan_visualization`: 扫描可视化 (slam_toolbox)
- `/submap_list`: 子图列表 (cartographer)

## 调试技巧

1. **检查坐标系**: 确保TF树正确发布
   ```bash
   ros2 run tf2_tools view_frames
   ```

2. **查看激活的topic**:
   ```bash
   ros2 topic list
   ros2 topic echo /scan
   ```

3. **使用rviz查看数据**:
   - 激光扫描数据
   - 地图构建过程
   - TF坐标系

## 常见问题

1. **没有地图输出**
   - 检查激光雷达topic是否正确
   - 确认TF坐标系配置正确
   - 查看节点日志是否有错误

2. **地图质量不佳**
   - 调整激光雷达参数
   - 修改SLAM算法参数
   - 检查移动速度是否过快

3. **内存使用过高**
   - 限制扫描缓冲区大小
   - 调整优化频率

## 扩展功能

您可以基于这个包进行扩展：
- 添加IMU数据融合
- 实现多传感器SLAM
- 集成路径规划
- 添加地图保存功能
- 实现定位模式

## 参考资料

- [slam_toolbox文档](https://github.com/SteveMacenski/slam_toolbox)
- [cartographer文档](https://google-cartographer-ros.readthedocs.io/)
- [ROS2 Navigation2文档](https://navigation.ros.org/)','content':