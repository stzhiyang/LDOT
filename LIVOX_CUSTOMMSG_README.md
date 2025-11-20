# Livox CustomMsg 支持说明

## 概述

本项目现在支持直接处理 Livox ROS Driver 2 的 CustomMsg 格式点云数据，无需修改 livox_ros_driver2 的配置。

## 问题背景

之前遇到的错误：
```
[ERROR] Client [/detector_node] wants topic /livox_lidar to have datatype/md5sum 
[sensor_msgs/PointCloud2/...], but our version has [livox_ros_driver2/CustomMsg/...]
```

这是因为 livox_ros_driver2 默认发布 CustomMsg 格式，而检测节点期望接收 PointCloud2 格式。

## 解决方案

### 方案一：修改 livox_ros_driver2 配置（之前的方案）

修改 livox launch 文件中的 `xfer_format` 参数：
- `xfer_format = 0`: Livox PointCloud2 (PointXYZRTLT) 格式
- `xfer_format = 1`: Livox CustomMsg 格式（默认）
- `xfer_format = 2`: 标准 PointCloud2 (pcl::PointXYZI) 格式

### 方案二：在检测节点中转换（新方案，推荐）

在检测节点中直接处理 CustomMsg 并转换为 PointCloud2，无需修改 livox 配置。

## 使用方法

### 配置参数

在 `detector_param.yaml` 中添加/修改以下参数：

```yaml
# 点云格式设置
use_livox_custom_msg: true   # true: 使用CustomMsg, false: 使用PointCloud2

# 点云话题
lidar_pointcloud_topic: /livox_lidar  # livox发布CustomMsg的话题
```

### 参数说明

- `use_livox_custom_msg`: 
  - `true`: 订阅 `livox_ros_driver2/CustomMsg` 格式的点云
  - `false`: 订阅标准 `sensor_msgs/PointCloud2` 格式的点云（默认）

### 示例配置

#### 使用 Livox CustomMsg（新方案）

```yaml
localization_mode: 1
use_livox_custom_msg: true
lidar_pointcloud_topic: /livox_lidar
odom_topic: /mavros/local_position/odom
```

#### 使用标准 PointCloud2

```yaml
localization_mode: 1
use_livox_custom_msg: false
lidar_pointcloud_topic: /cloud_registered
odom_topic: /mavros/local_position/odom
```

## 技术实现

### 转换流程

1. 订阅 `livox_ros_driver2::CustomMsg` 消息
2. 在回调函数中调用 `convertCustomMsgToPointCloud2()` 进行格式转换
3. 将转换后的 `sensor_msgs::PointCloud2` 传递给原有的处理流程

### 转换函数

```cpp
void dynamicDetector::convertCustomMsgToPointCloud2(
    const livox_ros_driver2::CustomMsgConstPtr& customMsg, 
    sensor_msgs::PointCloud2& cloud)
```

该函数将 Livox CustomMsg 中的点云数据（x, y, z）提取并转换为标准的 PointCloud2 格式。

### 新增回调函数

- `lidarCustomPoseCB()`: 处理 CustomMsg + PoseStamped
- `lidarCustomOdomCB()`: 处理 CustomMsg + Odometry

## 优势

1. **无需修改 livox 配置**：保持 livox_ros_driver2 的原始配置
2. **灵活切换**：通过配置文件轻松切换不同的点云格式
3. **兼容性好**：同时支持 CustomMsg 和 PointCloud2 两种格式
4. **代码复用**：转换后复用原有的点云处理逻辑

## 编译

确保你的工作空间中包含 livox_ros_driver2：

```bash
cd ~/LDOT_ws
catkin_make
source devel/setup.bash
```

## 运行

```bash
roslaunch onboard_detector detector.launch
```

## 注意事项

1. CustomMsg 转换只提取 x, y, z 坐标，reflectivity、tag、line 等信息会被丢弃
2. 如果需要使用这些额外信息，需要扩展转换函数
3. 确保 livox_ros_driver2 已正确编译并安装在你的 catkin 工作空间中

## 故障排查

### 编译错误：找不到 livox_ros_driver2/CustomMsg.h

确保 livox_ros_driver2 在你的 catkin 工作空间中：
```bash
ls ~/catkin_mid360/src/livox_ros_driver2
```

### 运行时错误：话题类型不匹配

检查配置文件中的 `use_livox_custom_msg` 参数是否与实际发布的话题类型匹配。

## 更新日期

2025-11-10
