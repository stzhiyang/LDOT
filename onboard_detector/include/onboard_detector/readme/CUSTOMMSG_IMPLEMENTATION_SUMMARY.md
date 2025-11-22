# Livox CustomMsg 转换方案实现总结

## 问题

您遇到的错误：
```
[ERROR] Client [/detector_node] wants topic /livox_lidar to have datatype/md5sum 
[sensor_msgs/PointCloud2/...], but our version has [livox_ros_driver2/CustomMsg/...]
```

## 解决方案

在 `/home/st/LDOT_ws/src/LDOT` 路径下实现了一个在代码中直接转换 CustomMsg 到 PointCloud2 的方案。

## 修改内容

### 1. 头文件修改 (dynamicDetector.h)

#### 添加的包含文件:
```cpp
#include <livox_ros_driver2/CustomMsg.h>
#include <sensor_msgs/point_cloud2_iterator.h>
```

#### 添加的成员变量:
```cpp
bool useLivoxCustomMsg_;  // 是否使用Livox CustomMsg格式
std::shared_ptr<message_filters::Subscriber<livox_ros_driver2::CustomMsg>> lidarCustomMsgSub_;
// 新增同步器
typedef message_filters::sync_policies::ApproximateTime<livox_ros_driver2::CustomMsg, geometry_msgs::PoseStamped> lidarCustomPoseSync;
typedef message_filters::sync_policies::ApproximateTime<livox_ros_driver2::CustomMsg, nav_msgs::Odometry> lidarCustomOdomSync;
std::shared_ptr<message_filters::Synchronizer<lidarCustomPoseSync>> lidarCustomPoseSync_;
std::shared_ptr<message_filters::Synchronizer<lidarCustomOdomSync>> lidarCustomOdomSync_;
```

#### 添加的成员函数:
```cpp
void lidarCustomPoseCB(const livox_ros_driver2::CustomMsgConstPtr& customMsg, 
                       const geometry_msgs::PoseStampedConstPtr& pose);
void lidarCustomOdomCB(const livox_ros_driver2::CustomMsgConstPtr& customMsg, 
                       const nav_msgs::OdometryConstPtr& odom);
void convertCustomMsgToPointCloud2(const livox_ros_driver2::CustomMsgConstPtr& customMsg, 
                                   sensor_msgs::PointCloud2& cloud);
```

### 2. 实现文件修改 (dynamicDetector.cpp)

#### initParam() 函数:
添加了读取 `use_livox_custom_msg` 参数的代码

#### registerCallback() 函数:
修改为根据 `useLivoxCustomMsg_` 参数选择不同的订阅器：
- 如果为 true：订阅 CustomMsg 并使用 lidarCustomPoseCB/lidarCustomOdomCB
- 如果为 false：订阅 PointCloud2 并使用 lidarPoseCB/lidarOdomCB

#### 新增函数实现:

**convertCustomMsgToPointCloud2()**: 
- 将 Livox CustomMsg 格式转换为标准 PointCloud2 格式
- 提取每个点的 x, y, z 坐标
- 使用 sensor_msgs::PointCloud2Modifier 和 PointCloud2Iterator

**lidarCustomPoseCB()** 和 **lidarCustomOdomCB()**:
- 接收 CustomMsg 消息
- 调用转换函数
- 将转换后的 PointCloud2 传递给原有的处理函数

### 3. CMakeLists.txt 修改

添加了 livox_ros_driver2 依赖：
```cmake
find_package(catkin REQUIRED COMPONENTS
  ...
  livox_ros_driver2
)

catkin_package(
  ...
  CATKIN_DEPENDS ... livox_ros_driver2
)
```

### 4. package.xml 修改

添加了依赖声明：
```xml
<build_depend>livox_ros_driver2</build_depend>
<exec_depend>livox_ros_driver2</exec_depend>
```

### 5. 配置文件修改 (detector_param.yaml)

添加了新参数：
```yaml
# 点云格式: true: 使用Livox CustomMsg格式, false: 使用标准PointCloud2格式 (默认)
use_livox_custom_msg: true
```

## 使用方法

### 方式 1: 使用 Livox CustomMsg (推荐)

1. 在 `detector_param.yaml` 中设置：
```yaml
use_livox_custom_msg: true
lidar_pointcloud_topic: /livox_lidar  # livox发布的话题
```

2. livox_ros_driver2 使用默认配置（xfer_format = 1）

### 方式 2: 使用标准 PointCloud2

1. 在 `detector_param.yaml` 中设置：
```yaml
use_livox_custom_msg: false
lidar_pointcloud_topic: /cloud_registered  # 发布PointCloud2的话题
```

2. 或者修改 livox_ros_driver2 的 launch 文件设置 xfer_format = 0 或 2

## 优势

1. ✅ **无需修改 livox 配置** - 保持 livox_ros_driver2 原有设置
2. ✅ **灵活切换** - 通过配置文件轻松切换格式
3. ✅ **向后兼容** - 同时支持两种格式
4. ✅ **代码复用** - 转换后使用原有处理逻辑
5. ✅ **编译成功** - 已验证可以成功编译

## 技术细节

### CustomMsg 结构
```cpp
std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
CustomPoint[] points
```

### CustomPoint 结构
```cpp
uint32 offset_time
float32 x, y, z
uint8 reflectivity
uint8 tag
uint8 line
```

### 转换逻辑
只提取 x, y, z 坐标，其他信息（reflectivity, tag, line）被丢弃。如需使用这些信息，需要扩展转换函数。

## 文件位置

- 头文件: `/home/st/LDOT_ws/src/LDOT/onboard_detector/include/onboard_detector/dynamicDetector.h`
- 实现文件: `/home/st/LDOT_ws/src/LDOT/onboard_detector/include/onboard_detector/dynamicDetector.cpp`
- 配置文件: `/home/st/LDOT_ws/src/LDOT/onboard_detector/cfg/detector_param.yaml`
- CMakeLists: `/home/st/LDOT_ws/src/LDOT/onboard_detector/CMakeLists.txt`
- package.xml: `/home/st/LDOT_ws/src/LDOT/onboard_detector/package.xml`
- 说明文档: `/home/st/LDOT_ws/src/LDOT/LIVOX_CUSTOMMSG_README.md`

## 下一步

1. 设置配置文件中的 `use_livox_custom_msg: true`
2. 重新编译: `cd /home/st/LDOT_ws && catkin_make`
3. Source 环境: `source devel/setup.bash`
4. 运行节点并测试

## 编译状态

✅ 编译成功 - 2025-11-10
