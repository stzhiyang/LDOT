# 多模型卡尔曼滤波器实现总结

## 概述
本次修改将单一的CA卡尔曼滤波器替换为多模型卡尔曼滤波系统，根据不同目标类别使用不同的运动模型。

## 新增文件

### 1. motionModel.h / motionModel.cpp
实现了三种运动模型的基类和具体实现：

#### CA_Model (恒定加速度模型)
- **用于**: 人、无人机
- **人的状态向量** (2D): `[x, y, vx, vy, ax, ay]` (6维)
- **无人机的状态向量** (3D): `[x, y, z, vx, vy, vz, ax, ay, az]` (9维)
- **测量**: 位置 `[x, y]` 或 `[x, y, z]`

#### CV_Model (恒定速度模型)
- **用于**: 其他类别
- **状态向量** (3D): `[x, y, z, vx, vy, vz]` (6维)
- **测量**: 位置 `[x, y, z]`

#### CTRA_Model (恒定转弯率和加速度模型)
- **用于**: 车辆
- **状态向量**: `[x, y, v, a, yaw, yaw_rate]` (6维)
  - `x, y`: 位置
  - `v`: 行驶速度 (标量)
  - `a`: 加速度
  - `yaw`: 偏航角
  - `yaw_rate`: 偏航角速度
- **测量**: 位置 `[x, y]`

### 2. multiModelKalmanFilter.h
实现了两种卡尔曼滤波器：

#### LinearKalmanFilter
- 用于线性模型 (CA, CV)
- 标准的预测-更新循环

#### ExtendedKalmanFilter
- 用于非线性模型 (CTRA)
- 使用雅可比矩阵进行线性化
- Joseph形式的协方差更新，提高数值稳定性

#### 工厂函数
`createKalmanFilter()` 根据目标类别自动创建相应的滤波器：
- `is_human` → 2D CA-KF
- `is_uav` → 3D CA-KF  
- `is_che` → CTRA-EKF
- `is_else` → 3D CV-KF

## 修改文件

### dynamicDetector.h
- 修改了头文件引用：删除 `kalmanFilter.h`，添加 `multiModelKalmanFilter.h`
- 修改了滤波器容器类型：
  ```cpp
  // 旧：
  std::vector<onboardDetector::kalman_filter> filters_;
  
  // 新：
  std::vector<std::shared_ptr<KalmanFilterBase>> filters_;
  ```
- 删除了旧的辅助函数声明：
  - `kalmanFilterMatrixVel()`
  - `kalmanFilterMatrixAcc()`
  - `getKalmanObservationVel()`
  - `getKalmanObservationAcc()`

### dynamicDetector.cpp

#### 1. 初始化部分 (首次检测)
```cpp
// 根据目标类别创建滤波器
auto newFilter = createKalmanFilter(
    bbox.is_human,
    bbox.is_che,
    bbox.is_uav,
    bbox.is_else
);
newFilter->setDt(this->dt_);
newFilter->initialize(detection);
```

#### 2. 跟踪部分 (kalmanFilterAndUpdateHist)
重写了整个函数以支持多模型：
- 对已匹配目标：
  - 执行预测步骤 `filter->predict()`
  - 根据目标类别准备相应维度的测量向量
  - 执行更新步骤 `filter->update(measurement)`
  - 从状态向量中提取位置、速度、加速度
  
- 对新目标：
  - 使用工厂函数创建相应的滤波器
  - 初始化滤波器

#### 3. 数据关联部分 (boxAssociation)
- 修改了状态提取方式，使用 `getState()` 返回 `VectorXd`
- 根据不同模型提取位置信息
- 修正了协方差矩阵索引 (因为状态向量结构改变)

#### 4. 删除的函数
- `kalmanFilterMatrixVel()` - 不再需要
- `kalmanFilterMatrixAcc()` - 不再需要
- `getKalmanObservationVel()` - 不再需要
- `getKalmanObservationAcc()` - 不再需要

### CMakeLists.txt
```cmake
# 旧：
add_library(${PROJECT_NAME}
   include/${PROJECT_NAME}/kalmanFilter.cpp
   ...
)

# 新：
add_library(${PROJECT_NAME}
   include/${PROJECT_NAME}/motionModel.cpp
   ...
)
```

## 运动模型详细说明

### 1. CA模型 (人/无人机)
**状态转移方程**:
```
x_{k+1} = x_k + vx*dt + ax*dt²/2
vx_{k+1} = vx_k + ax*dt
ax_{k+1} = ax_k
```

**优势**: 能够捕捉加速度变化，适合人的运动和无人机的机动

### 2. CV模型 (其他)
**状态转移方程**:
```
x_{k+1} = x_k + vx*dt
vx_{k+1} = vx_k
```

**优势**: 简单高效，适合速度相对恒定的物体

### 3. CTRA模型 (车辆)
**状态转移方程** (非线性):
```
当 |yaw_rate| < 0.001 (近似直线运动):
  x_{k+1} = x_k + (v*dt + a*dt²/2)*cos(yaw)
  y_{k+1} = y_k + (v*dt + a*dt²/2)*sin(yaw)
  
否则 (转弯运动):
  使用完整的CTRA运动方程 (见代码实现)
  
v_{k+1} = v_k + a*dt
yaw_{k+1} = yaw_k + yaw_rate*dt
a_{k+1} = a_k
yaw_rate_{k+1} = yaw_rate_k
```

**优势**: 准确描述车辆的转弯行为，考虑偏航角和转弯率

## 噪声参数

### 过程噪声 (Q矩阵)
- **CA**: 位置噪声小 (0.1)，速度中等 (1.0)，加速度大 (100.0)
- **CV**: 位置噪声小 (0.1)，速度中等 (10.0)
- **CTRA**: 各状态分量噪声 (0.1-10.0)

### 测量噪声 (R矩阵)
- 所有模型：位置测量噪声 = 0.1

### 初始协方差 (P矩阵)
- **CA**: 位置 (0.1)，速度 (1.0)，加速度 (10.0)
- **CV**: 位置 (0.1)，速度 (1.0)
- **CTRA**: x/y (0.1)，v (1.0)，a (10.0)，yaw (0.5)，yaw_rate (1.0)

## 使用方法

系统会自动根据 `box3D` 结构中的分类标志选择合适的滤波器：
- `is_human = true` → 2D CA-KF
- `is_uav = true` → 3D CA-KF
- `is_che = true` → CTRA-EKF
- `is_else = true` → 3D CV-KF

不需要手动指定，工厂函数会自动处理。

## 注意事项

1. **时间步长**: 确保在每次预测前调用 `setDt()` 设置正确的时间步长
2. **分类标志**: 必须在检测阶段正确设置 `box3D` 的分类标志
3. **测量维度**: 不同模型期望不同维度的测量向量，系统会自动处理
4. **角度归一化**: CTRA模型会自动将偏航角归一化到 [-π, π]

## 性能优势

1. **更准确的预测**: 针对不同运动模式优化
2. **更好的速度估计**: CA模型能更好地捕捉加速度
3. **车辆转弯处理**: CTRA模型准确描述车辆转弯
4. **数值稳定性**: EKF使用Joseph形式更新协方差

## 后续可能的改进

1. 自适应噪声协方差
2. 多模型交互 (IMM)
3. 根据实际性能调优噪声参数
4. 添加更多运动模型 (如CTRV, Bicycle)
