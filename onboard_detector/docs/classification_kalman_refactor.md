# 物体分类与卡尔曼滤波重构

## 问题概述

### 原始问题
1. **分类时机错误**：分类在卡尔曼滤波更新之后进行，导致滤波器使用旧的分类信息
2. **卡尔曼滤波测量维度错误**：所有模型强制使用3D测量 `[x, y, z]`，但Human和Vehicle模型应该只测量2D `[x, y]`
3. **双重预测问题**：模型切换时执行了两次predict()，导致误差累积
4. **不必要的2D/3D区分**：关联时区分2D/3D物体，但所有状态都是3D的

### 核心原则
- **所有模型的状态向量中位置都是三维的** `[x, y, z, ...]`（关键认识）
- 不同模型的**观测维度不同**：Human/Vehicle测量2D，UAV/Else测量3D

## 修改内容

### 1. 调整执行顺序 (`trackingCB`)

**旧流程**：
```
关联 → 滤波更新 → 分类 → 模型切换
```

**新流程**：
```
关联 → 分类 → 模型切换 → 滤波更新
```

**代码位置**：`dynamicDetector.cpp:1006-1054`

**关键变化**：
- 先对匹配成功的轨迹进行`classifyBox()`
- 立即执行`switchKalmanModel()`
- 确保`filteredBBoxes_`包含最新的分类信息
- 最后执行`kalmanFilterAndUpdateHist()`用正确的模型更新

### 2. 根据分类使用正确的测量维度 (`kalmanFilterAndUpdateHist`)

**代码位置**：`dynamicDetector.cpp:2000-2017`

```cpp
// Human CA 和 Vehicle CTRA: 只测量2D位置
if (currDetectedBBox.is_human || currDetectedBBox.is_che) {
    measurement.resize(2);
    measurement << x, y;
}
// UAV CA 和 Else CV: 测量3D位置
else {
    measurement.resize(3);
    measurement << x, y, z;
}
```

**删除的代码**：
- 计算速度和加速度的代码（~30行）
- 这些值从未被使用，卡尔曼滤波器通过状态估计自动得到

### 3. 根据模型类型正确提取状态

**代码位置**：`dynamicDetector.cpp:2019-2070`

不同模型的状态向量格式：
- **Human CA (7维)**: `[x, y, z, vx, vy, ax, ay]`
- **Vehicle CTRA (7维)**: `[x, y, z, v, a, yaw, yaw_rate]` → 需转换为笛卡尔坐标
- **UAV CA (9维)**: `[x, y, z, vx, vy, vz, ax, ay, az]`
- **Else CV (6维)**: `[x, y, z, vx, vy, vz]`

### 4. 避免双重预测 (`switchKalmanModel`)

**代码位置**：`dynamicDetector.cpp:1433-1443`

**关键认识**：
- `boxAssociation`中已经对所有滤波器执行`predict()`
- `switchKalmanModel`中获取的`oldState`是**预测后**的状态
- 用预测后的状态初始化新模型，**不需要再predict()**

```cpp
// 用旧模型预测后的状态初始化新模型
newFilter->initialize(newState);
// 不再 predict()!
```

### 5. 简化关联代价计算 (`boxAssociation`)

**代码位置**：`dynamicDetector.cpp:1630-1645`

**旧方法**：
- Human/Vehicle用2D马氏距离
- UAV/Else用3D马氏距离

**新方法**：
- 所有物体统一用3D马氏距离
- 马氏距离会自动根据协方差矩阵处理不确定性
- 如果z方向不确定性大，P(2,2)会大，z差异的权重自动降低

```cpp
// 统一使用3D马氏距离
Eigen::Matrix3d P_pos = P.block<3, 3>(0, 0);
double cost = computeAssociationCost3D(predBox, predStd, currBox, currStd, P_pos);
double gateThreshold = gateThreshold3D_;
```

### 6. 简化模型切换逻辑

**代码位置**：`dynamicDetector.cpp:1330-1449`

**删除的复杂逻辑**：
- 基于状态维度推断旧模型类型
- 尝试从历史记录中判断是Human还是Vehicle

**新逻辑**：
- 直接从`boxHist_[index][0]`获取旧的分类标志
- 判断分类是否变化
- 只在分类变化时执行切换

## 最终流程

### 正常跟踪（分类未变）
```
1. boxAssociation():
   - 所有滤波器: predict() (t-1 → t)
   - 统一用3D马氏距离计算关联代价

2. trackingCB():
   - 对匹配成功的轨迹:
     * classifyBox() (重新分类)
     * switchKalmanModel() (分类未变，直接return)

3. kalmanFilterAndUpdateHist():
   - 根据分类使用相应维度的测量更新
   - 根据模型类型正确提取状态
```

### 首次分类切换模型
```
1. boxAssociation():
   - 旧模型(3D CV).predict() (t-1 → t)

2. trackingCB():
   - classifyBox() (首次分类 → is_human=true)
   - switchKalmanModel():
     * oldState = filter->getState() (获取预测后的状态)
     * 提取(x, y, z, vx, vy, vz)
     * newFilter.initialize(newState)
     * 不predict()!

3. kalmanFilterAndUpdateHist():
   - update(measurement) (使用正确维度的测量)
```

## 代码清理

### 已删除
- ✅ 无用的速度和加速度计算代码（~30行）
- ✅ 2D/3D关联代价的区分逻辑（~30行）
- ✅ 基于状态维度推断模型类型的复杂逻辑（~20行）

### 保留但未使用（可选清理）
- `computeAssociationCost2D()` - 可以删除
- `computeMahalanobisDistance()` (2D版本) - 可以删除
- `gateThreshold2D_` - 可以删除
- `kfAvgFrames_` - 建议保留在配置中

## 关键要点

1. ✅ **不重复预测**：切换模型时使用预测后的状态初始化
2. ✅ **正确的测量维度**：Human/Vehicle用2D，UAV/Else用3D
3. ✅ **正确的状态提取**：根据模型类型提取不同格式的状态
4. ✅ **统一的关联逻辑**：所有物体用3D马氏距离
5. ✅ **清晰的执行顺序**：关联 → 分类 → 切换 → 更新

## 测试建议

1. **验证分类正确性**：检查`is_human`, `is_che`, `is_uav`, `is_else`标志
2. **验证模型切换**：观察日志中的"Switched model"信息
3. **验证状态连续性**：模型切换前后位置和速度应该平滑过渡
4. **验证关联稳定性**：匹配成功率应该保持较高水平
5. **观察卡方距离**：应该大部分在门限内

## 可能的问题和调试

### 如果分类不准确
- 检查`classifyBox`的阈值参数
- 检查`classificationStartFrame_`是否合理
- 验证历史最大尺寸`maxHistorySizes_`是否正确更新

### 如果模型切换失败
- 检查ROS日志中的切换信息
- 验证状态转换逻辑（特别是CTRA的yaw处理）
- 确认滤波器初始化参数正确

### 如果跟踪不稳定
- 调整`gateThreshold3D_`
- 检查协方差矩阵是否合理
- 验证测量噪声参数R
