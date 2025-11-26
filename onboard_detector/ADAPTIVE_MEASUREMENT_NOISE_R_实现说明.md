# 自适应测量噪声R实现说明

## 📋 修改概述

本次修改为LDOT动态障碍物检测系统的卡尔曼滤波器添加了**自适应测量噪声R**功能，可以根据历史测量数据的统计特性动态调整测量噪声协方差矩阵，提高不同场景下的跟踪精度。

---

## 🎯 核心原理

### 1. 基本思想
- **固定R的问题**: YAML配置中的测量噪声R是固定值，无法适应不同距离、不同目标清晰度、不同模型的实际测量噪声差异
- **自适应R的解决方案**: 通过分析最近K帧历史测量数据的抖动程度（标准差），动态估计当前的测量噪声水平

### 2. 算法流程

```
输入: track_id (轨迹索引), K (历史窗口大小)
输出: R_final (自适应调整的测量噪声矩阵)

步骤1: 从boxHist_[track_id]中提取最近K+1帧的位置测量 [x, y, z]
步骤2: 计算相邻帧差分: diff[k] = pos[k] - pos[k+1]
步骤3: 计算差分的方差: Var(diff)
步骤4: 理论修正: Var(noise) = Var(diff) / 2  (因为差分包含两次独立测量的噪声)
步骤5: 构建自适应R矩阵: R_adaptive = diag(Var_x, Var_y, Var_z)
步骤6: 模型特定调整:
       - Human: z轴 *= human_z_scale (0.5, 因为人在地面z轴变化小)
       - UAV: z轴 *= uav_z_scale (1.5, 因为无人机z轴运动剧烈)
       - CTRA (车辆): 转弯时xy轴 *= (1 + turn_scale * |yaw_rate|)
步骤7: 与原始R融合: R_final = weight*R_adaptive + (1-weight)*R_original
步骤8: 下限保护: R_final >= R_original * min_ratio
```

### 3. 数学推导

**为什么要除以2？**

相邻帧差分包含两次独立的测量噪声：
```
meas[k] = true_pos[k] + noise[k]
meas[k+1] = true_pos[k+1] + noise[k+1]

diff[k] = meas[k] - meas[k+1]
        = (true_pos[k] - true_pos[k+1]) + (noise[k] - noise[k+1])
        
Var(diff) = Var(true_motion) + Var(noise[k]) + Var(noise[k+1])
          ≈ Var(true_motion) + 2*Var(noise)  (假设噪声独立同分布)
          
如果目标匀速或低速: Var(true_motion) ≈ 0
则: Var(noise) ≈ Var(diff) / 2
```

---

## 📂 修改文件清单

### 1. 配置文件
**文件**: `cfg/detector_param.yaml`

**修改内容**: 添加自适应测量噪声R的配置参数
```yaml
kalman_filter:
  adaptive_measurement_noise:
    enable: true                  # 是否启用
    window_size: 10               # 使用最近K帧
    min_ratio: 0.3                # 下限保护比例
    adaptive_weight: 0.7          # 自适应权重
    human_z_scale: 0.5            # 人类z轴缩放
    uav_z_scale: 1.5              # 无人机z轴缩放
    ctra_turn_scale: 0.5          # 车辆转弯xy缩放系数
    ctra_turn_threshold: 0.1      # 转弯判定阈值
```

### 2. 卡尔曼滤波器基类
**文件**: `include/onboard_detector/multiModelKalmanFilter.h`

**修改内容**: 添加3个新方法
```cpp
// 设置测量噪声R (用于自适应调整)
void setMeasNoiseR(const Eigen::MatrixXd &R_new);

// 获取当前测量噪声R
const Eigen::MatrixXd &getMeasNoiseR() const;

// 获取底层运动模型 (用于访问原始参数)
std::shared_ptr<MotionModel> getModel();
```

### 3. 动态检测器头文件
**文件**: `include/onboard_detector/dynamicDetector.h`

**修改内容**: 
1. 添加成员变量（8个配置参数）
2. 添加函数声明
```cpp
private:
  // 自适应测量噪声R参数
  bool adaptiveREnabled_;
  int adaptiveRWindowSize_;
  double adaptiveRMinRatio_;
  double adaptiveRWeight_;
  double adaptiveRHumanZScale_;
  double adaptiveRUavZScale_;
  double adaptiveRCtraTurnScale_;
  double adaptiveRCtraTurnThreshold_;

  // 函数声明
  Eigen::MatrixXd computeAdaptiveMeasNoiseR(int track_id);
```

### 4. 动态检测器实现文件
**文件**: `include/onboard_detector/dynamicDetector.cpp`

**修改内容**:

#### 4.1 `initParam()` 函数 (约100行)
- 从YAML读取所有自适应R参数
- 打印配置信息到ROS日志

#### 4.2 新增 `computeAdaptiveMeasNoiseR()` 函数 (约180行)
- 完整实现自适应R计算算法
- 包含详细注释和调试输出

#### 4.3 `kalmanFilterAndUpdateHist()` 函数修改 (约8行)
- 在`update()`之前调用`computeAdaptiveMeasNoiseR()`
- 通过`setMeasNoiseR()`设置自适应R
```cpp
// 如果启用自适应测量噪声R，则计算并设置
if (this->adaptiveREnabled_) {
  Eigen::MatrixXd R_adaptive = this->computeAdaptiveMeasNoiseR(
      filtersTemp.size() - 1);
  filtersTemp.back()->setMeasNoiseR(R_adaptive);
}
```

---

## 🔧 使用方法

### 启用/禁用功能

在 `detector_param.yaml` 中设置：
```yaml
adaptive_measurement_noise:
  enable: true   # 启用
  # enable: false  # 禁用 (使用YAML中固定的meas_noise)
```

### 调试输出

设置ROS日志级别为DEBUG可以看到详细的自适应R信息：
```bash
# 临时设置
rosservice call /onboard_detector/set_logger_level "logger: 'ros.onboard_detector'
level: 'debug'"

# 或在launch文件中设置
<node name="onboard_detector" pkg="onboard_detector" type="onboard_detector_node" output="screen">
  <param name="log_level" value="debug"/>
</node>
```

输出示例：
```
[DEBUG] Track 3 adaptive R: [0.0234, 0.0189, 0.0067], original R: [0.1, 0.1, 0.1], hist_len=25, actual_K=10
[DEBUG] Track 5 (Vehicle turning, yaw_rate=0.35): xy R scaled by 1.175
```

---

## 📊 参数调优建议

### window_size (历史窗口大小)
- **默认值**: 10帧
- **建议范围**: 5-20帧
- **影响**: 
  - 太小 → 统计不稳定，噪声估计抖动大
  - 太大 → 响应慢，无法快速适应场景变化

### min_ratio (下限保护比例)
- **默认值**: 0.3 (不低于原始R的30%)
- **建议范围**: 0.2-0.5
- **影响**: 防止自适应R过小导致滤波器过于相信测量值

### adaptive_weight (自适应权重)
- **默认值**: 0.7 (70%相信自适应，30%相信原始)
- **建议范围**: 0.5-0.9
- **影响**:
  - 接近1.0 → 完全相信自适应估计
  - 接近0.5 → 保守，更多依赖原始配置

### 模型特定缩放因子

| 参数 | 默认值 | 适用场景 |
|------|--------|----------|
| `human_z_scale` | 0.5 | 人在平地行走，z轴噪声应减小 |
| `uav_z_scale` | 1.5 | 无人机z轴运动剧烈，增加容忍度 |
| `ctra_turn_scale` | 0.5 | 车辆转弯时聚类中心抖动系数 |
| `ctra_turn_threshold` | 0.1 rad/s | 判定车辆正在转弯的阈值 |

---

## ⚠️ 注意事项

### 1. 历史数据不足的处理
- 新目标（历史<2帧）: 自动返回原始R
- 短历史（<window_size）: 使用实际历史长度计算，权重动态调整

### 2. track_id的对应关系
在`kalmanFilterAndUpdateHist()`中：
- `h_idx`: 历史轨迹在旧容器中的索引
- `filtersTemp.size() - 1`: 当前滤波器在临时容器中的索引
- **关键**: 调用`computeAdaptiveMeasNoiseR()`时使用的是临时容器索引，但函数内部要访问的是继承来的历史数据

### 3. 性能影响
- 每帧每个匹配成功的轨迹都会计算一次自适应R
- 计算复杂度: O(K) 其中K是window_size
- 对于10-20个目标，window_size=10，额外开销可忽略

### 4. 适用场景
**适合使用的场景**:
- ✅ 目标距离变化大（远近测量噪声差异大）
- ✅ 部分遮挡/点云密度变化大
- ✅ 环境光照变化（影响激光雷达测量质量）

**不推荐使用的场景**:
- ❌ 目标高速机动（差分会包含过多运动信息）
- ❌ 历史数据极短（<5帧）的短暂追踪
- ❌ 测量噪声基本恒定的理想环境

---

## 🧪 测试验证

### 验证方法1: 日志检查
```bash
# 启动节点
roslaunch onboard_detector detector.launch

# 观察日志
# 应该看到类似输出:
# [INFO] Adaptive measurement noise R enabled: true
# [INFO] Adaptive R window size: 10
# ...
```

### 验证方法2: 对比实验
1. 设置 `enable: false`，运行并记录跟踪效果
2. 设置 `enable: true`，运行并记录跟踪效果
3. 对比指标：
   - 轨迹连续性
   - 位置估计误差
   - 对突然运动的响应速度

### 验证方法3: 极端场景测试
- 目标从近到远移动
- 目标被部分遮挡
- 目标高速转弯

---

## 📈 预期效果

| 场景 | 固定R | 自适应R |
|------|-------|---------|
| 近距离清晰目标 | R过大，滤波器不够相信测量 | R自动减小，更快响应 |
| 远距离/遮挡目标 | R过小，对噪声敏感 | R自动增大，更平滑 |
| 车辆转弯 | 位置估计可能跳变 | xy方向R增大，更稳定 |
| 无人机上下飞 | z轴可能过度平滑 | z轴R适当增大 |

---

## 🐛 调试技巧

### 如果发现轨迹不稳定
1. 检查R是否过小：增大`min_ratio`
2. 检查窗口是否太小：增大`window_size`
3. 检查权重是否太高：降低`adaptive_weight`

### 如果发现响应太慢
1. 检查R是否过大：降低`min_ratio`
2. 检查权重是否太低：提高`adaptive_weight`

### 查看实际R值
添加调试代码：
```cpp
ROS_INFO_STREAM("R diagonal: " << R_final(0,0) << ", " 
                               << R_final(1,1) << ", " 
                               << R_final(2,2));
```

---

## 📚 相关理论

### 卡尔曼滤波中R的作用
```
新息协方差: S = H*P*H' + R
卡尔曼增益: K = P*H'*S^{-1}
状态更新:   x = x + K*(z - H*x)

R越大 → S越大 → K越小 → 更相信预测
R越小 → S越小 → K越大 → 更相信测量
```

### 自适应滤波器分类
- **创新自适应**: 基于新息序列 (innovation)
- **残差自适应**: 基于残差序列 (residual)
- **历史统计自适应**: ✅ 本实现采用此方法

---

## 📞 联系与支持

如有问题，请检查：
1. 编译是否成功（无错误和警告）
2. YAML配置是否正确加载（查看启动日志）
3. 是否运行在支持的ROS版本（Noetic/Melodic）

---

**实现日期**: 2025-11-26  
**实现版本**: v1.0  
**作者**: Antigravity AI Assistant
