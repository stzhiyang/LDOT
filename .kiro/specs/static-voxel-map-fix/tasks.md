# 实现计划

- [x] 1. 添加新配置参数
  - [x] 1.1 在 `dynamicDetector.h` 中添加新成员变量
    - 添加 `confirmedDynamicFrames_` 向量用于跟踪每个轨迹的连续动态帧数
    - 添加 `voxelClearDynamicFrames_` 参数
    - 添加 `shapeStabilityThreshold_` 参数
    - _需求: 1.1, 2.1_
  - [x] 1.2 在 `dynamicDetector.cpp` 的 `initParam()` 中添加参数读取
    - 从ROS参数服务器读取 `voxel_clear_dynamic_frames`
    - 从ROS参数服务器读取 `shape_stability_threshold`
    - _需求: 1.1, 2.1_
  - [x] 1.3 在 `detector_param.yaml` 中添加默认参数配置
    - _需求: 1.1, 2.1_

- [x] 2. 实现动态一致性检查前置机制
  - [x] 2.1 修改 `classificationCB` 函数中的体素清除逻辑
    - 在确认动态后更新 `confirmedDynamicFrames_` 计数器
    - 只有当计数器达到 `voxelClearDynamicFrames_` 时才调用 `clearDynamicRegions`
    - 当物体不再被判定为动态时重置计数器
    - _需求: 1.1, 1.2, 1.3_
  - [ ]* 2.2 编写属性测试：动态一致性检查前置保证
    - **Property 1: 动态一致性检查前置保证**
    - **验证: 需求 1.1, 1.2, 1.3**

- [x] 3. 实现形状稳定性检查机制
  - [x] 3.1 在 `classificationCB` 中添加PCA变化率计算
    - 计算当前帧与历史帧的PCA标准差变化率
    - 当变化率低于阈值时抑制 `is_rotation_dynamic`
    - _需求: 2.1, 2.2_
  - [x] 3.2 添加尺寸变化检测逻辑
    - 检查历史尺寸变化是否超过 `sizeMergeThresh_`
    - 超过阈值时抑制 `is_rotation_dynamic`
    - _需求: 2.3_
  - [ ]* 3.3 编写属性测试：形状稳定性抑制误判
    - **Property 2: 形状稳定性抑制误判**
    - **验证: 需求 2.1, 2.2**
  - [ ]* 3.4 编写属性测试：遮挡检测正确性
    - **Property 3: 遮挡检测正确性**
    - **验证: 需求 2.3**

- [x] 4. 同步轨迹管理
  - [x] 4.1 在轨迹创建时初始化 `confirmedDynamicFrames_`
    - 在 `kalmanFilterAndUpdateHist` 函数中同步更新
    - _需求: 1.1_
  - [x] 4.2 在轨迹删除时同步删除对应计数器
    - 确保向量大小与轨迹数量一致
    - _需求: 1.1_

- [x] 5. 检查点 - 确保所有测试通过
  - 确保所有测试通过，如有问题请询问用户。

- [x] 6. 验证静态恢复机制
  - [x] 6.1 验证现有体素累积机制在清除后能正常恢复
    - 确认 `updateMap` 函数在体素被清除后能正常累积 hit_count
    - _需求: 3.1, 3.2_
  - [ ]* 6.2 编写属性测试：静态恢复机制
    - **Property 4: 静态恢复机制**
    - **验证: 需求 3.1, 3.2**

- [x] 7. 最终检查点 - 确保所有测试通过
  - 确保所有测试通过，如有问题请询问用户。
