# 实现计划

- [x] 1. 创建服务定义和配置参数
  - [x] 1.1 创建GetPredictedTrajectories.srv服务定义文件
    - 定义请求字段：current_position, range, prediction_horizon, prediction_dt
    - 定义响应字段：obstacle_ids, current_positions, current_velocities, sizes, trajectory_lengths, trajectory_positions, trajectory_velocities, position_covariances
    - _需求: 3.1, 3.2_
  - [x] 1.2 在detector_param.yaml中添加轨迹预测配置参数
    - 添加trajectory_prediction配置组
    - 包含default_horizon, default_dt, collision_inflation, max_trajectory_points参数
    - _需求: 4.1, 4.2, 4.3, 4.4_
  - [x] 1.3 更新CMakeLists.txt添加新服务文件
    - 在add_service_files中添加GetPredictedTrajectories.srv
    - _需求: 3.1_

- [x] 2. 扩展静态点滤波器实现碰撞检测
  - [x] 2.1 在StaticPointFilter类中添加碰撞检测接口
    - 添加checkCollision(const Eigen::Vector3d& point)方法
    - 添加checkBoxCollision(const Eigen::Vector3d& center, const Eigen::Vector3d& size, double inflation)方法
    - 复用现有的getVoxelKey和isPointStatic逻辑
    - _需求: 2.1, 2.3_
  - [ ]* 2.2 编写碰撞检测属性测试
    - **Property 3: 碰撞截断正确性**
    - **验证: 需求 2.2, 2.4**

- [x] 3. 实现轨迹预测功能
  - [x] 3.1 在dynamicDetector.h中添加轨迹预测相关声明
    - 添加TrajectoryPoint结构体定义
    - 添加预测参数成员变量
    - 添加predictTrajectory函数声明
    - 添加新服务的服务器声明
    - _需求: 1.1, 1.2_
  - [x] 3.2 在dynamicDetector.cpp中实现参数初始化
    - 在initParam()中读取trajectory_prediction配置参数
    - 设置默认值处理
    - _需求: 4.1, 4.2, 4.3, 4.4_
  - [x] 3.3 实现predictTrajectory函数
    - 实现CA模型多步预测（人/无人机）
    - 实现CV模型多步预测（其他）
    - 实现CTRA模型多步预测（车辆）
    - 实现协方差传播
    - 集成碰撞检测和轨迹截断逻辑
    - _需求: 1.1, 1.2, 1.4, 2.1, 2.2_
  - [ ]* 3.4 编写轨迹预测属性测试
    - **Property 1: 轨迹点数量一致性**
    - **Property 2: 轨迹数据完整性**
    - **验证: 需求 1.1, 1.2, 1.4**

- [x] 4. 实现ROS服务接口
  - [x] 4.1 在registerCallback()中注册新服务
    - 注册GetPredictedTrajectories服务
    - _需求: 3.1_
  - [x] 4.2 实现getPredictedTrajectories服务回调函数
    - 解析请求参数，处理无效参数
    - 遍历动态障碍物，调用predictTrajectory
    - 组装扁平化响应数据
    - _需求: 1.1, 1.2, 1.3, 3.3, 3.4_
  - [ ]* 4.3 编写服务接口属性测试
    - **Property 4: 轨迹长度有效性**
    - **Property 5: 无效参数处理**
    - **Property 6: 空障碍物处理**
    - **验证: 需求 2.4, 3.3, 3.4**

- [ ] 5. Checkpoint - 确保所有测试通过
  - 确保所有测试通过，如有问题请询问用户。

- [x] 6. 创建测试可视化脚本
  - [x] 6.1 创建test_trajectory_prediction.py脚本
    - 实现服务客户端调用逻辑
    - 实现轨迹线Marker发布（LINE_STRIP）
    - 实现不确定性椭圆Marker发布（SPHERE）
    - 添加周期性调用和可视化更新
    - _需求: 5.1, 5.2, 5.3_

- [ ] 7. 最终Checkpoint - 确保所有测试通过
  - 确保所有测试通过，如有问题请询问用户。
