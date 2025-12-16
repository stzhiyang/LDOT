/*
    FILE: dynamicDetector.cpp
    ---------------------------------
    function implementation of dynamic osbtacle detector
*/
#include <cmath>   // for std::isfinite
#include <numeric> // for std::iota
#include <ldot_detector/dynamicDetector.h>
#include <ldot_detector/paramLoader.h>

// ===================================================================
// 初始化
// ===================================================================
namespace onboardDetector {
// 默认构造函数
dynamicDetector::dynamicDetector() {
  this->ns_ = "ldot_detector";
  this->hint_ = "[LDOT]";
  this->isStaticMapReady_ = false;
}

// 带节点句柄的构造函数
dynamicDetector::dynamicDetector(const ros::NodeHandle &nh) {
  this->ns_ = "ldot_detector";
  this->hint_ = "[LDOT]";
  this->nh_ = nh;
  this->isStaticMapReady_ = false;
  this->initParam();
  this->registerPub();
  this->registerCallback();
}

void dynamicDetector::initDetector(const ros::NodeHandle &nh) {
  this->nh_ = nh;
  this->initParam();
  this->registerPub();
  this->registerCallback();
}

void dynamicDetector::initParam() {
  // 使用 ParamLoader 加载所有参数
  ParamLoader loader(this->nh_, this->ns_, this->hint_);
  loader.loadAllParams(this);
}

void dynamicDetector::registerPub() {
  //===========================原始点云过滤与检测器可视化========================================
  // 原始激光雷达点可视化发布
  this->rawLidarPointsPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(
      this->ns_ + "/raw_lidar_point_cloud", 10);

  // 过滤后点云可视化发布
  this->downSamplePointsPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(
      this->ns_ + "/downsampled_point_cloud", 10);

  // 聚类检测后的点云发布
  this->filteredPointsPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(
      this->ns_ + "/filtered_point_cloud", 10);

  // 聚类检测后的边界框发布
  this->filteredBBoxesPub_ =
      this->nh_.advertise<visualization_msgs::MarkerArray>(
          this->ns_ + "/filtered_bboxes", 10);

  //============================数据跟踪可视化===============================================
  // 跟踪的边界框发布
  this->trackedBBoxesPub_ =
      this->nh_.advertise<visualization_msgs::MarkerArray>(
          this->ns_ + "/tracked_bboxes", 10);

  // 历史轨迹发布
  this->historyTrajPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(
      this->ns_ + "/history_trajectories", 10);

  //===========================动态检测可视化===============================================
  // 动态点云发布
  this->dynamicPointsPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(
      this->ns_ + "/dynamic_point_cloud", 10);

  // 动态边界框发布
  this->dynamicBBoxesPub_ =
      this->nh_.advertise<visualization_msgs::MarkerArray>(
          this->ns_ + "/dynamic_bboxes", 10);

  // 原始动态点云发布，没过滤的在动态box中的原始点云
  this->rawDynamicPointsPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(
      this->ns_ + "/raw_dynamic_point_cloud", 10);

  // 动态障碍物专用轨迹发布
  this->dynamicTrajPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(
      this->ns_ + "/dynamic_trajectories", 10);

}

void dynamicDetector::registerCallback() {
  // 启动时间将在收到第一帧点云时记录（避免仿真时间未初始化的问题）
  this->systemStartTime_ = ros::Time(0);  // 初始化为0，表示尚未记录
  ROS_INFO_STREAM(this->hint_ << " Detector initialized. Static map warmup duration: " 
                  << this->staticMapWarmupDuration_ << "s (will start when first cloud received)");

  this->odomSub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(
      this->nh_, this->odomTopicName_, 25));

  if (this->useLivoxCustomMsg_) {
    // 使用Livox CustomMsg格式
    this->lidarCustomMsgSub_.reset(
        new message_filters::Subscriber<livox_ros_driver2::CustomMsg>(
            this->nh_, this->lidarTopicName_, 50));
    this->lidarCustomOdomSync_.reset(
        new message_filters::Synchronizer<lidarCustomOdomSync>(
            lidarCustomOdomSync(100), *this->lidarCustomMsgSub_,
            *this->odomSub_));
    this->lidarCustomOdomSync_->registerCallback(
        boost::bind(&dynamicDetector::lidarCustomOdomCB, this, _1, _2));
  } else {
    // 使用标准PointCloud2格式
    this->lidarCloudSub_.reset(
        new message_filters::Subscriber<sensor_msgs::PointCloud2>(
            this->nh_, this->lidarTopicName_, 50));
    this->lidarOdomSync_.reset(
        new message_filters::Synchronizer<lidarOdomSync>(
            lidarOdomSync(100), *this->lidarCloudSub_, *this->odomSub_));
    this->lidarOdomSync_->registerCallback(
        boost::bind(&dynamicDetector::lidarOdomCB, this, _1, _2));
  }

  // 激光雷达检测定时器
  this->lidarDetectionTimer_ = this->nh_.createTimer(
      ros::Duration(this->dt_), &dynamicDetector::lidarDetectionCB, this);

  // 跟踪定时器
  this->trackingTimer_ = this->nh_.createTimer(
      ros::Duration(this->dt_), &dynamicDetector::trackingCB, this);

  // 分类定时器
  this->classificationTimer_ = this->nh_.createTimer(
      ros::Duration(this->dt_), &dynamicDetector::classificationCB, this);

  // 可视化定时器
  this->visTimer_ = this->nh_.createTimer(ros::Duration(this->dt_),
                                          &dynamicDetector::visCB, this);

  // 获取动态障碍物服务
  this->getDynamicObstacleServer_ =
      this->nh_.advertiseService("ldot_detector/get_dynamic_obstacles",
                                 &dynamicDetector::getDynamicObstacles, this);

  // 获取预测轨迹服务
  this->getPredictedTrajectoriesServer_ =
      this->nh_.advertiseService("ldot_detector/get_predicted_trajectories",
                                 &dynamicDetector::getPredictedTrajectories, this);
}



// ===================================================================
// 点云预处理
// ===================================================================
void dynamicDetector::lidarCustomOdomCB(
    const livox_ros_driver2::CustomMsgConstPtr &customMsg,
    const nav_msgs::OdometryConstPtr &odom) {
  auto start_time = std::chrono::high_resolution_clock::now();

  // 将CustomMsg转换为PointCloud2
  sensor_msgs::PointCloud2 cloudMsg;
  this->convertCustomMsgToPointCloud2(customMsg, cloudMsg);

  // 转换为ConstPtr并调用原有的处理函数
  sensor_msgs::PointCloud2ConstPtr cloudMsgPtr =
      boost::make_shared<sensor_msgs::PointCloud2>(cloudMsg);
  this->lidarOdomCB(cloudMsgPtr, odom);

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      end_time - start_time);

  size_t input_points = cloudMsg.width * cloudMsg.height;
  size_t output_points = (this->lidarCloud_) ? this->lidarCloud_->size() : 0;

  ROS_INFO_THROTTLE(1.0,
                    "%s: lidarCustomOdomCB took %.3f ms, points: %lu -> %lu",
                    this->hint_.c_str(), duration.count() / 1000.0,
                    input_points, output_points);
}

// 将Livox CustomMsg格式转换为PointCloud2格式
void dynamicDetector::convertCustomMsgToPointCloud2(
    const livox_ros_driver2::CustomMsgConstPtr &customMsg,
    sensor_msgs::PointCloud2 &cloud) {
  // auto start_time = std::chrono::high_resolution_clock::now();

  // 设置PointCloud2的基本信息
  cloud.header = customMsg->header;
  cloud.height = 1;
  cloud.width = customMsg->point_num;
  cloud.is_bigendian = false;
  cloud.is_dense = false;

  // 定义PointCloud2的字段
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(customMsg->point_num);

  // 创建迭代器访问xyz字段
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  // 将CustomMsg中的点转换到PointCloud2
  for (size_t i = 0; i < customMsg->points.size(); ++i) {
    const auto &point = customMsg->points[i];
    *iter_x = point.x;
    *iter_y = point.y;
    *iter_z = point.z;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  // auto end_time = std::chrono::high_resolution_clock::now();
  // auto duration =
  // std::chrono::duration_cast<std::chrono::microseconds>(end_time -
  // start_time); ROS_INFO_THROTTLE(1.0, "%s: CustomMsg to PointCloud2
  // conversion took %.3f ms for %u points",
  //                   this->hint_.c_str(), duration.count() / 1000.0,
  //                   customMsg->point_num);
}

// 里程计回调函数，处理点云和里程计数据
void dynamicDetector::lidarOdomCB(
    const sensor_msgs::PointCloud2ConstPtr &cloudMsg,
    const nav_msgs::OdometryConstPtr &odom) {
  // [Performance Timing] 测量回调函数耗时
  // auto start_time = std::chrono::high_resolution_clock::now();

  std::lock_guard<std::mutex> lock(cloudMutex_); // 加锁保护共享数据

  // 用于可视化
  this->hasSensorPose_ = true;
  this->latestCloud_ = cloudMsg;
  this->lastCloudTime_ = cloudMsg->header.stamp; // 记录时间戳

  // --- 提前更新位姿信息 ---
  Eigen::Matrix4d lidarPoseMatrix;
  this->getLidarPose(odom, lidarPoseMatrix);

  this->position_(0) = odom->pose.pose.position.x;
  this->position_(1) = odom->pose.pose.position.y;
  this->position_(2) = odom->pose.pose.position.z;
  Eigen::Quaterniond quat(
      odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
      odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
  this->orientation_ = quat.toRotationMatrix();

  this->positionLidar_(0) = lidarPoseMatrix(0, 3);
  this->positionLidar_(1) = lidarPoseMatrix(1, 3);
  this->positionLidar_(2) = lidarPoseMatrix(2, 3);
  this->orientationLidar_ = lidarPoseMatrix.block<3, 3>(0, 0);

  // 局部点云转换
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloudMsg, *tempCloud);

  // --- 优化：一次性滤波（X、Y范围）并进行均匀密度降采样 ---
  pcl::PointCloud<pcl::PointXYZ>::Ptr preTransformCloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  // 范围过滤后保留的点数不确定，预分配为原始点云大小以避免多次重新分配
  preTransformCloud->reserve(tempCloud->size());

  double x_max = this->localLidarRange_.x();
  double y_max = this->localLidarRange_.y();

  for (const pcl::PointXYZ &pt : tempCloud->points) {
    // 先做范围检查（最快的操作）
    if (std::abs(pt.x) > x_max || std::abs(pt.y) > y_max) {
      continue;
    }
    preTransformCloud->push_back(pt);
  }

  // --- 坐标变换 ---
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.linear() = this->orientationLidar_;
  transform.translation() = this->positionLidar_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*preTransformCloud, *transformedCloud, transform);

  // --- 过滤地面和天花板（在变换后进行，避免重复操作） ---
  pcl::PointCloud<pcl::PointXYZ>::Ptr groundRoofFilterCloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  groundRoofFilterCloud->reserve(transformedCloud->size());

  for (const pcl::PointXYZ &pt : transformedCloud->points) {
    if (pt.z >= this->groundHeight_ && pt.z <= this->roofHeight_) {
      groundRoofFilterCloud->push_back(pt);
    }
  }

  // --- 自适应Voxel Grid下采样（两阶段精细控制） ---
  pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud;

  if (this->enableVoxelDownsampling_ &&
      static_cast<int>(groundRoofFilterCloud->size()) >
          this->voxelTargetPointCount_) {
    // 初始化
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelFilteredCloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    float adaptiveLeafSize = this->voxelBaseLeafSize_;
    int iteration = 0;
    const int maxIterations = 10;      // 防止无限循环
    const float toleranceRatio = 1.2f; // 允许20%的容差范围

    pcl::PointCloud<pcl::PointXYZ>::Ptr currentCloud = groundRoofFilterCloud;

    // 迭代调整体素大小，直到点数接近目标值
    while (
        static_cast<int>(currentCloud->size()) >
            static_cast<int>(this->voxelTargetPointCount_ * toleranceRatio) &&
        iteration < maxIterations) {

      pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
      voxelFilter.setInputCloud(currentCloud);
      voxelFilter.setLeafSize(adaptiveLeafSize, adaptiveLeafSize,
                              adaptiveLeafSize);
      voxelFilter.filter(*voxelFilteredCloud);

      // 检查是否达到目标
      if (static_cast<int>(voxelFilteredCloud->size()) <=
          this->voxelTargetPointCount_) {
        break; // 达到目标，退出循环
      }

      // 更新参数准备下一次迭代
      currentCloud = voxelFilteredCloud;
      adaptiveLeafSize *= 1.2f; // 增大体素20%
      ++iteration;

      // 准备下一次迭代的输出云
      voxelFilteredCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    }

    finalCloud = currentCloud;

    // 输出下采样信息（用于调试和性能监控）
    // ROS_INFO_THROTTLE(
    //     2.0,
    //     "%s: Voxel downsampling: %lu -> %lu points (iters=%d,
    //     leafSize=%.3fm)", this->hint_.c_str(), groundRoofFilterCloud->size(),
    //     finalCloud->size(), iteration, adaptiveLeafSize);
  } else {
    // 不启用下采样或点数未超过阈值
    finalCloud = groundRoofFilterCloud;
  }

  // 存储处理后的点云
  this->lidarCloud_ = finalCloud;
  hasNewCloud_ = true; // 标记有新数据可用

  // 发布降采样后的点云
  sensor_msgs::PointCloud2 outputCloud;
  pcl::toROSMsg(*this->lidarCloud_, outputCloud);
  outputCloud.header.frame_id = "map";
  outputCloud.header.stamp = cloudMsg->header.stamp;
  this->downSamplePointsPub_.publish(outputCloud);

  // [Performance Timing] 输出耗时
  // auto end_time = std::chrono::high_resolution_clock::now();
  // auto duration =
  // std::chrono::duration_cast<std::chrono::microseconds>(end_time -
  // start_time); ROS_INFO_THROTTLE(1.0, "%s: lidarOdomCB took %.3f ms, points:
  // %lu -> %lu",
  //                  this->hint_.c_str(), duration.count() / 1000.0,
  //                  tempCloud->size(), this->lidarCloud_->size());
  // ROS_INFO_THROTTLE(1.0, "new pointCloud%s", this->hasNewCloud_);
}



// ===================================================================
// 检测
// ===================================================================
void dynamicDetector::lidarDetectionCB(const ros::TimerEvent &event) {
  auto start_time = std::chrono::high_resolution_clock::now();
  // 检查是否有新点云数据
  if (!hasNewCloud_) {
    // 这是正常的时序行为，定时器和点云数据不完全同步
    ROS_DEBUG_THROTTLE(5.0, "%s: No new cloud data available for detection",
                       this->hint_.c_str());
    return;
  }

  // 检查数据时效性（避免处理过时数据）
  // if ((event.current_real - lastCloudTime_).toSec() > 0.5) {
  //   ROS_WARN_THROTTLE(
  //       5.0, "%s: Cloud data too old (%.3f s), skipping detection",
  //       this->hint_.c_str(), (event.current_real - lastCloudTime_).toSec());
  //   return;
  // }

  {
    std::lock_guard<std::mutex> lock(cloudMutex_);
    this->lidarDetect();
    hasNewCloud_ = false; // 标记数据已处理
  }

  this->newDetectFlag_ = true; // get a new detection
  hasNewDetection_ = true;     // 标记有新检测结果

  // [Performance Timing] 输出耗时
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      end_time - start_time);
  ROS_INFO_THROTTLE(1.0, "%s: lidarDetectionCB took %.3f ms",
                    this->hint_.c_str(), duration.count() / 1000.0);
  
  lastProcessTime_ = ros::Time::now(); // 更新最后处理时间
}

void dynamicDetector::lidarDetect() {
  // 检查是否有激光雷达点云数据（提前返回避免不必要的处理）
  if (this->lidarCloud_ == NULL) {
    ROS_WARN_THROTTLE(1.0, "%s: No point cloud available for detection",
                      this->hint_.c_str());
    return;
  }

  // 1. 始终更新静态地图，使用当前ROS时间，并传入传感器位置（全局坐标系）
  double currentTime = ros::Time::now().toSec();
  this->staticFilter_->updateMap(this->lidarCloud_, currentTime, this->positionLidar_);

  // 检查静态地图预热阶段
  // 如果是第一帧，记录启动时间
  if (this->systemStartTime_.toSec() < 0.001) {
    this->systemStartTime_ = ros::Time::now();
    ROS_INFO_STREAM(this->hint_ << " First cloud received. Starting static map warmup (" 
                    << this->staticMapWarmupDuration_ << "s)...");
  }
  
  double elapsedTime = (ros::Time::now() - this->systemStartTime_).toSec();
  if (!this->isStaticMapReady_) {
    if (elapsedTime < this->staticMapWarmupDuration_) {
      // 预热阶段：只更新静态地图，不进行检测
      ROS_INFO_THROTTLE(1.0, "%s: Static map warmup phase (%.1f/%.1f s). Only updating static map...",
                        this->hint_.c_str(), elapsedTime, this->staticMapWarmupDuration_);
      return;
    } else {
      // 预热完成
      this->isStaticMapReady_ = true;
      ROS_INFO_STREAM(this->hint_ << " Static map warmup completed! Starting dynamic detection...");
    }
  }

  // 2. 收集保护区域（动态物体边界框）- 只收集一次，供点级和聚类级过滤共用
  std::vector<onboardDetector::box3D> protectedBoxes;
  if (this->staticFilterEnabled_ || this->staticClusterFilterEnabled_) {
    std::lock_guard<std::mutex> lock(this->bboxMutex_);
    for (const auto &track : this->boxHist_) {
      if (!track.empty() && track[0].is_dynamic) {
        protectedBoxes.push_back(track[0]);
      }
    }
  }

  // 3. 执行静态点过滤 (点级，可选)
  if (this->staticFilterEnabled_) {
    this->staticFilter_->filterPoints(this->lidarCloud_, protectedBoxes);
  }

  // 执行检测（检测器已在initParam中初始化）
  // 将点云数据传递给检测器并执行DBSCAN聚类
  this->lidarDetector_->getPointcloud(this->lidarCloud_);
  this->lidarDetector_->setSensorPosition(this->positionLidar_);  // 设置传感器位置（用于自适应DBSCAN）
  this->lidarDetector_->lidarDBSCAN();

  std::vector<onboardDetector::Cluster> lidarClustersRaw =
      this->lidarDetector_->getClusters();
  std::vector<onboardDetector::box3D> lidarBBoxesRaw =
      this->lidarDetector_->getBBoxes();
  std::vector<onboardDetector::box3D> lidarBBoxesFiltered;
  std::vector<onboardDetector::Cluster> lidarClustersFiltered;

  // 遍历所有边界框，过滤掉尺寸过大的对象并进行分类
  for (int i = 0; i < int(lidarBBoxesRaw.size()); ++i) {
    onboardDetector::box3D lidarBBox = lidarBBoxesRaw[i];
    if (lidarBBox.x_width > this->maxObjectSize_(0) ||
       lidarBBox.y_width > this->maxObjectSize_(1) ||
       lidarBBox.z_width > this->maxObjectSize_(2)) {
      continue;
    }

    lidarBBoxesFiltered.push_back(lidarBBox);
    lidarClustersFiltered.push_back(lidarClustersRaw[i]);
  }

  // 4. 执行静态聚类过滤 (聚类级) - 移至尺寸过滤之后以减少计算量
  if (this->staticClusterFilterEnabled_) {
    this->staticFilter_->filterClusters(lidarClustersFiltered,
                                        lidarBBoxesFiltered,
                                        this->staticClusterFilterRatio_,
                                        protectedBoxes);
  }

  // 保存过滤后的结果
  this->lidarBBoxes_ = lidarBBoxesFiltered;
  this->lidarClusters_ = lidarClustersFiltered;

  // 临时存储来自激光雷达的边界框及其点云特征（先缓存点云簇用于NMS）
  std::vector<onboardDetector::box3D> lidarBBoxesTemp;
  std::vector<std::vector<Eigen::Vector3d>> lidarPcClustersTemp;
  std::vector<Eigen::Vector3d> lidarPcClusterCentersTemp;
  std::vector<Eigen::Vector3d> lidarPcClusterStdsTemp; // 存储激光雷达输出

  // 将簇点云转成Eigen格式以便NMS处理；延迟计算质心与标准差直到NMS之后
  std::vector<std::vector<Eigen::Vector3d>> tmpPcClusters;
  tmpPcClusters.reserve(lidarClustersFiltered.size());
  for (size_t i = 0; i < lidarClustersFiltered.size(); ++i) {
    onboardDetector::Cluster cluster = lidarClustersFiltered[i];
    std::vector<Eigen::Vector3d> pcCluster;
    pcCluster.reserve(cluster.points->size());
    for (const pcl::PointXYZ &point : cluster.points->points) {
      pcCluster.emplace_back(point.x, point.y, point.z);
    }
    tmpPcClusters.push_back(std::move(pcCluster));
  }

  // 在生成特征之前进行帧内去重(NMS)以减少不必要计算
  if (this->enableDetectionNMS_ && tmpPcClusters.size() > 1) {
    size_t beforeNMS = lidarBBoxesFiltered.size();
    this->applyDetectionNMS(lidarBBoxesFiltered, tmpPcClusters,
                            lidarPcClusterCentersTemp,
                            lidarPcClusterStdsTemp);
    size_t afterNMS = lidarBBoxesFiltered.size();
    if (beforeNMS != afterNMS) {
      ROS_INFO_THROTTLE(1.0, "%s: Detection NMS (pre-feature): %lu -> %lu boxes",
                        this->hint_.c_str(), beforeNMS, afterNMS);
    }
  }

  // 将（已NMS或未NMS）结果转回用于后续处理的临时容器
  for (size_t i = 0; i < lidarBBoxesFiltered.size(); ++i) {
    onboardDetector::box3D lidarBBox = lidarBBoxesFiltered[i];
    std::vector<Eigen::Vector3d> &pcCluster = tmpPcClusters[i];

    // 提取点云簇的质心
    Eigen::Vector3d clusterCenter(0, 0, 0);
    for (const auto &pt : pcCluster) {
      clusterCenter += pt;
    }
    if (!pcCluster.empty()) clusterCenter /= static_cast<double>(pcCluster.size());

    // 计算点云簇的标准差（如果applyDetectionNMS已经计算过，保留其值）
    Eigen::Vector3d clusterStd(0, 0, 0);
    if (lidarPcClusterStdsTemp.size() == lidarBBoxesFiltered.size()) {
      clusterStd = lidarPcClusterStdsTemp[i];
    } else {
      for (const auto &pt : pcCluster) {
        Eigen::Vector3d diff = pt - clusterCenter;
        clusterStd.x() += diff.x() * diff.x();
        clusterStd.y() += diff.y() * diff.y();
        clusterStd.z() += diff.z() * diff.z();
      }
      if (!pcCluster.empty()) {
        clusterStd /= static_cast<double>(pcCluster.size());
        clusterStd = clusterStd.cwiseSqrt();
      }
    }

    // 存入临时变量
    lidarBBoxesTemp.push_back(lidarBBox);
    lidarPcClustersTemp.push_back(pcCluster);
    lidarPcClusterCentersTemp.push_back(clusterCenter);
    lidarPcClusterStdsTemp.push_back(clusterStd);
  }

  // 更新最终的过滤结果
  {
    std::lock_guard<std::mutex> lock(this->bboxMutex_);
    this->filteredBBoxes_ = lidarBBoxesTemp;
    this->filteredPcClusters_ = lidarPcClustersTemp;
    this->filteredPcClusterCenters_ = lidarPcClusterCentersTemp;
    this->filteredPcClusterStds_ = lidarPcClusterStdsTemp;
  }
}

/*!
 * @brief 帧内检测去重(NMS) - 合并同一物体的多个重叠检测框
 * @param bboxes 检测框列表（会被修改）
 * @param pcClusters 点云聚类列表（会被修改）
 * @param pcClusterCenters 点云中心列表（会被修改）
 * @param pcClusterStds 点云标准差列表（会被修改）
 * 算法逻辑：
 * 1. 按边界框体积从大到小排序（保留较大检测，抑制较小重复检测）
 * 2. 遍历每个检测框，判断是否应该合并（IoU高 或 中心距离近）
 * 3. 如果满足合并条件，则合并两个检测（合并点云、重新计算边界框）
 */
void dynamicDetector::applyDetectionNMS(
    std::vector<onboardDetector::box3D> &bboxes,
    std::vector<std::vector<Eigen::Vector3d>> &pcClusters,
    std::vector<Eigen::Vector3d> &pcClusterCenters,
    std::vector<Eigen::Vector3d> &pcClusterStds) {

  if (bboxes.size() <= 1) {return; }

  int n = bboxes.size();

  // 计算每个边界框的体积（用作排序依据：保留较大的检测）
  std::vector<double> volumes(n);
  std::vector<Eigen::Vector3d> centers(n);
  std::vector<double> avgSizes(n);
  std::vector<double> distThresholds(n);
  for (int i = 0; i < n; ++i) {
    volumes[i] = bboxes[i].x_width * bboxes[i].y_width * bboxes[i].z_width;
    centers[i] = Eigen::Vector3d(bboxes[i].x, bboxes[i].y, bboxes[i].z);
    double avgSize = (bboxes[i].x_width + bboxes[i].y_width + bboxes[i].z_width) / 3.0;
    avgSizes[i] = avgSize;
    distThresholds[i] = avgSize * this->detectionNMSDistScale_; // scaled by parameter
  }

  // 按体积从大到小排序的索引
  std::vector<int> sortedIdx(n);
  std::iota(sortedIdx.begin(), sortedIdx.end(), 0);
  std::sort(sortedIdx.begin(), sortedIdx.end(),
            [&volumes](int a, int b) { return volumes[a] > volumes[b]; });

  // 标记被抑制的检测
  std::vector<bool> suppressed(n, false);

  // 存储合并后的结果
  std::vector<onboardDetector::box3D> mergedBBoxes;
  std::vector<std::vector<Eigen::Vector3d>> mergedPcClusters;
  std::vector<Eigen::Vector3d> mergedPcClusterCenters;
  std::vector<Eigen::Vector3d> mergedPcClusterStds;

  for (int _i = 0; _i < n; ++_i) {
    int i = sortedIdx[_i];
    if (suppressed[i])
      continue;

    // 收集所有应该合并的检测框（包括自己）
    std::vector<int> toMerge;
    toMerge.push_back(i);

    // 查找所有与当前框应该合并的检测框
    for (int _j = _i + 1; _j < n; ++_j) {
      int j = sortedIdx[_j];
      if (suppressed[j])continue;
      
      double iou = this->compute3DIoU(bboxes[i], bboxes[j]);

      // 计算中心点距离（使用平方距离避免不必要的开方）
      double dx = centers[i].x() - centers[j].x();
      double dy = centers[i].y() - centers[j].y();
      double dz = centers[i].z() - centers[j].z();
      double centerDistSqr = dx * dx + dy * dy + dz * dz;

      // 计算两个框的平均尺寸（用于自适应距离阈值）
      // 使用之前缓存好的平均尺寸和距离阈值
      // avgSizes is cached and used to compute distThresholds (above)
      double distThreshold = (distThresholds[i] + distThresholds[j]) / 2.0;
      double distThresholdSqr = distThreshold * distThreshold;

      // 合并条件：IoU高 或 中心距离近
      bool shouldMerge = (iou > this->detectionNMSIoUThreshold_) ||
             (centerDistSqr < distThresholdSqr);

      if (shouldMerge) {
        // 标记为抑制
        suppressed[j] = true;toMerge.push_back(j);
      }
    }

    // 合并所有收集到的检测框
    // 1. 合并点云（使用移动语义，并预分配内存以避免反复分配）
    std::vector<Eigen::Vector3d> mergedPc;
    size_t totalPts = 0;
    for (int idx : toMerge) totalPts += pcClusters[idx].size();
    mergedPc.reserve(totalPts);

    // 为合并后的统计量做准备（避免再次遍历点云）
    Eigen::Vector3d sumPos(0, 0, 0);
    Eigen::Vector3d sumSq(0, 0, 0); // sum of squares for variance
    size_t mergedPtCount = 0;

    for (int idx : toMerge) {
      // 移动每个点进入mergedPc（避免复制）
      for (auto &pt : pcClusters[idx]) {
        mergedPc.push_back(std::move(pt));
        sumPos += mergedPc.back();
        sumSq += mergedPc.back().cwiseProduct(mergedPc.back());
        ++mergedPtCount;
      }
      // 清理移动后的小向量容量（optional）
      std::vector<Eigen::Vector3d>().swap(pcClusters[idx]);
    }

    // 2. 从合并后的点云重新计算边界框（更准确），使用盒子边界的并集作为最小/最大值
    if (mergedPtCount == 0) {
      continue;
    }

    double minX = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double minY = std::numeric_limits<double>::max();
    double maxY = std::numeric_limits<double>::lowest();
    double minZ = std::numeric_limits<double>::max();
    double maxZ = std::numeric_limits<double>::lowest();
    // 使用原有边界框的边界作为合并后的包围盒边界，避免再次遍历所有点
    for (int idx : toMerge) {
      double bminX = bboxes[idx].x - bboxes[idx].x_width / 2.0;
      double bmaxX = bboxes[idx].x + bboxes[idx].x_width / 2.0;
      double bminY = bboxes[idx].y - bboxes[idx].y_width / 2.0;
      double bmaxY = bboxes[idx].y + bboxes[idx].y_width / 2.0;
      double bminZ = bboxes[idx].z - bboxes[idx].z_width / 2.0;
      double bmaxZ = bboxes[idx].z + bboxes[idx].z_width / 2.0;

      minX = std::min(minX, bminX);
      maxX = std::max(maxX, bmaxX);
      minY = std::min(minY, bminY);
      maxY = std::max(maxY, bmaxY);
      minZ = std::min(minZ, bminZ);
      maxZ = std::max(maxZ, bmaxZ);
    }

    // 计算新的边界框（位置使用点云质心）
    onboardDetector::box3D mergedBox;
    // 合并得到的边界框没有明确的原始簇 id，设置为 -1 表示未知/合并产生
    mergedBox.id = -1.0;
    // 计算点云质心
    Eigen::Vector3d mergedCenter = sumPos / static_cast<double>(mergedPc.size());
    // box位置使用点云质心
    mergedBox.x = mergedCenter.x();
    mergedBox.y = mergedCenter.y();
    mergedBox.z = mergedCenter.z();
    // 尺寸使用包围盒
    mergedBox.x_width = maxX - minX;
    mergedBox.y_width = maxY - minY;
    mergedBox.z_width = maxZ - minZ;

    // 计算点云标准差（PCA特征），使用在合并点云时就累加的sumSq与sumPos
    Eigen::Vector3d mergedStd(0, 0, 0);
    Eigen::Vector3d mean = mergedCenter;
    Eigen::Vector3d var = (sumSq / static_cast<double>(mergedPtCount)) -
                          mean.cwiseProduct(mean);
    // 防止数值不稳定导致负数
    for (int k = 0; k < 3; ++k) {
      if (var[k] < 0) var[k] = 0;
    }
    mergedStd = var.cwiseSqrt();

    // 保存合并后的结果
    mergedBBoxes.push_back(mergedBox);
    mergedPcClusters.push_back(mergedPc);
    mergedPcClusterCenters.push_back(mergedCenter);
    mergedPcClusterStds.push_back(mergedStd);
  }

  // 更新输出
  bboxes = mergedBBoxes;
  pcClusters = mergedPcClusters;
  pcClusterCenters = mergedPcClusterCenters;
  pcClusterStds = mergedPcClusterStds;
}



// ===================================================================
// 跟踪
// ===================================================================
// 跟踪定时器回调函数,有个问题，匹配时，多出的轨迹直接丢掉
void dynamicDetector::trackingCB(const ros::TimerEvent &) {
  // [Performance Timing] 测量回调函数耗时
  auto start_time = std::chrono::high_resolution_clock::now();

  // 静态地图预热阶段，跳过跟踪
  if (!this->isStaticMapReady_) {
    return;
  }

  // 检查是否有新检测结果
  if (!hasNewDetection_) {
    // 这是正常的时序行为，定时器和检测结果不完全同步
    ROS_DEBUG_THROTTLE(5.0, "%s: No new detection available for tracking",
                       this->hint_.c_str());
    return; // 跳过，避免重复处理相同数据
  }

  std::lock_guard<std::mutex> lock(bboxMutex_); // 加锁保护边界框数据

  // 数据关联线程（预测步骤在 boxAssociation 内部执行）
  std::vector<int> bestMatch;      // 存储当前检测与历史障碍物的匹配索引。
  this->boxAssociation(bestMatch); // 执行边界框关联。

  // --- 1. 先进行物体分类(针对匹配成功的旧轨迹) ---
  if (bestMatch.size()) {
    for (int i = 0; i < int(bestMatch.size()); ++i) {
      if (bestMatch[i] >= 0) { // 匹配成功的旧轨迹
        int histIndex = bestMatch[i];

        // 边界检查：确保 histIndex 在所有向量的有效范围内
        if (histIndex < 0 ||
            histIndex >= static_cast<int>(this->boxHist_.size()) ||
            histIndex >= static_cast<int>(this->maxHistorySizes_.size()) ||
            histIndex >= static_cast<int>(this->smallSizeCounter_.size())) {
          continue;  // 跳过无效索引
        }

        // 1.1 稳健的历史尺寸更新
        double curr_x = this->filteredBBoxes_[i].x_width;
        double curr_y = this->filteredBBoxes_[i].y_width;
        double curr_z = this->filteredBBoxes_[i].z_width;

        double max_x = this->maxHistorySizes_[histIndex].x();
        double max_y = this->maxHistorySizes_[histIndex].y();
        double max_z = this->maxHistorySizes_[histIndex].z();

        // 检查合并 (尺寸突增且点数突增)
        bool isMerge = false;
        // 仅在有历史记录时检查
        if (this->boxHist_[histIndex].size() > 1) {
          double sizeRatioX = curr_x / std::max(max_x, 0.1);
          double sizeRatioY = curr_y / std::max(max_y, 0.1);
          double sizeRatioZ = curr_z / std::max(max_z, 0.1);
          double maxRatio = std::max({sizeRatioX, sizeRatioY, sizeRatioZ});

          // 检查点数增加
          int currPoints = this->filteredPcClusters_[i].size();
          int prevPoints = this->pcHist_[histIndex][0].size(); // 上一帧
          double pointRatio =
              (double)currPoints / std::max((double)prevPoints, 1.0);

          if (maxRatio > this->sizeMergeThresh_ &&
              pointRatio > this->pointCountMergeThresh_) {
            isMerge = true;
            // ROS_WARN_STREAM(this->hint_ << " Merge detected for object " <<
            // histIndex
            //                 << ". Size ratio: " << maxRatio << ", Point
            //                 ratio: " << pointRatio
            //                 << ". Skipping max size update.");
          }
        }

        // 检查分离/重置 (尺寸持续小于最大值)
        bool isReset = false;
        // 检查当前尺寸是否显著小于最大值 (例如 < 80%)
        if (curr_x < max_x * 0.8 && curr_y < max_y * 0.8 &&
            curr_z < max_z * 0.8) {
          this->smallSizeCounter_[histIndex]++;
        } else {
          this->smallSizeCounter_[histIndex] =
              0; // 如果尺寸接近最大值，重置计数器
        }

        if (this->smallSizeCounter_[histIndex] > this->sizeResetFrames_) {
          isReset = true;
          // 将最大尺寸重置为当前尺寸
          this->maxHistorySizes_[histIndex] =
              Eigen::Vector3d(curr_x, curr_y, curr_z);
          this->smallSizeCounter_[histIndex] = 0;
          // ROS_INFO_STREAM(this->hint_ << " Size reset for object " <<
          // histIndex
          //                 << " after " << this->sizeResetFrames_ << " frames
          //                 of small size.");
        }

        // 如果未合并且未重置，更新历史最大尺寸
        if (!isMerge && !isReset) {
          if (curr_x > this->maxHistorySizes_[histIndex].x())
            this->maxHistorySizes_[histIndex].x() = curr_x;
          if (curr_y > this->maxHistorySizes_[histIndex].y())
            this->maxHistorySizes_[histIndex].y() = curr_y;
          if (curr_z > this->maxHistorySizes_[histIndex].z())
            this->maxHistorySizes_[histIndex].z() = curr_z;
        }

        // 1.2 检查是否需要进行分类
        // 首次分类：达到 classificationStartFrame_ 且从未分类过 (is_else 为
        // true) 后续分类：使用 ROS 时间间隔 classificationIntervalSec_ 判断
        bool needClassify = false;

        // 确保 lastClassifyTime_ 和 stableClassificationCount_ 与历史大小匹配
        if (lastClassifyTime_.size() < this->boxHist_.size()) {
          lastClassifyTime_.resize(this->boxHist_.size(), ros::Time(0));
        }
        if (stableClassificationCount_.size() < this->boxHist_.size()) {
          stableClassificationCount_.resize(this->boxHist_.size(), 0);
        }

        if (int(this->boxHist_[histIndex].size()) ==
            this->classificationStartFrame_) {
          // 首次达到分类阈值，进行分类
          needClassify = true;
          lastClassifyTime_[histIndex] = ros::Time::now();
        } else if (int(this->boxHist_[histIndex].size()) >
                   this->classificationStartFrame_) {
          // 已经分类过，检查时间间隔
          ros::Duration timeSince =
              ros::Time::now() - lastClassifyTime_[histIndex];
          if (timeSince.toSec() >= this->classificationIntervalSec_) {
            needClassify = true;
            lastClassifyTime_[histIndex] = ros::Time::now();
          }
        }

        // 检查分类是否已经固定（连续多次相同分类后不再更新）
        bool classificationLocked = this->boxHist_[histIndex][0].fix_size;

        if (needClassify && !classificationLocked) {
          // 保存分类前的状态用于比较
          bool prevIsHuman = this->boxHist_[histIndex][0].is_human;
          bool prevIsChe = this->boxHist_[histIndex][0].is_che;
          bool prevIsUav = this->boxHist_[histIndex][0].is_uav;

          Eigen::Vector4f centroid;
          centroid << this->filteredPcClusterCenters_[i](0),
              this->filteredPcClusterCenters_[i](1),
              this->filteredPcClusterCenters_[i](2), 1.0;

          // 对当前检测框进行分类,结果写入filteredBBoxes_[i]
          // 使用历史最大尺寸，传入轨迹索引用于xy距离检查
          this->classifyBox(this->filteredBBoxes_[i], centroid,
                            this->maxHistorySizes_[histIndex], histIndex);

          // 检查分类是否与上次相同（仅对明确分类：人/车/无人机）
          bool currIsSpecific = this->filteredBBoxes_[i].is_human || 
                                this->filteredBBoxes_[i].is_che || 
                                this->filteredBBoxes_[i].is_uav;
          bool prevIsSpecific = prevIsHuman || prevIsChe || prevIsUav;
          bool sameClassification = currIsSpecific && prevIsSpecific &&
                                    (this->filteredBBoxes_[i].is_human == prevIsHuman) &&
                                    (this->filteredBBoxes_[i].is_che == prevIsChe) &&
                                    (this->filteredBBoxes_[i].is_uav == prevIsUav);

          // 更新连续相同分类计数
          if (histIndex < static_cast<int>(this->stableClassificationCount_.size())) {
            if (sameClassification) {
              this->stableClassificationCount_[histIndex]++;
              // 达到阈值则固定尺寸和分类
              if (this->stableClassificationCount_[histIndex] >= this->fixSizeClassificationThreshold_) {
                this->filteredBBoxes_[i].fix_size = true;
              }
            } else {
              // 分类改变，重置计数
              this->stableClassificationCount_[histIndex] = currIsSpecific ? 1 : 0;
              this->filteredBBoxes_[i].fix_size = false;
            }
          }

          // 立即切换卡尔曼滤波模型(在更新之前)
          this->switchKalmanModel(histIndex, this->filteredBBoxes_[i]);
        } else {
          // 未达到分类或重新分类条件,继承历史分类结果,继承fix_size标志
          this->filteredBBoxes_[i].is_human =
              this->boxHist_[histIndex][0].is_human;
          this->filteredBBoxes_[i].is_che = this->boxHist_[histIndex][0].is_che;
          this->filteredBBoxes_[i].is_uav = this->boxHist_[histIndex][0].is_uav;
          this->filteredBBoxes_[i].is_else =
              this->boxHist_[histIndex][0].is_else;
          this->filteredBBoxes_[i].fix_size = 
              this->boxHist_[histIndex][0].fix_size;
        }
      }
    }
  }

  // --- 2. 卡尔曼滤波跟踪(此时filteredBBoxes_已包含最新的分类信息) ---
  if (bestMatch.size()) {
    this->kalmanFilterAndUpdateHist(bestMatch); // 更新卡尔曼滤波器和历史记录

    // --- 3. 移除重复轨迹（解决幽灵轨迹问题）---
    this->removeDuplicateTracks();
  } else { // 如果当前帧没有任何检测结果
    // 清空历史记录。
    this->boxHist_.clear();
    this->pcHist_.clear();
    this->pcCenterHist_.clear();
    this->pcStdHist_.clear();
    this->maxHistorySizes_.clear();
    this->smallSizeCounter_.clear();
    this->filters_.clear(); // 同时清空滤波器
    // 同时清空 stableClassificationCount_ 和 lastClassifyTime_（用于fix_size功能）
    this->stableClassificationCount_.clear();
    this->lastClassifyTime_.clear();
  }

  hasNewDetection_ = false; // 标记检测结果已处理
  hasNewTracking_ = true;   // 标记有新跟踪结果

  // [Performance Timing] 输出耗时
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      end_time - start_time);
  ROS_INFO_THROTTLE(1.0, "%s: trackingCB took %.3f ms", this->hint_.c_str(),
                    duration.count() / 1000.0);
}

// ----------------------------------------关联-------------------------------------
/*!
 * @brief 使用马氏距离和匈牙利算法进行数据关联
 * @param[out] bestMatch
 * 最佳匹配结果，bestMatch[i]表示第i个当前检测对应的历史轨迹索引（-1表示新目标）
 *
 * 关联流程：
 * 1. 首次检测：初始化历史记录和卡尔曼滤波器
 * 2. 后续检测：
 *    - 构建代价矩阵（基于马氏距离、尺寸差异、点云标准差差异）
 *    - 使用匈牙利算法求解最优匹配
 *    - 应用关联门限，拒绝不可靠匹配
 */
void dynamicDetector::boxAssociation(std::vector<int> &bestMatch) {
  int numCurrObjs = int(this->filteredBBoxes_.size());

  // 第一次检测：初始化所有目标
  if (this->boxHist_.size() == 0) {
    // 预留空间但不初始化，避免 resize + push_back 导致大小翻倍
    this->boxHist_.reserve(numCurrObjs);
    this->pcHist_.reserve(numCurrObjs);
    this->pcCenterHist_.reserve(numCurrObjs);
    this->pcStdHist_.reserve(numCurrObjs);
    this->maxHistorySizes_.reserve(numCurrObjs);
    this->smallSizeCounter_.reserve(numCurrObjs);
    this->filters_.reserve(numCurrObjs);
    this->trackMissedFrames_.reserve(numCurrObjs);
    this->trackedBBoxes_.reserve(numCurrObjs);
    // 第一帧：bestMatch[i] = i 表示自己匹配自己，避免在 kalmanFilterAndUpdateHist 中重复初始化
    bestMatch.resize(numCurrObjs);

    for (int i = 0; i < numCurrObjs; ++i) {
      // 设置匹配索引为自己，避免被当作新目标重复初始化
      bestMatch[i] = i;

      // 使用 push_back 统一添加元素
      std::deque<onboardDetector::box3D> newBoxHist;
      newBoxHist.push_back(this->filteredBBoxes_[i]);
      this->boxHist_.push_back(newBoxHist);

      std::deque<std::vector<Eigen::Vector3d>> newPcHist;
      newPcHist.push_back(this->filteredPcClusters_[i]);
      this->pcHist_.push_back(newPcHist);

      // 使用box位置作为质心历史（与后续帧保持一致）
      std::deque<Eigen::Vector3d> newPcCenterHist;
      Eigen::Vector3d boxCenter(this->filteredBBoxes_[i].x, 
                                 this->filteredBBoxes_[i].y, 
                                 this->filteredBBoxes_[i].z);
      newPcCenterHist.push_back(boxCenter);
      this->pcCenterHist_.push_back(newPcCenterHist);

      std::deque<Eigen::Vector3d> newPcStdHist;
      newPcStdHist.push_back(this->filteredPcClusterStds_[i]);
      this->pcStdHist_.push_back(newPcStdHist);

      // 初始化历史最大尺寸
      this->maxHistorySizes_.push_back(Eigen::Vector3d(
          this->filteredBBoxes_[i].x_width, this->filteredBBoxes_[i].y_width,
          this->filteredBBoxes_[i].z_width));

      this->smallSizeCounter_.push_back(0);
      this->trackMissedFrames_.push_back(0);

      // 强制所有目标使用 3D CV 模型，并初始化分类信息为 is_else
      auto &bbox = this->filteredBBoxes_[i];
      // 初始化分类信息：新目标默认为 is_else（与滤波器模型一致）
      bbox.is_human = false;
      bbox.is_che = false;
      bbox.is_uav = false;
      bbox.is_else = true;

      auto newFilter = createKalmanFilter(false, // is_human
                                          false, // is_che
                                          false, // is_uav
                                          true,  // is_else -> 强制使用 3D CV
                                          this->kfParams_);

      newFilter->setDt(this->dt_);

      // 统一使用 3D CV 初始化：[x, y, z, vx, vy, vz]
      Eigen::VectorXd detection(6);
      detection(0) = bbox.x;
      detection(1) = bbox.y;
      detection(2) = bbox.z;
      detection(3) = 0.0; // vx
      detection(4) = 0.0; // vy
      detection(5) = 0.0; // vz

      newFilter->initialize(detection);
      this->filters_.push_back(newFilter);

      // 初始化 trackedBBoxes_，第一帧的跟踪结果就是检测结果（已包含正确的分类信息）
      this->trackedBBoxes_.push_back(this->filteredBBoxes_[i]);
    }

    // 第一帧初始化完成，设置标志并返回，不需要再调用 kalmanFilterAndUpdateHist
    this->newDetectFlag_ = false;
    return;
  } else if (this->newDetectFlag_) {
    // 后续检测：使用匈牙利算法进行关联
    int numHistObjs = int(this->boxHist_.size());
    bestMatch.resize(numCurrObjs, -1);

    // 确保 filters_ 大小与 boxHist_ 一致
    if (this->filters_.size() != this->boxHist_.size()) {
      ROS_WARN_THROTTLE(1.0, "%s: filters_ size mismatch, skipping association",
                        this->hint_.c_str());
      return;
    }

    // 首先对所有历史轨迹的卡尔曼滤波器执行预测步骤
    for (int j = 0; j < numHistObjs; ++j) {
      if (this->filters_[j]) {
        this->filters_[j]->setDt(this->dt_);
        this->filters_[j]->predict();
      }
    }

    // 构建代价矩阵
    std::vector<std::vector<double>> costMatrix(
        numCurrObjs, std::vector<double>(numHistObjs, 1e9));

    //  遍历当前障碍物与所有历史轨迹的代价，即矩阵的行，为当前检测到的障碍物，列为按顺序排好的每个历史轨迹
    for (int i = 0; i < numCurrObjs; ++i) {
      const onboardDetector::box3D &currBox = this->filteredBBoxes_[i];
      const Eigen::Vector3d &currStd = this->filteredPcClusterStds_[i];

      for (int j = 0; j < numHistObjs; ++j) {
        // 边界检查和空指针检查
        if (this->boxHist_[j].empty() || this->pcStdHist_[j].empty() ||
            !this->filters_[j]) {
          continue;
        }

        // 获取历史轨迹的最新状态
        const onboardDetector::box3D &histBox = this->boxHist_[j][0];
        const Eigen::Vector3d &histStd = this->pcStdHist_[j][0];

        // 使用卡尔曼滤波器预测的位置构建预测bbox
        // 注意：predBox的位置来自卡尔曼滤波预测，尺寸保持历史值（尺寸不参与状态估计）
        onboardDetector::box3D predBox = histBox;
        const Eigen::VectorXd &filterStates = this->filters_[j]->getState();

        // 根据不同的滤波器类型提取预测位置
        // 注意: 所有模型的状态向量中位置都是三维的 [x, y, z, ...]
        predBox.x = filterStates(0);
        predBox.y = filterStates(1);
        predBox.z = filterStates(2);

        // 预测状态使用历史尺寸和点云标准差（这些不参与卡尔曼滤波）
        // predBox的尺寸已经在初始化时从histBox复制，无需额外设置
        const Eigen::Vector3d &predStd = histStd; // 点云标准差使用历史值

        // 计算关联代价：所有物体统一使用3D马氏距离
        // 注意：所有模型的状态向量中位置都是3维的[x,y,z,...]
        // 马氏距离会自动根据协方差矩阵处理不确定性
        // (例如，如果z方向不确定性大，z的差异对距离贡献会被自动降权)
        const Eigen::MatrixXd &P = this->filters_[j]->getCovariance();

        // 提取位置部分的协方差（状态向量的前3x3块）
        Eigen::Matrix3d P_pos = P.block<3, 3>(0, 0);

        double cost = this->computeAssociationCost3D(predBox, predStd, currBox,
                                                     currStd, P_pos);

        // 统一使用3D门限
        double gateThreshold = this->gateThreshold3D_;
        
        // 【改进】对于 coasting 轨迹（trackMissedFrames_[j] > 0），放宽门限
        // 这样可以更容易地匹配到新检测，防止错误地生成新轨迹
        if (j < static_cast<int>(this->trackMissedFrames_.size()) &&
            this->trackMissedFrames_[j] > 0) {
          gateThreshold *= this->coastingTrackGateRelaxFactor_;
        }

        // 应用关联门限(使用动态门限)
        if (cost < gateThreshold) {
          costMatrix[i][j] = cost;
          // 调试:输出通过门限的匹配
          // ROS_DEBUG_THROTTLE(
          //     0.5, "%s: Match [curr:%d->hist:%d] PASS: cost=%.2f <
          //     gate=%.2f", this->hint_.c_str(), i, j, cost, gateThreshold);
        } else {
          // 调试:输出被门限拒绝的匹配
          // ROS_WARN_THROTTLE(
          //     1.0,
          //     "%s: Match [curr:%d->hist:%d] REJECT: cost=%.2f >= gate=%.2f",
          //     this->hint_.c_str(), i, j, cost, gateThreshold);
        }
      }
    }

    // 使用匈牙利算法求解最优匹配
    this->hungarianAlgorithm(costMatrix, bestMatch);

    // // 统计并输出关联结果
    // int numMatched = 0;
    // int numNewTargets = 0;
    // for (int i = 0; i < numCurrObjs; ++i) {
    //   if (bestMatch[i] >= 0) {
    //     numMatched++;
    //   } else {
    //     numNewTargets++;
    //   }
    // }
    // int numLostTargets = numHistObjs - numMatched;

    // // 简洁的日志输出
    // ROS_INFO_THROTTLE(
    //     0.5, "%s: boxAssociation[currBox:%d histBox:%d] -> [o:%d +:%d -:%d]",
    //     this->hint_.c_str(), numCurrObjs, numHistObjs, numMatched,
    //     numNewTargets, numLostTargets);
  }

  this->newDetectFlag_ = false;
}

/*!
 * @brief 计算3D马氏距离
 * @param posDiff 位置差异向量 [dx, dy, dz]
 * @param covariance 协方差矩阵 3x3
 * @return 马氏距离的平方
 */
double dynamicDetector::computeMahalanobisDistance3D(
    const Eigen::Vector3d &posDiff, const Eigen::Matrix3d &covariance) {
  // 计算马氏距离，使用协方差矩阵的逆
  double det = covariance.determinant();
  if (std::abs(det) < 1e-10) {
    // 协方差矩阵奇异，退化为欧式距离
    return posDiff.squaredNorm();
  }

  // 添加正则化项，防止过拟合导致的协方差过小
  Eigen::Matrix3d covRegularized =
      covariance + 1e-4 * Eigen::Matrix3d::Identity();

  Eigen::Matrix3d covInv = covRegularized.inverse();
  double mahalDist = posDiff.transpose() * covInv * posDiff;
  if (!std::isfinite(mahalDist)) {
    // 计算异常，退化为欧式距离
    return posDiff.squaredNorm();
  }
  return mahalDist;
}

/*!
 * @brief 计算两个3D边界框的IoU (Intersection over Union)
 * @param box1 第一个边界框
 * @param box2 第二个边界框
 * @return IoU值，范围 [0, 1]，值越大表示重叠度越高
 *
 * IoU计算公式：IoU = Volume(Intersection) / Volume(Union)
 * 其中：
 * - Intersection: 两个边界框的交集体积
 * - Union: 两个边界框的并集体积 = Vol1 + Vol2 - Intersection
 */
double dynamicDetector::compute3DIoU(const onboardDetector::box3D &box1,
                                     const onboardDetector::box3D &box2) {
  // 计算每个边界框在x、y、z轴上的最小值和最大值
  double box1_x_min = box1.x - box1.x_width / 2.0;
  double box1_x_max = box1.x + box1.x_width / 2.0;
  double box1_y_min = box1.y - box1.y_width / 2.0;
  double box1_y_max = box1.y + box1.y_width / 2.0;
  double box1_z_min = box1.z - box1.z_width / 2.0;
  double box1_z_max = box1.z + box1.z_width / 2.0;

  double box2_x_min = box2.x - box2.x_width / 2.0;
  double box2_x_max = box2.x + box2.x_width / 2.0;
  double box2_y_min = box2.y - box2.y_width / 2.0;
  double box2_y_max = box2.y + box2.y_width / 2.0;
  double box2_z_min = box2.z - box2.z_width / 2.0;
  double box2_z_max = box2.z + box2.z_width / 2.0;

  // 计算x、y、z三个维度的重叠长度
  double x_overlap = std::max(0.0, std::min(box1_x_max, box2_x_max) -
                                       std::max(box1_x_min, box2_x_min));
  double y_overlap = std::max(0.0, std::min(box1_y_max, box2_y_max) -
                                       std::max(box1_y_min, box2_y_min));
  double z_overlap = std::max(0.0, std::min(box1_z_max, box2_z_max) -
                                       std::max(box1_z_min, box2_z_min));

  // 计算交集体积
  double intersection = x_overlap * y_overlap * z_overlap;

  // 计算两个边界框的体积
  double vol1 = box1.x_width * box1.y_width * box1.z_width;
  double vol2 = box2.x_width * box2.y_width * box2.z_width;

  // 计算并集体积
  double union_vol = vol1 + vol2 - intersection;

  // 避免除零
  if (union_vol < 1e-10) {
    return 0.0;
  }

  // 计算IoU
  double iou = intersection / union_vol;

  // 确保IoU在[0, 1]范围内
  return std::max(0.0, std::min(1.0, iou));
}

/*!
 * @brief 计算3D物体（无人机和其他类）的数据关联总代价
 * @param predBox 预测的边界框（来自卡尔曼滤波器）
 * @param predStd 预测时刻的点云标准差（未使用）
 * @param measBox 当前测量的边界框
 * @param measStd 当前测量的点云标准差（未使用）
 * @param covariance 预测位置的3D协方差矩阵
 * @return 总关联代价（越小越好）
 *
 * 代价函数组成：
 * 1. 位置代价：3D马氏距离（考虑x, y, z和不确定性）
 * 2. IoU代价：3D边界框重叠度（1-IoU）
 */
double dynamicDetector::computeAssociationCost3D(
    const onboardDetector::box3D &predBox, const Eigen::Vector3d &predStd,
    const onboardDetector::box3D &measBox, const Eigen::Vector3d &measStd,
    const Eigen::Matrix3d &covariance) {
  // 1. 计算3D位置代价（马氏距离）
  Eigen::Vector3d posDiff;
  posDiff << (measBox.x - predBox.x), (measBox.y - predBox.y),
      (measBox.z - predBox.z);
  double posCost = this->computeMahalanobisDistance3D(posDiff, covariance);

  // 2. 计算IoU代价（IoU越大，代价越小）
  double iou = this->compute3DIoU(predBox, measBox);
  double iouCost = 1.0 - iou; // IoU=1时代价为0，IoU=0时代价为1

  // 加权总代价
  double totalCost = this->associationPosCostWeight_ * posCost +
                     this->associationIoUCostWeight_ * iouCost;

  // 调试：输出各项代价的详细信息
  // ROS_DEBUG_THROTTLE(0.5,
  //                   "%s: Cost3D - pos:%.2f iou:%.2f(%.3f) total:%.2f",
  //                   this->hint_.c_str(), posCost, iouCost, iou, totalCost);

  return totalCost;
}

/*!
 * @brief 匈牙利算法求解最优分配问题
 * @param costMatrix 代价矩阵 [numCurr x numHist]
 * @param assignment 输出匹配结果，assignment[i] = j
 * 表示第i个当前检测匹配到第j个历史轨迹，-1表示未匹配
 *
 * 算法步骤：
 * 1. 行归约：每行减去该行最小值
 * 2. 列归约：每列减去该列最小值
 * 3. 贪婪匹配：优先选择代价小的匹配
 *
 * 注意：这是简化版匈牙利算法，适用于大多数情况
 */
void dynamicDetector::hungarianAlgorithm(
    const std::vector<std::vector<double>> &costMatrix,
    std::vector<int> &assignment) {
  if (costMatrix.empty()) {
    assignment.clear();
    return;
  }

  int numRows = costMatrix.size();
  int numCols = costMatrix[0].size();
  assignment.resize(numRows, -1);

  // 创建代价矩阵的副本用于修改
  std::vector<std::vector<double>> cost = costMatrix;

  // 1. 行归约，找到每行的最小值，即当前每个障碍物找到与其最合适的历史轨迹
  for (int i = 0; i < numRows; ++i) {
    double minVal = *std::min_element(cost[i].begin(), cost[i].end());
    if (minVal < 1e8) { // 只处理有效代价，找0元素代价
      for (int j = 0; j < numCols; ++j) {
        cost[i][j] -= minVal;
      }
    }
  }

  // 2. 列归约，找到每列的最小值，即历史轨迹找到与其最合适的当前障碍物
  // 这两步下来确保行列都有最合适的"零"
  for (int j = 0; j < numCols; ++j) {
    double minVal = 1e9;
    for (int i = 0; i < numRows; ++i) {
      minVal = std::min(minVal, cost[i][j]);
    }
    if (minVal < 1e8) {
      for (int i = 0; i < numRows; ++i) {
        cost[i][j] -= minVal;
      }
    }
  }

  // 3. 贪婪匹配（简化版）
  std::vector<bool> colUsed(numCols, false);
  std::vector<std::pair<double, std::pair<int, int>>> candidates;

  // 收集所有零代价的候选匹配
  for (int i = 0; i < numRows; ++i) {
    for (int j = 0; j < numCols; ++j) {
      if (cost[i][j] < 1e-6 && costMatrix[i][j] < 1e8) {
        candidates.push_back({costMatrix[i][j], {i, j}});
      }
    }
  }

  // 按原始代价排序
  std::sort(candidates.begin(), candidates.end());

  // 贪婪分配，遍历排序后的候选列表。如果某一对 (row, col)
  // 对应的行和列都还没有被占用，就锁定这个匹配。
  std::vector<bool> rowUsed(numRows, false);
  for (const auto &candidate : candidates) {
    int row = candidate.second.first;
    int col = candidate.second.second;

    if (!rowUsed[row] && !colUsed[col]) {
      assignment[row] = col;
      rowUsed[row] = true;
      colUsed[col] = true;
    }
  }
}

// --------------------------------物体分类----------------------------------------------

/*!
 * @brief 对单个边界框进行物体分类
 * @param bbox 待分类的边界框（引用传递，会修改其分类标志）
 * @param centroid 点云质心坐标 [x, y, z, 1]（世界坐标系）
 * @param maxHistorySize 历史最大尺寸 [max_x, max_y, max_z]
 */
void dynamicDetector::classifyBox(onboardDetector::box3D &bbox,
                                  const Eigen::Vector4f &centroid,
                                  const Eigen::Vector3d &maxHistorySize,
                                  int trackIndex) {
  // 重置分类标志
  bbox.is_human = false;
  bbox.is_che = false;
  bbox.is_uav = false;
  bbox.is_else = false;

  // 检查box与无人机的xy轴距离，如果小于阈值则继承分类
  if (trackIndex >= 0 && trackIndex < (int)this->boxHist_.size() &&
      !this->boxHist_[trackIndex].empty()) {
    // 计算box质心与无人机的xy距离
    double dx = centroid(0) - this->position_.x();
    double dy = centroid(1) - this->position_.y();
    double xy_distance = std::sqrt(dx * dx + dy * dy);

    // 如果xy距离小于阈值，继承前一帧的分类
    if (xy_distance < this->classifyXYDistanceThreshold_) {
      bbox.is_human = this->boxHist_[trackIndex][0].is_human;
      bbox.is_che = this->boxHist_[trackIndex][0].is_che;
      bbox.is_uav = this->boxHist_[trackIndex][0].is_uav;
      bbox.is_else = this->boxHist_[trackIndex][0].is_else;
      return; // 直接退出函数
    }
  }

  // 使用历史最大尺寸进行判断，抵抗遮挡和距离衰减
  double x_width = maxHistorySize.x();
  double y_width = maxHistorySize.y();
  double z_width = maxHistorySize.z();
  double centroid_z = centroid(2);  // 质心在世界坐标系中的高度

  // 计算x、y轴的最大值
  double xy_max = std::max(x_width, y_width);

  // 1. 分类为人：
  // - 尺寸：高瘦 (z > xy * ratio)
  // - 质心：靠下（质心高度 < 物体高度的一定比例）
  if (z_width >= xy_max * this->classifyHumanZWidthRatio_ &&
      centroid_z < z_width * this->classifyHumanCentroidZRatio_) {
    bbox.is_human = true;
  }
  // 2. 分类为车：
  // - 尺寸：扁平 (xy > z * ratio)
  // - 质心：靠下
  else if (xy_max >= z_width * this->classifyVehicleXYWidthRatio_ &&
           centroid_z < z_width * this->classifyVehicleCentroidZRatio_) {
    bbox.is_che = true;
  }
  // 3. 分类为无人机：
  // - 尺寸：小物体 (all < threshold)
  // - 质心：靠上 (悬浮)
  else if (x_width < this->classifyUAVMaxSize_ &&
           y_width < this->classifyUAVMaxSize_ &&
           z_width < this->classifyUAVMaxSize_ &&
           centroid_z > z_width * this->classifyUAVCentroidZRatio_) {
    bbox.is_uav = true;
  }
  // 4. 其他情况
  else {
    bbox.is_else = true;
  }
}

/*!
 * @brief 切换卡尔曼滤波模型
 * @param index 轨迹索引
 * @param bbox 当前边界框（包含最新的分类信息）
 */
void dynamicDetector::switchKalmanModel(int index,
                                        const onboardDetector::box3D &bbox) {
  // 边界检查
  if (index < 0 || index >= static_cast<int>(this->filters_.size()) ||
      !this->filters_[index]) {
    return;
  }

  // 获取当前滤波器
  auto &filter = this->filters_[index];
  Eigen::VectorXd oldState = filter->getState();
  int oldDim = oldState.size();

  // 获取历史轨迹的分类标志(boxHist_[index][0]是上一帧的分类)
  bool oldIsHuman = false;
  bool oldIsChe = false;
  bool oldIsUav = false;
  bool oldIsElse = false;

  if (this->boxHist_[index].size() > 0) {
    oldIsHuman = this->boxHist_[index][0].is_human;
    oldIsChe = this->boxHist_[index][0].is_che;
    oldIsUav = this->boxHist_[index][0].is_uav;
    oldIsElse = this->boxHist_[index][0].is_else;
  }

  // 判断分类是否发生变化
  bool classificationChanged =
      (bbox.is_human != oldIsHuman) || (bbox.is_che != oldIsChe) ||
      (bbox.is_uav != oldIsUav) || (bbox.is_else != oldIsElse);

  // 如果分类没变,无需切换
  if (!classificationChanged) {
    return;
  }

  // 检查是否是冗余切换 (例如 Else -> Else, 维度 6 -> 6)
  // 这种情况通常发生在历史记录刚初始化，oldIsElse可能不准确，但维度已经是6
  if (oldDim == 6 && bbox.is_else) {
    return;
  }

  // 准备新滤波器参数
  std::shared_ptr<KalmanFilterBase> newFilter = nullptr;
  Eigen::VectorXd newState;
  bool needSwitch = false;

  // 提取旧状态的基础信息 (x, y, z, vx, vy, vz)
  double x = 0, y = 0, z = 0, vx = 0, vy = 0, vz = 0;

  if (oldDim == 6) { // 3D CV [x, y, z, vx, vy, vz]
    x = oldState(0);
    y = oldState(1);
    z = oldState(2);
    vx = oldState(3);
    vy = oldState(4);
    vz = oldState(5);
  } else if (oldDim == 7) {
    // 7维模型：可能是 Human CA 或 Vehicle CTRA
    x = oldState(0);
    y = oldState(1);
    z = oldState(2);

    if (oldIsChe) {
      // 旧模型为 CTRA [x, y, z, v, a, yaw, yaw_rate]
      double v = oldState(3);
      double yaw = oldState(5);
      vx = v * cos(yaw);
      vy = v * sin(yaw);
      vz = 0;
    } else {
      // 旧模型为 Human CA [x, y, z, vx, vy, ax, ay]
      vx = oldState(3);
      vy = oldState(4);
      vz = 0;
    }
  } else if (oldDim == 9) { // 3D CA [x, y, z, vx, vy, vz, ax, ay, az]
    x = oldState(0);
    y = oldState(1);
    z = oldState(2);
    vx = oldState(3);
    vy = oldState(4);
    vz = oldState(5);
  }

  // 根据新的分类结果创建对应的滤波器
  if (bbox.is_human) {
    // 切换到 Human CA (2D CA, 7维)
    newFilter = createKalmanFilter(true, false, false, false, this->kfParams_);
    newState.resize(7);
    // Human State: [x, y, z, vx, vy, ax, ay]
    newState << x, y, z, vx, vy, 0, 0;
    needSwitch = true;
  } else if (bbox.is_che) {
    // 切换到 Vehicle CTRA (7维)
    newFilter = createKalmanFilter(false, true, false, false, this->kfParams_);
    newState.resize(7);
    // CTRA State: [x, y, z, v, a, yaw, yaw_rate]
    double v = sqrt(vx * vx + vy * vy);
    double yaw = atan2(vy, vx);
    newState << x, y, z, v, 0, yaw, 0;
    needSwitch = true;
  } else if (bbox.is_uav) {
    // 切换到 UAV CA (3D CA, 9维)
    newFilter = createKalmanFilter(false, false, true, false, this->kfParams_);
    newState.resize(9);
    // 3D CA State: [x, y, z, vx, vy, vz, ax, ay, az]
    newState << x, y, z, vx, vy, vz, 0, 0, 0;
    needSwitch = true;
  } else if (bbox.is_else) {
    // 切换到 3D CV (6维)
    newFilter = createKalmanFilter(false, false, false, true, this->kfParams_);
    newState.resize(6);
    // 3D CV State: [x, y, z, vx, vy, vz]
    newState << x, y, z, vx, vy, vz;
    needSwitch = true;
  }

  // 执行切换
  if (needSwitch && newFilter) {
    newFilter->setDt(this->dt_);
    // 用旧模型预测后的状态初始化新模型
    // 注意：oldState是在boxAssociation中predict()后的状态
    // 因此这里不需要再predict()，直接initialize即可
    newFilter->initialize(newState);
    this->filters_[index] = newFilter;

    // ROS_INFO_STREAM(this->hint_
    //                 << " Switched model for object " << index << " (Dim "
    //                 << oldDim << " -> " << newState.size() << ") to "
    //                 << (bbox.is_human
    //                         ? "Human"
    //                         : (bbox.is_che ? "Vehicle"
    //                                        : (bbox.is_uav ? "UAV" :
    //                                        "Else"))));
  }
}

// -------------------------------滤波、合并轨迹-----------------------------------------
// 使用卡尔曼滤波器并更新历史记录
void dynamicDetector::kalmanFilterAndUpdateHist(
    const std::vector<int> &bestMatch) {
  // --- 初始化临时容器 ---
  std::vector<std::deque<onboardDetector::box3D>> boxHistTemp;
  std::vector<std::deque<std::vector<Eigen::Vector3d>>> pcHistTemp;
  std::vector<std::deque<Eigen::Vector3d>> pcCenterHistTemp;
  std::vector<std::deque<Eigen::Vector3d>> pcStdHistTemp;
  std::vector<Eigen::Vector3d> maxHistorySizesTemp;
  std::vector<int> smallSizeCounterTemp;
  std::vector<std::shared_ptr<KalmanFilterBase>> filtersTemp;
  std::vector<int> trackMissedFramesTemp;
  std::vector<int> confirmedDynamicFramesTemp; // 连续动态帧数计数器
  std::vector<int> stationaryFrameCountTemp;   // 连续静止帧数计数器（动态转静态回退）
  std::vector<int> stableClassificationCountTemp; // 连续相同分类计数器（用于fix_size）
  std::vector<ros::Time> lastClassifyTimeTemp;    // 上次分类时间戳

  // 确保所有向量大小与 boxHist_ 一致
  size_t histSize = this->boxHist_.size();
  if (this->trackMissedFrames_.size() != histSize) {
    this->trackMissedFrames_.resize(histSize, 0);
  }
  if (this->smallSizeCounter_.size() != histSize) {
    this->smallSizeCounter_.resize(histSize, 0);
  }
  if (this->confirmedDynamicFrames_.size() != histSize) {
    this->confirmedDynamicFrames_.resize(histSize, 0);
  }
  if (this->stationaryFrameCount_.size() != histSize) {
    this->stationaryFrameCount_.resize(histSize, 0);
  }
  if (this->stableClassificationCount_.size() != histSize) {
    this->stableClassificationCount_.resize(histSize, 0);
  }
  if (this->lastClassifyTime_.size() != histSize) {
    this->lastClassifyTime_.resize(histSize, ros::Time(0));
  }

  // 为新出现的目标准备的空历史记录模板
  std::deque<onboardDetector::box3D> newSingleBoxHist;
  std::deque<std::vector<Eigen::Vector3d>> newSinglePcHist;
  std::deque<Eigen::Vector3d> newSinglePcCenterHist;
  std::deque<Eigen::Vector3d> newSinglePcStdHist;

  std::vector<onboardDetector::box3D>
      trackedBBoxesTemp; // 存储当前帧滤波后的所有目标框

  newSingleBoxHist.resize(0);
  newSinglePcHist.resize(0);
  newSinglePcCenterHist.resize(0);
  newSinglePcStdHist.resize(0);
  int numCurrObjs = this->filteredBBoxes_.size(); // 当前帧检测到的目标数量
  int numHistObjs = this->boxHist_.size();
  std::vector<bool> isHistMatched(numHistObjs, false);

  // --- 1. 处理当前检测到的目标 (匹配的旧目标 + 新目标) ---
  for (int i = 0; i < numCurrObjs; i++) {
    onboardDetector::box3D currDetectedBBox = this->filteredBBoxes_[i];
    onboardDetector::box3D newEstimatedBBox; // 用于存储卡尔曼滤波后的状态

    // bestMatch[i] 存储的是当前第 i 个检测框所匹配到的历史轨迹的索引
    if (bestMatch[i] >= 0) {
      // --- 情况1：目标匹配成功 (老目标) ---
      int h_idx = bestMatch[i];
      isHistMatched[h_idx] = true;
      trackMissedFramesTemp.push_back(0); // 重置丢失计数

      // 继承该目标之前的历史记录和滤波器
      boxHistTemp.push_back(this->boxHist_[h_idx]);
      pcHistTemp.push_back(this->pcHist_[h_idx]);
      pcCenterHistTemp.push_back(this->pcCenterHist_[h_idx]);
      pcStdHistTemp.push_back(this->pcStdHist_[h_idx]);
      maxHistorySizesTemp.push_back(this->maxHistorySizes_[h_idx]);
      smallSizeCounterTemp.push_back(this->smallSizeCounter_[h_idx]);
      // 同步继承 confirmedDynamicFrames_ 和 stationaryFrameCount_
      confirmedDynamicFramesTemp.push_back(this->confirmedDynamicFrames_[h_idx]);
      stationaryFrameCountTemp.push_back(this->stationaryFrameCount_[h_idx]);
      // 同步继承 stableClassificationCount_ 和 lastClassifyTime_（用于fix_size功能）
      stableClassificationCountTemp.push_back(this->stableClassificationCount_[h_idx]);
      lastClassifyTimeTemp.push_back(this->lastClassifyTime_[h_idx]);
      filtersTemp.push_back(this->filters_[h_idx]);

      // 构建测量向量：所有模型都测量3D位置 [x, y, z]
      Eigen::VectorXd measurement(3);
      measurement(0) = currDetectedBBox.x;
      measurement(1) = currDetectedBBox.y;
      measurement(2) = currDetectedBBox.z;

      // 执行更新步骤 (预测已在 trackingCB 中完成)
      filtersTemp.back()->update(measurement);
      // 从滤波器中提取更新后的状态
      const Eigen::VectorXd &state = filtersTemp.back()->getState();

      if (currDetectedBBox.is_human) {
        // Human CA: [x, y, z, vx, vy, ax, ay]
        newEstimatedBBox.x = state(0);
        newEstimatedBBox.y = state(1);
        newEstimatedBBox.z = state(2);
        newEstimatedBBox.Vx = state(3);
        newEstimatedBBox.Vy = state(4);
        newEstimatedBBox.Vz = 0.0;
        newEstimatedBBox.Ax = state(5);
        newEstimatedBBox.Ay = state(6);
        newEstimatedBBox.Az = 0.0;
      } else if (currDetectedBBox.is_che) {
        // Vehicle CTRA: [x, y, z, v, a, yaw, yaw_rate]
        newEstimatedBBox.x = state(0);
        newEstimatedBBox.y = state(1);
        newEstimatedBBox.z = state(2);
        double v = state(3);
        double yaw = state(5);
        newEstimatedBBox.Vx = v * cos(yaw);
        newEstimatedBBox.Vy = v * sin(yaw);
        newEstimatedBBox.Vz = 0.0;
        newEstimatedBBox.Ax = state(4) * cos(yaw); // a * cos(yaw)
        newEstimatedBBox.Ay = state(4) * sin(yaw); // a * sin(yaw)
        newEstimatedBBox.Az = 0.0;
      } else if (currDetectedBBox.is_uav) {
        // UAV CA: [x, y, z, vx, vy, vz, ax, ay, az]
        newEstimatedBBox.x = state(0);
        newEstimatedBBox.y = state(1);
        newEstimatedBBox.z = state(2);
        newEstimatedBBox.Vx = state(3);
        newEstimatedBBox.Vy = state(4);
        newEstimatedBBox.Vz = state(5);
        newEstimatedBBox.Ax = state(6);
        newEstimatedBBox.Ay = state(7);
        newEstimatedBBox.Az = state(8);
      } else { // is_else
        // Else CV: [x, y, z, vx, vy, vz]
        newEstimatedBBox.x = state(0);
        newEstimatedBBox.y = state(1);
        newEstimatedBBox.z = state(2);
        newEstimatedBBox.Vx = state(3);
        newEstimatedBBox.Vy = state(4);
        newEstimatedBBox.Vz = state(5);
        newEstimatedBBox.Ax = 0.0;
        newEstimatedBBox.Ay = 0.0;
        newEstimatedBBox.Az = 0.0;
      }

      // 边界框的尺寸处理
      // 获取历史最大尺寸
      const Eigen::Vector3d& maxSize = maxHistorySizesTemp.back();

      // 检查是否已固定尺寸
      if (currDetectedBBox.fix_size) {
        // 尺寸已固定，直接使用历史最大尺寸
        newEstimatedBBox.x_width = maxSize.x();
        newEstimatedBBox.y_width = maxSize.y();
        newEstimatedBBox.z_width = maxSize.z();
        newEstimatedBBox.fix_size = true;
      } else {
        // 尺寸未固定，使用指数平滑公式：smoothed = alpha * curr + (1 - alpha) * prev
        double prev_x_width = this->boxHist_[h_idx][0].x_width;
        double prev_y_width = this->boxHist_[h_idx][0].y_width;
        double prev_z_width = this->boxHist_[h_idx][0].z_width;

        newEstimatedBBox.x_width =
            this->boxSizeSmoothingAlpha_ * currDetectedBBox.x_width +
            (1.0 - this->boxSizeSmoothingAlpha_) * prev_x_width;
        newEstimatedBBox.y_width =
            this->boxSizeSmoothingAlpha_ * currDetectedBBox.y_width +
            (1.0 - this->boxSizeSmoothingAlpha_) * prev_y_width;
        newEstimatedBBox.z_width =
            this->boxSizeSmoothingAlpha_ * currDetectedBBox.z_width +
            (1.0 - this->boxSizeSmoothingAlpha_) * prev_z_width;

        // 约束边界框尺寸不低于历史最大尺寸的指定比例
        newEstimatedBBox.x_width = std::max(newEstimatedBBox.x_width, 
                                             maxSize.x() * this->sizeRetainRatio_);
        newEstimatedBBox.y_width = std::max(newEstimatedBBox.y_width, 
                                             maxSize.y() * this->sizeRetainRatio_);
        newEstimatedBBox.z_width = std::max(newEstimatedBBox.z_width, 
                                             maxSize.z() * this->sizeRetainRatio_);
        newEstimatedBBox.fix_size = false;
      }

      // 使用当前检测框中的最新分类结果
      newEstimatedBBox.is_dynamic = currDetectedBBox.is_dynamic;
      newEstimatedBBox.is_human = currDetectedBBox.is_human;
      newEstimatedBBox.is_che = currDetectedBBox.is_che;
      newEstimatedBBox.is_uav = currDetectedBBox.is_uav;
      newEstimatedBBox.is_else = currDetectedBBox.is_else;
    } else {
      // --- 情况2：目标未匹配 (新目标) ---
      trackMissedFramesTemp.push_back(0);

      boxHistTemp.push_back(newSingleBoxHist);
      pcHistTemp.push_back(newSinglePcHist);
      pcCenterHistTemp.push_back(newSinglePcCenterHist);
      pcStdHistTemp.push_back(newSinglePcStdHist);
      maxHistorySizesTemp.push_back(
          Eigen::Vector3d(currDetectedBBox.x_width, currDetectedBBox.y_width,
                          currDetectedBBox.z_width)); // 初始化最大尺寸
      smallSizeCounterTemp.push_back(0);
      // 同步初始化 confirmedDynamicFrames_ 和 stationaryFrameCount_
      confirmedDynamicFramesTemp.push_back(0);
      stationaryFrameCountTemp.push_back(0);
      // 同步初始化 stableClassificationCount_ 和 lastClassifyTime_（用于fix_size功能）
      stableClassificationCountTemp.push_back(0);
      lastClassifyTimeTemp.push_back(ros::Time(0));

      // 强制所有新轨迹使用 3D CV 模型
      auto newFilter = createKalmanFilter(false, // is_human
                                          false, // is_che
                                          false, // is_uav
                                          true,  // is_else -> 强制使用 3D CV
                                          this->kfParams_);

      newFilter->setDt(this->dt_);

      // 统一使用 3D CV 初始化：[x, y, z, vx, vy, vz]
      Eigen::VectorXd detection(6);
      detection(0) = currDetectedBBox.x;
      detection(1) = currDetectedBBox.y;
      detection(2) = currDetectedBBox.z;
      detection(3) = 0.0; // vx
      detection(4) = 0.0; // vy
      detection(5) = 0.0; // vz

      // 初始化滤波器
      newFilter->initialize(detection);
      filtersTemp.push_back(newFilter);

      // 对于新目标，其初始估计状态就是它的第一次测量值
      newEstimatedBBox = currDetectedBBox;
      newEstimatedBBox.Vx = 0.0;
      newEstimatedBBox.Vy = 0.0;
      newEstimatedBBox.Vz = 0.0;
      newEstimatedBBox.Ax = 0.0;
      newEstimatedBBox.Ay = 0.0;
      newEstimatedBBox.Az = 0.0;
      // 初始化分类信息：新目标默认为 is_else（与滤波器模型一致）
      newEstimatedBBox.is_human = false;
      newEstimatedBBox.is_che = false;
      newEstimatedBBox.is_uav = false;
      newEstimatedBBox.is_else = true;
    }

    // --- 更新历史记录队列 ---
    if (int(boxHistTemp.back().size()) == this->histSize_) {
      boxHistTemp.back().pop_back();
      pcHistTemp.back().pop_back();
      pcCenterHistTemp.back().pop_back();
      pcStdHistTemp.back().pop_back();
    }

    // 将当前帧的最新估计状态和信息从队列头部推入
    boxHistTemp.back().push_front(newEstimatedBBox);
    pcHistTemp.back().push_front(this->filteredPcClusters_[i]);
    // 使用经过KF/EKF平滑的box位置替代原始点云质心，提高轨迹平滑性
    Eigen::Vector3d smoothedCenter(newEstimatedBBox.x, newEstimatedBBox.y, newEstimatedBBox.z);
    pcCenterHistTemp.back().push_front(smoothedCenter);
    pcStdHistTemp.back().push_front(this->filteredPcClusterStds_[i]);

    // 将当前帧的最终跟踪结果存入 trackedBBoxesTemp
    trackedBBoxesTemp.push_back(newEstimatedBBox);
  }

  // --- 2. 处理未匹配的历史目标 (Coasting) ---
  for (int j = 0; j < numHistObjs; ++j) {
    if (!isHistMatched[j]) {
      int missed = this->trackMissedFrames_[j] + 1;
      if (missed < this->maxMissedFrames_) {
        // 保留该轨迹 (Coasting)
        trackMissedFramesTemp.push_back(missed);

        boxHistTemp.push_back(this->boxHist_[j]);
        pcHistTemp.push_back(this->pcHist_[j]);
        pcCenterHistTemp.push_back(this->pcCenterHist_[j]);
        pcStdHistTemp.push_back(this->pcStdHist_[j]);
        maxHistorySizesTemp.push_back(this->maxHistorySizes_[j]);
        smallSizeCounterTemp.push_back(this->smallSizeCounter_[j]);
        // 同步继承 confirmedDynamicFrames_ 和 stationaryFrameCount_
        confirmedDynamicFramesTemp.push_back(this->confirmedDynamicFrames_[j]);
        stationaryFrameCountTemp.push_back(this->stationaryFrameCount_[j]);
        // 同步继承 stableClassificationCount_ 和 lastClassifyTime_（用于fix_size功能）
        stableClassificationCountTemp.push_back(this->stableClassificationCount_[j]);
        lastClassifyTimeTemp.push_back(this->lastClassifyTime_[j]);
        filtersTemp.push_back(this->filters_[j]);

        // 获取预测状态 (已在 trackingCB 中 predict)
        const Eigen::VectorXd &state = filtersTemp.back()->getState();

        // 构建预测的 BBox
        onboardDetector::box3D predBBox;
        // 使用上一帧的属性作为基础
        if (boxHistTemp.back().size() > 0) {
          predBBox = boxHistTemp.back().front();
        }

        // 根据模型类型提取预测状态
        // 注意：这里我们使用历史轨迹的分类信息
        if (!predBBox.is_dynamic) {
          // 静态物体 Coasting：强制静止
          // 位置保持上一帧的值 (predBBox.x/y/z 已经从
          // boxHistTemp.back().front() 复制)
          predBBox.Vx = 0.0;
          predBBox.Vy = 0.0;
          predBBox.Vz = 0.0;
          predBBox.Ax = 0.0;
          predBBox.Ay = 0.0;
          predBBox.Az = 0.0;

          // 重置 KF 状态以防止内部漂移
          Eigen::VectorXd staticState = state; // 复制一份，保留维度
          int dim = state.size();

          // 位置重置为上一帧位置
          staticState(0) = predBBox.x;
          staticState(1) = predBBox.y;
          staticState(2) = predBBox.z;

          // 速度和加速度重置为 0
          if (dim == 6) { // 3D CV [x, y, z, vx, vy, vz]
            staticState(3) = 0;
            staticState(4) = 0;
            staticState(5) = 0;
          } else if (dim == 7) { // Human CA or Vehicle CTRA
            if (predBBox.is_che) {
              // CTRA: [x, y, z, v, a, yaw, yaw_rate]
              staticState(3) = 0; // v
              staticState(4) = 0; // a
              staticState(6) = 0; // yaw_rate
              // yaw (index 5) 保持不变
            } else {
              // Human: [x, y, z, vx, vy, ax, ay]
              staticState(3) = 0;
              staticState(4) = 0; // vx, vy
              staticState(5) = 0;
              staticState(6) = 0; // ax, ay
            }
          } else if (dim == 9) { // 3D CA
            for (int k = 3; k < 9; ++k)
              staticState(k) = 0;
          }

          filtersTemp.back()->initialize(staticState);
        } else if (predBBox.is_human) {
          predBBox.x = state(0);
          predBBox.y = state(1);
          predBBox.z = state(2);
          predBBox.Vx = state(3);
          predBBox.Vy = state(4);
          predBBox.Vz = 0.0;
          predBBox.Ax = state(5);
          predBBox.Ay = state(6);
          predBBox.Az = 0.0;
        } else if (predBBox.is_che) {
          predBBox.x = state(0);
          predBBox.y = state(1);
          predBBox.z = state(2);
          double v = state(3);
          double yaw = state(5);
          predBBox.Vx = v * cos(yaw);
          predBBox.Vy = v * sin(yaw);
          predBBox.Vz = 0.0;
          predBBox.Ax = state(4) * cos(yaw);
          predBBox.Ay = state(4) * sin(yaw);
          predBBox.Az = 0.0;
        } else if (predBBox.is_uav) {
          predBBox.x = state(0);
          predBBox.y = state(1);
          predBBox.z = state(2);
          predBBox.Vx = state(3);
          predBBox.Vy = state(4);
          predBBox.Vz = state(5);
          predBBox.Ax = state(6);
          predBBox.Ay = state(7);
          predBBox.Az = state(8);
        } else {
          predBBox.x = state(0);
          predBBox.y = state(1);
          predBBox.z = state(2);
          predBBox.Vx = state(3);
          predBBox.Vy = state(4);
          predBBox.Vz = state(5);
          predBBox.Ax = 0.0;
          predBBox.Ay = 0.0;
          predBBox.Az = 0.0;
        }

        // 尺寸保持不变 (predBBox 已经复制了上一帧的尺寸)

        // 更新历史队列
        if (int(boxHistTemp.back().size()) == this->histSize_) {
          boxHistTemp.back().pop_back();
          pcHistTemp.back().pop_back();
          pcCenterHistTemp.back().pop_back();
          pcStdHistTemp.back().pop_back();
        }

        boxHistTemp.back().push_front(predBBox);
        // 点云数据推入空值，但质心使用预测的box位置（保持与正常帧一致）
        pcHistTemp.back().push_front(std::vector<Eigen::Vector3d>());
        Eigen::Vector3d predCenter(predBBox.x, predBBox.y, predBBox.z);
        pcCenterHistTemp.back().push_front(predCenter);
        pcStdHistTemp.back().push_front(Eigen::Vector3d::Zero());

        trackedBBoxesTemp.push_back(predBBox);
      }
      // else: 丢弃 (missed >= maxMissedFrames_)
    }
  }

  // --- 更新类的成员变量 ---
  this->boxHist_ = boxHistTemp;
  this->pcHist_ = pcHistTemp;
  this->pcCenterHist_ = pcCenterHistTemp;
  this->pcStdHist_ = pcStdHistTemp;
  this->maxHistorySizes_ = maxHistorySizesTemp;
  this->smallSizeCounter_ = smallSizeCounterTemp;
  this->filters_ = filtersTemp;
  this->trackedBBoxes_ = trackedBBoxesTemp;
  this->trackMissedFrames_ = trackMissedFramesTemp;
  this->confirmedDynamicFrames_ = confirmedDynamicFramesTemp;
  this->stationaryFrameCount_ = stationaryFrameCountTemp;
  // 同步更新 stableClassificationCount_ 和 lastClassifyTime_（用于fix_size功能）
  this->stableClassificationCount_ = stableClassificationCountTemp;
  this->lastClassifyTime_ = lastClassifyTimeTemp;
}

/*!
 * @brief 移除重复/重叠的轨迹（用于解决幽灵轨迹问题）
 *
 * 检测逻辑：
 * - 使用多维度判断：IoU重叠、中心距离、速度方向相似度
 * - 可以处理有交集和无交集的重复轨迹
 * - 保留更可靠的轨迹（丢失帧数少、历史更长）
 * - 删除不可靠的轨迹（丢失帧数多、新生成的轨迹）
 */
void dynamicDetector::removeDuplicateTracks() {
  if (this->boxHist_.size() <= 1) {
    return; // 只有一条或零条轨迹，无需去重
  }

  // 确保 trackMissedFrames_ 大小与 boxHist_ 一致
  if (this->trackMissedFrames_.size() != this->boxHist_.size()) {
    this->trackMissedFrames_.resize(this->boxHist_.size(), 0);
  }

  std::vector<bool> toRemove(this->boxHist_.size(), false);

  // 检查所有轨迹对
  for (size_t i = 0; i < this->boxHist_.size(); ++i) {
    if (toRemove[i] || this->boxHist_[i].empty())
      continue;

    for (size_t j = i + 1; j < this->boxHist_.size(); ++j) {
      if (toRemove[j] || this->boxHist_[j].empty())
        continue;

      // 使用智能判断函数检测是否为重复轨迹
      if (this->areDuplicateTracks(i, j)) {
        // 边界检查
        int missed_i = (i < this->trackMissedFrames_.size()) ? this->trackMissedFrames_[i] : 0;
        int missed_j = (j < this->trackMissedFrames_.size()) ? this->trackMissedFrames_[j] : 0;
        size_t histLen_i = this->boxHist_[i].size();
        size_t histLen_j = this->boxHist_[j].size();

        // 优先保留：
        // 1. 丢失帧数少的（更可靠）
        // 2. 如果丢失帧数相同，保留历史更长的（更稳定）
        bool removeI = false;
        if (missed_i > missed_j) {
          removeI = true;
        } else if (missed_i == missed_j) {
          // 丢失帧数相同，保留历史更长的
          if (histLen_i < histLen_j) {
            removeI = true;
          } else if (histLen_i == histLen_j) {
            // 历史长度也相同，保留第一个（idx小的）
            removeI = false;
          }
        }

        if (removeI) {
          toRemove[i] = true;
          // ROS_WARN_THROTTLE(1.0,
          //                   "%s: Removing duplicate track %zu (missed=%d, "
          //                   "histLen=%zu, duplicate with track %zu)",
          //                   this->hint_.c_str(), i, missed_i, histLen_i, j);
          break; // i 已被标记删除，无需继续比较
        } else {
          toRemove[j] = true;
          // ROS_WARN_THROTTLE(1.0,
          //                   "%s: Removing duplicate track %zu (missed=%d, "
          //                   "histLen=%zu, duplicate with track %zu)",
          //                   this->hint_.c_str(), j, missed_j, histLen_j, i);
        }
      }
    }
  }

  // 执行删除（倒序删除避免索引偏移）
  for (int i = this->boxHist_.size() - 1; i >= 0; --i) {
    if (toRemove[i]) {
      this->boxHist_.erase(this->boxHist_.begin() + i);
      this->pcHist_.erase(this->pcHist_.begin() + i);
      this->pcCenterHist_.erase(this->pcCenterHist_.begin() + i);
      this->pcStdHist_.erase(this->pcStdHist_.begin() + i);
      this->maxHistorySizes_.erase(this->maxHistorySizes_.begin() + i);
      this->smallSizeCounter_.erase(this->smallSizeCounter_.begin() + i);
      // 同步删除动态转静态回退计数器
      if (i < static_cast<int>(this->stationaryFrameCount_.size())) {
        this->stationaryFrameCount_.erase(this->stationaryFrameCount_.begin() + i);
      }
      // 同步删除确认动态帧数计数器
      if (i < static_cast<int>(this->confirmedDynamicFrames_.size())) {
        this->confirmedDynamicFrames_.erase(this->confirmedDynamicFrames_.begin() + i);
      }
      // 同步删除点数历史记录（鲁棒性增强）
      if (i < static_cast<int>(this->pointCountHist_.size())) {
        this->pointCountHist_.erase(this->pointCountHist_.begin() + i);
      }
      // 同步删除滞后状态（鲁棒性增强）
      if (i < static_cast<int>(this->previousDynamicState_.size())) {
        this->previousDynamicState_.erase(this->previousDynamicState_.begin() + i);
      }
      // 同步删除 stableClassificationCount_ 和 lastClassifyTime_（用于fix_size功能）
      if (i < static_cast<int>(this->stableClassificationCount_.size())) {
        this->stableClassificationCount_.erase(this->stableClassificationCount_.begin() + i);
      }
      if (i < static_cast<int>(this->lastClassifyTime_.size())) {
        this->lastClassifyTime_.erase(this->lastClassifyTime_.begin() + i);
      }

      this->filters_.erase(this->filters_.begin() + i);
      this->trackedBBoxes_.erase(this->trackedBBoxes_.begin() + i);
      this->trackMissedFrames_.erase(this->trackMissedFrames_.begin() + i);
    }
  }
}


/*!
 * @brief 判断两条轨迹是否为重复轨迹（同一物体）
 * @param idx1 轨迹1的索引
 * @param idx2 轨迹2的索引
 * @return true 如果是重复轨迹，false 否则
 * 
 * 判断标准：
 * 1. IoU重叠度（处理有交集的情况）
 * 2. 中心距离（处理无交集但距离近的情况）
 * 3. 速度方向相似度（运动一致性）
 */
bool dynamicDetector::areDuplicateTracks(int idx1, int idx2) {
  // 边界检查
  if (idx1 < 0 || idx1 >= static_cast<int>(this->boxHist_.size()) ||
      idx2 < 0 || idx2 >= static_cast<int>(this->boxHist_.size())) {
    return false;
  }
  if (this->boxHist_[idx1].empty() || this->boxHist_[idx2].empty()) {
    return false;
  }

  const auto &bbox1 = this->boxHist_[idx1][0];
  const auto &bbox2 = this->boxHist_[idx2][0];

  // 1. 计算 IoU
  double iou = this->compute3DIoU(bbox1, bbox2);
  if (iou > this->duplicateTrackIoUThreshold_) {
    return true;  // 有明显重叠
  }

  // 2. 计算中心距离
  double dx = bbox1.x - bbox2.x;
  double dy = bbox1.y - bbox2.y;
  double dz = bbox1.z - bbox2.z;
  double centerDist = std::sqrt(dx * dx + dy * dy + dz * dz);

  // 计算物体的平均尺寸作为距离判断的参考
  double avgSize1 = (bbox1.x_width + bbox1.y_width + bbox1.z_width) / 3.0;
  double avgSize2 = (bbox2.x_width + bbox2.y_width + bbox2.z_width) / 3.0;
  double avgSize = (avgSize1 + avgSize2) / 2.0;

  // 如果中心距离小于阈值（考虑物体尺寸），可能是同一物体
  if (centerDist < this->duplicateTrackDistanceThreshold_ * avgSize) {
    // 3. 检查速度方向相似度（仅当两者都在运动时）
    double v1 = std::sqrt(bbox1.Vx * bbox1.Vx + bbox1.Vy * bbox1.Vy + bbox1.Vz * bbox1.Vz);
    double v2 = std::sqrt(bbox2.Vx * bbox2.Vx + bbox2.Vy * bbox2.Vy + bbox2.Vz * bbox2.Vz);

    // 至少有一个静止，则仅基于距离判断
    if (v1 < 0.1 || v2 < 0.1) {
      return (centerDist < avgSize * 1.5);  // 静止物体距离阈值更严格
    }

    // 两者都在运动，计算速度方向的余弦相似度
    double vdot = bbox1.Vx * bbox2.Vx + bbox1.Vy * bbox2.Vy + bbox1.Vz * bbox2.Vz;
    double cosSimilarity = vdot / (v1 * v2);

    // 速度方向相似 + 距离近 = 同一物体
    if (cosSimilarity > this->duplicateTrackVelocitySimilarityThreshold_) {
      return true;
    }
  }

  return false;
}



// ===================================================================
// 动静态分类
// ===================================================================
// 动静态分类定时器回调函数
void dynamicDetector::classificationCB(const ros::TimerEvent &) {
  // // [Performance Timing] 测量回调函数耗时
  auto start_time = std::chrono::high_resolution_clock::now();

  // 静态地图预热阶段，跳过分类
  if (!this->isStaticMapReady_) {
    return;
  }

  // 检查是否有新跟踪结果
  if (!hasNewTracking_) {
    return; // 跳过，避免重复处理相同数据
  }

  std::lock_guard<std::mutex> lock(bboxMutex_); // 加锁保护边界框数据

  // 创建一个临时向量来存储当前帧检测到的动态边界框
  std::vector<onboardDetector::box3D> dynamicBBoxesTemp;
  
  // 确保点数历史记录向量大小与轨迹数量一致
  while (this->pointCountHist_.size() < this->pcHist_.size()) {
    this->pointCountHist_.push_back(std::deque<int>());
  }
  // 确保滞后状态向量大小与轨迹数量一致
  while (this->previousDynamicState_.size() < this->pcHist_.size()) {
    this->previousDynamicState_.push_back(false);
  }

  // 遍历所有被跟踪目标的点云/边界框历史。
  // 默认只判断xy平面的动态性，但对于无人机（is_uav）和其他3D类（is_else），保留z轴速度用于3D动态判别
  for (size_t i = 0; i < this->pcHist_.size(); ++i) {
    // ===================================================================================
    // 情况一：历史记录长度不足以进行分类
    // 确定用于比较的当前帧与历史帧之间的时间间隔（帧数）
    int curFrameGap;
    if (int(this->pcHist_[i].size()) < this->skipFrame_ + 1) {
      // 如果历史记录不够长，就用现有的最远一帧进行比较
      curFrameGap = this->pcHist_[i].size() - 1;
    } else {
      // 否则，使用参数设定的帧间隔
      curFrameGap = this->skipFrame_;
    }
    // ===================================================================================

    // ==================================================================================
    // 情况二：强制动态（如果一个障碍物在过去一段时间内被频繁分类为动态，则强制认定其为动态）
    int dynaFrames = 0;
    if (int(this->boxHist_[i].size()) > this->forceDynaCheckRange_) {
      for (int j = 1; j < this->forceDynaCheckRange_ + 1; ++j) {
        if (this->boxHist_[i][j].is_dynamic) {
          ++dynaFrames;
        }
      }
    }

    if (dynaFrames >= this->forceDynaFrames_) {
      this->boxHist_[i][0].is_dynamic = true;
      dynamicBBoxesTemp.push_back(this->boxHist_[i][0]);
      continue;
    }
    // ===================================================================================

    // 获取当前帧和历史计算帧的点云
    std::vector<Eigen::Vector3d> currPc = this->pcHist_[i][0];
    std::vector<Eigen::Vector3d> prevPc = this->pcHist_[i][curFrameGap];
    
    // ===================================================================================
    // 【鲁棒性增强1】点云稀疏自适应 - 根据点数和距离动态调整速度阈值
    // ===================================================================================
    int currPointCount = static_cast<int>(currPc.size());
    // 更新点数历史
    this->pointCountHist_[i].push_front(currPointCount);
    if (this->pointCountHist_[i].size() > 10) {
      this->pointCountHist_[i].pop_back();
    }
    
    // 计算物体到传感器的距离（在全局坐标系下）
    double dx = this->boxHist_[i][0].x - this->positionLidar_.x();
    double dy = this->boxHist_[i][0].y - this->positionLidar_.y();
    double objDist = std::sqrt(dx * dx + dy * dy);
    
    // 自适应速度阈值：点数少或距离远时提高阈值，减少误判
    double adaptiveVelThresh = this->dynaVelThresh_;
    // 点数因子：点数少于阈值时提高阈值（最多2倍）
    if (currPointCount < this->minReliablePoints_ && currPointCount > 0) {
      double pointFactor = 1.0 + (1.0 - static_cast<double>(currPointCount) / this->minReliablePoints_);
      adaptiveVelThresh *= std::min(pointFactor, 2.0);
    }
    // 距离因子：距离远时提高阈值（每5米增加20%，最多1.5倍）
    double distFactor = 1.0 + std::min(objDist / 25.0, 0.5);
    adaptiveVelThresh *= distFactor;
    
    // ===================================================================================
    // 【鲁棒性增强2】遮挡检测 - 检测点数突变，标记为可能遮挡
    // ===================================================================================
    bool possibleOcclusion = false;
    if (this->pointCountHist_[i].size() >= 3) {
      // 计算历史平均点数（排除当前帧）
      double avgPointCount = 0.0;
      for (size_t k = 1; k < this->pointCountHist_[i].size(); ++k) {
        avgPointCount += this->pointCountHist_[i][k];
      }
      avgPointCount /= (this->pointCountHist_[i].size() - 1);
      
      // 如果当前点数下降超过阈值，标记为可能遮挡
      if (avgPointCount > 0 && currPointCount < avgPointCount * this->pointCountDropThreshold_) {
        possibleOcclusion = true;
      }
    }

    // 初始化速度向量
    Eigen::Vector3d Vcur(0., 0., 0.); // 单个点的速度
    Eigen::Vector3d Vbox(0., 0., 0.); // 整个边界框的平均速度
    Eigen::Vector3d Vkf(0., 0., 0.);  // 卡尔曼滤波器估计的速度

    int numPoints = currPc.size(); // 点云中的总点数，用于计算投票率
    int votes = 0;                 // “动态”票数

    // 计算边界框中心点的速度
    Vbox(0) = (this->boxHist_[i][0].x - this->boxHist_[i][curFrameGap].x) /
              (this->dt_ * curFrameGap);
    Vbox(1) = (this->boxHist_[i][0].y - this->boxHist_[i][curFrameGap].y) /
              (this->dt_ * curFrameGap);
    Vbox(2) = (this->boxHist_[i][0].z - this->boxHist_[i][curFrameGap].z) /
              (this->dt_ * curFrameGap);

    // 获取卡尔曼滤波器估计的速度，根据不同模型维度进行计算
    // 边界检查
    if (i >= this->filters_.size() || !this->filters_[i]) {
      continue;
    }
    Eigen::VectorXd state = this->filters_[i]->getState();
    int dim = state.size();
    if (dim == 6) {
      // 3D CV: [x, y, z, vx, vy, vz]
      Vkf(0) = state(3);
      Vkf(1) = state(4);
      // include z velocity (vz) when available
      Vkf(2) = state(5);
    } else if (dim == 7) {
      // 7维可能是 Human CA 或 Vehicle CTRA，依据历史分类决定
      bool isVehicle = this->boxHist_[i][0].is_che; // Vehicle CTRA
      if (isVehicle) {
        // CTRA: [x, y, z, v, a, yaw, yaw_rate]
        double v = state(3);
        double yaw = state(5);
        Vkf(0) = v * cos(yaw);
        Vkf(1) = v * sin(yaw);
      } else {
        // Human CA: [x, y, z, vx, vy, ax, ay]
        Vkf(0) = state(3);
        Vkf(1) = state(4);
      }
    } else if (dim == 9) {
      // 3D CA: [x, y, z, vx, vy, vz, ax, ay, az]
      Vkf(0) = state(3);
      Vkf(1) = state(4);
      // include z velocity (vz)
      Vkf(2) = state(5);
    } else {
      // fallback to historical speed
      Vkf(0) = this->boxHist_[i][0].Vx;
      Vkf(1) = this->boxHist_[i][0].Vy;
      Vkf(2) = this->boxHist_[i][0].Vz; // use historical vz if available
    }

    // 检查尺寸稳定性（解决遮挡导致的误判问题）
    // 由于已经把静态簇过滤了，不需要这个尺寸稳定性检测了
    bool isSizeStable = true;

    // 遍历当前点云中的每一个点，通过与历史点云比较来“投票”
    for (size_t j = 0; j < currPc.size(); ++j) {
      double minDist = this->classificationMinNeighborDist_; // 初始化一个较大的最小距离，从参数文件读取
      Eigen::Vector3d nearestVect;
      // 在历史点云中为当前点寻找最近邻点
      for (size_t k = 0; k < prevPc.size(); k++) {
        double dist = (currPc[j] - prevPc[k]).norm();
        if (abs(dist) < minDist) {
          minDist = dist;
          nearestVect = currPc[j] - prevPc[k]; // 记录位移向量
        }
      }
      // 计算该点的速度
      Vcur = nearestVect / (this->dt_ * curFrameGap);
      // 默认情况下（人物/车辆），忽略Z轴速度，以提高平面判别鲁棒性
      // 但如果被标注为无人机或else类别，则保留Z轴速度（3D运动）用于分类
      if (!(this->boxHist_[i][0].is_uav || this->boxHist_[i][0].is_else)) {
        Vcur(2) = 0;
      }
      // 计算点的速度向量与边界框整体速度向量的余弦相似度
      double velSim = Vcur.dot(Vbox) / (Vcur.norm() * Vbox.norm());

      // 如果速度方向相反，且尺寸稳定，则认为该点是噪声或匹配错误，不计入总点数
      // 如果尺寸不稳定（可能因遮挡导致质心偏移），则不进行此过滤，保留所有点作为分母
      if (isSizeStable && velSim < 0) {
        --numPoints;
      } else {
        // 如果点的速度超过动态阈值，则投一票“动态”
        if (Vcur.norm() > this->dynaVelThresh_) {
          ++votes;
        }
      }
    }

    // --- 根据投票结果和速度阈值判断是否为动态 ---
    // 计算动态票的比例
    double voteRatio = (numPoints > 0) ? double(votes) / double(numPoints) : 0;
    // 获取卡尔曼滤波器估计的速度大小
    double velNorm = Vkf.norm();
    
    // ===================================================================================
    // 【鲁棒性增强3】抖动过滤与滞后机制
    // ===================================================================================
    // 如果检测到可能遮挡，提高投票阈值要求
    double adaptiveVoteThresh = this->dynaVoteThresh_;
    if (possibleOcclusion) {
      adaptiveVoteThresh = std::min(0.95, this->dynaVoteThresh_ + 0.15);
    }
    
    // 滞后机制：已经是动态的物体用较低阈值，静态物体用较高阈值
    // 防止LiDAR抖动导致静态物体在动态/静态之间频繁切换
    bool wasDynamic = this->previousDynamicState_[i];
    double effectiveVelThresh = wasDynamic ? 
        (adaptiveVelThresh * this->hysteresisLower_) :  // 动态->静态：用较低阈值（更难变静态）
        adaptiveVelThresh;                               // 静态->动态：用正常阈值
    
    // 动态判定条件：点云投票率足够高 && 卡尔曼滤波器估计的线速度足够快
    bool is_dynamic_candidate =
        (voteRatio >= adaptiveVoteThresh && velNorm >= effectiveVelThresh);
    
    // 更新滞后状态
    this->previousDynamicState_[i] = is_dynamic_candidate || this->boxHist_[i][0].is_dynamic;

    if (is_dynamic_candidate) {
      // 如果满足条件，首先标记为“动态候选”
      this->boxHist_[i][0].is_dynamic_candidate = true;

      // --- 动态一致性检查 ---
      // 检查过去几帧是否也一直被认为是动态的，以增加鲁棒性
      int dynaConsistCount = 0;
      if (int(this->boxHist_[i].size()) >= this->dynamicConsistThresh_) {
        for (int j = 0; j < this->dynamicConsistThresh_; ++j) {
          // 如果是动态候选、已经是动态，则计数
          if (this->boxHist_[i][j].is_dynamic_candidate or
              this->boxHist_[i][j].is_dynamic) {
            ++dynaConsistCount;
          }
        }
      }
      // 如果连续几帧都满足条件
      if (dynaConsistCount == this->dynamicConsistThresh_) {
        // 则正式标记为动态，并添加到本轮的动态障碍物列表中
        this->boxHist_[i][0].is_dynamic = true;
        dynamicBBoxesTemp.push_back(this->boxHist_[i][0]);
      }
    }
  }

  // // ==================================================================================
  // // 【动态转静态回退机制】结合位置变化和运动方向一致性判断
  // // 核心思想：真实运动方向连续，点云抖动方向随机
  // // 确保 stationaryFrameCount_ 向量大小与轨迹数量一致
  // while (this->stationaryFrameCount_.size() < this->boxHist_.size()) {
  //   this->stationaryFrameCount_.push_back(0);
  // }
  
  // // 遍历所有轨迹，检查是否需要回退为静态
  // for (size_t i = 0; i < this->boxHist_.size(); ++i) {
  //   if (this->boxHist_[i].empty()) continue;
    
  //   // 只对当前被标记为动态的物体进行检查
  //   if (this->boxHist_[i][0].is_dynamic) {
  //     bool isStationary = false;
  //     double posChange = 0.0;
  //     double dirConsistency = 1.0;  // 方向一致性，默认为1（一致）
      
  //     // 使用与动静态分类相同的帧间隔（skipFrame_）来计算方向一致性
  //     // 这样位移向量更长，方向更稳定，能更好地区分真实运动和抖动
  //     int k = 1;
  //     size_t requiredFrames = static_cast<size_t>(2 * k + 1);
      
  //     if (this->boxHist_[i].size() >= requiredFrames) {
  //       // 计算两段间隔为k帧的位移向量
  //       // motion1: 帧0 -> 帧k
  //       // motion2: 帧k -> 帧2k
  //       Eigen::Vector3d motion1(
  //           this->boxHist_[i][0].x - this->boxHist_[i][k].x,
  //           this->boxHist_[i][0].y - this->boxHist_[i][k].y,
  //           this->boxHist_[i][0].z - this->boxHist_[i][k].z
  //       );
  //       Eigen::Vector3d motion2(
  //           this->boxHist_[i][k].x - this->boxHist_[i][2 * k].x,
  //           this->boxHist_[i][k].y - this->boxHist_[i][2 * k].y,
  //           this->boxHist_[i][k].z - this->boxHist_[i][2 * k].z
  //       );
        
  //       double norm1 = motion1.norm();
  //       double norm2 = motion2.norm();
  //       posChange = norm1;  // 最近k帧的累积位移
        
  //       // 计算方向一致性（余弦相似度）
  //       // dirConsistency 接近 1.0 = 方向一致（真实运动）
  //       // dirConsistency 接近 0 或负值 = 方向随机（点云抖动）
  //       if (norm1 > 1e-6 && norm2 > 1e-6) {
  //         dirConsistency = motion1.dot(motion2) / (norm1 * norm2);
  //       }
        
  //       // 将位置变化转换为速度（除以时间间隔）进行阈值比较
  //       double impliedVel = posChange / (k * this->dt_);
        
  //       // 判断是否为静止或抖动：
  //       // 1. 速度低于阈值 -> 静止
  //       // 2. 方向一致性低（<0.5，即夹角>60度）且速度不高 -> 抖动，视为静止
  //       bool lowVelocity = (impliedVel < this->staticFallbackVelThresh_);
  //       bool isJitter = (dirConsistency < this->motionDirConsistencyThresh_) && 
  //                       (impliedVel < this->staticFallbackVelThresh_ * 3.0);  // 抖动判断用更宽松的速度阈值
        
  //       isStationary = lowVelocity || isJitter;
        
  //     } else if (this->boxHist_[i].size() >= 2) {
  //       // 历史数据不足时，退化为纯速度判断
  //       double dx = this->boxHist_[i][0].x - this->boxHist_[i][1].x;
  //       double dy = this->boxHist_[i][0].y - this->boxHist_[i][1].y;
  //       double dz = this->boxHist_[i][0].z - this->boxHist_[i][1].z;
  //       posChange = std::sqrt(dx * dx + dy * dy + dz * dz);
        
  //       double impliedVel = posChange / this->dt_;
  //       isStationary = (impliedVel < this->staticFallbackVelThresh_);
  //     }
      
  //     if (isStationary) {
  //       this->stationaryFrameCount_[i]++;
        
  //       // 如果连续静止帧数达到阈值，回退为静态
  //       if (this->stationaryFrameCount_[i] >= this->staticFallbackFrames_) {
  //         this->boxHist_[i][0].is_dynamic = false;
  //         this->boxHist_[i][0].is_dynamic_candidate = false;
          
  //         // 从动态列表中移除
  //         auto it = std::find_if(
  //             dynamicBBoxesTemp.begin(), dynamicBBoxesTemp.end(),
  //             [&](const onboardDetector::box3D &box) {
  //               return std::abs(box.x - this->boxHist_[i][0].x) < 0.01 &&
  //                      std::abs(box.y - this->boxHist_[i][0].y) < 0.01 &&
  //                      std::abs(box.z - this->boxHist_[i][0].z) < 0.01;
  //             });
  //         if (it != dynamicBBoxesTemp.end()) {
  //           dynamicBBoxesTemp.erase(it);
  //         }

  //         // 【静态恢复机制】将回退为静态的物体区域立即标记为静态体素
  //         // 这样可以避免该区域在一段时间内被当作"未知"区域处理
  //         if (this->staticClusterFilterEnabled_) {
  //           std::vector<onboardDetector::box3D> revertedBoxes;
  //           revertedBoxes.push_back(this->boxHist_[i][0]);
  //           this->staticFilter_->boostStaticRegions(revertedBoxes);
  //         }

  //         ROS_INFO_THROTTLE(
  //             1.0,
  //             "%s: Object %zu reverted to static (pos_change=%.3f m, "
  //             "dir_consistency=%.2f, stationary for %d frames)",
  //             this->hint_.c_str(), i, posChange, dirConsistency, 
  //             this->stationaryFrameCount_[i]);

  //         // 重置计数器
  //         this->stationaryFrameCount_[i] = 0;
  //       }
  //     } else {
  //       // 如果是真实运动（速度足够且方向一致），重置静止帧计数
  //       this->stationaryFrameCount_[i] = 0;
  //     }
  //   } else {
  //     // 非动态物体，重置计数器
  //     if (i < this->stationaryFrameCount_.size()) {
  //       this->stationaryFrameCount_[i] = 0;
  //     }
  //   }
  // }
  // ==================================================================================

  // 直接更新最终的动态障碍物列表（已移除尺寸过滤）
  this->dynamicBBoxes_ = dynamicBBoxesTemp;

  // 【动态反哺机制】清理已确认动态物体历史轨迹区域的体素
  // 【修复】只有连续多帧确认为动态的物体才触发体素清除，防止短暂误判导致静态标记丢失
  if (this->staticClusterFilterEnabled_) {
    // 确保 confirmedDynamicFrames_ 向量大小与轨迹数量一致
    while (this->confirmedDynamicFrames_.size() < this->boxHist_.size()) {
      this->confirmedDynamicFrames_.push_back(0);
    }
    
    // 收集需要清除体素的动态物体（连续动态帧数达到阈值）
    std::vector<onboardDetector::box3D> boxesToClear;
    for (size_t i = 0; i < this->boxHist_.size(); ++i) {
      if (!this->boxHist_[i].empty() && this->boxHist_[i][0].is_dynamic) {
        // 增加连续动态帧数计数
        this->confirmedDynamicFrames_[i]++;
        // 只有连续动态帧数达到阈值才触发体素清除
        if (this->confirmedDynamicFrames_[i] >= this->voxelClearDynamicFrames_) {
          boxesToClear.push_back(this->boxHist_[i][0]);
        }
      } else {
        // 如果当前帧不是动态，重置计数器
        if (i < this->confirmedDynamicFrames_.size()) {
          this->confirmedDynamicFrames_[i] = 0;
        }
      }
    }
    
    // 只对达到阈值的物体执行体素清除
    if (!boxesToClear.empty()) {
      this->staticFilter_->clearDynamicRegions(boxesToClear);
    }
  }

  hasNewTracking_ = false; // 标记跟踪结果已处理

  // [Performance Timing] 输出耗时
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      end_time - start_time);
  ROS_INFO_THROTTLE(1.0, "%s: classificationCB took %.3f ms",
                    this->hint_.c_str(), duration.count() / 1000.0);
}



// ===================================================================
// 可视化
// ===================================================================
void dynamicDetector::visCB(const ros::TimerEvent &) {
  // [Performance Timing] 测量回调函数耗时
  auto start_time = std::chrono::high_resolution_clock::now();
  // ============================================================================
  // 方案3（可选）：使用 try_lock 避免阻塞，如果锁被占用则跳过本次可视化
  // 如需启用，请取消下面的注释，并注释掉后面的分离锁代码
  // ============================================================================
  // std::unique_lock<std::mutex> lock_cloud(cloudMutex_, std::try_to_lock);
  // std::unique_lock<std::mutex> lock_bbox(bboxMutex_, std::try_to_lock);
  // 
  // if (!lock_cloud.owns_lock() || !lock_bbox.owns_lock()) {
  //   ROS_DEBUG_THROTTLE(2.0, "%s: Skipping visualization (locks busy)", this->hint_.c_str());
  //   return; // 锁被占用，跳过本次可视化
  // }
  // ============================================================================

  // 方案2（当前启用）：分离锁的使用，减少同时持有多个锁的时间
  // 优点：减少对其他线程的阻塞，提高系统并发性能
  
  //----------------------------第一部分：只需要 bboxMutex_ 的可视化----------------------------------------
  {
    std::lock_guard<std::mutex> lock(bboxMutex_);
    
    // 发布过滤后的边界框（青色）
    this->publish3dBox(this->filteredBBoxes_, this->filteredBBoxesPub_, 0, 1, 1);
    
    // 发布经过卡尔曼滤波跟踪后的边界框（黄色）
    this->publish3dBox(this->trackedBBoxes_, this->trackedBBoxesPub_, 1, 1, 0);
    
    // 发布最终被分类为动态的边界框（蓝色）
    this->publish3dBox(this->dynamicBBoxes_, this->dynamicBBoxesPub_, 0, 0, 1);
    
    // 发布被跟踪物体的历史轨迹线
    this->publishHistoryTraj();
    
    // 发布动态障碍物的专用轨迹可视化（轨迹线、轨迹点、速度箭头等）
    this->publishDynamicBoxTrajectory();
  } // 释放 bboxMutex_，让其他线程可以继续工作

  //----------------------------第二部分：需要 cloudMutex_ 和 bboxMutex_ 的可视化----------------------------------------
  {
    // 这部分需要同时访问点云和边界框数据
    // 按照固定顺序加锁：先 cloudMutex_，再 bboxMutex_，避免死锁
    std::lock_guard<std::mutex> lock_cloud(cloudMutex_);
    std::lock_guard<std::mutex> lock_bbox(bboxMutex_);
    
    // 从原始（未降采样）的激光雷达数据中提取并发布动态点云，以获得更密集的视觉效果
    this->publishRawDynamicPoints();
    
    // 发布过滤后的点云
    this->publishFilteredPoints();
    
    // 提取并发布属于动态障碍物的点云
    std::vector<Eigen::Vector3d> dynamicPoints;
    this->getDynamicPc(dynamicPoints);
    this->publishPoints(dynamicPoints, this->dynamicPointsPub_);
  } // 释放所有锁

  // [Performance Timing] 输出耗时
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration =
  std::chrono::duration_cast<std::chrono::microseconds>(end_time -
  start_time); ROS_INFO_THROTTLE(1.0, "%s: visCB took %.3f ms",
  this->hint_.c_str(), duration.count() / 1000.0);
}

// 获取动态点云
void dynamicDetector::getDynamicPc(std::vector<Eigen::Vector3d> &dynamicPc) {
  Eigen::Vector3d curPoint;
  for (size_t i = 0; i < this->filteredPcClusters_.size(); ++i) {
    for (size_t j = 0; j < this->filteredPcClusters_[i].size(); ++j) {
      curPoint = this->filteredPcClusters_[i][j];
      for (size_t k = 0; k < this->dynamicBBoxes_.size(); ++k) {
        if (abs(curPoint(0) - this->dynamicBBoxes_[k].x) <=
                this->dynamicBBoxes_[k].x_width / 2 and
            abs(curPoint(1) - this->dynamicBBoxes_[k].y) <=
                this->dynamicBBoxes_[k].y_width / 2 and
            abs(curPoint(2) - this->dynamicBBoxes_[k].z) <=
                this->dynamicBBoxes_[k].z_width / 2) {
          dynamicPc.push_back(curPoint);
          break;
        }
      }
    }
  }
}

// 发布点云
void dynamicDetector::publishPoints(const std::vector<Eigen::Vector3d> &points,
                                    const ros::Publisher &publisher) {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (size_t i = 0; i < points.size(); ++i) {
    pt.x = points[i](0);
    pt.y = points[i](1);
    pt.z = points[i](2);
    cloud.push_back(pt);
  }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = "map";

  sensor_msgs::PointCloud2 cloudMsg;
  pcl::toROSMsg(cloud, cloudMsg);
  // 使用传感器数据的时间戳，确保与Gazebo同步
  cloudMsg.header.stamp = (this->lastCloudTime_.toSec() > 0) ? this->lastCloudTime_ : ros::Time::now();
  publisher.publish(cloudMsg);
}

// 发布3D边界框
void dynamicDetector::publish3dBox(const std::vector<box3D> &boxes,
                                   const ros::Publisher &publisher, double r,
                                   double g, double b) {
  // 创建一个MarkerArray消息，用于批量发布多个Marker
  visualization_msgs::MarkerArray markers;
  
  // 使用传感器数据的时间戳，确保与Gazebo同步
  // 如果没有有效的时间戳，则使用当前时间
  ros::Time stamp = (this->lastCloudTime_.toSec() > 0) ? this->lastCloudTime_ : ros::Time::now();

  // 遍历所有传入的边界框
  for (size_t i = 0; i < boxes.size(); i++) {
    // 为每个边界框创建一个LINE_LIST类型的Marker
    visualization_msgs::Marker line;
    line.header.frame_id = "map"; // 设置Marker的坐标系为"map"
    line.header.stamp = stamp;    // 使用传感器数据的时间戳，与Gazebo同步
    line.ns = "box3D";            // 设置Marker的命名空间
    line.id = i;                  // 为Marker设置唯一的ID
    line.type = visualization_msgs::Marker::
        LINE_LIST; // Marker类型为线列表，用于绘制立方体的边
    line.action = visualization_msgs::Marker::ADD; // 操作类型为添加或修改
    line.scale.x = 0.06;                           // 设置线的宽度

    // 设置线的颜色和透明度
    line.color.r = r;
    line.color.g = g;
    line.color.b = b;
    line.color.a = 1.0;

    line.lifetime = ros::Duration(0.2); // Marker的生命周期，设置为3倍dt_以避免闪烁

    // 设置Marker的姿态，这里表示无旋转
    line.pose.orientation.x = 0.0;
    line.pose.orientation.y = 0.0;
    line.pose.orientation.z = 0.0;
    line.pose.orientation.w = 1.0;

    // 设置Marker的中心位置
    line.pose.position.x = boxes[i].x;
    line.pose.position.y = boxes[i].y;

    // 获取边界框的宽度和长度
    double x_width = boxes[i].x_width;
    double y_width = boxes[i].y_width;

    // --- 计算Marker在Z轴上的位置和高度 ---
    // 这里的计算方式似乎是为了让边界框的底部接触地面（z=0）
    double top = boxes[i].z + boxes[i].z_width / 2.0; // 计算边界框的最高点Z值
    double z_width = top / 2.0;     // 将可视化Marker的高度设为最高点的一半
    line.pose.position.z = z_width; // 将可视化Marker的中心Z坐标设为该值

    // 定义立方体的8个顶点
    geometry_msgs::Point corner[8];
    corner[0].x = -x_width / 2.0;
    corner[0].y = -y_width / 2.0;
    corner[0].z = -z_width;
    corner[1].x = -x_width / 2.0;
    corner[1].y = y_width / 2.0;
    corner[1].z = -z_width;
    corner[2].x = x_width / 2.0;
    corner[2].y = y_width / 2.0;
    corner[2].z = -z_width;
    corner[3].x = x_width / 2.0;
    corner[3].y = -y_width / 2.0;
    corner[3].z = -z_width;

    corner[4].x = -x_width / 2.0;
    corner[4].y = -y_width / 2.0;
    corner[4].z = z_width;
    corner[5].x = -x_width / 2.0;
    corner[5].y = y_width / 2.0;
    corner[5].z = z_width;
    corner[6].x = x_width / 2.0;
    corner[6].y = y_width / 2.0;
    corner[6].z = z_width;
    corner[7].x = x_width / 2.0;
    corner[7].y = -y_width / 2.0;
    corner[7].z = z_width;

    // 定义连接8个顶点的12条边
    int edgeIdx[12][2] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0}, // 底部四条边
        {4, 5}, {5, 6}, {6, 7}, {7, 4}, // 顶部四条边
        {0, 4}, {1, 5}, {2, 6}, {3, 7}  // 连接上下面的四条垂直边
    };

    // 将12条边的端点添加到Marker的点列表中
    for (int e = 0; e < 12; e++) {
      line.points.push_back(corner[edgeIdx[e][0]]);
      line.points.push_back(corner[edgeIdx[e][1]]);
    }

    // 将配置好的Marker添加到MarkerArray中
    markers.markers.push_back(line);
  }

  // 通过发布器将整个MarkerArray发布出去
  publisher.publish(markers);
}

// 发布历史轨迹
void dynamicDetector::publishHistoryTraj() {
  visualization_msgs::MarkerArray trajMsg;
  int countMarker = 0;
  // 使用传感器数据的时间戳，确保与Gazebo同步
  ros::Time stamp = (this->lastCloudTime_.toSec() > 0) ? this->lastCloudTime_ : ros::Time::now();
  for (size_t i = 0; i < this->boxHist_.size(); ++i) {
    // std::cout << "this->boxHist_[i].size() = "  << this->boxHist_[i].size()
    // << std::endl;
    if (this->boxHist_[i].size() > 5) {
      visualization_msgs::Marker traj;
      traj.header.frame_id = "map";
      traj.header.stamp = stamp;
      traj.ns = "dynamic_detector";
      traj.id = countMarker;
      traj.type = visualization_msgs::Marker::LINE_LIST;
      traj.scale.x = 0.03;
      traj.scale.y = 0.03;
      traj.scale.z = 0.03;
      traj.color.a = 1.0; // Don't forget to set the alpha!
      traj.color.r = 0.0;
      traj.color.g = 1.0;
      traj.color.b = 0.0;
      traj.pose.orientation.w = 1.0;
      traj.pose.orientation.x = 0.0;
      traj.pose.orientation.y = 0.0;
      traj.pose.orientation.z = 0.0;
      traj.lifetime = ros::Duration(0.2); // 设置生命周期，避免闪烁
      for (size_t j = 0; j < this->boxHist_[i].size() - 1; ++j) {
        geometry_msgs::Point p1, p2;
        onboardDetector::box3D box1 = this->boxHist_[i][j];
        onboardDetector::box3D box2 = this->boxHist_[i][j + 1];
        p1.x = box1.x;
        p1.y = box1.y;
        p1.z = box1.z;
        p2.x = box2.x;
        p2.y = box2.y;
        p2.z = box2.z;
        traj.points.push_back(p1);
        traj.points.push_back(p2);
      }

      ++countMarker;
      trajMsg.markers.push_back(traj);
    }
  }
  this->historyTrajPub_.publish(trajMsg);
}

/*!
 * \brief 发布动态障碍物的历史轨迹可视化
 * 
 * 该函数专门用于可视化被识别为动态的障碍物的运动轨迹。
 * 可视化包括三个部分：
 * 1. 轨迹线：连接历史位置点的彩色线条
 * 2. 轨迹点：历史位置上的球体标记
 * 3. 速度箭头：显示当前运动方向和速度的箭头
 */
void dynamicDetector::publishDynamicBoxTrajectory() {
  visualization_msgs::MarkerArray trajMarkers;
  int markerId = 0;
  // 使用传感器数据的时间戳，确保与Gazebo同步
  ros::Time stamp = (this->lastCloudTime_.toSec() > 0) ? this->lastCloudTime_ : ros::Time::now();

  // 遍历所有被跟踪的物体
  for (size_t i = 0; i < this->boxHist_.size(); ++i) {
    // 检查该物体是否有足够的历史数据
    if (this->boxHist_[i].empty()) {
      continue;
    }

    // 只显示被标记为动态的障碍物
    bool isDynamic = this->boxHist_[i][0].is_dynamic;
    if (!isDynamic) {
      continue;
    }

    // 轨迹长度至少要有3个点才显示
    if (this->boxHist_[i].size() < 3) {
      continue;
    }

    // --- 1. 创建轨迹线 (LINE_STRIP) ---
    visualization_msgs::Marker trajLine;
    trajLine.header.frame_id = "map";
    trajLine.header.stamp = stamp;
    trajLine.ns = "dynamic_trajectory_lines";
    trajLine.id = markerId++;
    trajLine.type = visualization_msgs::Marker::LINE_STRIP;
    trajLine.action = visualization_msgs::Marker::ADD;
    trajLine.pose.orientation.w = 1.0;

    // 轨迹线宽度
    trajLine.scale.x = 0.05;

    // 使用统一的轨迹颜色（青色）
    trajLine.color.r = 0.0;
    trajLine.color.g = 0.8;
    trajLine.color.b = 0.8;
    trajLine.color.a = 0.8;
    trajLine.lifetime = ros::Duration(0.2);

    // 添加轨迹点（从旧到新）
    for (int j = this->boxHist_[i].size() - 1; j >= 0; --j) {
      geometry_msgs::Point p;
      p.x = this->boxHist_[i][j].x;
      p.y = this->boxHist_[i][j].y;
      p.z = this->boxHist_[i][j].z;
      trajLine.points.push_back(p);
    }

    trajMarkers.markers.push_back(trajLine);

    // --- 2. 创建速度箭头 (ARROW) ---
    // 获取最新的速度信息
    double vx = this->boxHist_[i][0].Vx;
    double vy = this->boxHist_[i][0].Vy;
    double vz = this->boxHist_[i][0].Vz;
    double velNorm = sqrt(vx * vx + vy * vy + vz * vz);

    // 只有当速度大于阈值时才显示箭头
    if (velNorm > 0.1) {
      visualization_msgs::Marker velArrow;
      velArrow.header.frame_id = "map";
      velArrow.header.stamp = stamp;
      velArrow.ns = "dynamic_velocity_arrows";
      velArrow.id = markerId++;
      velArrow.type = visualization_msgs::Marker::ARROW;
      velArrow.action = visualization_msgs::Marker::ADD;
      
      // 初始化四元数为恒等值（无旋转）
      velArrow.pose.orientation.x = 0.0;
      velArrow.pose.orientation.y = 0.0;
      velArrow.pose.orientation.z = 0.0;
      velArrow.pose.orientation.w = 1.0;
      
      // 箭头的起点和终点
      geometry_msgs::Point start, end;
      start.x = this->boxHist_[i][0].x;
      start.y = this->boxHist_[i][0].y;
      start.z = this->boxHist_[i][0].z;
      
      // 箭头长度与速度成正比（缩放因子可调整）
      double arrowScale = 0.7; // 0.5秒的运动距离
      end.x = start.x + vx * arrowScale;
      end.y = start.y + vy * arrowScale;
      end.z = start.z + vz * arrowScale;
      
      velArrow.points.push_back(start);
      velArrow.points.push_back(end);
      
      // 箭头粗细
      velArrow.scale.x = 0.1;  // 箭杆直径
      velArrow.scale.y = 0.15; // 箭头直径
      velArrow.scale.z = 0.2;  // 箭头长度
      
      // 箭头颜色（亮黄色，易于区分）
      velArrow.color.r = 1.0;
      velArrow.color.g = 1.0;
      velArrow.color.b = 0.0;
      velArrow.color.a = 0.9;
      velArrow.lifetime = ros::Duration(0.2);
      
      trajMarkers.markers.push_back(velArrow);
    }

    // --- 3. 创建文本标签显示分类和速度信息 ---
    visualization_msgs::Marker textLabel;
    textLabel.header.frame_id = "map";
    textLabel.header.stamp = stamp;
    textLabel.ns = "dynamic_trajectory_labels";
    textLabel.id = markerId++;
    textLabel.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    textLabel.action = visualization_msgs::Marker::ADD;

    // 文本位置（在物体上方）
    textLabel.pose.position.x = this->boxHist_[i][0].x;
    textLabel.pose.position.y = this->boxHist_[i][0].y;
    textLabel.pose.position.z = this->boxHist_[i][0].z +
                                this->boxHist_[i][0].z_width  /2.0 + 0.5;

    // 文本大小
    textLabel.scale.z = 0.25;

    // 文本颜色（白色）
    textLabel.color.r = 1.0;
    textLabel.color.g = 1.0;
    textLabel.color.b = 1.0;
    textLabel.color.a = 1.0;
    textLabel.lifetime = ros::Duration(0.2);

    // 获取物体分类信息
    std::string classStr;
    if (this->boxHist_[i][0].is_human) {
      classStr = "Human";
    } else if (this->boxHist_[i][0].is_che) {
      classStr = "Vehicle";
    } else if (this->boxHist_[i][0].is_uav) {
      classStr = "UAV";
    } else {
      classStr = "Other";
    }

    // 文本内容：分类 + 速度 + 轨迹帧数
    std::ostringstream textStream;
    textStream << classStr << " V:" << std::fixed << std::setprecision(2)
               << velNorm << "m/s F:" << this->boxHist_[i].size();
    textLabel.text = textStream.str();

    trajMarkers.markers.push_back(textLabel);
  }

  // 发布所有标记到专用的动态轨迹话题
  this->dynamicTrajPub_.publish(trajMarkers);
}

// 发布过滤后的点
void dynamicDetector::publishFilteredPoints() {
  sensor_msgs::PointCloud2 filteredPointsMsg;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>());
  for (size_t i = 0; i < this->filteredPcClusters_.size(); ++i) {
    std_msgs::ColorRGBA color;
    color.r = 0.5;
    color.g = 0.5;
    color.b = 0.5;
    color.a = 1.0;

    for (size_t j = 0; j < this->filteredPcClusters_[i].size(); ++j) {
      pcl::PointXYZRGB point;
      point.x = this->filteredPcClusters_[i][j](0);
      point.y = this->filteredPcClusters_[i][j](1);
      point.z = this->filteredPcClusters_[i][j](2);
      point.r = color.r * 255;
      point.g = color.g * 255;
      point.b = color.b * 255;
      colored_cloud->push_back(point);
    }
  }
  pcl::toROSMsg(*colored_cloud, filteredPointsMsg);
  filteredPointsMsg.header.frame_id = "map";
  // 使用传感器数据的时间戳，确保与Gazebo同步
  filteredPointsMsg.header.stamp = (this->lastCloudTime_.toSec() > 0) ? this->lastCloudTime_ : ros::Time::now();
  this->filteredPointsPub_.publish(filteredPointsMsg);
}

// 发布原始动态点
void dynamicDetector::publishRawDynamicPoints() {
  // 检查是否有最新的点云数据，如果没有则直接返回
  if (not this->latestCloud_) {
    return;
  }
  try {
    // 创建一个PCL点云指针，用于存储全局坐标系下的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr globalCloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    // 检查是否有传感器位姿信息
    if (this->hasSensorPose_) {
      // 如果有位姿，则将ROS消息格式的原始点云转换到PCL格式
      pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(
          new pcl::PointCloud<pcl::PointXYZ>());
      pcl::fromROSMsg(*this->latestCloud_, *tempCloud);

      // 创建一个仿射变换矩阵，用于将点云从激光雷达坐标系转换到全局（map）坐标系
      Eigen::Affine3d transform = Eigen::Affine3d::Identity();
      transform.linear() = this->orientationLidar_;   // 设置旋转部分
      transform.translation() = this->positionLidar_; // 设置平移部分

      // 对点云应用变换
      pcl::transformPointCloud(*tempCloud, *globalCloud, transform);

      // 创建一个ROS点云消息用于发布可视化
      sensor_msgs::PointCloud2 cloudMsg;
      pcl::toROSMsg(*globalCloud, cloudMsg);
      cloudMsg.header.frame_id = "map";         // 设置坐标系为 "map"
      // 使用传感器数据的时间戳，确保与Gazebo同步
      cloudMsg.header.stamp = (this->lastCloudTime_.toSec() > 0) ? this->lastCloudTime_ : ros::Time::now();
      this->rawLidarPointsPub_.publish(
          cloudMsg); // 发布转换到全局坐标系的原始点云

      // 终端输出测试：打印发布点云信息到ROS日志/终端
      // std::cout << ": No time step parameter found. Use default: 0.033." <<
      // std::endl; ROS_INFO_STREAM(this->hint_ << " publishRawDynamicPoints:
      // transformed pointcloud published. "
      //                 << "hasSensorPose=" << (this->hasSensorPose_ ? "true"
      //                 : "false")
      //                 << ", points=" << globalCloud->points.size());
    } else {
      // 如果没有位姿信息，直接将ROS消息转换为PCL点云（假设其已在全局坐标系）
      pcl::fromROSMsg(*this->latestCloud_, *globalCloud);
    }

    // 创建一个向量，用于存储属于动态障碍物的点的坐标
    std::vector<Eigen::Vector3d> dynamicEigenPoints;

    // 遍历所有已识别的动态边界框
    for (const auto &box : this->dynamicBBoxes_) {
      // 如果该边界框未被标记为动态，则跳过
      if (!box.is_dynamic)
        continue;

      // 计算边界框的最小和最大坐标
      double xmin = box.x - box.x_width / 2.0;
      double xmax = box.x + box.x_width / 2.0;
      double ymin = box.y - box.y_width / 2.0;
      double ymax = box.y + box.y_width / 2.0;
      double zmin = box.z - box.z_width / 2.0;
      double zmax = box.z + box.z_width / 2.0;

      // 遍历全局点云中的每一个点
      for (const auto &point : globalCloud->points) {
        // 检查点是否在当前动态边界框内部
        if (point.x >= xmin && point.x <= xmax && point.y >= ymin &&
            point.y <= ymax && point.z >= zmin && point.z <= zmax) {
          // 如果点在框内，则将其添加到动态点向量中
          dynamicEigenPoints.push_back(
              Eigen::Vector3d(point.x, point.y, point.z));
        }
      }
    }

    // 如果没有找到任何动态点，则直接返回
    if (dynamicEigenPoints.empty()) {
      return;
    }

    // 调用publishPoints函数，将提取出的动态点云发布出去
    this->publishPoints(dynamicEigenPoints, this->rawDynamicPointsPub_);
  }
  // 捕获并报告PCL库可能抛出的异常
  catch (const pcl::PCLException &e) {
    ROS_ERROR("PCL Exception during dynamic point extraction: %s", e.what());
  }
  // 捕获并报告标准C++库可能抛出的异常
  catch (const std::exception &e) {
    ROS_ERROR("Standard Exception during dynamic point extraction: %s",
              e.what());
  }
  // 捕获所有其他类型的未知异常
  catch (...) {
    ROS_ERROR("Unknown error during dynamic point extraction.");
  }
}

// ===================================================================
// 预测
// ===================================================================
// 获取动态障碍物的服务回调函数。对获取的障碍物按与机器人的距离从小到大排序
bool dynamicDetector::getDynamicObstacles(
    ldot_detector::GetDynamicObstacles::Request &req,
    ldot_detector::GetDynamicObstacles::Response &res) {
  
  // 记录服务开始时间
  auto start_time = std::chrono::high_resolution_clock::now();

  // 定义结构体用于存储动态障碍物的完整信息（包括滤波器索引）
  struct DynamicObstacleInfo {
    double distance;                    // 与机器人的距离
    onboardDetector::box3D bbox;        // 边界框数据
    int filterIndex;                    // 对应的滤波器索引
    Eigen::VectorXd filterState;        // 滤波器状态
    Eigen::MatrixXd filterCovariance;   // 滤波器协方差
  };

  // 使用局部拷贝来减少锁持有时间
  std::vector<DynamicObstacleInfo> obstaclesWithInfo;
  {
    std::lock_guard<std::mutex> lock(bboxMutex_); // 加锁保护动态边界框数据
    
    // 检查是否有有效的跟踪数据
    if (this->boxHist_.empty()) {
      ROS_WARN_THROTTLE(2.0, "%s: No tracked obstacles available", this->hint_.c_str());
      return true; // 返回空结果，但服务调用成功
    }

    // 从服务请求中获取机器人当前的位置
    Eigen::Vector3d currPos = Eigen::Vector3d(
        req.current_position.x, req.current_position.y, req.current_position.z);

    // 遍历所有历史轨迹，找出被标记为动态的障碍物
    for (size_t i = 0; i < this->boxHist_.size(); ++i) {
      // 检查历史轨迹是否为空
      if (this->boxHist_[i].empty()) {
        continue;
      }

      // 获取最新帧的边界框
      const onboardDetector::box3D &bbox = this->boxHist_[i][0];
      
      // 只处理被标记为动态的障碍物
      if (!bbox.is_dynamic) {
        continue;
      }

      // 检查对应的滤波器是否存在且已初始化
      if (i >= this->filters_.size() || !this->filters_[i] || 
          !this->filters_[i]->isInitialized()) {
        continue;
      }

      // 计算与机器人的距离
      Eigen::Vector3d obsPos(bbox.x, bbox.y, bbox.z);
      Eigen::Vector3d diff = currPos - obsPos;
      double distance = diff.norm();

      // 如果障碍物在请求的范围之内，则将其添加到列表中
      if (distance <= req.range) {
        DynamicObstacleInfo info;
        info.distance = distance;
        info.bbox = bbox;
        info.filterIndex = static_cast<int>(i);
        info.filterState = this->filters_[i]->getState();
        info.filterCovariance = this->filters_[i]->getCovariance();
        obstaclesWithInfo.push_back(info);
      }
    }
  } // 锁在这里自动释放

  // 检查是否有有效的动态障碍物
  if (obstaclesWithInfo.empty()) {
    ROS_DEBUG_THROTTLE(2.0, "%s: No dynamic obstacles in range", this->hint_.c_str());
    return true; // 返回空结果，但服务调用成功
  }

  // 按距离从小到大对障碍物进行排序
  std::sort(obstaclesWithInfo.begin(), obstaclesWithInfo.end(),
            [](const DynamicObstacleInfo &a, const DynamicObstacleInfo &b) {
              return a.distance < b.distance;
            });

  // 将排序后的障碍物信息填充到服务响应中
  for (const auto &info : obstaclesWithInfo) {
    const onboardDetector::box3D &bbox = info.bbox;
    const Eigen::VectorXd &state = info.filterState;
    const Eigen::MatrixXd &P = info.filterCovariance;
    int dim = state.size();

    geometry_msgs::Vector3 pos;
    geometry_msgs::Vector3 vel;
    geometry_msgs::Vector3 size;

    // 填充当前位置
    pos.x = bbox.x;
    pos.y = bbox.y;
    pos.z = bbox.z;

    // 填充尺寸
    size.x = bbox.x_width;
    size.y = bbox.y_width;
    size.z = bbox.z_width;

    // 根据不同的滤波器模型提取速度
    double vx = 0, vy = 0, vz = 0;
    
    if (dim == 6) {
      // 3D CV模型: [x, y, z, vx, vy, vz]
      vx = state(3);
      vy = state(4);
      vz = state(5);
    } else if (dim == 7) {
      // 7维可能是 Human CA 或 Vehicle CTRA
      bool isVehicle = bbox.is_che;
      if (isVehicle) {
        // CTRA模型: [x, y, z, v, a, yaw, yaw_rate]
        double v = state(3);
        double yaw = state(5);
        vx = v * cos(yaw);
        vy = v * sin(yaw);
      } else {
        // Human CA模型: [x, y, z, vx, vy, ax, ay]
        vx = state(3);
        vy = state(4);
      }
    } else if (dim == 9) {
      // 3D CA模型 (UAV): [x, y, z, vx, vy, vz, ax, ay, az]
      vx = state(3);
      vy = state(4);
      vz = state(5);
    }

    // 填充速度
    vel.x = vx;
    vel.y = vy;
    vel.z = vz;

    // 障碍物类型
    std::string obstacleType;
    if (bbox.is_human) {
      obstacleType = "human";
    } else if (bbox.is_che) {
      obstacleType = "vehicle";
    } else if (bbox.is_uav) {
      obstacleType = "uav";
    } else {
      obstacleType = "other";
    }

    // 将基本数据添加到响应中
    res.position.push_back(pos);
    res.velocity.push_back(vel);
    res.size.push_back(size);
    res.obstacle_types.push_back(obstacleType);

    // 添加状态向量维度
    res.state_dims.push_back(static_cast<uint32_t>(dim));

    // 添加状态向量（扁平化）
    for (int i = 0; i < dim; ++i) {
      res.states.push_back(state(i));
    }

    // 添加协方差矩阵（扁平化，按行存储）
    for (int i = 0; i < dim; ++i) {
      for (int j = 0; j < dim; ++j) {
        res.covariances.push_back(P(i, j));
      }
    }
  }

  // 计算并输出服务耗时
  auto end_time = std::chrono::high_resolution_clock::now();
  double duration_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
  ROS_INFO_THROTTLE(1.0, "%s: GetDynamicObstacles service took %.2f ms, returned %zu obstacles",
            this->hint_.c_str(), duration_ms, res.position.size());
  return true; // 表示服务成功完成
}

// 获取预测轨迹的服务回调函数
// 返回动态障碍物的长期预测轨迹，支持碰撞检测截断
bool dynamicDetector::getPredictedTrajectories(
    ldot_detector::GetPredictedTrajectories::Request &req,
    ldot_detector::GetPredictedTrajectories::Response &res) {

  // 记录服务开始时间
  auto start_time = std::chrono::high_resolution_clock::now();
  
  // 解析请求参数，处理无效参数使用默认值
  double horizon = req.prediction_horizon;
  double dt = req.prediction_dt;
  double range = req.range;

  // 无效参数处理：使用默认值（需求3.4）
  if (horizon <= 0) {
    horizon = this->trajPredDefaultHorizon_;
    ROS_DEBUG_THROTTLE(2.0, "%s: Invalid prediction_horizon, using default: %.2f",
                       this->hint_.c_str(), horizon);
  }
  if (dt <= 0) {
    dt = this->trajPredDefaultDt_;
    ROS_DEBUG_THROTTLE(2.0, "%s: Invalid prediction_dt, using default: %.2f",
                       this->hint_.c_str(), dt);
  }
  if (range <= 0) {
    range = 10.0;  // 默认查询范围10米
    ROS_DEBUG_THROTTLE(2.0, "%s: Invalid range, using default: %.2f",
                       this->hint_.c_str(), range);
  }

  // 定义结构体用于存储动态障碍物的完整信息
  struct DynamicObstacleInfo {
    double distance;                    // 与机器人的距离
    onboardDetector::box3D bbox;        // 边界框数据
    int filterIndex;                    // 对应的滤波器索引
  };

  // 使用局部拷贝来减少锁持有时间
  std::vector<DynamicObstacleInfo> obstaclesWithInfo;
  {
    std::lock_guard<std::mutex> lock(bboxMutex_);  // 加锁保护动态边界框数据

    // 检查是否有有效的跟踪数据（需求3.3：无动态障碍物返回空列表）
    if (this->boxHist_.empty()) {
      ROS_DEBUG_THROTTLE(2.0, "%s: No tracked obstacles available", this->hint_.c_str());
      return true;  // 返回空结果，但服务调用成功
    }

    // 从服务请求中获取机器人当前的位置
    Eigen::Vector3d currPos = Eigen::Vector3d(
        req.current_position.x, req.current_position.y, req.current_position.z);

    // 遍历所有历史轨迹，找出被标记为动态的障碍物
    for (size_t i = 0; i < this->boxHist_.size(); ++i) {
      // 检查历史轨迹是否为空
      if (this->boxHist_[i].empty()) {
        continue;
      }

      // 获取最新帧的边界框
      const onboardDetector::box3D &bbox = this->boxHist_[i][0];

      // 只处理被标记为动态的障碍物
      if (!bbox.is_dynamic) {
        continue;
      }

      // 检查对应的滤波器是否存在且已初始化
      if (i >= this->filters_.size() || !this->filters_[i] ||
          !this->filters_[i]->isInitialized()) {
        continue;
      }

      // 计算与机器人的距离
      Eigen::Vector3d obsPos(bbox.x, bbox.y, bbox.z);
      Eigen::Vector3d diff = currPos - obsPos;
      double distance = diff.norm();

      // 如果障碍物在请求的范围之内，则将其添加到列表中
      if (distance <= range) {
        DynamicObstacleInfo info;
        info.distance = distance;
        info.bbox = bbox;
        info.filterIndex = static_cast<int>(i);
        obstaclesWithInfo.push_back(info);
      }
    }
  }  // 锁在这里自动释放

  // 检查是否有有效的动态障碍物（需求3.3：无动态障碍物返回空列表）
  if (obstaclesWithInfo.empty()) {
    ROS_DEBUG_THROTTLE(2.0, "%s: No dynamic obstacles in range", this->hint_.c_str());
    return true;  // 返回空结果，但服务调用成功
  }

  // 按距离从小到大对障碍物进行排序
  std::sort(obstaclesWithInfo.begin(), obstaclesWithInfo.end(),
            [](const DynamicObstacleInfo &a, const DynamicObstacleInfo &b) {
              return a.distance < b.distance;
            });

  // 遍历动态障碍物，调用predictTrajectory生成预测轨迹
  for (size_t i = 0; i < obstaclesWithInfo.size(); ++i) {
    const DynamicObstacleInfo &info = obstaclesWithInfo[i];
    const onboardDetector::box3D &bbox = info.bbox;

    // 调用轨迹预测函数
    std::vector<TrajectoryPoint> trajectory;
    this->predictTrajectory(info.filterIndex, bbox, horizon, dt, trajectory);

    // 跳过空轨迹
    if (trajectory.empty()) {
      continue;
    }

    // 填充响应数据
    // 障碍物ID（使用滤波器索引作为ID）
    res.obstacle_ids.push_back(static_cast<uint32_t>(info.filterIndex));

    // 障碍物类型（根据分类标志确定）
    std::string obstacleType;
    if (bbox.is_human) {
      obstacleType = "human";
    } else if (bbox.is_che) {
      obstacleType = "vehicle";
    } else if (bbox.is_uav) {
      obstacleType = "uav";
    } else {
      obstacleType = "other";
    }
    res.obstacle_types.push_back(obstacleType);

    // 当前位置
    geometry_msgs::Vector3 currPos;
    currPos.x = bbox.x;
    currPos.y = bbox.y;
    currPos.z = bbox.z;
    res.current_positions.push_back(currPos);

    // 当前速度（从第一个轨迹点获取）
    geometry_msgs::Vector3 currVel;
    currVel.x = trajectory[0].velocity.x();
    currVel.y = trajectory[0].velocity.y();
    currVel.z = trajectory[0].velocity.z();
    res.current_velocities.push_back(currVel);

    // 障碍物尺寸
    geometry_msgs::Vector3 size;
    size.x = bbox.x_width;
    size.y = bbox.y_width;
    size.z = bbox.z_width;
    res.sizes.push_back(size);

    // 轨迹长度（碰撞截断后的实际长度）
    res.trajectory_lengths.push_back(static_cast<uint32_t>(trajectory.size()));

    // 扁平化轨迹数据
    for (const auto &point : trajectory) {
      // 轨迹点位置
      geometry_msgs::Vector3 pos;
      pos.x = point.position.x();
      pos.y = point.position.y();
      pos.z = point.position.z();
      res.trajectory_positions.push_back(pos);

      // 轨迹点速度
      geometry_msgs::Vector3 vel;
      vel.x = point.velocity.x();
      vel.y = point.velocity.y();
      vel.z = point.velocity.z();
      res.trajectory_velocities.push_back(vel);

      // 位置协方差对角元素
      geometry_msgs::Vector3 cov;
      cov.x = point.covariance.x();
      cov.y = point.covariance.y();
      cov.z = point.covariance.z();
      res.position_covariances.push_back(cov);
    }
  }

  // 计算并输出服务耗时
  auto end_time = std::chrono::high_resolution_clock::now();
  double duration_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
  ROS_INFO_THROTTLE(1.0, "%s: GetPredictedTrajectories service took %.2f ms, returned %zu obstacles",
            this->hint_.c_str(), duration_ms, res.obstacle_ids.size());

  return true;  // 服务调用成功
}

// 轨迹预测函数实现
// 基于卡尔曼滤波器状态进行多步轨迹外推，并进行碰撞检测截断
void dynamicDetector::predictTrajectory(int filterIndex,
                                        const onboardDetector::box3D &bbox,
                                        double horizon, double dt,
                                        std::vector<TrajectoryPoint> &trajectory) {
  trajectory.clear();

  // 参数有效性检查
  if (horizon <= 0 || dt <= 0) {
    ROS_WARN_THROTTLE(1.0, "%s: Invalid prediction parameters (horizon=%.2f, dt=%.2f)",
                      this->hint_.c_str(), horizon, dt);
    return;
  }

  // 检查滤波器索引有效性
  if (filterIndex < 0 || filterIndex >= static_cast<int>(this->filters_.size())) {
    ROS_WARN_THROTTLE(1.0, "%s: Invalid filter index %d", this->hint_.c_str(), filterIndex);
    return;
  }

  // 检查滤波器是否存在且已初始化
  if (!this->filters_[filterIndex] || !this->filters_[filterIndex]->isInitialized()) {
    ROS_WARN_THROTTLE(1.0, "%s: Filter %d not initialized", this->hint_.c_str(), filterIndex);
    return;
  }

  // 获取滤波器状态和协方差
  Eigen::VectorXd state = this->filters_[filterIndex]->getState();
  Eigen::MatrixXd P = this->filters_[filterIndex]->getCovariance();
  int dim = state.size();

  // 计算预测步数（限制最大点数）
  int numSteps = static_cast<int>(std::floor(horizon / dt)) + 1;
  numSteps = std::min(numSteps, this->trajPredMaxPoints_);

  // 获取障碍物尺寸用于碰撞检测
  Eigen::Vector3d obstacleSize(bbox.x_width, bbox.y_width, bbox.z_width);

  // 根据不同的滤波器模型进行预测
  // 判断模型类型：
  // dim == 6: CV模型 [x, y, z, vx, vy, vz]
  // dim == 7: Human CA模型 [x, y, z, vx, vy, ax, ay] 或 CTRA模型 [x, y, z, v, a, yaw, yaw_rate]
  // dim == 9: UAV CA模型 [x, y, z, vx, vy, vz, ax, ay, az]

  for (int step = 0; step < numSteps; ++step) {
    double t = step * dt;
    TrajectoryPoint point;
    point.timestamp = t;

    // 根据模型类型计算预测位置和速度
    if (dim == 6) {
      // CV模型: [x, y, z, vx, vy, vz]
      // 恒速度运动学方程: x(t) = x0 + vx*t
      double x0 = state(0), y0 = state(1), z0 = state(2);
      double vx = state(3), vy = state(4), vz = state(5);

      point.position = Eigen::Vector3d(x0 + vx * t, y0 + vy * t, z0 + vz * t);
      point.velocity = Eigen::Vector3d(vx, vy, vz);

      // 协方差传播（简化版本：使用状态转移矩阵的位置部分）
      // 对于CV模型，位置协方差随时间增长
      // P_pos(t) ≈ P_pos(0) + t² * P_vel(0) + 2*t*P_pos_vel(0)
      // 简化为对角元素
      double sigma_x = std::sqrt(P(0, 0) + t * t * P(3, 3));
      double sigma_y = std::sqrt(P(1, 1) + t * t * P(4, 4));
      double sigma_z = std::sqrt(P(2, 2) + t * t * P(5, 5));
      point.covariance = Eigen::Vector3d(sigma_x * sigma_x, sigma_y * sigma_y, sigma_z * sigma_z);

    } else if (dim == 7) {
      // 判断是Human CA模型还是CTRA模型
      bool isVehicle = bbox.is_che;

      if (isVehicle) {
        // CTRA模型: [x, y, z, v, a, yaw, yaw_rate]
        double x0 = state(0), y0 = state(1), z0 = state(2);
        double v = state(3), a = state(4);
        double yaw = state(5), omega = state(6);

        double x_pred, y_pred, yaw_pred;
        double vx_pred, vy_pred;

        // CTRA运动学方程
        const double eps = 1e-6;
        if (std::abs(omega) > eps) {
          // 转弯情况
          double v_t = v + a * t;
          double yaw_t = yaw + omega * t;

          // 位置积分（考虑加速度的CTRA方程）
          x_pred = x0 + (v / omega) * (std::sin(yaw_t) - std::sin(yaw)) +
                   (a / (omega * omega)) *
                       (std::cos(yaw) - std::cos(yaw_t) + omega * t * std::sin(yaw_t));
          y_pred = y0 + (v / omega) * (-std::cos(yaw_t) + std::cos(yaw)) +
                   (a / (omega * omega)) *
                       (std::sin(yaw) - std::sin(yaw_t) + omega * t * std::cos(yaw_t));
          yaw_pred = yaw_t;

          // 速度分量
          vx_pred = v_t * std::cos(yaw_pred);
          vy_pred = v_t * std::sin(yaw_pred);
        } else {
          // 直行情况（退化为CA模型）
          double v_t = v + a * t;
          x_pred = x0 + v * t * std::cos(yaw) + 0.5 * a * t * t * std::cos(yaw);
          y_pred = y0 + v * t * std::sin(yaw) + 0.5 * a * t * t * std::sin(yaw);
          yaw_pred = yaw;

          vx_pred = v_t * std::cos(yaw_pred);
          vy_pred = v_t * std::sin(yaw_pred);
        }

        point.position = Eigen::Vector3d(x_pred, y_pred, z0);
        point.velocity = Eigen::Vector3d(vx_pred, vy_pred, 0.0);

        // 协方差传播（简化）
        double sigma_x = std::sqrt(P(0, 0) + t * t * P(3, 3));
        double sigma_y = std::sqrt(P(1, 1) + t * t * P(3, 3));
        double sigma_z = std::sqrt(P(2, 2));
        point.covariance = Eigen::Vector3d(sigma_x * sigma_x, sigma_y * sigma_y, sigma_z * sigma_z);

      } else {
        // Human CA模型: [x, y, z, vx, vy, ax, ay]
        // 恒加速度运动学方程: x(t) = x0 + vx*t + 0.5*ax*t²
        double x0 = state(0), y0 = state(1), z0 = state(2);
        double vx = state(3), vy = state(4);
        double ax = state(5), ay = state(6);

        point.position = Eigen::Vector3d(x0 + vx * t + 0.5 * ax * t * t,
                                         y0 + vy * t + 0.5 * ay * t * t, z0);
        point.velocity = Eigen::Vector3d(vx + ax * t, vy + ay * t, 0.0);

        // 协方差传播
        double sigma_x = std::sqrt(P(0, 0) + t * t * P(3, 3) + 0.25 * t * t * t * t * P(5, 5));
        double sigma_y = std::sqrt(P(1, 1) + t * t * P(4, 4) + 0.25 * t * t * t * t * P(6, 6));
        double sigma_z = std::sqrt(P(2, 2));
        point.covariance = Eigen::Vector3d(sigma_x * sigma_x, sigma_y * sigma_y, sigma_z * sigma_z);
      }

    } else if (dim == 9) {
      // UAV CA模型: [x, y, z, vx, vy, vz, ax, ay, az]
      // 恒加速度运动学方程（3D）
      double x0 = state(0), y0 = state(1), z0 = state(2);
      double vx = state(3), vy = state(4), vz = state(5);
      double ax = state(6), ay = state(7), az = state(8);

      point.position = Eigen::Vector3d(x0 + vx * t + 0.5 * ax * t * t,
                                       y0 + vy * t + 0.5 * ay * t * t,
                                       z0 + vz * t + 0.5 * az * t * t);
      point.velocity = Eigen::Vector3d(vx + ax * t, vy + ay * t, vz + az * t);

      // 协方差传播
      double sigma_x = std::sqrt(P(0, 0) + t * t * P(3, 3) + 0.25 * t * t * t * t * P(6, 6));
      double sigma_y = std::sqrt(P(1, 1) + t * t * P(4, 4) + 0.25 * t * t * t * t * P(7, 7));
      double sigma_z = std::sqrt(P(2, 2) + t * t * P(5, 5) + 0.25 * t * t * t * t * P(8, 8));
      point.covariance = Eigen::Vector3d(sigma_x * sigma_x, sigma_y * sigma_y, sigma_z * sigma_z);

    } else {
      // 未知模型类型，使用简单的位置外推
      ROS_WARN_THROTTLE(5.0, "%s: Unknown filter dimension %d, using simple extrapolation",
                        this->hint_.c_str(), dim);
      point.position = Eigen::Vector3d(state(0), state(1), state(2));
      point.velocity = Eigen::Vector3d(0, 0, 0);
      point.covariance = Eigen::Vector3d(1.0, 1.0, 1.0);
    }

    // 限制预测协方差上限（为跟踪器协方差上限的倍数）
    if (this->kfParams_.enable_cov_limit) {
      double maxPredCov = this->kfParams_.max_pos_cov * this->kfParams_.prediction_cov_multiplier;
      point.covariance.x() = std::min(point.covariance.x(), maxPredCov);
      point.covariance.y() = std::min(point.covariance.y(), maxPredCov);
      point.covariance.z() = std::min(point.covariance.z(), maxPredCov);
    }

    // 检查预测值是否有效（非NaN/Inf）
    if (!std::isfinite(point.position.x()) || !std::isfinite(point.position.y()) ||
        !std::isfinite(point.position.z()) || !std::isfinite(point.velocity.x()) ||
        !std::isfinite(point.velocity.y()) || !std::isfinite(point.velocity.z())) {
      ROS_WARN_THROTTLE(1.0, "%s: Invalid prediction at step %d, truncating trajectory",
                        this->hint_.c_str(), step);
      break;
    }

    // 碰撞检测：检查预测点是否与静态体素地图发生碰撞
    if (this->staticFilter_ && step > 0) {
      bool collision = this->staticFilter_->checkBoxCollision(
          point.position, obstacleSize, this->trajPredCollisionInflation_);

      if (collision) {
        // 检测到碰撞，截断轨迹
        ROS_DEBUG_THROTTLE(1.0, "%s: Trajectory collision detected at step %d (t=%.2fs)",
                           this->hint_.c_str(), step, t);
        break;
      }
    }

    // 添加轨迹点
    trajectory.push_back(point);
  }

  // 确保至少有一个轨迹点（起始点）
  if (trajectory.empty() && numSteps > 0) {
    TrajectoryPoint startPoint;
    startPoint.timestamp = 0.0;
    startPoint.position = Eigen::Vector3d(state(0), state(1), state(2));

    // 根据模型类型设置速度
    if (dim == 6) {
      startPoint.velocity = Eigen::Vector3d(state(3), state(4), state(5));
    } else if (dim == 7) {
      if (bbox.is_che) {
        double v = state(3), yaw = state(5);
        startPoint.velocity = Eigen::Vector3d(v * std::cos(yaw), v * std::sin(yaw), 0.0);
      } else {
        startPoint.velocity = Eigen::Vector3d(state(3), state(4), 0.0);
      }
    } else if (dim == 9) {
      startPoint.velocity = Eigen::Vector3d(state(3), state(4), state(5));
    } else {
      startPoint.velocity = Eigen::Vector3d(0, 0, 0);
    }

    startPoint.covariance = Eigen::Vector3d(P(0, 0), P(1, 1), P(2, 2));
    trajectory.push_back(startPoint);
  }
}

} // namespace onboardDetector