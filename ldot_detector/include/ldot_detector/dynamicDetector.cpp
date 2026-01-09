/*
    FILE: dynamicDetector.cpp
    ---------------------------------
    function implementation of dynamic osbtacle detector
*/
#include <cmath>   // for std::isfinite
#include <numeric> // for std::iota
#include <chrono>  // for timing
#include <iomanip> // for std::put_time
#include <sstream> // for std::stringstream
#include <unordered_map> // for std::unordered_map
#include <unordered_set> // for std::unordered_set
#include <random>   // for std::random_device, std::mt19937, std::shuffle
#include <algorithm> // for std::shuffle
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
  this->lastConversionTime_ = 0.0;
}

// 带节点句柄的构造函数
dynamicDetector::dynamicDetector(const ros::NodeHandle &nh) {
  this->ns_ = "ldot_detector";
  this->hint_ = "[LDOT]";
  this->nh_ = nh;
  this->isStaticMapReady_ = false;
  this->lastConversionTime_ = 0.0;
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
  
  // 计时输出配置
  this->nh_.param(this->ns_ + "/enable_timing_output", this->enableTimingOutput_, false);
  
  if (this->enableTimingOutput_) {
    // 自动生成带时间戳的文件名
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::stringstream ss;
    ss << "/home/st/LDOT_ws/src/LDOT/ldot_detector/timing/test_ldot_timing_"
       << std::put_time(std::localtime(&now_time_t), "%Y%m%d_%H%M%S")
       << "_" << std::setfill('0') << std::setw(3) << now_ms.count()
       << ".csv";
    this->timingFilePath_ = ss.str();
    
    // 创建目录（如果不存在）
    system("mkdir -p /home/st/LDOT_ws/src/LDOT/ldot_detector/timing");
    
    this->timingOutputFile_.open(this->timingFilePath_, std::ios::out);
    if (this->timingOutputFile_.is_open()) {
      // 写入 CSV 表头
      this->timingOutputFile_ << "Timestamp,PreprocessTime,DetectionTime,TrackingTime,ClassificationTime,TotalTime" << std::endl;
      ROS_INFO_STREAM(this->hint_ << " ========================================");
      ROS_INFO_STREAM(this->hint_ << " Timing output enabled!");
      ROS_INFO_STREAM(this->hint_ << " Output file: " << this->timingFilePath_);
      ROS_INFO_STREAM(this->hint_ << " ========================================");
    } else {
      ROS_ERROR_STREAM(this->hint_ << " Failed to open timing output file: " << this->timingFilePath_);
      this->enableTimingOutput_ = false;
    }
  }
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

  // M-detector 风格：独立订阅里程计和点云，数据存入缓冲区队列
  this->odomSub_ = this->nh_.subscribe<nav_msgs::Odometry>(
      this->odomTopicName_, 200, &dynamicDetector::odomBufferCB, this);

  // 使用标准PointCloud2格式（FAST-LIO输出）
  this->lidarCloudSub_ = this->nh_.subscribe<sensor_msgs::PointCloud2>(
      this->lidarTopicName_, 200, &dynamicDetector::cloudBufferCB, this);

  // 处理定时器：10ms 周期，从缓冲区取数据配对处理
  this->processTimer_ = this->nh_.createTimer(ros::Duration(0.01), 
                                               &dynamicDetector::processTimerCB, this);

  // 可视化定时器（独立线程，只读取双缓冲数据）
  this->visTimer_ = this->nh_.createTimer(ros::Duration(0.01),
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
// M-detector 风格的缓冲区回调函数
// ===================================================================

// 里程计回调：将数据存入缓冲区队列
void dynamicDetector::odomBufferCB(const nav_msgs::OdometryConstPtr &odom) {
  // 只保存完整的里程计消息用于与点云配对
  this->buffer_odoms_.push_back(odom);
}

// 标准点云回调：将数据存入缓冲区队列
void dynamicDetector::cloudBufferCB(const sensor_msgs::PointCloud2ConstPtr &cloudMsg) {
  this->buffer_clouds_.push_back(cloudMsg);
}

// 处理定时器回调：从缓冲区取队首数据配对处理
void dynamicDetector::processTimerCB(const ros::TimerEvent &e) {
  // 检查缓冲区是否都有数据
  bool hasCloud = !this->buffer_clouds_.empty();
  bool hasOdom = !this->buffer_odoms_.empty();
  
  if (!hasCloud || !hasOdom) {
    return;  // 数据不足，等待下一次
  }
  
  // 从缓冲区取队首数据
  nav_msgs::OdometryConstPtr curOdom = this->buffer_odoms_.front();
  this->buffer_odoms_.pop_front();
  
  sensor_msgs::PointCloud2ConstPtr curCloud = this->buffer_clouds_.front();
  this->buffer_clouds_.pop_front();
  this->lidarOdomCB(curCloud, curOdom);
}

// 里程计回调函数，处理点云和里程计数据
void dynamicDetector::lidarOdomCB(
    const sensor_msgs::PointCloud2ConstPtr &cloudMsg,
    const nav_msgs::OdometryConstPtr &odom) {
  
  // 保存最新点云用于可视化（无锁，通过双缓冲保护）
  this->latestCloud_ = cloudMsg;
  this->lastCloudTime_ = cloudMsg->header.stamp; // 记录时间戳

  // ===== 静态地图初始化完成后才开始计时 =====
  // 先检查静态地图是否就绪（通过预先调用一次检测来判断）
  // 在预热阶段，只更新静态地图，不进行完整处理
  if (!this->isStaticMapReady_) {
    // 预热阶段：只做预处理和静态地图更新
    this->lidarCloud_ = this->preprocessPointCloud(cloudMsg, odom);
    
    // 发布降采样后的点云
    sensor_msgs::PointCloud2 outputCloud;
    pcl::toROSMsg(*this->lidarCloud_, outputCloud);
    outputCloud.header.frame_id = "map";
    outputCloud.header.stamp = cloudMsg->header.stamp;
    this->downSamplePointsPub_.publish(outputCloud);
    
    // 尝试检测（内部会检查预热状态并更新静态地图）
    this->runDetection();
    return;  // 预热期间不计时，直接返回
  }
  
  // ===== 静态地图已就绪，开始完整流程并计时 =====
  auto callbackStart = std::chrono::high_resolution_clock::now();
  
  // 点云预处理（范围过滤、坐标变换、地面/天花板过滤、下采样）
  auto preprocessStart = std::chrono::high_resolution_clock::now();
  this->lidarCloud_ = this->preprocessPointCloud(cloudMsg, odom);
  auto preprocessEnd = std::chrono::high_resolution_clock::now();
  double preprocessMs = std::chrono::duration<double, std::milli>(preprocessEnd - preprocessStart).count();

  // 发布降采样后的点云
  sensor_msgs::PointCloud2 outputCloud;
  pcl::toROSMsg(*this->lidarCloud_, outputCloud);
  outputCloud.header.frame_id = "map";
  outputCloud.header.stamp = cloudMsg->header.stamp;
  this->downSamplePointsPub_.publish(outputCloud);

  // 1. 检测
  auto detectionStart = std::chrono::high_resolution_clock::now();
  this->runDetection();
  auto detectionEnd = std::chrono::high_resolution_clock::now();
  double detectionMs = std::chrono::duration<double, std::milli>(detectionEnd - detectionStart).count();
  
  // 2. 跟踪
  auto trackingStart = std::chrono::high_resolution_clock::now();
  this->runTracking();
  auto trackingEnd = std::chrono::high_resolution_clock::now();
  double trackingMs = std::chrono::duration<double, std::milli>(trackingEnd - trackingStart).count();
  
  // 3. 分类
  auto classificationStart = std::chrono::high_resolution_clock::now();
  this->runClassification();
  auto classificationEnd = std::chrono::high_resolution_clock::now();
  double classificationMs = std::chrono::duration<double, std::milli>(classificationEnd - classificationStart).count();
  
  // 4. 将处理结果复制到写缓冲区，然后交换缓冲区
  this->copyToWriteBuffer();
  this->swapBuffers();
  
  // 计算总耗时（从回调开始到所有处理完成）
  auto callbackEnd = std::chrono::high_resolution_clock::now();
  double totalMs = std::chrono::duration<double, std::milli>(callbackEnd - callbackStart).count();
  
  // 输出到 CSV 文件（只在静态地图就绪后才写入）
  if (this->enableTimingOutput_ && this->timingOutputFile_.is_open()) {
    this->timingOutputFile_ << cloudMsg->header.stamp << "," 
                            << preprocessMs << "," 
                            << detectionMs << "," 
                            << trackingMs << "," 
                            << classificationMs << "," 
                            << totalMs << std::endl;
  }
  
  ROS_INFO_THROTTLE(1.0, "%s: Main process completed in %.1f ms (Preprocess: %.1f, Detection: %.1f, Tracking: %.1f, Classification: %.1f)", 
                    this->hint_.c_str(), totalMs, preprocessMs, detectionMs, trackingMs, classificationMs);
  
  lastProcessTime_ = ros::Time::now();
}


// ===================================================================
// 点云预处理
// ===================================================================
/*!
 * \brief 点云预处理函数 - 将原始点云转换为处理后的点云
 * \param cloudMsg 原始点云消息（FAST-LIO输出的全局坐标系点云，已去畸变）
 * \param odom 里程计消息
 * \return 处理后的点云（已完成范围过滤、地面/天花板过滤、下采样）
 * 
 * 处理流程：
 * 1. 更新位姿信息（机体位姿）
 * 2. 范围过滤（相对于机体位置）
 * 3. 地面和天花板过滤（Z方向）
 * 4. 自适应Voxel Grid下采样
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr dynamicDetector::preprocessPointCloud(
    const sensor_msgs::PointCloud2ConstPtr &cloudMsg,
    const nav_msgs::OdometryConstPtr &odom) {
  // [性能计时] 测量函数耗时
  auto start_time = std::chrono::high_resolution_clock::now();
  
  // --- 1. 更新位姿信息（使用机体位姿，FAST-LIO输出的点云已在全局坐标系） ---
  this->position_(0) = odom->pose.pose.position.x;
  this->position_(1) = odom->pose.pose.position.y;
  this->position_(2) = odom->pose.pose.position.z;
  Eigen::Quaterniond quat(
      odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
      odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
  this->orientation_ = quat.toRotationMatrix();

  // --- 2. 点云转换（FAST-LIO输出已在全局坐标系，无需坐标变换） ---
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloudMsg, *tempCloud);
  
  // 记录原始点云数量
  size_t originalPointCount = tempCloud->size();

  // --- 3. 范围过滤（相对于机体位置，使用矩形范围） ---
  pcl::PointCloud<pcl::PointXYZ>::Ptr rangeFilteredCloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  rangeFilteredCloud->reserve(tempCloud->size());

  double range_x = this->localLidarRange_.x();
  double range_y = this->localLidarRange_.y();
  
  for (const pcl::PointXYZ &pt : tempCloud->points) {
    // 计算点到机体的距离（在全局坐标系）
    double dx = std::abs(pt.x - this->position_(0));
    double dy = std::abs(pt.y - this->position_(1));
    
    // 使用矩形范围判断：|dx| <= range_x && |dy| <= range_y
    if (dx <= range_x && dy <= range_y) {
      rangeFilteredCloud->push_back(pt);
    }
  }

  // --- 4. 地面和天花板过滤（Z方向） ---
  pcl::PointCloud<pcl::PointXYZ>::Ptr groundRoofFilterCloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  groundRoofFilterCloud->reserve(rangeFilteredCloud->size());

  for (const pcl::PointXYZ &pt : rangeFilteredCloud->points) {
    if (pt.z >= this->groundHeight_ && pt.z <= this->roofHeight_) {
      groundRoofFilterCloud->push_back(pt);
    }
  }

  // --- 5. 基于体素内点云数量上限的自适应降采样 ---
  // 策略：对每个体素内的点云数量进行限制，超过上限时随机保留部分点
  pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud(
      new pcl::PointCloud<pcl::PointXYZ>());

  if (this->enableVoxelDownsampling_) {
    // 预分配内存，减少动态扩容开销
    finalCloud->reserve(groundRoofFilterCloud->size());
    
    // 使用更高效的体素分组方式
    std::unordered_map<int, std::vector<int>> voxelMap;
    voxelMap.reserve(groundRoofFilterCloud->size() / 10); // 预估体素数量
    
    // 计算每个点的体素索引并分组
    const float invLeafSize = 1.0f / this->voxelBaseLeafSize_; // 预计算倒数，避免除法
    for (size_t i = 0; i < groundRoofFilterCloud->size(); ++i) {
      const pcl::PointXYZ &pt = groundRoofFilterCloud->points[i];
      
      // 使用位运算和乘法代替除法，提高计算速度
      int voxelX = static_cast<int>(std::floor(pt.x * invLeafSize));
      int voxelY = static_cast<int>(std::floor(pt.y * invLeafSize));
      int voxelZ = static_cast<int>(std::floor(pt.z * invLeafSize));
      
      // 使用更简单的哈希函数，减少计算开销
      int voxelKey = ((voxelX * 73856093) ^ (voxelY * 19349663)) ^ (voxelZ * 83492791);
      
      voxelMap[voxelKey].push_back(i);
    }
    
    // 对每个体素内的点进行降采样
    std::random_device rd;
    std::mt19937 gen(rd());
    
    for (const auto &voxelPair : voxelMap) {
      const std::vector<int> &pointIndices = voxelPair.second;
      const size_t pointCount = pointIndices.size();
      
      if (pointCount <= static_cast<size_t>(this->voxelTargetPointCount_)) {
        // 如果体素内点数不超过上限，全部保留
        for (int idx : pointIndices) {
          finalCloud->push_back(groundRoofFilterCloud->points[idx]);
        }
      } else {
        // 使用 Fisher-Yates 洗牌算法的部分实现，只随机选择前N个点
        // 这比完全洗牌更高效
        std::vector<int> selectedIndices;
        selectedIndices.reserve(this->voxelTargetPointCount_);
        
        // 简单的随机采样：随机选择N个不重复的索引
        std::uniform_int_distribution<int> dist(0, pointCount - 1);
        std::unordered_set<int> selectedSet;
        selectedSet.reserve(this->voxelTargetPointCount_);
        
        while (selectedSet.size() < static_cast<size_t>(this->voxelTargetPointCount_)) {
          int randomIdx = dist(gen);
          if (selectedSet.insert(randomIdx).second) {
            selectedIndices.push_back(pointIndices[randomIdx]);
          }
        }
        
        // 添加选中的点
        for (int idx : selectedIndices) {
          finalCloud->push_back(groundRoofFilterCloud->points[idx]);
        }
      }
    }
  } else {
    finalCloud = groundRoofFilterCloud;
  }

  // [性能计时] 输出耗时和点云数量变化
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      end_time - start_time);
  ROS_INFO_THROTTLE(1.0, "%s: preprocessPointCloud took %.3f ms, points: %lu -> %lu",
                    this->hint_.c_str(), duration.count() / 1000.0,
                    originalPointCount, finalCloud->size());

  return finalCloud;
}

// ===================================================================
// 检测
// ===================================================================
void dynamicDetector::runDetection() {
  // auto start_time = std::chrono::high_resolution_clock::now();
  // 检查是否有激光雷达点云数据（提前返回避免不必要的处理）
  if (this->lidarCloud_ == NULL) {
    ROS_WARN_THROTTLE(1.0, "%s: No point cloud available for detection",
                      this->hint_.c_str());
    return;
  }

  // 1. 始终更新静态地图，使用当前ROS时间，并传入机体位置（全局坐标系）
  double currentTime = ros::Time::now().toSec();
  this->staticFilter_->updateMap(this->lidarCloud_, currentTime, this->position_);

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
      return;  // 预热中，直接返回
    } else {
      // 预热完成
      this->isStaticMapReady_ = true;
      ROS_INFO_STREAM(this->hint_ << " Static map warmup completed! Starting dynamic detection...");
    }
  }

  // 2. 收集保护区域（动态物体边界框）- 只收集一次，供点级和聚类级过滤共用
  std::vector<onboardDetector::box3D> protectedBoxes;
  if (this->staticFilterEnabled_) {
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
  this->lidarDetector_->setSensorPosition(this->position_);  // 设置机体位置（用于自适应DBSCAN）
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

  // 临时存储质心和标准差（用于NMS）
  std::vector<Eigen::Vector3d> tmpPcClusterCenters;
  std::vector<Eigen::Vector3d> tmpPcClusterStds;

  // 在生成特征之前进行帧内去重(NMS)以减少不必要计算
  if (this->enableDetectionNMS_ && tmpPcClusters.size() > 1) {
    // size_t beforeNMS = lidarBBoxesFiltered.size();
    this->applyDetectionNMS(lidarBBoxesFiltered, tmpPcClusters,
                            tmpPcClusterCenters, tmpPcClusterStds);
    // size_t afterNMS = lidarBBoxesFiltered.size();
    // if (beforeNMS != afterNMS) {
    //   ROS_INFO_THROTTLE(1.0, "%s: Detection NMS (pre-feature): %lu -> %lu boxes",
    //                     this->hint_.c_str(), beforeNMS, afterNMS);
    // }
  }

  // 清空成员变量，准备存储新的检测结果
  this->filteredBBoxes_.clear();
  this->filteredPcClusters_.clear();
  this->filteredPcClusterCenters_.clear();
  this->filteredPcClusterStds_.clear();
  
  // 预分配内存以提高性能
  this->filteredBBoxes_.reserve(lidarBBoxesFiltered.size());
  this->filteredPcClusters_.reserve(lidarBBoxesFiltered.size());
  this->filteredPcClusterCenters_.reserve(lidarBBoxesFiltered.size());
  this->filteredPcClusterStds_.reserve(lidarBBoxesFiltered.size());

  // 将（已NMS或未NMS）结果直接存入成员变量
  for (size_t i = 0; i < lidarBBoxesFiltered.size(); ++i) {
    onboardDetector::box3D lidarBBox = lidarBBoxesFiltered[i];
    std::vector<Eigen::Vector3d> &pcCluster = tmpPcClusters[i];

    // 提取点云簇的质心
    Eigen::Vector3d clusterCenter(0, 0, 0);
    for (const auto &pt : pcCluster) {
      clusterCenter += pt;
    }
    if (!pcCluster.empty()) clusterCenter /= static_cast<double>(pcCluster.size());
    
    // ===== 质心补偿：使bbox的质心与点云质心保持一致 =====
    // 注意：lidarBBox已经在lidarDBSCAN()或applyDetectionNMS()中补偿过
    // 这里将点云质心也更新为bbox的中心位置，保持一致性
    clusterCenter.x() = lidarBBox.x;
    clusterCenter.y() = lidarBBox.y;
    clusterCenter.z() = lidarBBox.z;
    // ===== 质心同步结束 =====

    // 计算点云簇的标准差（如果applyDetectionNMS已经计算过，保留其值）
    Eigen::Vector3d clusterStd(0, 0, 0);
    if (tmpPcClusterStds.size() == lidarBBoxesFiltered.size()) {
      clusterStd = tmpPcClusterStds[i];
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

    // 直接存入成员变量（避免临时变量和额外的复制）
    this->filteredBBoxes_.push_back(lidarBBox);
    this->filteredPcClusters_.push_back(pcCluster);
    this->filteredPcClusterCenters_.push_back(clusterCenter);
    this->filteredPcClusterStds_.push_back(clusterStd);
  }
  
  // [Performance Timing] 输出耗时
  // auto end_time = std::chrono::high_resolution_clock::now();
  // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
  //     end_time - start_time);
  // ROS_INFO_THROTTLE(1.0, "%s: runDetection took %.3f ms",
  //                   this->hint_.c_str(), duration.count() / 1000.0);
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

    // 2. 从合并后的点云重新计算边界框（更准确、更鲁棒）
    if (mergedPtCount == 0) {
      continue;
    }

    // 计算点云质心
    Eigen::Vector3d mergedCenter = sumPos / static_cast<double>(mergedPc.size());
    
    // 对于X和Y轴，使用传统的min/max方法
    double minX = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double minY = std::numeric_limits<double>::max();
    double maxY = std::numeric_limits<double>::lowest();
    
    // 收集所有Z坐标用于鲁棒估计
    std::vector<double> z_values;
    z_values.reserve(mergedPc.size());
    
    for (const auto& pt : mergedPc) {
      minX = std::min(minX, pt.x());
      maxX = std::max(maxX, pt.x());
      minY = std::min(minY, pt.y());
      maxY = std::max(maxY, pt.y());
      z_values.push_back(pt.z());
    }
    
    // 对Z轴使用百分位数方法过滤离群点
    std::sort(z_values.begin(), z_values.end());
    size_t n = z_values.size();
    size_t lower_idx = std::max(size_t(1), static_cast<size_t>(n * 0.02));
    size_t upper_idx = std::min(n - 1, static_cast<size_t>(n * 0.98));
    double z_min_robust = z_values[lower_idx];
    double z_max_robust = z_values[upper_idx];

    // 计算新的边界框
    onboardDetector::box3D mergedBox;
    // 合并得到的边界框没有明确的原始簇 id，设置为 -1 表示未知/合并产生
    mergedBox.id = -1.0;
    // 尺寸：XY使用包围盒，Z使用鲁棒估计
    mergedBox.x_width = maxX - minX;
    mergedBox.y_width = maxY - minY;
    mergedBox.z_width = z_max_robust - z_min_robust;
    
    // box位置：XY使用点云质心（需要补偿），Z使用鲁棒估计的中心
    mergedBox.x = mergedCenter.x();
    mergedBox.y = mergedCenter.y();
    mergedBox.z = (z_min_robust + z_max_robust) / 2.0;
    
    // ===== NMS质心补偿：对合并后的检测框也进行质心补偿 =====
    if (this->enableCentroidCompensation_) {
      Eigen::Vector3d objectPos(mergedBox.x, mergedBox.y, mergedBox.z);
      Eigen::Vector3d radarToObject = objectPos - this->position_;  // 使用机体位置
      double distance = radarToObject.norm();
      
      if (distance >= this->centroidCompMinDistance_ && distance <= this->centroidCompMaxDistance_) {
        Eigen::Vector3d direction = radarToObject.normalized();
        
        // 使用较大的水平尺寸作为补偿基准
        double sizeInDirection = std::max(mergedBox.x_width, mergedBox.y_width);
        
        // 计算补偿距离
        double compensationDist = sizeInDirection * this->centroidCompensationRatio_;
        
        // 应用补偿（只补偿XY平面）
        mergedBox.x += direction.x() * compensationDist;
        mergedBox.y += direction.y() * compensationDist;
        
        // 同步更新质心（用于后续特征计算）
        mergedCenter.x() = mergedBox.x;
        mergedCenter.y() = mergedBox.y;
      }
    }
    // ===== NMS质心补偿结束 =====

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
void dynamicDetector::runTracking() {
  // auto start_time = std::chrono::high_resolution_clock::now();

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
  if (this->filteredBBoxes_.size() > 0) {
    if (bestMatch.size() > 0) {
      this->kalmanFilterAndUpdateHist(bestMatch); // 更新卡尔曼滤波器和历史记录

      // --- 3. 移除重复轨迹（解决幽灵轨迹问题）---
      this->removeDuplicateTracks();
    }
  } else { // 如果当前帧没有任何检测结果
    // 清空历史记录。
    this->boxHist_.clear();
    this->pcHist_.clear();
    this->pcCenterHist_.clear();
    this->pcStdHist_.clear();
    this->maxHistorySizes_.clear();
    this->smallSizeCounter_.clear();
    this->filters_.clear(); // 同时清空滤波器
    this->stableClassificationCount_.clear();
    this->lastClassifyTime_.clear();
  }

  // [Performance Timing] 输出耗时
  // auto end_time = std::chrono::high_resolution_clock::now();
  // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
  //     end_time - start_time);
  // ROS_INFO_THROTTLE(1.0, "%s: runTracking took %.3f ms", this->hint_.c_str(),
  //                   duration.count() / 1000.0);
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

    for (int i = 0; i < numCurrObjs; ++i) {
      // 设置匹配索引为自己，避免被当作新目标重复初始化

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

    // 第一帧初始化完成，直接返回，不需要再调用 kalmanFilterAndUpdateHist
    return;
  } else {
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
    //     1, "%s: boxAssociation[currBox:%d histBox:%d] -> [o:%d +:%d -:%d]",
    //     this->hint_.c_str(), numCurrObjs, numHistObjs, numMatched,
    //     numNewTargets, numLostTargets);
  }
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
      covariance + 1e-2 * Eigen::Matrix3d::Identity();

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
    // 距离足够近，直接判定为同一物体
    return true;
  }

  // 3. 检查速度方向相似度（仅当距离判断不满足时，作为补充判断）
  double v1 = std::sqrt(bbox1.Vx * bbox1.Vx + bbox1.Vy * bbox1.Vy + bbox1.Vz * bbox1.Vz);
  double v2 = std::sqrt(bbox2.Vx * bbox2.Vx + bbox2.Vy * bbox2.Vy + bbox2.Vz * bbox2.Vz);

  // 两者都在运动时，检查速度方向相似度作为补充判断
  if (v1 >= 0.01 && v2 >= 0.01) {
    // 计算速度方向的余弦相似度
    double vdot = bbox1.Vx * bbox2.Vx + bbox1.Vy * bbox2.Vy + bbox1.Vz * bbox2.Vz;
    double cosSimilarity = vdot / (v1 * v2);

    // 速度方向相似 + 距离在合理范围内 = 同一物体
    if (cosSimilarity > this->duplicateTrackVelocitySimilarityThreshold_ &&
        centerDist < this->duplicateTrackDistanceThreshold_ * avgSize ) {
      return true;
    }
  }

  return false;
}



// ===================================================================
// 动静态分类
// ===================================================================
void dynamicDetector::runClassification() {
  // auto start_time = std::chrono::high_resolution_clock::now();

  // 创建一个临时向量来存储当前帧检测到的动态边界框
  std::vector<onboardDetector::box3D> dynamicBBoxesTemp;

  // 遍历所有被跟踪目标的点云/边界框历史。
  // 默认只判断xy平面的动态性，但对于无人机（is_uav）和其他3D类（is_else），保留z轴速度用于3D动态判别
  for (size_t i = 0; i < this->pcHist_.size(); ++i) {
    // 使用最新的两帧进行比较（当前帧和上一帧）
    // 如果历史记录不足2帧，跳过该轨迹的动静态判断
    if (this->pcHist_[i].size() < 2) {
      continue;
    }
    int curFrameGap = 1;

    // ==================================================================================
    // 强制动态（如果一个障碍物在过去一段时间内被频繁分类为动态，则强制认定其为动态）
    // 但需要额外检查当前速度，防止静态物体因历史误判而被持续标记为动态
    int dynaFrames = 0;
    if (int(this->boxHist_[i].size()) > this->forceDynaCheckRange_) {
      for (int j = 1; j < this->forceDynaCheckRange_ + 1; ++j) {
        if (this->boxHist_[i][j].is_dynamic) {
          ++dynaFrames;
        }
      }
    }

    // 获取卡尔曼滤波器估计的速度（提前计算，用于强制动态的速度检查）
    // 根据不同模型维度进行计算，默认考虑三轴速度
    Eigen::Vector3d Vkf(0., 0., 0.);
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
    // 获取卡尔曼滤波器估计的速度大小
    double velNorm = Vkf.norm();

    // 强制动态判定（历史动态惯性机制）
    // 条件1：历史帧数条件 + 当前速度仍然足够快（原有逻辑）
    // 条件2：历史动态比例极高（>80%）时，即使当前低速也保持动态状态（防止低速/转弯误判）
    //        但需要额外检查：如果连续多帧速度都很低，则允许转为静态
    bool forceDynamic = false;
    
    // 计算历史动态比例
    double dynaRatio = (this->forceDynaCheckRange_ > 0) ? 
                       double(dynaFrames) / double(this->forceDynaCheckRange_) : 0.0;
    
    if (dynaFrames >= this->forceDynaFrames_ && velNorm >= this->dynaVelThresh_) {
      // 原有逻辑：历史动态帧数足够 + 当前速度足够
      forceDynamic = true;
    } else if (dynaRatio >= 0.7 && dynaFrames >= this->dynamicConsistThresh_) {
      // 新增逻辑：历史动态比例极高（>80%），即使当前低速也保持动态
      // 这解决了动态物体低速或原地转弯时被误判为静态的问题
      // 但需要检查是否真的停下来了（连续低速帧数）
      int lowSpeedFrames = 0;
      const int maxLowSpeedFrames = this->forceDynaCheckRange_;  // 连续低速超过10帧才允许转为静态
      forceDynamic = true;
      for (int j = 0; j < std::min(maxLowSpeedFrames, int(this->boxHist_[i].size())); ++j) {
        // 计算历史帧的速度
        double histVel = std::sqrt(
            this->boxHist_[i][j].Vx * this->boxHist_[i][j].Vx +
            this->boxHist_[i][j].Vy * this->boxHist_[i][j].Vy);
        if (histVel < this->dynaVelThresh_ * 0.5) {
          lowSpeedFrames++;
          // 只有连续低速帧数未达到阈值时才保持动态
          if (lowSpeedFrames > maxLowSpeedFrames) {
            forceDynamic = false;
          }
        } else {
          break;  // 一旦有高速帧就停止计数
        }
      }
    }
    
    if (forceDynamic) {
      this->boxHist_[i][0].is_dynamic = true;
      dynamicBBoxesTemp.push_back(this->boxHist_[i][0]);
      continue;
    }
    // ===================================================================================

    // 获取当前帧和历史计算帧的点云
    std::vector<Eigen::Vector3d> currPc = this->pcHist_[i][0];
    std::vector<Eigen::Vector3d> prevPc = this->pcHist_[i][curFrameGap];

    // 初始化速度向量
    Eigen::Vector3d Vcur(0., 0., 0.); // 单个点的速度
    Eigen::Vector3d Vbox(0., 0., 0.); // 整个边界框的平均速度

    int numPoints = currPc.size(); // 点云中的总点数，用于计算投票率
    int votes = 0;                 // “动态”票数

    // 计算边界框中心点的速度
    Vbox(0) = (this->boxHist_[i][0].x - this->boxHist_[i][curFrameGap].x) /
              (this->dt_ * curFrameGap);
    Vbox(1) = (this->boxHist_[i][0].y - this->boxHist_[i][curFrameGap].y) /
              (this->dt_ * curFrameGap);
    Vbox(2) = (this->boxHist_[i][0].z - this->boxHist_[i][curFrameGap].z) /
              (this->dt_ * curFrameGap);

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
      if (velSim < 0) {
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
    
    // 动态判定条件：点云投票率足够高 && 卡尔曼滤波器估计的线速度足够快
    bool is_dynamic_candidate =
        (voteRatio >= this->dynaVoteThresh_ && velNorm >= this->dynaVelThresh_);

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

  // 直接更新最终的动态障碍物列表（已移除尺寸过滤）
  this->dynamicBBoxes_ = dynamicBBoxesTemp;

  // [Performance Timing] 输出耗时
  // auto end_time = std::chrono::high_resolution_clock::now();
  // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
  //     end_time - start_time);
  // ROS_INFO_THROTTLE(1.0, "%s: runClassification took %.3f ms",
  //                   this->hint_.c_str(), duration.count() / 1000.0);
}



// ===================================================================
// 可视化回调函数
// 从双缓冲读取数据，每个发布器对应一个独立的可视化函数
// ===================================================================
void dynamicDetector::visCB(const ros::TimerEvent &) {
  // auto start_time = std::chrono::high_resolution_clock::now();
  
  // 检查是否有数据可用
  if (!dataReady_.load()) {
    ROS_DEBUG_THROTTLE(2.0, "%s: No data ready for visualization", this->hint_.c_str());
    return;
  }
  
  // 获取读缓冲区的引用（只读，无需加锁）
  const SharedData& readBuffer = getReadBuffer();
  
  // ===================================================================
  // 10个可视化发布，每个对应一个函数，点云预处理的可视化在预处理里面
  // ===================================================================
  
  // 1. 原始激光雷达点云 (rawLidarPointsPub_)
  this->visRawLidarPoints(readBuffer);
  
  // 2. 过滤后的点云 (filteredPointsPub_)
  this->visFilteredPoints(readBuffer);
  
  // 3. 过滤后的边界框 (filteredBBoxesPub_) - 青色
  this->visFilteredBBoxes(readBuffer);
  
  // 4. 跟踪的边界框 (trackedBBoxesPub_) - 黄色
  this->visTrackedBBoxes(readBuffer);
  
  // 5. 历史轨迹 (historyTrajPub_)
  this->visHistoryTraj(readBuffer);
  
  // 6. 动态边界框 (dynamicBBoxesPub_) - 蓝色
  this->visDynamicBBoxes(readBuffer);
  
  // 7. 动态点云 (dynamicPointsPub_)
  this->visDynamicPoints(readBuffer);
  
  // 8. 原始动态点云 (rawDynamicPointsPub_)
  this->visRawDynamicPoints(readBuffer);
  
  // 9. 动态轨迹可视化 (dynamicTrajPub_)
  this->visDynamicTraj(readBuffer);

  // [Performance Timing] 输出耗时
  // auto end_time = std::chrono::high_resolution_clock::now();
  // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
  //     end_time - start_time);
  // ROS_INFO_THROTTLE(1.0, "%s: visCB took %.3f ms",
  //                   this->hint_.c_str(), duration.count() / 1000.0);
}

// ===================================================================
// 独立的可视化函数（每个发布器对应一个）
// ===================================================================

// 1. 发布原始激光雷达点云（从双缓冲读取，无锁）
void dynamicDetector::visRawLidarPoints(const SharedData& buffer) {
  if (!buffer.hasCloud) return;
  
  try {
    // FAST-LIO输出的点云已在全局坐标系，直接发布
    sensor_msgs::PointCloud2 cloudMsg = buffer.latestCloud;
    cloudMsg.header.frame_id = "map";
    cloudMsg.header.stamp = (buffer.timestamp.toSec() > 0) ? buffer.timestamp : ros::Time::now();
    this->rawLidarPointsPub_.publish(cloudMsg);
  } catch (...) {
    ROS_ERROR_THROTTLE(5.0, "%s: Error in visRawLidarPoints", this->hint_.c_str());
  }
}

// 2. 发布过滤后的点云
void dynamicDetector::visFilteredPoints(const SharedData& buffer) {
  sensor_msgs::PointCloud2 filteredPointsMsg;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>());
  
  for (size_t i = 0; i < buffer.filteredPcClusters.size(); ++i) {
    for (size_t j = 0; j < buffer.filteredPcClusters[i].size(); ++j) {
      pcl::PointXYZRGB point;
      point.x = buffer.filteredPcClusters[i][j](0);
      point.y = buffer.filteredPcClusters[i][j](1);
      point.z = buffer.filteredPcClusters[i][j](2);
      point.r = 128; point.g = 128; point.b = 128;
      colored_cloud->push_back(point);
    }
  }
  pcl::toROSMsg(*colored_cloud, filteredPointsMsg);
  filteredPointsMsg.header.frame_id = "map";
  filteredPointsMsg.header.stamp = (buffer.timestamp.toSec() > 0) ? buffer.timestamp : ros::Time::now();
  this->filteredPointsPub_.publish(filteredPointsMsg);
}

// 3. 发布过滤后的边界框（青色）
void dynamicDetector::visFilteredBBoxes(const SharedData& buffer) {
  ros::Time stamp = (buffer.timestamp.toSec() > 0) ? buffer.timestamp : ros::Time::now();
  this->publish3dBox(buffer.filteredBBoxes, this->filteredBBoxesPub_, 0, 1, 1, stamp);
}

// 4. 发布跟踪的边界框（黄色）
void dynamicDetector::visTrackedBBoxes(const SharedData& buffer) {
  ros::Time stamp = (buffer.timestamp.toSec() > 0) ? buffer.timestamp : ros::Time::now();
  this->publish3dBox(buffer.trackedBBoxes, this->trackedBBoxesPub_, 1, 1, 0, stamp);
}

// 5. 发布历史轨迹
void dynamicDetector::visHistoryTraj(const SharedData& buffer) {
  visualization_msgs::MarkerArray trajMsg;
  int countMarker = 0;
  ros::Time stamp = (buffer.timestamp.toSec() > 0) ? buffer.timestamp : ros::Time::now();
  
  for (size_t i = 0; i < buffer.boxHist.size(); ++i) {
    if (buffer.boxHist[i].size() > 5) {
      visualization_msgs::Marker traj;
      traj.header.frame_id = "map";
      traj.header.stamp = stamp;
      traj.ns = "dynamic_detector";
      traj.id = countMarker;
      traj.type = visualization_msgs::Marker::LINE_LIST;
      traj.scale.x = 0.03;
      traj.scale.y = 0.03;
      traj.scale.z = 0.03;
      traj.color.a = 1.0;
      traj.color.r = 0.0;
      traj.color.g = 1.0;
      traj.color.b = 0.0;
      traj.pose.orientation.w = 1.0;
      traj.lifetime = ros::Duration(0.1);
      
      for (size_t j = 0; j < buffer.boxHist[i].size() - 1; ++j) {
        geometry_msgs::Point p1, p2;
        p1.x = buffer.boxHist[i][j].x;
        p1.y = buffer.boxHist[i][j].y;
        p1.z = buffer.boxHist[i][j].z;
        p2.x = buffer.boxHist[i][j + 1].x;
        p2.y = buffer.boxHist[i][j + 1].y;
        p2.z = buffer.boxHist[i][j + 1].z;
        traj.points.push_back(p1);
        traj.points.push_back(p2);
      }
      ++countMarker;
      trajMsg.markers.push_back(traj);
    }
  }
  this->historyTrajPub_.publish(trajMsg);
}

// 6. 发布动态边界框（蓝色）
void dynamicDetector::visDynamicBBoxes(const SharedData& buffer) {
  ros::Time stamp = (buffer.timestamp.toSec() > 0) ? buffer.timestamp : ros::Time::now();
  this->publish3dBox(buffer.dynamicBBoxes, this->dynamicBBoxesPub_, 0, 0, 1, stamp);
}

// 7. 发布动态点云
void dynamicDetector::visDynamicPoints(const SharedData& buffer) {
  std::vector<Eigen::Vector3d> dynamicPc;
  for (size_t i = 0; i < buffer.filteredPcClusters.size(); ++i) {
    for (size_t j = 0; j < buffer.filteredPcClusters[i].size(); ++j) {
      const Eigen::Vector3d& curPoint = buffer.filteredPcClusters[i][j];
      for (size_t k = 0; k < buffer.dynamicBBoxes.size(); ++k) {
        if (std::abs(curPoint(0) - buffer.dynamicBBoxes[k].x) <= buffer.dynamicBBoxes[k].x_width / 2 &&
            std::abs(curPoint(1) - buffer.dynamicBBoxes[k].y) <= buffer.dynamicBBoxes[k].y_width / 2 &&
            std::abs(curPoint(2) - buffer.dynamicBBoxes[k].z) <= buffer.dynamicBBoxes[k].z_width / 2) {
          dynamicPc.push_back(curPoint);
          break;
        }
      }
    }
  }
  ros::Time stamp = (buffer.timestamp.toSec() > 0) ? buffer.timestamp : ros::Time::now();
  this->publishPoints(dynamicPc, this->dynamicPointsPub_, stamp);
}

// 8. 发布原始动态点云（从双缓冲读取，无锁）
void dynamicDetector::visRawDynamicPoints(const SharedData& buffer) {
  if (!buffer.hasCloud) return;
  
  try {
    // FAST-LIO输出的点云已在全局坐标系，直接使用
    pcl::PointCloud<pcl::PointXYZ>::Ptr globalCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(buffer.latestCloud, *globalCloud);

    std::vector<Eigen::Vector3d> dynamicEigenPoints;
    for (const auto &box : buffer.dynamicBBoxes) {
      if (!box.is_dynamic) continue;
      double xmin = box.x - box.x_width / 2.0, xmax = box.x + box.x_width / 2.0;
      double ymin = box.y - box.y_width / 2.0, ymax = box.y + box.y_width / 2.0;
      double zmin = box.z - box.z_width / 2.0, zmax = box.z + box.z_width / 2.0;

      for (const auto &point : globalCloud->points) {
        if (point.x >= xmin && point.x <= xmax && point.y >= ymin &&
            point.y <= ymax && point.z >= zmin && point.z <= zmax) {
          dynamicEigenPoints.push_back(Eigen::Vector3d(point.x, point.y, point.z));
        }
      }
    }
    if (!dynamicEigenPoints.empty()) {
      ros::Time stamp = (buffer.timestamp.toSec() > 0) ? buffer.timestamp : ros::Time::now();
      this->publishPoints(dynamicEigenPoints, this->rawDynamicPointsPub_, stamp);
    }
  } catch (...) {
    ROS_ERROR_THROTTLE(5.0, "%s: Error in visRawDynamicPoints", this->hint_.c_str());
  }
}

// 9. 发布动态轨迹可视化
void dynamicDetector::visDynamicTraj(const SharedData& buffer) {
  visualization_msgs::MarkerArray trajMarkers;
  int markerId = 0;
  ros::Time stamp = (buffer.timestamp.toSec() > 0) ? buffer.timestamp : ros::Time::now();

  for (size_t i = 0; i < buffer.boxHist.size(); ++i) {
    if (buffer.boxHist[i].empty()) continue;
    if (!buffer.boxHist[i][0].is_dynamic) continue;
    if (buffer.boxHist[i].size() < 3) continue;

    // 轨迹线
    visualization_msgs::Marker trajLine;
    trajLine.header.frame_id = "map";
    trajLine.header.stamp = stamp;
    trajLine.ns = "dynamic_trajectory_lines";
    trajLine.id = markerId++;
    trajLine.type = visualization_msgs::Marker::LINE_STRIP;
    trajLine.action = visualization_msgs::Marker::ADD;
    trajLine.pose.orientation.w = 1.0;
    trajLine.scale.x = 0.05;
    trajLine.color.r = 0.0; trajLine.color.g = 0.8; trajLine.color.b = 0.8; trajLine.color.a = 0.8;
    trajLine.lifetime = ros::Duration(0.1);

    for (int j = buffer.boxHist[i].size() - 1; j >= 0; --j) {
      geometry_msgs::Point p;
      p.x = buffer.boxHist[i][j].x;
      p.y = buffer.boxHist[i][j].y;
      p.z = buffer.boxHist[i][j].z;
      trajLine.points.push_back(p);
    }
    trajMarkers.markers.push_back(trajLine);

    // 速度箭头
    double vx = buffer.boxHist[i][0].Vx;
    double vy = buffer.boxHist[i][0].Vy;
    double vz = buffer.boxHist[i][0].Vz;
    double velNorm = std::sqrt(vx * vx + vy * vy + vz * vz);

    if (velNorm > 0.1) {
      visualization_msgs::Marker velArrow;
      velArrow.header.frame_id = "map";
      velArrow.header.stamp = stamp;
      velArrow.ns = "dynamic_velocity_arrows";
      velArrow.id = markerId++;
      velArrow.type = visualization_msgs::Marker::ARROW;
      velArrow.action = visualization_msgs::Marker::ADD;
      velArrow.pose.orientation.w = 1.0;
      
      geometry_msgs::Point start, end;
      start.x = buffer.boxHist[i][0].x;
      start.y = buffer.boxHist[i][0].y;
      start.z = buffer.boxHist[i][0].z;
      double arrowScale = 0.7;
      end.x = start.x + vx * arrowScale;
      end.y = start.y + vy * arrowScale;
      end.z = start.z + vz * arrowScale;
      velArrow.points.push_back(start);
      velArrow.points.push_back(end);
      velArrow.scale.x = 0.1; velArrow.scale.y = 0.15; velArrow.scale.z = 0.2;
      velArrow.color.r = 1.0; velArrow.color.g = 1.0; velArrow.color.b = 0.0; velArrow.color.a = 0.9;
      velArrow.lifetime = ros::Duration(0.1);
      trajMarkers.markers.push_back(velArrow);
    }

    // 文本标签
    visualization_msgs::Marker textLabel;
    textLabel.header.frame_id = "map";
    textLabel.header.stamp = stamp;
    textLabel.ns = "dynamic_trajectory_labels";
    textLabel.id = markerId++;
    textLabel.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    textLabel.action = visualization_msgs::Marker::ADD;
    textLabel.pose.position.x = buffer.boxHist[i][0].x;
    textLabel.pose.position.y = buffer.boxHist[i][0].y;
    textLabel.pose.position.z = buffer.boxHist[i][0].z + buffer.boxHist[i][0].z_width / 2.0 + 0.5;
    textLabel.scale.z = 0.25;
    textLabel.color.r = 1.0; textLabel.color.g = 1.0; textLabel.color.b = 1.0; textLabel.color.a = 1.0;
    textLabel.lifetime = ros::Duration(0.1);

    std::string classStr;
    if (buffer.boxHist[i][0].is_human) classStr = "Human";
    else if (buffer.boxHist[i][0].is_che) classStr = "Vehicle";
    else if (buffer.boxHist[i][0].is_uav) classStr = "UAV";
    else classStr = "Other";

    std::ostringstream textStream;
    textStream << classStr << " V:" << std::fixed << std::setprecision(2)
               << velNorm << "m/s F:" << buffer.boxHist[i].size();
    textLabel.text = textStream.str();
    trajMarkers.markers.push_back(textLabel);
  }
  this->dynamicTrajPub_.publish(trajMarkers);
}

// 发布点云
void dynamicDetector::publishPoints(const std::vector<Eigen::Vector3d> &points,
                                    const ros::Publisher &publisher,
                                    const ros::Time &stamp) {
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
  // 使用传入的时间戳
  cloudMsg.header.stamp = stamp;
  publisher.publish(cloudMsg);
}

// 发布3D边界框
void dynamicDetector::publish3dBox(const std::vector<box3D> &boxes,
                                   const ros::Publisher &publisher, double r,
                                   double g, double b, const ros::Time &stamp) {
  // 创建一个MarkerArray消息，用于批量发布多个Marker
  visualization_msgs::MarkerArray markers;

  // 遍历所有传入的边界框
  for (size_t i = 0; i < boxes.size(); i++) {
    // 为每个边界框创建一个LINE_LIST类型的Marker
    visualization_msgs::Marker line;
    line.header.frame_id = "map"; // 设置Marker的坐标系为"map"
    line.header.stamp = stamp;     // 使用传入的时间戳
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

    line.lifetime = ros::Duration(0.1); // Marker的生命周期，设置为3倍dt_以避免闪烁

    // 设置Marker的姿态，这里表示无旋转
    line.pose.orientation.x = 0.0;
    line.pose.orientation.y = 0.0;
    line.pose.orientation.z = 0.0;
    line.pose.orientation.w = 1.0;

    // 设置Marker的中心位置
    line.pose.position.x = boxes[i].x;
    line.pose.position.y = boxes[i].y;

    // 获取边界框的宽度、长度和高度
    double x_width = boxes[i].x_width;
    double y_width = boxes[i].y_width;
    double z_width = boxes[i].z_width;

    // 直接使用边界框的Z坐标作为可视化中心位置
    line.pose.position.z = boxes[i].z;

    // 定义立方体的8个顶点（相对于box中心的偏移）
    geometry_msgs::Point corner[8];
    corner[0].x = -x_width / 2.0;
    corner[0].y = -y_width / 2.0;
    corner[0].z = -z_width / 2.0;
    corner[1].x = -x_width / 2.0;
    corner[1].y = y_width / 2.0;
    corner[1].z = -z_width / 2.0;
    corner[2].x = x_width / 2.0;
    corner[2].y = y_width / 2.0;
    corner[2].z = -z_width / 2.0;
    corner[3].x = x_width / 2.0;
    corner[3].y = -y_width / 2.0;
    corner[3].z = -z_width / 2.0;

    corner[4].x = -x_width / 2.0;
    corner[4].y = -y_width / 2.0;
    corner[4].z = z_width / 2.0;
    corner[5].x = -x_width / 2.0;
    corner[5].y = y_width / 2.0;
    corner[5].z = z_width / 2.0;
    corner[6].x = x_width / 2.0;
    corner[6].y = y_width / 2.0;
    corner[6].z = z_width / 2.0;
    corner[7].x = x_width / 2.0;
    corner[7].y = -y_width / 2.0;
    corner[7].z = z_width / 2.0;

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

// ===================================================================
// 预测服务
// ===================================================================
// 获取动态障碍物的服务回调函数。对获取的障碍物按与机器人的距离从小到大排序
bool dynamicDetector::getDynamicObstacles(
    ldot_detector::GetDynamicObstacles::Request &req,
    ldot_detector::GetDynamicObstacles::Response &res) {
  
  // 记录服务开始时间
  auto start_time = std::chrono::high_resolution_clock::now();

  // 检查是否有数据可用
  if (!dataReady_.load()) {
    ROS_WARN_THROTTLE(2.0, "%s: No data ready for service", this->hint_.c_str());
    return true;
  }

  // 定义结构体用于存储动态障碍物的完整信息（包括滤波器索引）
  struct DynamicObstacleInfo {
    double distance;                    // 与机器人的距离
    onboardDetector::box3D bbox;        // 边界框数据
    int filterIndex;                    // 对应的滤波器索引
    Eigen::VectorXd filterState;        // 滤波器状态
    Eigen::MatrixXd filterCovariance;   // 滤波器协方差
  };

  // 从双缓冲读取数据（只读，无需加锁）
  const SharedData& readBuffer = getReadBuffer();
  std::vector<DynamicObstacleInfo> obstaclesWithInfo;
  
  // 检查是否有有效的跟踪数据
  if (readBuffer.boxHist.empty()) {
    ROS_WARN_THROTTLE(2.0, "%s: No tracked obstacles available", this->hint_.c_str());
    return true;
  }

  // 从服务请求中获取机器人当前的位置
  Eigen::Vector3d currPos = Eigen::Vector3d(
      req.current_position.x, req.current_position.y, req.current_position.z);

  // 遍历所有历史轨迹，找出被标记为动态的障碍物
  for (size_t i = 0; i < readBuffer.boxHist.size(); ++i) {
    if (readBuffer.boxHist[i].empty()) continue;

    const onboardDetector::box3D &bbox = readBuffer.boxHist[i][0];
    if (!bbox.is_dynamic) continue;

    // 检查对应的滤波器是否存在且已初始化
    if (i >= readBuffer.filters.size() || !readBuffer.filters[i] || 
        !readBuffer.filters[i]->isInitialized()) {
      continue;
    }

    // 计算与机器人的距离
    Eigen::Vector3d obsPos(bbox.x, bbox.y, bbox.z);
    double distance = (currPos - obsPos).norm();

    if (distance <= req.range) {
      DynamicObstacleInfo info;
      info.distance = distance;
      info.bbox = bbox;
      info.filterIndex = static_cast<int>(i);
      info.filterState = readBuffer.filters[i]->getState();
      info.filterCovariance = readBuffer.filters[i]->getCovariance();
      obstaclesWithInfo.push_back(info);
    }
  }

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

// 获取预测轨迹的服务回调函数（从双缓冲读取数据，无需加锁）
// 返回动态障碍物的长期预测轨迹，支持碰撞检测截断
bool dynamicDetector::getPredictedTrajectories(
    ldot_detector::GetPredictedTrajectories::Request &req,
    ldot_detector::GetPredictedTrajectories::Response &res) {

  // 记录服务开始时间
  auto start_time = std::chrono::high_resolution_clock::now();
  
  // 检查是否有数据可用
  if (!dataReady_.load()) {
    ROS_WARN_THROTTLE(2.0, "%s: No data ready for service", this->hint_.c_str());
    return true;
  }
  
  // 解析请求参数，处理无效参数使用默认值
  double horizon = req.prediction_horizon;
  double dt = req.prediction_dt;
  double range = req.range;

  if (horizon <= 0) {
    horizon = this->trajPredDefaultHorizon_;
  }
  if (dt <= 0) {
    dt = this->trajPredDefaultDt_;
  }
  if (range <= 0) {
    range = 10.0;
  }

  // 定义结构体用于存储动态障碍物的完整信息
  struct DynamicObstacleInfo {
    double distance;
    onboardDetector::box3D bbox;
    int filterIndex;
  };

  // 从双缓冲读取数据（只读，无需加锁）
  const SharedData& readBuffer = getReadBuffer();
  std::vector<DynamicObstacleInfo> obstaclesWithInfo;

  if (readBuffer.boxHist.empty()) {
    ROS_DEBUG_THROTTLE(2.0, "%s: No tracked obstacles available", this->hint_.c_str());
    return true;
  }

  Eigen::Vector3d currPos = Eigen::Vector3d(
      req.current_position.x, req.current_position.y, req.current_position.z);

  for (size_t i = 0; i < readBuffer.boxHist.size(); ++i) {
    if (readBuffer.boxHist[i].empty()) continue;

    const onboardDetector::box3D &bbox = readBuffer.boxHist[i][0];
    if (!bbox.is_dynamic) continue;

    if (i >= readBuffer.filters.size() || !readBuffer.filters[i] ||
        !readBuffer.filters[i]->isInitialized()) {
      continue;
    }

    Eigen::Vector3d obsPos(bbox.x, bbox.y, bbox.z);
    double distance = (currPos - obsPos).norm();

    if (distance <= range) {
      DynamicObstacleInfo info;
      info.distance = distance;
      info.bbox = bbox;
      info.filterIndex = static_cast<int>(i);
      obstaclesWithInfo.push_back(info);
    }
  }

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

    // 调用轨迹预测函数（使用缓冲区中的滤波器）
    std::vector<TrajectoryPoint> trajectory;
    if (info.filterIndex >= 0 && info.filterIndex < static_cast<int>(readBuffer.filters.size())) {
      this->predictTrajectoryFromFilter(readBuffer.filters[info.filterIndex], bbox, horizon, dt, trajectory);
    }

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

/*!
 * @brief 轨迹预测函数（直接使用滤波器指针）
 * 用于从双缓冲读取数据时调用，基于卡尔曼滤波器状态进行多步轨迹外推
 */
void dynamicDetector::predictTrajectoryFromFilter(
    const std::shared_ptr<KalmanFilterBase>& filter,
    const onboardDetector::box3D &bbox,
    double horizon, double dt,
    std::vector<TrajectoryPoint> &trajectory) {
  trajectory.clear();

  if (horizon <= 0 || dt <= 0) return;
  if (!filter || !filter->isInitialized()) return;

  Eigen::VectorXd state = filter->getState();
  Eigen::MatrixXd P = filter->getCovariance();
  int dim = state.size();

  int numSteps = static_cast<int>(std::floor(horizon / dt)) + 1;
  numSteps = std::min(numSteps, this->trajPredMaxPoints_);

  Eigen::Vector3d obstacleSize(bbox.x_width, bbox.y_width, bbox.z_width);

  for (int step = 0; step < numSteps; ++step) {
    double t = step * dt;
    TrajectoryPoint point;
    point.timestamp = t;

    if (dim == 6) {
      double x0 = state(0), y0 = state(1), z0 = state(2);
      double vx = state(3), vy = state(4), vz = state(5);
      point.position = Eigen::Vector3d(x0 + vx * t, y0 + vy * t, z0 + vz * t);
      point.velocity = Eigen::Vector3d(vx, vy, vz);
      double sigma_x = std::sqrt(P(0, 0) + t * t * P(3, 3));
      double sigma_y = std::sqrt(P(1, 1) + t * t * P(4, 4));
      double sigma_z = std::sqrt(P(2, 2) + t * t * P(5, 5));
      point.covariance = Eigen::Vector3d(sigma_x * sigma_x, sigma_y * sigma_y, sigma_z * sigma_z);
    } else if (dim == 7) {
      bool isVehicle = bbox.is_che;
      if (isVehicle) {
        double x0 = state(0), y0 = state(1), z0 = state(2);
        double v = state(3), a = state(4), yaw = state(5), omega = state(6);
        double x_pred, y_pred, vx_pred, vy_pred;
        const double eps = 1e-6;
        if (std::abs(omega) > eps) {
          double v_t = v + a * t;
          double yaw_t = yaw + omega * t;
          x_pred = x0 + (v / omega) * (std::sin(yaw_t) - std::sin(yaw));
          y_pred = y0 + (v / omega) * (-std::cos(yaw_t) + std::cos(yaw));
          vx_pred = v_t * std::cos(yaw_t);
          vy_pred = v_t * std::sin(yaw_t);
        } else {
          double v_t = v + a * t;
          x_pred = x0 + v * t * std::cos(yaw) + 0.5 * a * t * t * std::cos(yaw);
          y_pred = y0 + v * t * std::sin(yaw) + 0.5 * a * t * t * std::sin(yaw);
          vx_pred = v_t * std::cos(yaw);
          vy_pred = v_t * std::sin(yaw);
        }
        point.position = Eigen::Vector3d(x_pred, y_pred, z0);
        point.velocity = Eigen::Vector3d(vx_pred, vy_pred, 0.0);
        double sigma_x = std::sqrt(P(0, 0) + t * t * P(3, 3));
        double sigma_y = std::sqrt(P(1, 1) + t * t * P(3, 3));
        point.covariance = Eigen::Vector3d(sigma_x * sigma_x, sigma_y * sigma_y, P(2, 2));
      } else {
        double x0 = state(0), y0 = state(1), z0 = state(2);
        double vx = state(3), vy = state(4), ax = state(5), ay = state(6);
        point.position = Eigen::Vector3d(x0 + vx * t + 0.5 * ax * t * t,
                                         y0 + vy * t + 0.5 * ay * t * t, z0);
        point.velocity = Eigen::Vector3d(vx + ax * t, vy + ay * t, 0.0);
        double sigma_x = std::sqrt(P(0, 0) + t * t * P(3, 3));
        double sigma_y = std::sqrt(P(1, 1) + t * t * P(4, 4));
        point.covariance = Eigen::Vector3d(sigma_x * sigma_x, sigma_y * sigma_y, P(2, 2));
      }
    } else if (dim == 9) {
      double x0 = state(0), y0 = state(1), z0 = state(2);
      double vx = state(3), vy = state(4), vz = state(5);
      double ax = state(6), ay = state(7), az = state(8);
      point.position = Eigen::Vector3d(x0 + vx * t + 0.5 * ax * t * t,
                                       y0 + vy * t + 0.5 * ay * t * t,
                                       z0 + vz * t + 0.5 * az * t * t);
      point.velocity = Eigen::Vector3d(vx + ax * t, vy + ay * t, vz + az * t);
      double sigma_x = std::sqrt(P(0, 0) + t * t * P(3, 3));
      double sigma_y = std::sqrt(P(1, 1) + t * t * P(4, 4));
      double sigma_z = std::sqrt(P(2, 2) + t * t * P(5, 5));
      point.covariance = Eigen::Vector3d(sigma_x * sigma_x, sigma_y * sigma_y, sigma_z * sigma_z);
    } else {
      point.position = Eigen::Vector3d(state(0), state(1), state(2));
      point.velocity = Eigen::Vector3d(0, 0, 0);
      point.covariance = Eigen::Vector3d(1.0, 1.0, 1.0);
    }

    if (!std::isfinite(point.position.x()) || !std::isfinite(point.position.y()) ||
        !std::isfinite(point.position.z())) {
      break;
    }

    if (this->staticFilter_ && step > 0) {
      if (this->staticFilter_->checkBoxCollision(point.position, obstacleSize, 
                                                  this->trajPredCollisionInflation_)) {
        break;
      }
    }
    trajectory.push_back(point);
  }

  if (trajectory.empty() && numSteps > 0) {
    TrajectoryPoint startPoint;
    startPoint.timestamp = 0.0;
    startPoint.position = Eigen::Vector3d(state(0), state(1), state(2));
    if (dim == 6) {
      startPoint.velocity = Eigen::Vector3d(state(3), state(4), state(5));
    } else if (dim == 7) {
      if (bbox.is_che) {
        startPoint.velocity = Eigen::Vector3d(state(3) * std::cos(state(5)), 
                                               state(3) * std::sin(state(5)), 0.0);
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

// ===================================================================
// 双缓冲辅助函数
// ===================================================================

/*!
 * @brief 将当前处理结果复制到写缓冲区
 * 在主处理流程完成后调用，准备数据供可视化和服务线程读取
 */
void dynamicDetector::copyToWriteBuffer() {
  SharedData& writeBuffer = getWriteBuffer();
  
  // 复制原始点云和位姿数据（用于可视化原始点云）
  if (this->latestCloud_) {
    writeBuffer.latestCloud = *this->latestCloud_;
    writeBuffer.hasCloud = true;
  } else {
    writeBuffer.hasCloud = false;
  }
  writeBuffer.position = this->position_;
  writeBuffer.orientation = this->orientation_;
  
  // 复制检测结果
  writeBuffer.filteredBBoxes = this->filteredBBoxes_;
  writeBuffer.filteredPcClusters = this->filteredPcClusters_;
  writeBuffer.filteredPcClusterCenters = this->filteredPcClusterCenters_;
  writeBuffer.filteredPcClusterStds = this->filteredPcClusterStds_;
  
  // 复制跟踪结果
  writeBuffer.trackedBBoxes = this->trackedBBoxes_;
  writeBuffer.boxHist = this->boxHist_;
  writeBuffer.pcHist = this->pcHist_;
  writeBuffer.pcCenterHist = this->pcCenterHist_;
  writeBuffer.pcStdHist = this->pcStdHist_;
  writeBuffer.filters = this->filters_;
  
  // 复制分类结果
  writeBuffer.dynamicBBoxes = this->dynamicBBoxes_;
  
  // 复制时间戳
  writeBuffer.timestamp = this->lastCloudTime_;
}

/*!
 * @brief 交换读写缓冲区
 * 原子操作，确保可视化/服务线程读取的是完整的数据
 */
void dynamicDetector::swapBuffers() {
  int oldWrite = writeBufferIndex_.load();
  int oldRead = readBufferIndex_.load();
  writeBufferIndex_.store(oldRead);
  readBufferIndex_.store(oldWrite);
  dataReady_.store(true);
}
} // namespace onboardDetector