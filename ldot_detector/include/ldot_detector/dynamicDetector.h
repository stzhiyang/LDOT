/*
    FILE: dynamicDetector.h
    ---------------------------------
    header file of dynamic obstacle detector
*/
#ifndef ONBOARDDETECTOR_DYNAMICDETECTOR_H
#define ONBOARDDETECTOR_DYNAMICDETECTOR_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <atomic>
#include <boost/math/distributions/chi_squared.hpp> // 用于根据置信度计算卡方分布阈值
#include <livox_ros_driver2/CustomMsg.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ldot_detector/GetDynamicObstacles.h>
#include <ldot_detector/GetPredictedTrajectories.h>
#include <ldot_detector/dbscan.h>
#include <ldot_detector/lidarDetector.h>
#include <ldot_detector/multiModelKalmanFilter.h>
#include <ldot_detector/staticPointFilter.h>
#include <ldot_detector/utils.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/MarkerArray.h>

namespace onboardDetector {

// 前向声明
class ParamLoader;

// 轨迹预测点结构体
// 用于存储预测轨迹中每个时间步的状态信息
struct TrajectoryPoint {
  Eigen::Vector3d position;   // 预测位置
  Eigen::Vector3d velocity;   // 预测速度
  Eigen::Vector3d covariance; // 位置协方差对角元素 (σx², σy², σz²)
  double timestamp;           // 相对时间戳（从当前时刻开始的秒数）
};

// ===================================================================
// 双缓冲数据结构 - 用于线程安全的数据共享
// 处理线程写入 writeBuffer，可视化/服务线程读取 readBuffer
// ===================================================================
struct SharedData {
  // 原始点云和位姿数据（用于可视化原始点云）
  sensor_msgs::PointCloud2 latestCloud;      // 原始点云消息
  Eigen::Vector3d positionLidar;             // 激光雷达位置
  Eigen::Matrix3d orientationLidar;          // 激光雷达姿态
  bool hasCloud = false;                     // 是否有有效点云
  
  // 检测结果
  std::vector<onboardDetector::box3D> filteredBBoxes;
  std::vector<std::vector<Eigen::Vector3d>> filteredPcClusters;
  std::vector<Eigen::Vector3d> filteredPcClusterCenters;
  std::vector<Eigen::Vector3d> filteredPcClusterStds;
  
  // 跟踪结果
  std::vector<onboardDetector::box3D> trackedBBoxes;
  std::vector<std::deque<onboardDetector::box3D>> boxHist;
  std::vector<std::deque<std::vector<Eigen::Vector3d>>> pcHist;
  std::vector<std::deque<Eigen::Vector3d>> pcCenterHist;
  std::vector<std::deque<Eigen::Vector3d>> pcStdHist;
  std::vector<std::shared_ptr<KalmanFilterBase>> filters;
  
  // 分类结果
  std::vector<onboardDetector::box3D> dynamicBBoxes;
  
  // 时间戳
  ros::Time timestamp;
};

class dynamicDetector {
  // 允许 ParamLoader 访问私有成员
  friend class ParamLoader;

private:
  // ===================================================================
  // ROS 基础设施
  // ===================================================================
  std::string ns_;                       // 命名空间，用于ROS话题和参数
  std::string hint_;                     // 日志输出前缀
  ros::NodeHandle nh_;                   // ROS节点句柄

  // 订阅器与同步器
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> lidarCloudSub_;
  std::shared_ptr<message_filters::Subscriber<livox_ros_driver2::CustomMsg>> lidarCustomMsgSub_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odomSub_;
  
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> lidarOdomSync;
  std::shared_ptr<message_filters::Synchronizer<lidarOdomSync>> lidarOdomSync_;
  
  typedef message_filters::sync_policies::ApproximateTime<livox_ros_driver2::CustomMsg, nav_msgs::Odometry> lidarCustomOdomSync;
  std::shared_ptr<message_filters::Synchronizer<lidarCustomOdomSync>> lidarCustomOdomSync_;

  // 定时器
  ros::Timer visTimer_;                  // 可视化发布定时器（独立线程）

  // 发布器
  ros::Publisher filteredBBoxesPub_;     // 过滤后的边界框
  ros::Publisher trackedBBoxesPub_;      // 跟踪的边界框
  ros::Publisher dynamicBBoxesPub_;      // 动态边界框
  ros::Publisher filteredPointsPub_;     // 过滤后的点云
  ros::Publisher dynamicPointsPub_;      // 动态点云
  ros::Publisher rawDynamicPointsPub_;   // 原始动态点云
  ros::Publisher downSamplePointsPub_;   // 降采样后的点云
  ros::Publisher rawLidarPointsPub_;     // 原始激光雷达点云
  ros::Publisher historyTrajPub_;        // 历史轨迹
  ros::Publisher dynamicTrajPub_;        // 动态轨迹

  // 服务
  ros::ServiceServer getDynamicObstacleServer_;       // 获取动态障碍物的服务
  ros::ServiceServer getPredictedTrajectoriesServer_; // 获取预测轨迹的服务

  // ===================================================================
  // 检测器实例
  // ===================================================================
  std::shared_ptr<onboardDetector::lidarDetector> lidarDetector_;    // 激光雷达检测器
  std::shared_ptr<onboardDetector::StaticPointFilter> staticFilter_; // 静态点滤波器

  // ===================================================================
  // 系统配置参数
  // ===================================================================
  // ROS话题配置
  bool useLivoxCustomMsg_;                 // 是否使用Livox CustomMsg格式
  std::string lidarTopicName_;             // 激光雷达点云话题
  std::string odomTopicName_;              // 里程计话题
  
  // 坐标变换
  Eigen::Matrix4d body2Lidar_;             // 机体坐标系到激光雷达坐标系的变换矩阵
  
  // 系统时间参数
  double dt_;                              // 系统运行时间步长

  // ===================================================================
  // 点云预处理参数
  // ===================================================================
  // 高度过滤
  double groundHeight_;                  // 地面高度阈值
  double roofHeight_;                    // 天花板高度阈值
  
  // Voxel Grid下采样
  bool enableVoxelDownsampling_;         // 是否启用自适应下采样
  float voxelBaseLeafSize_;              // 基础体素大小（米）
  int voxelTargetPointCount_;            // 目标点云数量

  // ===================================================================
  // DBSCAN聚类参数
  // ===================================================================
  int lidarDBMinPoints_;                 // 最小点数
  double lidarDBEpsilon_;                // 搜索半径
  bool lidarDBUseAdaptive_;              // 是否启用自适应DBSCAN
  double lidarDBDistanceScale_;          // 距离缩放因子

  // ===================================================================
  // 静态点滤波参数
  // ===================================================================
  bool staticFilterEnabled_;             // 是否启用静态点滤波
  float staticFilterVoxelSize_;          // 体素大小
  int staticFilterHitThreshold_;         // 命中阈值
  double staticFilterTimeThreshold_;     // 时间阈值
  bool staticFilterUseNeighborVoting_;   // 是否启用邻域投票
  int staticFilterMinNeighborVotes_;     // 最小邻域投票数
  
  bool staticClusterFilterEnabled_;      // 是否启用静态聚类滤波
  float staticClusterFilterRatio_;       // 静态聚类过滤比例

  // ===================================================================
  // 目标跟踪与数据关联参数
  // ===================================================================
  // 数据关联
  double associationGateConfidence_;                 // 数据关联的置信度 (0~1)
  double gateThreshold3D_;                           // 3D门限（卡方阈值）
  double associationPosCostWeight_;                  // 位置代价权重
  double associationIoUCostWeight_;                  // 3D IoU代价权重
  double coastingTrackGateRelaxFactor_;              // coasting轨迹的门限放松因子
  
  // 跟踪历史管理
  int histSize_;                                     // 跟踪历史长度
  int maxMissedFrames_;                              // 最大丢失帧数
  std::vector<int> trackMissedFrames_;               // 每个轨迹连续丢失的帧数
  
  // 重复轨迹检测
  double duplicateTrackIoUThreshold_;                // IoU阈值
  double duplicateTrackDistanceThreshold_;           // 距离阈值（平均尺寸缩放因子）
  double duplicateTrackVelocitySimilarityThreshold_; // 速度相似度阈值
  
  // 包围框尺寸管理
  double boxSizeSmoothingAlpha_;                     // 尺寸平滑系数
  double sizeRetainRatio_;                           // 尺寸保持比例阈值

  // ===================================================================
  // 检测过滤参数
  // ===================================================================
  Eigen::Vector3d maxObjectSize_;                    // 物体的最大尺寸阈值
  
  // 帧内去重(NMS)
  bool enableDetectionNMS_;                          // 是否启用检测NMS
  double detectionNMSIoUThreshold_;                  // NMS的IoU阈值
  double detectionNMSDistScale_;                     // NMS距离阈值缩放参数

  // ===================================================================
  // 动态/静态分类参数
  // ===================================================================
  // 基础分类参数
  int skipFrame_;                                    // 点云比较时跳过的帧数
  double dynaVelThresh_;                             // 判定为动态的速度阈值
  double dynaVoteThresh_;                            // 判定为动态的投票比例阈值
  double classificationMinNeighborDist_;             // 点云匹配距离
  
  // 动态一致性检查
  int forceDynaFrames_;                              // 强制判定为动态的帧数阈值
  int forceDynaCheckRange_;                          // 检查强制动态的历史范围
  int dynamicConsistThresh_;                         // 动态一致性检查的帧数阈值
  

  
  // 尺寸管理
  double sizeMergeThresh_;                           // 尺寸合并阈值
  double pointCountMergeThresh_;                     // 点数合并阈值
  int sizeResetFrames_;                              // 尺寸重置帧数
  
  // 鲁棒性增强
  int minReliablePoints_;                            // 最小可靠点数
  double pointCountDropThreshold_;                   // 点数下降阈值（遮挡检测）
  double hysteresisLower_;                           // 滞后系数
  std::vector<bool> previousDynamicState_;           // 每个轨迹的前一帧动态状态
  std::vector<std::deque<int>> pointCountHist_;      // 每个轨迹的历史点数
  
  // 静态体素地图误判修复
  int voxelClearDynamicFrames_;                      // 触发体素清除所需的连续动态帧数

  // ===================================================================
  // 物体分类参数
  // ===================================================================
  // 分类阈值
  double classifyHumanZWidthRatio_;                  // 人：z轴宽度 >= x/y轴的倍数
  double classifyHumanCentroidZRatio_;               // 人：质心z高度 < z轴宽度的倍数
  double classifyVehicleXYWidthRatio_;               // 车：x/y轴最大宽度 >= z轴的倍数
  double classifyVehicleCentroidZRatio_;             // 车：质心z高度 < z轴宽度的倍数
  double classifyUAVMaxSize_;                        // 无人机：x/y/z轴宽度 < 该值(米)
  double classifyUAVCentroidZRatio_;                 // 无人机：质心z高度 > z轴宽度的倍数
  double classifyXYDistanceThreshold_;               // box与无人机xy轴距离阈值(米)
  
  // 分类与模型切换
  int classificationStartFrame_;                     // 跟踪多少帧后开始分类
  double classificationIntervalSec_;                 // 重新分类间隔（秒）
  std::vector<ros::Time> lastClassifyTime_;          // 每个轨迹上一次分类的时间戳
  std::vector<int> stableClassificationCount_;       // 每个轨迹连续相同分类的次数
  int fixSizeClassificationThreshold_;               // 固定尺寸所需的连续相同分类次数

  // ===================================================================
  // 卡尔曼滤波器参数
  // ===================================================================
  KF_Params kfParams_;                               // 卡尔曼滤波器参数

  // ===================================================================
  // 轨迹预测参数
  // ===================================================================
  double trajPredDefaultHorizon_;                    // 默认预测时域（秒）
  double trajPredDefaultDt_;                         // 默认预测步长（秒）
  double trajPredCollisionInflation_;                // 碰撞检测膨胀系数（米）
  int trajPredMaxPoints_;                            // 最大轨迹点数限制

  // ===================================================================
  // 传感器位姿数据
  // ===================================================================
  Eigen::Vector3d position_;         // 机器人当前位置（世界坐标系）
  Eigen::Matrix3d orientation_;      // 机器人当前姿态（世界坐标系）
  Eigen::Vector3d positionLidar_;    // 激光雷达当前位置（世界坐标系）
  Eigen::Matrix3d orientationLidar_; // 激光雷达当前姿态（世界坐标系）
  Eigen::Vector3d localLidarRange_;  // 激光雷达局部检测范围（X、Y方向）
  
  // 运动补偿相关
  bool enableMotionCompensation_;    // 是否启用运动补偿
  int odomHistorySize_;              // 里程计历史队列大小
  std::deque<nav_msgs::Odometry> odomHistory_;  // 里程计历史记录

  // ===================================================================
  // 点云处理数据
  // ===================================================================
  sensor_msgs::PointCloud2ConstPtr latestCloud_;           // 最新的原始激光雷达消息（用于可视化）
  pcl::PointCloud<pcl::PointXYZ>::Ptr lidarCloud_ = NULL; // 处理后的激光雷达点云

  // ===================================================================
  // 检测结果
  // ===================================================================
  std::vector<onboardDetector::box3D> lidarBBoxes_;                  // 激光雷达检测的原始边界框
  std::vector<onboardDetector::Cluster> lidarClusters_;              // 激光雷达点云聚类结果
  std::vector<onboardDetector::box3D> filteredBBoxes_;               // 过滤后的边界框
  std::vector<std::vector<Eigen::Vector3d>> filteredPcClusters_;     // 过滤后的点云聚类
  std::vector<Eigen::Vector3d> filteredPcClusterCenters_;            // 点云聚类中心
  std::vector<Eigen::Vector3d> filteredPcClusterStds_;               // 点云聚类标准差
  
  // ===================================================================
  // 跟踪和分类结果
  // ===================================================================
  std::vector<onboardDetector::box3D> trackedBBoxes_;  // 跟踪的边界框
  std::vector<onboardDetector::box3D> dynamicBBoxes_;  // 动态边界框

  // ===================================================================
  // 跟踪历史数据
  // ===================================================================
  std::vector<std::deque<onboardDetector::box3D>> boxHist_;           // 边界框历史
  std::vector<std::deque<std::vector<Eigen::Vector3d>>> pcHist_;      // 点云历史
  std::vector<std::deque<Eigen::Vector3d>> pcCenterHist_;             // 点云中心历史
  std::vector<std::deque<Eigen::Vector3d>> pcStdHist_;                // 点云标准差历史
  std::vector<Eigen::Vector3d> maxHistorySizes_;                      // 历史最大尺寸
  std::vector<int> smallSizeCounter_;                                 // 小尺寸计数器
  std::vector<std::shared_ptr<KalmanFilterBase>> filters_;            // 卡尔曼滤波器
  std::vector<int> confirmedDynamicFrames_;                           // 连续动态帧数计数器

  // ===================================================================
  // 静态地图初始化
  // ===================================================================
  double staticMapWarmupDuration_;                   // 静态地图预热时长（秒）
  ros::Time systemStartTime_;                        // 系统启动时间
  bool isStaticMapReady_;                            // 静态地图是否已准备好

  // ===================================================================
  // 计时输出（CSV 格式）
  // ===================================================================
  bool enableTimingOutput_;                          // 是否启用计时输出
  std::string timingFilePath_;                       // 计时文件路径
  std::ofstream timingOutputFile_;                   // 计时输出文件流
  double lastConversionTime_;                        // 上一次格式转换的耗时（毫秒）

  // ===================================================================
  // 线程安全与数据同步 - 双缓冲机制（无锁设计）
  // ===================================================================
  ros::Time lastCloudTime_;                          // 最后一次接收点云的时间戳
  ros::Time lastProcessTime_;                        // 最后一次处理的时间戳
  
  SharedData sharedBuffers_[2];                      // 双缓冲数组
  std::atomic<int> writeBufferIndex_{0};             // 当前写入缓冲区索引
  std::atomic<int> readBufferIndex_{1};              // 当前读取缓冲区索引
  std::atomic<bool> dataReady_{false};               // 数据是否准备好供读取

public:
  // ===================================================================
  // 构造与初始化
  // ===================================================================
  dynamicDetector();
  dynamicDetector(const ros::NodeHandle &nh);
  void initDetector(const ros::NodeHandle &nh);
  void initParam();                              // 初始化ROS参数
  void registerPub();                            // 注册所有发布者
  void registerCallback();                       // 注册所有订阅者和定时器

  // ===================================================================
  // 回调函数
  // ===================================================================
  // 传感器数据回调
  void lidarOdomCB(const sensor_msgs::PointCloud2ConstPtr &cloudMsg,
                   const nav_msgs::OdometryConstPtr &odom);
  void lidarCustomOdomCB(const livox_ros_driver2::CustomMsgConstPtr &customMsg,
                         const nav_msgs::OdometryConstPtr &odom);
  
  // 可视化定时器回调
  void visCB(const ros::TimerEvent &);
  
  // 服务回调
  bool getDynamicObstacles(ldot_detector::GetDynamicObstacles::Request &req,
                          ldot_detector::GetDynamicObstacles::Response &res);
  bool getPredictedTrajectories(ldot_detector::GetPredictedTrajectories::Request &req,
                               ldot_detector::GetPredictedTrajectories::Response &res);

  // ===================================================================
  // 主处理流程
  // ===================================================================
  pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessPointCloud(
      const sensor_msgs::PointCloud2ConstPtr &cloudMsg,
      const nav_msgs::OdometryConstPtr &odom);
  void runDetection();                           // 执行检测
  void runTracking();                            // 执行跟踪
  void runClassification();                      // 执行分类
  
  // 运动补偿相关函数
  void updateOdomHistory(const nav_msgs::OdometryConstPtr &odom);
  bool interpolatePose(double timestamp, Eigen::Vector3d &position, 
                       Eigen::Quaterniond &orientation);
  pcl::PointCloud<pcl::PointXYZ>::Ptr applyMotionCompensation(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
      const livox_ros_driver2::CustomMsgConstPtr &customMsg,
      const nav_msgs::OdometryConstPtr &frameEndOdom);

  // ===================================================================
  // 检测模块
  // ===================================================================
  void applyDetectionNMS(std::vector<onboardDetector::box3D> &bboxes,
                        std::vector<std::vector<Eigen::Vector3d>> &pcClusters,
                        std::vector<Eigen::Vector3d> &pcClusterCenters,
                        std::vector<Eigen::Vector3d> &pcClusterStds);

  // ===================================================================
  // 跟踪模块
  // ===================================================================
  void boxAssociation(std::vector<int> &bestMatch);
  void kalmanFilterAndUpdateHist(const std::vector<int> &bestMatch);
  void removeDuplicateTracks();
  bool areDuplicateTracks(int idx1, int idx2);
  
  // 数据关联辅助函数
  double computeMahalanobisDistance3D(const Eigen::Vector3d &posDiff,
                                     const Eigen::Matrix3d &covariance);
  double compute3DIoU(const onboardDetector::box3D &box1,
                     const onboardDetector::box3D &box2);
  double computeAssociationCost3D(const onboardDetector::box3D &predBox,
                                 const Eigen::Vector3d &predStd,
                                 const onboardDetector::box3D &measBox,
                                 const Eigen::Vector3d &measStd,
                                 const Eigen::Matrix3d &covariance);
  void hungarianAlgorithm(const std::vector<std::vector<double>> &costMatrix,
                         std::vector<int> &assignment);

  // ===================================================================
  // 分类模块
  // ===================================================================
  void classifyBox(onboardDetector::box3D &bbox, 
                  const Eigen::Vector4f &centroid,
                  const Eigen::Vector3d &maxHistorySize, 
                  int trackIndex = -1);
  void switchKalmanModel(int index, const onboardDetector::box3D &bbox);

  // ===================================================================
  // 轨迹预测
  // ===================================================================
  void predictTrajectoryFromFilter(const std::shared_ptr<KalmanFilterBase>& filter,
                                   const onboardDetector::box3D &bbox,
                                   double horizon, double dt,
                                   std::vector<TrajectoryPoint> &trajectory);

  // ===================================================================
  // 可视化发布
  // ===================================================================
  void visRawLidarPoints(const SharedData& buffer);
  void visFilteredPoints(const SharedData& buffer);
  void visFilteredBBoxes(const SharedData& buffer);
  void visTrackedBBoxes(const SharedData& buffer);
  void visHistoryTraj(const SharedData& buffer);
  void visDynamicBBoxes(const SharedData& buffer);
  void visDynamicPoints(const SharedData& buffer);
  void visRawDynamicPoints(const SharedData& buffer);
  void visDynamicTraj(const SharedData& buffer);
  
  // 可视化辅助函数
  void publishPoints(const std::vector<Eigen::Vector3d> &points,
                    const ros::Publisher &publisher);
  void publish3dBox(const std::vector<onboardDetector::box3D> &bboxes,
                   const ros::Publisher &publisher, double r, double g, double b);

  // ===================================================================
  // 双缓冲机制
  // ===================================================================
  SharedData& getWriteBuffer() { return sharedBuffers_[writeBufferIndex_.load()]; }
  const SharedData& getReadBuffer() const { return sharedBuffers_[readBufferIndex_.load()]; }
  void swapBuffers();
  void copyToWriteBuffer();

  // ===================================================================
  // 工具函数
  // ===================================================================
  void getLidarPose(const nav_msgs::OdometryConstPtr &odom,
                   Eigen::Matrix4d &lidarPoseMatrix);
  void convertCustomMsgToPointCloud2(const livox_ros_driver2::CustomMsgConstPtr &customMsg,
                                    sensor_msgs::PointCloud2 &cloud);
};

/*!
 * \brief 根据里程计信息计算激光雷达位姿矩阵（使用Odometry消息）
 * \param odom 机器人里程计信息
 * \param lidarPoseMatrix 输出参数，激光雷达的位姿矩阵
 */
inline void
dynamicDetector::getLidarPose(const nav_msgs::OdometryConstPtr &odom,
                              Eigen::Matrix4d &lidarPoseMatrix) {
  Eigen::Quaterniond quat;
  quat = Eigen::Quaterniond(
      odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
      odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
  Eigen::Matrix3d rot = quat.toRotationMatrix();

  // convert body pose to camera pose
  Eigen::Matrix4d map2body;
  map2body.setZero();
  map2body.block<3, 3>(0, 0) = rot;
  map2body(0, 3) = odom->pose.pose.position.x;
  map2body(1, 3) = odom->pose.pose.position.y;
  map2body(2, 3) = odom->pose.pose.position.z;
  map2body(3, 3) = 1.0;

  lidarPoseMatrix = map2body * this->body2Lidar_;
}

} // namespace onboardDetector

#endif
