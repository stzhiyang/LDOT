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
#include <chrono>
#include <geometry_msgs/PoseStamped.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <onboard_detector/GetDynamicObstacles.h>
#include <onboard_detector/dbscan.h>
#include <onboard_detector/lidarDetector.h>
#include <onboard_detector/multiModelKalmanFilter.h>
#include <onboard_detector/staticPointFilter.h>
#include <onboard_detector/utils.h>
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
class dynamicDetector {
private:
  // ROS相关句柄、订阅者、发布者和定时器
  std::string ns_;   // 命名空间，用于ROS话题和参数
  std::string hint_; // 日志输出前缀

  // ROS句柄、订阅者、发布者等
  ros::NodeHandle nh_;
  // 消息过滤器，用于同步不同传感器的数据
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>>
      lidarCloudSub_; // 激光雷达点云订阅器
  std::shared_ptr<message_filters::Subscriber<livox_ros_driver2::CustomMsg>>
      lidarCustomMsgSub_; // Livox CustomMsg订阅器
  std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>>
      poseSub_; // 位姿订阅器
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, geometry_msgs::PoseStamped>
      lidarPoseSync; // 激光-位姿同步策略
  std::shared_ptr<message_filters::Synchronizer<lidarPoseSync>>
      lidarPoseSync_; // 同步器实例
  typedef message_filters::sync_policies::ApproximateTime<
      livox_ros_driver2::CustomMsg, geometry_msgs::PoseStamped>
      lidarCustomPoseSync; // Livox-位姿同步策略
  std::shared_ptr<message_filters::Synchronizer<lidarCustomPoseSync>>
      lidarCustomPoseSync_; // 同步器实例
  std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>>
      odomSub_; // 里程计订阅器
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, nav_msgs::Odometry>
      lidarOdomSync; // 激光-里程计同步策略
  std::shared_ptr<message_filters::Synchronizer<lidarOdomSync>>
      lidarOdomSync_; // 同步器实例
  typedef message_filters::sync_policies::ApproximateTime<
      livox_ros_driver2::CustomMsg, nav_msgs::Odometry>
      lidarCustomOdomSync; // Livox-里程计同步策略
  std::shared_ptr<message_filters::Synchronizer<lidarCustomOdomSync>>
      lidarCustomOdomSync_; // 同步器实例

  // 定时器，用于周期性执行检测、跟踪、分类和可视化任务
  ros::Timer lidarDetectionTimer_; // 激光雷达检测定时器
  ros::Timer trackingTimer_;       // 目标跟踪定时器
  ros::Timer classificationTimer_; // 动态/静态分类定时器
  ros::Timer visTimer_;            // 可视化发布定时器

  // 发布器，用于发布中间结果和最终结果以供调试和可视化
  ros::Publisher lidarBBoxesPub_;         // 激光雷达检测的3D边界框
  ros::Publisher filteredBBoxesPub_;      // 最终过滤后的3D边界框
  ros::Publisher trackedBBoxesPub_;       // 跟踪中的3D边界框
  ros::Publisher dynamicBBoxesPub_;       // 最终识别出的动态3D边界框
  ros::Publisher filteredDepthPointsPub_; // 过滤后的深度点云
  ros::Publisher lidarClustersPub_;       // 激光雷达点云聚类
  ros::Publisher filteredPointsPub_;      // 过滤后的融合点云
  ros::Publisher dynamicPointsPub_;       // 动态障碍物的点云
  ros::Publisher rawDynamicPointsPub_;    // 原始传感器数据中的动态点云
  ros::Publisher downSamplePointsPub_;    // 降采样后的点云
  ros::Publisher rawLidarPointsPub_;      // 原始激光雷达点云
  ros::Publisher historyTrajPub_;         // 跟踪物体的历史轨迹
  ros::Publisher velVisPub_;              // 跟踪物体的速度可视化

  // 服务
  ros::ServiceServer getDynamicObstacleServer_; // 获取动态障碍物的服务

  // 检测器实例
  std::shared_ptr<onboardDetector::lidarDetector>
      lidarDetector_; // 激光雷达检测器
  std::shared_ptr<onboardDetector::StaticPointFilter>
      staticFilter_; // 静态点滤波器

  // 激光雷达参数
  Eigen::Matrix4d body2Lidar_; // 机体坐标系到激光雷达坐标系的变换矩阵

  // ROS话题名称与模式参数
  int localizationMode_;       // 定位模式 (0: Pose, 1: Odometry)
  bool useLivoxCustomMsg_;     // 是否使用Livox CustomMsg格式 (true: CustomMsg,
                               // false: PointCloud2)
  std::string lidarTopicName_; // 激光雷达点云话题
  std::string poseTopicName_;  // 位姿话题
  std::string odomTopicName_;  // 里程计话题

  // 系统参数
  double dt_; // 系统运行时间步长

  // DBSCAN通用参数
  double groundHeight_; // 地面高度阈值，用于滤除地面点
  double roofHeight_;   // 天花板高度阈值，用于滤除天花板点

  // 激光雷达DBSCAN聚类参数
  int lidarDBMinPoints_;        // 激光雷达DBSCAN的最小点数
  double lidarDBEpsilon_;       // 激光雷达DBSCAN的搜索半径
  bool lidarDBUseAdaptive_;     // 是否启用基于距离的自适应DBSCAN
  double lidarDBDistanceScale_; // 自适应DBSCAN的距离缩放因子

  // 点云数量控制参数 - Voxel Grid自适应下采样
  bool enableVoxelDownsampling_; // 是否启用Voxel Grid自适应下采样
  float voxelBaseLeafSize_;      // 基础体素大小（米）
  int voxelTargetPointCount_;    // 目标点云数量

  // 静态点滤波器参数
  bool staticFilterEnabled_;
  float staticFilterVoxelSize_;
  int staticFilterHitThreshold_;
  double staticFilterTimeThreshold_;

  // 静态聚类滤波器参数
  bool staticClusterFilterEnabled_;
  float staticClusterFilterRatio_;

  // 目标跟踪与数据关联参数
  double associationGateConfidence_;   // 数据关联的置信度 (0~1)
  double gateThreshold3D_;             // 3D门限: 根据置信度计算三维卡方阈值
  double associationPosCostWeight_;    // 位置代价的权重
  double associationIoUCostWeight_;    // 3D IoU代价的权重
  int histSize_;                       // 跟踪历史的长度
  int kfAvgFrames_;                    // 用于计算观测速度的帧数
  int maxMissedFrames_;                // 最大丢失帧数
  std::vector<int> trackMissedFrames_; // 每个轨迹连续丢失的帧数
  double duplicateTrackIoUThreshold_;  // 重复轨迹检测的IoU阈值
  double boxSizeSmoothingAlpha_;       // 包围框尺寸平滑系数

  // 动态/静态分类参数
  int skipFrame_;               // 点云比较时跳过的帧数
  double dynaVelThresh_;        // 判定为动态的线速度阈值
  double dynaAngularVelThresh_; // 判定为动态的角速度阈值（原地转弯检测）
  double dynaVoteThresh_;       // 判定为动态的投票比例阈值
  int forceDynaFrames_;      // 在历史中被判定为动态的帧数，超过则强制认为是动态
  int forceDynaCheckRange_;  // 检查强制动态的历史范围
  int dynamicConsistThresh_; // 动态一致性检查的帧数阈值
  double boxSizeChangeThresh_; // 尺寸变化阈值

  // 尺寸约束参数
  Eigen::Vector3d maxObjectSize_; // 物体的最大尺寸阈值

  // 分类阈值参数
  double classifyHumanZWidthRatio_;      // 人：z轴宽度 >= x/y轴的倍数
  double classifyHumanCentroidZRatio_;   // 人：质心z高度 < z轴宽度的倍数
  double classifyVehicleXYWidthRatio_;   // 车：x/y轴最大宽度 >= z轴的倍数
  double classifyVehicleCentroidZRatio_; // 车：质心z高度 < z轴宽度的倍数
  double classifyUAVMaxSize_;            // 无人机：x/y/z轴宽度 < 该值(米)
  double classifyUAVCentroidZRatio_;     // 无人机：质心z高度 > z轴宽度的倍数

  // 分类与模型切换参数
  int classificationStartFrame_;       // 跟踪多少帧后开始进行分类和模型切换
  double classifyHumanPcaRatio_;       // PCA特征: z_std / xy_std 的阈值
  double classifyVehiclePcaRatio_;     // PCA特征: xy_std / z_std 的阈值
  double classifyCloseRangeThreshold_; // 近距离分类阈值
  double classificationIntervalSec_; // 首次分类后，基于时间的重新分类间隔（秒）
  std::vector<ros::Time> lastClassifyTime_; // 每个轨迹上一次分类的时间戳

  // 卡尔曼滤波器参数
  KF_Params kfParams_;

  // 传感器原始数据
  Eigen::Vector3d position_;         // 机器人当前位置
  Eigen::Matrix3d orientation_;      // 机器人当前姿态
  Eigen::Vector3d positionLidar_;    // 激光雷达当前位置
  Eigen::Matrix3d orientationLidar_; // 激光雷达当前姿态
  bool hasSensorPose_;               // 是否已获取到传感器位姿
  Eigen::Vector3d localLidarRange_;  // 激光雷达局部检测范围

  // 激光雷达处理数据
  sensor_msgs::PointCloud2ConstPtr latestCloud_; // 最新的原始激光雷达消息
  pcl::PointCloud<pcl::PointXYZ>::Ptr lidarCloud_ =
      NULL;                                             // 处理后的激光雷达点云
  std::vector<onboardDetector::Cluster> lidarClusters_; // 激光雷达点云聚类结果

  // 检测器中间数据
  int projPointsNum_ = 0;                              // 投影点数量
  std::vector<onboardDetector::box3D> filteredBBoxes_; // 最终过滤后的边界框
  std::vector<std::vector<Eigen::Vector3d>>
      filteredPcClusters_; // 最终过滤后的点云聚类
  std::vector<Eigen::Vector3d>
      filteredPcClusterCenters_; // 最终过滤后点云聚类的中心
  std::vector<Eigen::Vector3d>
      filteredPcClusterStds_; // 最终过滤后点云聚类的标准差
  std::vector<onboardDetector::box3D> lidarBBoxes_; // 由激光雷达检测到的边界框
  std::vector<onboardDetector::box3D>
      trackedBBoxes_; // 经过卡尔曼滤波跟踪的边界框
  std::vector<onboardDetector::box3D> dynamicBBoxes_; // 被分类为动态的边界框

  // 跟踪与关联数据
  bool newDetectFlag_; // 是否有新检测结果的标志
  std::vector<std::deque<onboardDetector::box3D>>
      boxHist_; // 每个被跟踪物体的边界框历史
  std::vector<std::deque<std::vector<Eigen::Vector3d>>>
      pcHist_; // 每个被跟踪物体的点云历史
  std::vector<std::deque<Eigen::Vector3d>>
      pcCenterHist_; // 每个被跟踪物体的点云中心历史
  std::vector<std::deque<Eigen::Vector3d>>
      pcStdHist_; // 每个被跟踪物体的点云标准差历史
  std::vector<Eigen::Vector3d> maxHistorySizes_; // 每个被跟踪物体的历史最大尺寸
  std::vector<std::shared_ptr<KalmanFilterBase>>
      filters_; // 每个被跟踪物体对应的多模型卡尔曼滤波器

  // 线程安全与数据同步
  std::mutex cloudMutex_;                    // 保护点云数据的互斥锁
  std::mutex bboxMutex_;                     // 保护边界框数据的互斥锁
  std::atomic<bool> hasNewCloud_{false};     // 是否有新点云数据
  std::atomic<bool> hasNewDetection_{false}; // 是否有新检测结果
  std::atomic<bool> hasNewTracking_{false};  // 是否有新跟踪结果
  ros::Time lastCloudTime_;                  // 最后一次接收点云的时间戳
  ros::Time lastProcessTime_;                // 最后一次处理的时间戳

public:
  // 构造与析构函数
  dynamicDetector();
  dynamicDetector(const ros::NodeHandle &nh);
  void initDetector(const ros::NodeHandle &nh);

  // 初始化函数
  void initParam();        // 初始化ROS参数
  void registerPub();      // 注册所有发布者
  void registerCallback(); // 注册所有订阅者和定时器

  // 服务回调函数
  bool
  getDynamicObstacles(onboard_detector::GetDynamicObstacles::Request &req,
                      onboard_detector::GetDynamicObstacles::Response &res);

  // 传感器数据回调函数
  void lidarPoseCB(const sensor_msgs::PointCloud2ConstPtr &cloudMsg,
                   const geometry_msgs::PoseStampedConstPtr &pose);
  void lidarOdomCB(const sensor_msgs::PointCloud2ConstPtr &cloudMsg,
                   const nav_msgs::OdometryConstPtr &odom);
  void lidarCustomPoseCB(const livox_ros_driver2::CustomMsgConstPtr &customMsg,
                         const geometry_msgs::PoseStampedConstPtr &pose);
  void lidarCustomOdomCB(const livox_ros_driver2::CustomMsgConstPtr &customMsg,
                         const nav_msgs::OdometryConstPtr &odom);

  // 定时器回调函数
  void lidarDetectionCB(const ros::TimerEvent &); // 激光雷达检测主循环
  void trackingCB(const ros::TimerEvent &);       // 跟踪主循环
  void classificationCB(const ros::TimerEvent &); // 分类主循环
  void visCB(const ros::TimerEvent &);            // 可视化主循环

  // 检测模块函数
  void lidarDetect(); // 执行激光雷达检测
  void
  classifyBox(onboardDetector::box3D &bbox, const Eigen::Vector4f &centroid,
              const Eigen::Vector3d &clusterStd,
              const Eigen::Vector3d &maxHistorySize); // 对单个边界框进行分类
  void
  switchKalmanModel(int index,
                    const onboardDetector::box3D &bbox); // 切换卡尔曼滤波模型

  // 数据关联与跟踪函数
  void
  boxAssociation(std::vector<int>
                     &bestMatch); // 边界框数据关联（基于马氏距离和匈牙利算法）
  double computeMahalanobisDistance3D(
      const Eigen::Vector3d &posDiff,
      const Eigen::Matrix3d &covariance); // 计算3D马氏距离
  double compute3DIoU(const onboardDetector::box3D &box1,
                      const onboardDetector::box3D &box2); // 计算3D IoU
  double computeAssociationCost3D(
      const onboardDetector::box3D &predBox, const Eigen::Vector3d &predStd,
      const onboardDetector::box3D &measBox, const Eigen::Vector3d &measStd,
      const Eigen::Matrix3d &covariance); // 计算3D物体的关联代价
  void hungarianAlgorithm(const std::vector<std::vector<double>> &costMatrix,
                          std::vector<int> &assignment); // 匈牙利算法
  void removeDuplicateTracks();                          // 移除重复/重叠的轨迹
  void kalmanFilterAndUpdateHist(
      const std::vector<int> &bestMatch); // 卡尔曼滤波与更新历史

  // 可视化函数
  void getDynamicPc(std::vector<Eigen::Vector3d> &dynamicPc); // 获取动态点云
  void publishPoints(const std::vector<Eigen::Vector3d> &points,
                     const ros::Publisher &publisher); // 发布点云
  void publish3dBox(const std::vector<onboardDetector::box3D> &bboxes,
                    const ros::Publisher &publisher, double r, double g,
                    double b);    // 发布3D边界框
  void publishHistoryTraj();      // 发布历史轨迹
  void publishVelVis();           // 发布速度可视化信息
  void publishLidarClusters();    // 发布激光雷达聚类
  void publishFilteredPoints();   // 发布过滤后的点云
  void publishRawDynamicPoints(); // 发布原始动态点云

  // 用户接口函数
  void getDynamicObstacles(
      std::vector<onboardDetector::box3D> &incomeDynamicBBoxes,
      const Eigen::Vector3d &robotSize = Eigen::Vector3d(
          0.0, 0.0, 0.0)); // 获取动态障碍物（考虑机器人尺寸膨胀）
  void
  getDynamicObstaclesHist(std::vector<std::vector<Eigen::Vector3d>> &posHist,
                          std::vector<std::vector<Eigen::Vector3d>> &velHist,
                          std::vector<std::vector<Eigen::Vector3d>> &sizeHist,
                          const Eigen::Vector3d &robotSize = Eigen::Vector3d(
                              0.0, 0.0, 0.0)); // 获取动态障碍物的历史轨迹

  // 内联辅助函数
  void getLidarPose(const geometry_msgs::PoseStampedConstPtr &pose,
                    Eigen::Matrix4d &lidarPoseMatrix); // 获取激光雷达位姿
  void getLidarPose(const nav_msgs::OdometryConstPtr &odom,
                    Eigen::Matrix4d &lidarPoseMatrix); // 获取激光雷达位姿
  void convertCustomMsgToPointCloud2(
      const livox_ros_driver2::CustomMsgConstPtr &customMsg,
      sensor_msgs::PointCloud2 &cloud); // 将CustomMsg转换为PointCloud2
};

/*!
 * \brief 根据位姿信息计算激光雷达位姿矩阵（使用PoseStamped消息）
 * \param pose 机器人位姿信息
 * \param lidarPoseMatrix 输出参数，激光雷达的位姿矩阵
 */
inline void
dynamicDetector::getLidarPose(const geometry_msgs::PoseStampedConstPtr &pose,
                              Eigen::Matrix4d &lidarPoseMatrix) {
  Eigen::Quaterniond quat;
  quat = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                            pose->pose.orientation.y, pose->pose.orientation.z);
  Eigen::Matrix3d rot = quat.toRotationMatrix();

  // convert body pose to camera pose
  Eigen::Matrix4d map2body;
  map2body.setZero();
  map2body.block<3, 3>(0, 0) = rot;
  map2body(0, 3) = pose->pose.position.x;
  map2body(1, 3) = pose->pose.position.y;
  map2body(2, 3) = pose->pose.position.z;
  map2body(3, 3) = 1.0;

  lidarPoseMatrix = map2body * this->body2Lidar_;
}

/*!
 * \brief 根据位姿信息计算激光雷达位姿矩阵（使用Odometry消息）
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
