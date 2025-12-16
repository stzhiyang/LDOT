/*
    FILE: staticPointFilter.h
    ---------------------------------
    静态点滤波器头文件
*/
#ifndef ONBOARDDETECTOR_STATICPOINTFILTER_H
#define ONBOARDDETECTOR_STATICPOINTFILTER_H

#include <Eigen/Dense>
#include <ldot_detector/lidarDetector.h>
#include <ldot_detector/utils.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <unordered_map>

namespace onboardDetector {

// 体素状态结构体
struct VoxelStatus {
  int hit_count;
  double last_seen_time;

  VoxelStatus() : hit_count(0), last_seen_time(0.0) {}
};

class StaticPointFilter {
public:
  StaticPointFilter();
  ~StaticPointFilter();

  // 设置参数
  void setParams(bool enabled, float voxel_size, int hit_threshold,
                 double time_threshold, bool use_neighbor_voting = true,
                 int min_neighbor_votes = 3);

  // 仅更新地图
  // sensor_position: 传感器在全局坐标系中的位置（用于近距离累积抑制）
  void updateMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                 double current_time,
                 const Eigen::Vector3d &sensor_position);

  // 点级过滤 (原 filter 函数)
  void filterPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                    const std::vector<onboardDetector::box3D> &protected_boxes =
                        std::vector<onboardDetector::box3D>());

  // 聚类级过滤
  // 注意：使用成员变量 sensor_position_，需要先调用 updateMap 更新传感器位置
  void
  filterClusters(std::vector<onboardDetector::Cluster> &clusters,
                 std::vector<onboardDetector::box3D> &bboxes,
                 float static_ratio_threshold,
                 const std::vector<onboardDetector::box3D> &protected_boxes =
                     std::vector<onboardDetector::box3D>());

  // 清理动态物体历史轨迹区域的体素（动态反哺机制）
  void
  clearDynamicRegions(const std::vector<onboardDetector::box3D> &dynamic_boxes);

  // 加速恢复静态区域的体素（当动态物体回退为静态时调用）
  // 主动增加该区域体素的hit_count，使其更快被标记为静态
  void boostStaticRegions(const std::vector<onboardDetector::box3D> &static_boxes);

  // 清理旧体素
  void cleanMap(double current_time);

  // 碰撞检测接口
  // 检查单个点是否与静态地图碰撞
  bool checkCollision(const Eigen::Vector3d &point);

  // 检查带尺寸的包围框是否与静态地图碰撞
  // center: 包围框中心位置
  // size: 包围框尺寸 (x_width, y_width, z_width)
  // inflation: 膨胀系数（米），用于安全裕度
  bool checkBoxCollision(const Eigen::Vector3d &center,
                         const Eigen::Vector3d &size, double inflation = 0.0);

private:
  // 计算体素键值的辅助函数
  long long getVoxelKey(const pcl::PointXYZ &point);

  // 检查点是否在边界框内
  bool isPointInBox(const pcl::PointXYZ &pt, const onboardDetector::box3D &box);

  // 检查点是否为静态 (基于当前地图) - 原始版本
  bool isPointStatic(const pcl::PointXYZ &pt);

  // 【新增】距离自适应阈值 - 远距离降低判定门槛
  // 使用成员变量 sensor_position_ 计算距离
  int getAdaptiveThreshold(const pcl::PointXYZ &pt);

  // 【新增】带邻域投票的静态点判断 - 利用空间连续性
  // 使用成员变量 sensor_position_ 计算距离
  bool isPointStaticWithNeighbors(const pcl::PointXYZ &pt);

  bool enabled_;
  float voxel_size_;
  int hit_threshold_;
  double time_threshold_;

  // 【新增】邻域投票相关参数
  bool use_neighbor_voting_;      // 是否启用邻域投票
  int min_neighbor_votes_;        // 最小邻域投票数

  // 帧计数器，用于控制清理频率
  int frame_count_;

  // 体素地图：键值 -> 状态
  std::unordered_map<long long, VoxelStatus> voxel_map_;

  // 【新增】传感器位置（全局坐标系），用于距离计算
  // 在 updateMap 时更新，供 isPointStatic 等函数使用
  Eigen::Vector3d sensor_position_;
};

} // namespace onboardDetector

#endif // ONBOARDDETECTOR_STATICPOINTFILTER_H
