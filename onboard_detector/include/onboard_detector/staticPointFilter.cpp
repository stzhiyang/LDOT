/*
    FILE: staticPointFilter.cpp
    ---------------------------------
    静态点滤波器实现
*/
#include <cmath>
#include <onboard_detector/staticPointFilter.h>
#include <onboard_detector/utils.h>

namespace onboardDetector {

StaticPointFilter::StaticPointFilter()
    : enabled_(false), voxel_size_(0.1), hit_threshold_(5),
      time_threshold_(5.0) {}

StaticPointFilter::~StaticPointFilter() {}

void StaticPointFilter::setParams(bool enabled, float voxel_size,
                                  int hit_threshold, double time_threshold) {
  enabled_ = enabled;
  voxel_size_ = voxel_size;
  hit_threshold_ = hit_threshold;
  time_threshold_ = time_threshold;
}

long long StaticPointFilter::getVoxelKey(const pcl::PointXYZ &point) {
  // 向下取整
  int x_idx = std::floor(point.x / voxel_size_);
  int y_idx = std::floor(point.y / voxel_size_);
  int z_idx = std::floor(point.z / voxel_size_);

  // 每个坐标的质数。为了下面进行异或运算和防止不同点云占据同一个体素格子，即哈希函数的运算
  const long long p1 = 73856093;
  const long long p2 = 19349663;
  const long long p3 = 83492791;

  // 这个就是哈希函数，给每一个小方块一个独一无二的 ID，方便快速查找
  return (long long)(x_idx * p1) ^ (long long)(y_idx * p2) ^
         (long long)(z_idx * p3);

  
}

bool StaticPointFilter::isPointInBox(const pcl::PointXYZ &pt,
                                     const onboardDetector::box3D &box) {
  // 计算点相对于box中心的位置
  float dx = pt.x - box.x;
  float dy = pt.y - box.y;
  float dz = pt.z - box.z;

  // 如果box有yaw旋转，将点转换到box的局部坐标系
  // 通过反向旋转(-yaw)将点从世界坐标系转到box坐标系
  if (std::abs(box.yaw) > 1e-6) {
    float cos_yaw = std::cos(-box.yaw);
    float sin_yaw = std::sin(-box.yaw);
    float dx_local = dx * cos_yaw - dy * sin_yaw;
    float dy_local = dx * sin_yaw + dy * cos_yaw;
    dx = dx_local;
    dy = dy_local;
  }

  // 在box局部坐标系中进行AABB检查
  float half_x = box.x_width / 2.0;
  float half_y = box.y_width / 2.0;
  float half_z = box.z_width / 2.0;

  if (std::abs(dx) <= half_x && std::abs(dy) <= half_y &&
      std::abs(dz) <= half_z) {
    return true;
  }
  return false;
}

//判断体素格子的命中次数，如果大于阈值为静态，返回true
bool StaticPointFilter::isPointStatic(const pcl::PointXYZ &pt) {
  long long key = getVoxelKey(pt);
  if (voxel_map_.find(key) != voxel_map_.end()) {
    return voxel_map_[key].hit_count > hit_threshold_;
  }
  return false;
}

void StaticPointFilter::updateMap(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double current_time) {
  if (cloud->empty()) {
    return;
  }

  for (const auto &point : cloud->points) {
    long long key = getVoxelKey(point);

    // 更新体素状态
    VoxelStatus &status = voxel_map_[key]; //讲哈希值存入一维哈希表
    status.last_seen_time = current_time;

    // 【关键优化】近距离体素累积抑制，防止动态物体被快速标记为静态
    // 计算点到传感器的距离（假设传感器在原点）
    double dist = std::sqrt(point.x * point.x + point.y * point.y);

    // 近距离（<3m）：降低累积速度，每3帧才累积1次
    // 这样可以防止近距离的动态物体（如人、车）被快速标记为静态
    bool should_increment = true;
    if (dist < 3.0) {
      // 使用模运算实现每3帧累积1次
      // 基于当前计数的模运算，确保不同体素有不同的累积节奏
      should_increment = (status.hit_count % 3 == 0);
    }

    // 增加体素格子命中计数，但进行截断以避免溢出
    if (should_increment && status.hit_count <= hit_threshold_ + 1) {
      status.hit_count++;
    }
  }

  // 定期清理地图
  cleanMap(current_time);
}

void StaticPointFilter::filterPoints(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const std::vector<onboardDetector::box3D> &protected_boxes) {
  if (!enabled_ || cloud->empty()) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  filtered_cloud->reserve(cloud->size());

  for (const auto &point : cloud->points) {
    // 检查是否为静态
    if (isPointStatic(point)) {
      bool protected_point = false;
      // 检查是否在保护区域内（历史轨迹为动态的区域）
      for (const auto &box : protected_boxes) {
        if (isPointInBox(point, box)) {
          protected_point = true;
          // 使用置信度衰减而非清零，避免动态物体离开后背景恢复过慢
          // 衰减1次可以平滑处理动静转换，同时保留部分历史信息
          long long key = getVoxelKey(point);
          if (voxel_map_.find(key) != voxel_map_.end()) {
            voxel_map_[key].hit_count =
                std::max(0, voxel_map_[key].hit_count - 1);
          }
          break;
        }
      }

      if (protected_point) {
        filtered_cloud->push_back(point);
      }
      // 否则，它是静态的且不在保护区内，移除它
    } else {
      // 非静态，保留
      filtered_cloud->push_back(point);
    }
  }

  // 用过滤后的点云替换原始点云
  *cloud = *filtered_cloud;
}

void StaticPointFilter::filterClusters(
    std::vector<onboardDetector::Cluster> &clusters,
    std::vector<onboardDetector::box3D> &bboxes, float static_ratio_threshold) {
  if (clusters.empty()) {
    return;
  }

  // 使用 erase-remove 惯用语同时过滤 clusters 和 bboxes
  // 由于我们需要保持两个 vector 的同步，我们手动遍历
  size_t write_idx = 0;
  for (size_t i = 0; i < clusters.size(); ++i) {
    const auto &cluster = clusters[i];
    const auto &bbox = bboxes[i];
    int static_points = 0;
    int total_points = cluster.points->size();

    if (total_points == 0)
      continue;

    for (const auto &pt : cluster.points->points) {
      if (isPointStatic(pt)) {
        static_points++;
      }
    }

    float ratio = (float)static_points / total_points;

    // 自适应阈值选择：根据簇的特征动态调整过滤阈值
    float adaptive_threshold = static_ratio_threshold;

    // 计算簇到传感器的距离（2D平面距离）
    double dist_to_sensor = std::sqrt(bbox.x * bbox.x + bbox.y * bbox.y);

    // 【关键策略】距离自适应：近距离动态物体优先保护
    // 近距离（<3m）：使用极严格的阈值，防止误判动态障碍物
    // 远距离（>8m）：使用宽松阈值，更激进地过滤背景
    if (dist_to_sensor < 3.0) {
      // 近距离（<3m）：大幅提高阈值0.3，极难被判定为静态（极度保守）
      // 0.6 + 0.3 = 0.9，需要90%以上的点是静态才会被过滤
      adaptive_threshold = std::min(0.95f, static_ratio_threshold + 0.3f);
    } else if (dist_to_sensor > 8.0) {
      // 远距离（>8m）：降低阈值0.15，更容易被判定为静态（更激进）
      adaptive_threshold = std::max(0.35f, static_ratio_threshold - 0.15f);
    }

    // 策略2：大尺寸物体（可能是建筑物、墙壁等）使用更宽松的阈值
    // 但近距离的大物体（如停着的车）不应被过滤
    float max_dim = std::max({bbox.x_width, bbox.y_width, bbox.z_width});
    if (max_dim > 2.0 && dist_to_sensor > 3.0) {
      // 只对远距离的大物体降低阈值
      adaptive_threshold = std::max(0.3f, adaptive_threshold - 0.15f);
    }

    // 策略3：如果bbox有速度信息且速度很小，且距离较远时才使用宽松阈值
    // 近距离的低速物体（如人站立）不应被误判为静态
    double velocity = std::sqrt(bbox.Vx * bbox.Vx + bbox.Vy * bbox.Vy);
    if (velocity < 0.1 && dist_to_sensor > 3.0) {
      // 只对远距离的低速物体降低阈值
      adaptive_threshold = std::max(0.3f, adaptive_threshold - 0.1f);
    }

    // 如果静态点比例低于（自适应）阈值，则保留该聚类
    if (ratio <= adaptive_threshold) {
      if (write_idx != i) {
        clusters[write_idx] = clusters[i];
        bboxes[write_idx] = bboxes[i];
      }
      write_idx++;
    }
    // 否则（ratio > threshold），该聚类被视为静态背景，被"跳过"（移除）
  }

  // 调整大小以移除末尾的元素
  clusters.resize(write_idx);
  bboxes.resize(write_idx);
}

void StaticPointFilter::clearDynamicRegions(
    const std::vector<onboardDetector::box3D> &dynamic_boxes) {
  if (dynamic_boxes.empty()) {
    return;
  }

  for (const auto &box : dynamic_boxes) {
    // 计算需要遍历的体素范围
    // 基于box尺寸计算搜索半径（向外扩展1个体素以确保覆盖）
    float max_half_width =
        std::max({box.x_width, box.y_width, box.z_width}) / 2.0f;
    int range = std::ceil(max_half_width / voxel_size_) + 1;

    // 生成box中心点
    pcl::PointXYZ center;
    center.x = box.x;
    center.y = box.y;
    center.z = box.z;

    // 遍历box周围的体素网格
    for (int dx = -range; dx <= range; ++dx) {
      for (int dy = -range; dy <= range; ++dy) {
        for (int dz = -range; dz <= range; ++dz) {
          // 计算体素中心点
          pcl::PointXYZ voxel_center;
          voxel_center.x = center.x + dx * voxel_size_;
          voxel_center.y = center.y + dy * voxel_size_;
          voxel_center.z = center.z + dz * voxel_size_;

          // 检查体素中心是否在box内
          if (isPointInBox(voxel_center, box)) {
            long long key = getVoxelKey(voxel_center);
            auto it = voxel_map_.find(key);

            if (it != voxel_map_.end()) {
              // 降低计数而非清零（平滑处理，保留部分历史信息）
              // 每次降低3，比保护区域的衰减（-1）更激进
              it->second.hit_count = std::max(0, it->second.hit_count - 3);

              // 如果计数已经很低，直接删除该体素以释放内存
              if (it->second.hit_count == 0) {
                voxel_map_.erase(it);
              }
            }
          }
        }
      }
    }
  }
}

void StaticPointFilter::cleanMap(double current_time) {
  for (auto it = voxel_map_.begin(); it != voxel_map_.end();) {
    if (current_time - it->second.last_seen_time > time_threshold_) {
      it = voxel_map_.erase(it);
    } else {
      ++it;
    }
  }
}

} // namespace onboardDetector
