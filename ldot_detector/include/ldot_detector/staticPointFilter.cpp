/*
    FILE: staticPointFilter.cpp
    ---------------------------------
    静态点滤波器实现
*/
#include <cmath>
#include <ldot_detector/staticPointFilter.h>
#include <ldot_detector/utils.h>

namespace onboardDetector {

StaticPointFilter::StaticPointFilter()
    : enabled_(false), voxel_size_(0.1), hit_threshold_(5),
      time_threshold_(5.0), use_neighbor_voting_(true), min_neighbor_votes_(4),
      frame_count_(0), sensor_position_(Eigen::Vector3d::Zero()) {}

StaticPointFilter::~StaticPointFilter() {}

void StaticPointFilter::setParams(bool enabled, float voxel_size,
                                  int hit_threshold, double time_threshold,
                                  bool use_neighbor_voting,
                                  int min_neighbor_votes) {
  enabled_ = enabled;
  voxel_size_ = voxel_size;
  hit_threshold_ = hit_threshold;
  time_threshold_ = time_threshold;
  use_neighbor_voting_ = use_neighbor_voting;
  min_neighbor_votes_ = min_neighbor_votes;
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

// 判断体素格子的命中次数，如果大于阈值为静态，返回true
// 注意：此函数依赖 sensor_position_ 成员变量，需要先调用 updateMap 更新传感器位置
bool StaticPointFilter::isPointStatic(const pcl::PointXYZ &pt) {
  // 如果启用邻域投票，使用增强版判断
  if (use_neighbor_voting_) {
    return isPointStaticWithNeighbors(pt);
  }
  
  // 原始逻辑：使用距离自适应阈值
  long long key = getVoxelKey(pt);
  if (voxel_map_.find(key) != voxel_map_.end()) {
    int adaptive_thresh = getAdaptiveThreshold(pt);
    return voxel_map_[key].hit_count > adaptive_thresh;
  }
  return false;
}

// 【新增】距离自适应阈值 - 远距离降低判定门槛，补偿点云稀疏性
// 使用成员变量 sensor_position_ 计算点到传感器的距离
int StaticPointFilter::getAdaptiveThreshold(const pcl::PointXYZ &pt) {
  // 计算点到传感器的2D距离（在全局坐标系下）
  double dx = pt.x - sensor_position_.x();
  double dy = pt.y - sensor_position_.y();
  double dist = std::sqrt(dx * dx + dy * dy);
  
  if (dist < 5.0) {
    // 近距离（<5m）：使用标准阈值，保持严格判定
    return hit_threshold_;
  } else if (dist < 10.0) {
    // 中距离（5-10m）：降低1，适度放宽
    return std::max(2, hit_threshold_ - 1);
  } else if (dist < 20.0) {
    // 远距离（10-20m）：降低2，补偿稀疏性
    return std::max(2, hit_threshold_ - 2);
  } else {
    // 极远距离（>20m）：降低3，最小为2
    return std::max(2, hit_threshold_ - 3);
  }
}

// 【新增】带邻域投票的静态点判断 - 利用空间连续性
// 核心思想：稀疏点云中，单个体素可能累积不够，但如果周围邻居都是静态的，
// 则该点大概率也是静态的（空间连续性假设）
// 使用成员变量 sensor_position_ 计算距离
bool StaticPointFilter::isPointStaticWithNeighbors(const pcl::PointXYZ &pt) {
  long long key = getVoxelKey(pt);
  int adaptive_thresh = getAdaptiveThreshold(pt);
  
  // 1. 首先检查自身
  int self_hits = 0;
  auto self_it = voxel_map_.find(key);
  if (self_it != voxel_map_.end()) {
    self_hits = self_it->second.hit_count;
  }
  
  // 如果自身已经达到阈值，直接返回静态
  if (self_hits > adaptive_thresh) {
    return true;
  }
  
  // 2. 自身未达标，检查邻域投票
  // 只有当自身有一定累积（至少达到阈值的1/2）时才考虑邻域投票
  if (self_hits < adaptive_thresh / 2) {
    return false;
  }
  
  // 3. 6邻域投票（上下左右前后，不含对角线以减少计算量）
  int neighbor_votes = 0;
  
  // 6个方向的偏移
  float offsets[6][3] = {
    {voxel_size_, 0, 0}, {-voxel_size_, 0, 0},
    {0, voxel_size_, 0}, {0, -voxel_size_, 0},
    {0, 0, voxel_size_}, {0, 0, -voxel_size_}
  };
  
  for (int i = 0; i < 6; ++i) {
    pcl::PointXYZ neighbor;
    neighbor.x = pt.x + offsets[i][0];
    neighbor.y = pt.y + offsets[i][1];
    neighbor.z = pt.z + offsets[i][2];
    
    long long nkey = getVoxelKey(neighbor);
    auto it = voxel_map_.find(nkey);
    if (it != voxel_map_.end()) {
      // 邻居必须是真正的静态点（达到阈值）才能投票
      // 避免"半静态"的邻居互相抬轿子
      if (it->second.hit_count > adaptive_thresh) {
        neighbor_votes++;
        // 提前退出：如果已经达到投票阈值，无需继续检查
        if (neighbor_votes >= min_neighbor_votes_) {
          // 4. 综合判断：自身接近达标 且 邻域投票支持
          if (self_hits > (adaptive_thresh * 0.67)) {
            return true;
          }
        }
      }
    }
  }
  
  return false;
}

void StaticPointFilter::updateMap(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double current_time,
    const Eigen::Vector3d &sensor_position) {
  if (cloud->empty()) {
    return;
  }

  // 更新传感器位置（供后续 isPointStatic 等函数使用）
  sensor_position_ = sensor_position;

  for (const auto &point : cloud->points) {
    long long key = getVoxelKey(point);

    // 更新体素状态
    VoxelStatus &status = voxel_map_[key]; // 将哈希值存入一维哈希表
    status.last_seen_time = current_time;

    // 【关键优化】近距离体素累积抑制，防止动态物体被快速标记为静态
    // 计算点到传感器的2D距离（在全局坐标系下）
    double dx = point.x - sensor_position.x();
    double dy = point.y - sensor_position.y();
    double dist = dx * dx + dy * dy;

    // 近距离（<3m）：降低累积速度，每2帧才累积1次
    // 这样可以防止近距离的动态物体（如人、车）被快速标记为静态
    bool should_increment = true;
    if (dist < 4.0) {
      // 基于当前计数的模运算，确保不同体素有不同的累积节奏
      should_increment = (status.hit_count % 2 == 0);
    }

    // 增加体素格子命中计数，但进行截断以避免溢出
    if (should_increment && status.hit_count <= hit_threshold_ + 1) {
      status.hit_count++;
    }
  }

  // 每10帧清理一次地图
  frame_count_++;
  if (frame_count_ >= 10) {
    cleanMap(current_time);
    frame_count_ = 0;
  }
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
    std::vector<onboardDetector::box3D> &bboxes, float static_ratio_threshold,
    const std::vector<onboardDetector::box3D> &protected_boxes) {
  if (clusters.empty()) {
    return;
  }

  // 使用 erase-remove 惯用语同时过滤 clusters 和 bboxes
  // 由于我们需要保持两个 vector 的同步，我们手动遍历
  size_t write_idx = 0;
  for (size_t i = 0; i < clusters.size(); ++i) {
    const auto &cluster = clusters[i];
    const auto &bbox = bboxes[i];
    int total_points = cluster.points->size();

    if (total_points == 0)
      continue;

    // 【保护区域检查】如果簇在保护区域内，强制保留（提前检查，避免不必要的静态点计算）
    bool is_protected = false;
    pcl::PointXYZ bbox_center(bbox.x, bbox.y, bbox.z);
    for (const auto &protected_box : protected_boxes) {
      if (isPointInBox(bbox_center, protected_box)) {
        is_protected = true;
        break;
      }
    }

    // 如果在保护区域内，直接保留，跳过后续的静态阈值判断
    if (is_protected) {
      if (write_idx != i) {
        clusters[write_idx] = clusters[i];
        bboxes[write_idx] = bboxes[i];
      }
      write_idx++;
      continue;
    }

    // 计算静态点数量
    int static_points = 0;
    for (const auto &pt : cluster.points->points) {
      if (isPointStatic(pt)) {
        static_points++;
      }
    }
    float ratio = (float)static_points / total_points;

    // ==================== 简化的自适应阈值策略 ====================
    // 只保留影响最大的因素：距离和几何特征

    // 计算簇到传感器的距离（2D平面距离，在全局坐标系下）
    double dx = bbox.x - sensor_position_.x();
    double dy = bbox.y - sensor_position_.y();
    double dist_to_sensor = std::sqrt(dx * dx + dy * dy);

    // 基础阈值
    float adaptive_threshold = static_ratio_threshold;

    // -------------------- 策略1：距离自适应 --------------------
    // 距离越近，保护系数越高（阈值越高，越难被过滤）
    if (dist_to_sensor < 3.0) {
      // 极近距离（<3m）：最大保护
      adaptive_threshold += 0.15f;
    } else if (dist_to_sensor < 5.0) {
      // 近距离（3-5m）：线性过渡
      adaptive_threshold += 0.15f * (5.0f - (float)dist_to_sensor) / 2.0f;
    } else if (dist_to_sensor > 10.0) {
      // 远距离（>10m）：更激进过滤
      adaptive_threshold -= 0.15f * std::min(1.0f, (float)(dist_to_sensor - 10.0) / 10.0f);
    }

    // -------------------- 策略2：几何特征自适应（简化版） --------------------
    float max_dim = std::max({bbox.x_width, bbox.y_width, bbox.z_width});
    float min_dim = std::min({bbox.x_width, bbox.y_width, bbox.z_width});
    float volume = bbox.x_width * bbox.y_width * bbox.z_width;

    // 行人高度范围（0.5-2.2m）+ 近距离：增加保护
    if (bbox.z_width > 0.5f && bbox.z_width < 2.2f && dist_to_sensor < 5.0) {
      adaptive_threshold += 0.1f;
    }

    // 细长高大物体（如电线杆、树干）：更容易过滤
    float aspect_ratio = max_dim / (min_dim + 0.01f);
    if (aspect_ratio > 5.0f && max_dim > 2.0f) {
      adaptive_threshold -= 0.1f;
    }

    // 远距离大体积物体：更激进过滤
    if (volume > 24.0f && dist_to_sensor > 8.0) {
      adaptive_threshold -= 0.15f;
    }

    // 限制在合理范围内 [0.25, 0.95]
    adaptive_threshold = std::max(0.25f, std::min(0.90f, adaptive_threshold));

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

void StaticPointFilter::cleanMap(double current_time) {
  for (auto it = voxel_map_.begin(); it != voxel_map_.end();) {
    if (current_time - it->second.last_seen_time > time_threshold_) {
      it = voxel_map_.erase(it);
    } else {
      ++it;
    }
  }
}

bool StaticPointFilter::checkBoxCollision(const Eigen::Vector3d &center,
                                          const Eigen::Vector3d &size,
                                          double inflation) {
  // 计算膨胀后的半尺寸
  double half_x = (size.x() / 2.0) + inflation;
  double half_y = (size.y() / 2.0) + inflation;
  double half_z = (size.z() / 2.0) + inflation;

  // 计算需要检查的体素范围
  int x_range = static_cast<int>(std::ceil(half_x / voxel_size_)) + 1;
  int y_range = static_cast<int>(std::ceil(half_y / voxel_size_)) + 1;
  int z_range = static_cast<int>(std::ceil(half_z / voxel_size_)) + 1;

  // 遍历包围框范围内的所有体素
  for (int dx = -x_range; dx <= x_range; ++dx) {
    for (int dy = -y_range; dy <= y_range; ++dy) {
      for (int dz = -z_range; dz <= z_range; ++dz) {
        // 计算体素中心点
        pcl::PointXYZ voxel_center;
        voxel_center.x = static_cast<float>(center.x() + dx * voxel_size_);
        voxel_center.y = static_cast<float>(center.y() + dy * voxel_size_);
        voxel_center.z = static_cast<float>(center.z() + dz * voxel_size_);

        // 检查体素中心是否在膨胀后的包围框内
        double local_x = std::abs(voxel_center.x - center.x());
        double local_y = std::abs(voxel_center.y - center.y());
        double local_z = std::abs(voxel_center.z - center.z());

        if (local_x <= half_x && local_y <= half_y && local_z <= half_z) {
          // 体素在包围框内，检查是否为静态体素
          if (isPointStatic(voxel_center)) {
            // 发现碰撞，立即返回
            return true;
          }
        }
      }
    }
  }

  // 未发现碰撞
  return false;
}

} // namespace onboardDetector
