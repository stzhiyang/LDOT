/*
    FILE: dbscan.h
    ------------------
    helper class function definitions for dbscan
    文件：dbscan.h
    ------------------
    DBSCAN 辅助类的函数定义
*/
#include <ldot_detector/dbscan.h>
#include <iostream>
#include <algorithm>

namespace onboardDetector{
    // 构建KD-Tree用于快速邻域搜索
    void DBSCAN::buildKdTree()
    {
        m_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        m_cloud->width = m_pointSize;
        m_cloud->height = 1;
        m_cloud->points.resize(m_pointSize);
        
        for (size_t i = 0; i < m_pointSize; ++i)
        {
            m_cloud->points[i].x = m_points[i].x;
            m_cloud->points[i].y = m_points[i].y;
            m_cloud->points[i].z = m_points[i].z;
        }
        
        m_kdtree.setInputCloud(m_cloud);
    }

    // 使用KD-Tree进行邻域搜索（优化版本）
    vector<int> DBSCAN::calculateClusterKdTree(Point point, int pointIdx)
    {
        vector<int> clusterIndex;
        vector<int> pointIdxRadiusSearch;
        vector<float> pointRadiusSquaredDistance;
        
        // 获取当前点的自适应epsilon值
        double adaptiveEps = getAdaptiveEpsilon(point);
        double searchRadius = sqrt(adaptiveEps);  // KD-Tree使用实际距离，不是距离平方
        
        pcl::PointXYZ searchPoint;
        searchPoint.x = point.x;
        searchPoint.y = point.y;
        searchPoint.z = point.z;
        
        // 使用KD-Tree进行半径搜索，时间复杂度O(log N)
        if (m_kdtree.radiusSearch(searchPoint, searchRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
        {
            clusterIndex = pointIdxRadiusSearch;
        }
        
        return clusterIndex;
    }

    // 使用KD-Tree优化的expandCluster函数
    int DBSCAN::expandClusterKdTree(int pointIdx, int clusterID)
    {
        Point point = m_points[pointIdx];
        
        // 使用KD-Tree找到当前点的邻域内的所有点（作为种子点）
        vector<int> clusterSeeds = calculateClusterKdTree(point, pointIdx);

        // 如果邻域内的点数小于m_minPoints，则该点不是核心点，可能为噪声点
        if (clusterSeeds.size() < m_minPoints)
        {
            m_points[pointIdx].clusterID = NOISE;
            return FAILURE;
        }
        else
        {
            // 将所有种子点分配给当前聚类
            for (int idx : clusterSeeds)
            {
                m_points[idx].clusterID = clusterID;
            }
            
            // 从种子点列表中移除核心点自身，避免重复处理
            clusterSeeds.erase(std::remove(clusterSeeds.begin(), clusterSeeds.end(), pointIdx), clusterSeeds.end());

            // 遍历所有种子点，继续扩展聚类
            for (size_t i = 0; i < clusterSeeds.size(); ++i)
            {
                int seedIdx = clusterSeeds[i];
                
                // 使用KD-Tree找到当前种子点的邻域
                vector<int> clusterNeighbors = calculateClusterKdTree(m_points[seedIdx], seedIdx);

                // 如果这个种子点也是一个核心点
                if (clusterNeighbors.size() >= m_minPoints)
                {
                    for (int neighborIdx : clusterNeighbors)
                    {
                        // 如果邻域中的点是未分类或噪声点
                        if (m_points[neighborIdx].clusterID == UNCLASSIFIED || 
                            m_points[neighborIdx].clusterID == NOISE)
                        {
                            // 如果是未分类的点，则将其添加到种子列表中以供后续扩展
                            if (m_points[neighborIdx].clusterID == UNCLASSIFIED)
                            {
                                clusterSeeds.push_back(neighborIdx);
                            }
                            // 将该邻域点分配给当前聚类
                            m_points[neighborIdx].clusterID = clusterID;
                        }
                    }
                }
            }

            return SUCCESS;
        }
    }

    // 运行DBSCAN聚类算法（KD-Tree优化版本）
    int DBSCAN::run()
    {
        // 从1开始初始化聚类ID
        int clusterID = 1;
        
        // 遍历所有点（使用索引而不是迭代器以配合KD-Tree）
        for (size_t i = 0; i < m_points.size(); ++i)
        {
            // 如果点尚未被分类
            if (m_points[i].clusterID == UNCLASSIFIED)
            {
                // 尝试从该点开始扩展一个新的聚类（使用KD-Tree优化版本）
                if (expandClusterKdTree(i, clusterID) != FAILURE)
                {
                    // 如果成功，为下一个聚类准备新的ID
                    clusterID += 1;
                }
            }
        }

        return 0;
    }

    // 计算点到传感器的距离（点云在全局坐标系下）
    inline double DBSCAN::calculateDistanceToSensor(const Point& point)
    {
        double dx = point.x - m_sensorX;
        double dy = point.y - m_sensorY;
        double dz = point.z - m_sensorZ;
        return sqrt(dx * dx + dy * dy + dz * dz);
    }

    // 获取基于距离的自适应epsilon值
    // 原理：距离传感器越远的点，点云密度越稀疏，需要更大的epsilon
    inline double DBSCAN::getAdaptiveEpsilon(const Point& point)
    {
        if (!m_useAdaptiveEps) {
            // 如果不使用自适应epsilon，返回基础epsilon的平方（因为calculateDistance返回距离的平方）
            return m_epsilon * m_epsilon;
        }
        
        // 计算点到传感器的距离（在全局坐标系下）
        double distToSensor = calculateDistanceToSensor(point);
        
        // 自适应epsilon = 基础epsilon + 距离 * 缩放因子
        // 这样可以根据点的深度动态调整邻域半径
        double adaptiveEps = m_epsilon + distToSensor * m_distanceScale;
        
        // 返回平方值，因为calculateDistance返回距离的平方
        return adaptiveEps * adaptiveEps;
    }
}