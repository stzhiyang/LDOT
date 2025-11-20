/*
    FILE: dbscan.h
    ------------------
    helper class function definitions for dbscan
    文件：dbscan.h
    ------------------
    DBSCAN 辅助类的函数定义
*/
#include <onboard_detector/dbscan.h>
#include <iostream>

namespace onboardDetector{
    // 运行DBSCAN聚类算法
    int DBSCAN::run()
    {
        // 从1开始初始化聚类ID
        int clusterID = 1;
        vector<Point>::iterator iter;
        // 遍历所有点
        for(iter = m_points.begin(); iter != m_points.end(); ++iter)
        {
            // 如果点尚未被分类
            if ( iter->clusterID == UNCLASSIFIED )
            {
                // 尝试从该点开始扩展一个新的聚类
                if ( expandCluster(*iter, clusterID) != FAILURE )
                {
                    // 如果成功，为下一个聚类准备新的ID
                    clusterID += 1;
                }
            }
        }

        return 0;
    }

    // 从一个核心点扩展一个聚类
    int DBSCAN::expandCluster(Point point, int clusterID)
    {    
        // 找到当前点的邻域内的所有点（作为种子点）
        vector<int> clusterSeeds = calculateCluster(point);

        // 如果邻域内的点数小于m_minPoints，则该点不是核心点，可能为噪声点
        if ( clusterSeeds.size() < m_minPoints )
        {
            point.clusterID = NOISE;
            return FAILURE;
        }
        else
        {
            // 将所有种子点分配给当前聚类
            int index = 0, indexCorePoint = 0;
            vector<int>::iterator iterSeeds;
            for( iterSeeds = clusterSeeds.begin(); iterSeeds != clusterSeeds.end(); ++iterSeeds)
            {
                m_points.at(*iterSeeds).clusterID = clusterID; // m_points中找到*iterSeeds
                // 找到核心点在种子点列表中的索引
                if (m_points.at(*iterSeeds).x == point.x && m_points.at(*iterSeeds).y == point.y && m_points.at(*iterSeeds).z == point.z )
                {
                    indexCorePoint = index;
                }
                ++index;
            }
            // 从种子点列表中移除核心点自身，避免重复处理
            clusterSeeds.erase(clusterSeeds.begin()+indexCorePoint);

            // 遍历所有种子点，继续扩展聚类
            for( vector<int>::size_type i = 0, n = clusterSeeds.size(); i < n; ++i )
            {
                // 找到当前种子点的邻域
                vector<int> clusterNeighors = calculateCluster(m_points.at(clusterSeeds[i]));

                // 如果这个种子点也是一个核心点
                if ( clusterNeighors.size() >= m_minPoints )
                {
                    vector<int>::iterator iterNeighors;
                    for ( iterNeighors = clusterNeighors.begin(); iterNeighors != clusterNeighors.end(); ++iterNeighors )
                    {
                        // 如果邻域中的点是未分类或噪声点
                        if ( m_points.at(*iterNeighors).clusterID == UNCLASSIFIED || m_points.at(*iterNeighors).clusterID == NOISE )
                        {
                            // 如果是未分类的点，则将其添加到种子列表中以供后续扩展
                            if ( m_points.at(*iterNeighors).clusterID == UNCLASSIFIED )
                            {
                                clusterSeeds.push_back(*iterNeighors);
                                n = clusterSeeds.size();
                            }
                            // 将该邻域点分配给当前聚类
                            m_points.at(*iterNeighors).clusterID = clusterID;
                        }
                    }
                }
            }

            return SUCCESS;
        }
    }

    // 计算并返回一个点在其epsilon邻域内的所有点的索引指针地址
    vector<int> DBSCAN::calculateCluster(Point point)
    {
        int index = 0;
        vector<Point>::iterator iter;
        vector<int> clusterIndex;
        // 获取当前点的自适应epsilon值
        double adaptiveEps = getAdaptiveEpsilon(point);
        // 遍历所有点
        for( iter = m_points.begin(); iter != m_points.end(); ++iter)
        {
            // 如果两个点之间的距离（的平方）小于或等于epsilon（的平方）
            if ( calculateDistance(point, *iter) <= adaptiveEps )
            {
                // 将该点的索引添加到邻域索引列表中
                clusterIndex.push_back(index);
            }
            index++;
        }
        return clusterIndex;
    }

    // 计算两个点之间的欧氏距离的平方
    inline double DBSCAN::calculateDistance(const Point& pointCore, const Point& pointTarget )
    {
        return pow(pointCore.x - pointTarget.x,2)+pow(pointCore.y - pointTarget.y,2)+pow(pointCore.z - pointTarget.z,2);
    }

    // 计算点到原点（传感器位置）的距离
    inline double DBSCAN::calculateDistanceToOrigin(const Point& point)
    {
        return sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
    }

    // 获取基于距离的自适应epsilon值
    // 原理：距离传感器越远的点，点云密度越稀疏，需要更大的epsilon
    inline double DBSCAN::getAdaptiveEpsilon(const Point& point)
    {
        if (!m_useAdaptiveEps) {
            // 如果不使用自适应epsilon，返回基础epsilon的平方（因为calculateDistance返回距离的平方）
            return m_epsilon * m_epsilon;
        }
        
        // 计算点到原点的距离
        double distToOrigin = calculateDistanceToOrigin(point);
        
        // 自适应epsilon = 基础epsilon + 距离 * 缩放因子
        // 这样可以根据点的深度动态调整邻域半径
        double adaptiveEps = m_epsilon + distToOrigin * m_distanceScale;
        
        // 返回平方值，因为calculateDistance返回距离的平方
        return adaptiveEps * adaptiveEps;
    }
}