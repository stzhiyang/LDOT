/*
    FILE: lidarDetector.cpp
    ------------------
    class function definitions for lidar-based obstacle detector
*/
#include <ldot_detector/lidarDetector.h>
namespace onboardDetector{
    lidarDetector::lidarDetector(){
        this->eps_ = 0.5;
        this->minPts_ = 10;
        this->useAdaptive_ = false;
        this->distanceScale_ = 0.05;
        this->cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        this->sensorPosition_ = Eigen::Vector3d::Zero();
    }

    void lidarDetector::setParams(double eps, int minPts, bool useAdaptive, double distScale){
        this->eps_ = eps;
        this->minPts_ = minPts;
        this->useAdaptive_ = useAdaptive;
        this->distanceScale_ = distScale;
    }

    void lidarDetector::getPointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
        this->cloud_ = cloud;
    }

    void lidarDetector::setSensorPosition(const Eigen::Vector3d& position){
        this->sensorPosition_ = position;
    }

    /*
    FUNCTION: lidarDBSCAN
    ---------------------
    使用DBSCAN算法对输入点云进行聚类处理，并计算每个聚类的边界框
    
    主要步骤:
    1. 检查输入点云是否有效
    2. 将PCL点云数据转换为DBSCAN可处理的格式
    3. 执行DBSCAN聚类算法
    4. 统计聚类数量并组织聚类数据
    5. 计算每个聚类的质心、尺寸和边界框参数
    
    输出:
    - 更新clusters_成员变量：包含所有检测到的点云聚类
    - 更新bboxes_成员变量：包含所有聚类对应的3D边界框
    */
    void lidarDetector::lidarDBSCAN(){
        // 检查输入点云是否为空
        if(!cloud_ || cloud_->empty()){
            return;
        }
        
        // 将PCL点云数据转换为自定义Point结构体向量
        std::vector<Point> points;
        for(size_t i=0; i<cloud_->size(); ++i){
            Point p;
            p.x = cloud_->points[i].x;
            p.y = cloud_->points[i].y;
            p.z = cloud_->points[i].z;
            p.clusterID = UNCLASSIFIED;
            points.push_back(p);
        }

        // 创建并运行DBSCAN聚类算法（传入传感器位置用于自适应epsilon计算）
        DBSCAN dbscan(minPts_, eps_, points, useAdaptive_, distanceScale_,
                      static_cast<float>(sensorPosition_.x()),
                      static_cast<float>(sensorPosition_.y()),
                      static_cast<float>(sensorPosition_.z()));
        dbscan.run();  // 得到分好类的自定义Points

        // 统计聚类数量（查找最大的clusterID）
        int clusterNum = 0;
        for (size_t i=0; i<dbscan.m_points.size(); ++i){
            onboardDetector::Point pDB = dbscan.m_points[i];
            if (pDB.clusterID > clusterNum){
                clusterNum = pDB.clusterID;
            }
        }

        // 根据聚类结果构建Cluster对象向量
        std::vector<onboardDetector::Cluster> clustersTemp;
        clustersTemp.resize(clusterNum);
        
        // 这里说明了clusters_容器中点云簇的序号代表，其放置位置的索引，即按序号放入
        for(size_t i=0; i<dbscan.m_points.size(); ++i){
            if (dbscan.m_points[i].clusterID > 0){
                pcl::PointXYZ point;
                point.x = dbscan.m_points[i].x;
                point.y = dbscan.m_points[i].y;
                point.z = dbscan.m_points[i].z;
                clustersTemp[dbscan.m_points[i].clusterID-1].points->push_back(point);
                clustersTemp[dbscan.m_points[i].clusterID-1].cluster_id = dbscan.m_points[i].clusterID;
            }
        }
        this->clusters_ = clustersTemp;

        // 计算每个聚类的质心、尺寸并构造对应的3D边界框，&是引用，即操作clusters_
        std::vector<onboardDetector::box3D> bboxesTemp; 
        //因为是按序号操作clusters_的序号、位置处理的，所以bboxes_与clusters_的成员一一对应
        for(auto& cluster : this->clusters_){
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cluster.points, centroid);
            cluster.centroid = centroid;
            
            // 对于Z轴，使用统计方法过滤离群点以获得更鲁棒的高度估计
            // 收集所有Z坐标并排序
            std::vector<float> z_values;
            z_values.reserve(cluster.points->size());
            for(const auto& pt : cluster.points->points) {
                z_values.push_back(pt.z);
            }
            std::sort(z_values.begin(), z_values.end());
            
            // 使用百分位数方法：去除最高和最低5%的点（对于小聚类至少保留3个点）
            size_t n = z_values.size();
            size_t lower_idx = std::max(size_t(1), static_cast<size_t>(n * 0.1));
            size_t upper_idx = std::min(n - 1, static_cast<size_t>(n * 0.9));
            
            float z_min_robust = z_values[lower_idx];
            float z_max_robust = z_values[upper_idx];
            
            // 对于X和Y轴，仍使用传统的min/max方法（因为水平方向通常更可靠）
            pcl::PointXYZ minPt, maxPt;
            pcl::getMinMax3D(*cluster.points, minPt, maxPt);
            
            cluster.dimensions = Eigen::Vector3f(maxPt.x - minPt.x, maxPt.y - minPt.y, z_max_robust - z_min_robust);

            onboardDetector::box3D bbox;
            bbox.x = centroid(0);
            bbox.y = centroid(1);
            // Z坐标使用鲁棒估计的中心
            bbox.z = (z_min_robust + z_max_robust) / 2.0f;
            bbox.x_width = maxPt.x - minPt.x;
            bbox.y_width = maxPt.y - minPt.y;
            // Z高度使用鲁棒估计
            bbox.z_width = z_max_robust - z_min_robust;
            bbox.id = cluster.cluster_id;
            bboxesTemp.push_back(bbox);
        }
        this->bboxes_ = bboxesTemp;
    }

    std::vector<onboardDetector::Cluster>& lidarDetector::getClusters(){
        return this->clusters_;
    }

    std::vector<onboardDetector::box3D>& lidarDetector::getBBoxes(){
        return this->bboxes_;
    }
}
