/*
    FILE: lidarDetector.cpp
    ------------------
    class function definitions for lidar-based obstacle detector
*/
#include <onboard_detector/lidarDetector.h>
namespace onboardDetector{
    lidarDetector::lidarDetector(){
        this->eps_ = 0.5;
        this->minPts_ = 10;
        this->cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    }

    void lidarDetector::setParams(double eps, int minPts){
        this->eps_ = eps;
        this->minPts_ = minPts;
    }

    void lidarDetector::getPointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
        this->cloud_ = cloud;
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

        // 创建并运行DBSCAN聚类算法
        DBSCAN dbscan(minPts_, eps_, points);
        dbscan.run();

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

        // 计算每个聚类的质心、尺寸并构造对应的3D边界框
        std::vector<onboardDetector::box3D> bboxesTemp;
        for(auto& cluster : this->clusters_){
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cluster.points, centroid);
            cluster.centroid = centroid;
            pcl::PointXYZ minPt, maxPt;
            pcl::getMinMax3D(*cluster.points, minPt, maxPt);
            cluster.dimensions = Eigen::Vector3f(maxPt.x - minPt.x, maxPt.y - minPt.y, maxPt.z - minPt.z); 

            onboardDetector::box3D bbox;
            bbox.x = centroid(0);
            bbox.y = centroid(1);
            bbox.z = centroid(2);
            bbox.x_width = maxPt.x - minPt.x;
            bbox.y_width = maxPt.y - minPt.y;
            bbox.z_width = maxPt.z - minPt.z;
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
