#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试 ROS 服务预测结果可视化脚本

该脚本用于：
1. 调用 GetDynamicObstacles 服务获取动态障碍物信息
2. 可视化预测位置和协方差椭圆（表达预测不确定性）
"""

import rospy
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from onboard_detector.srv import GetDynamicObstacles, GetDynamicObstaclesRequest


class ServiceVisualizationTester:
    """动态障碍物服务测试与可视化类"""
    
    def __init__(self):
        rospy.init_node('service_visualization_tester', anonymous=True)
        
        # 参数配置
        self.service_name = rospy.get_param('~service_name', '/onboard_detector/get_dynamic_obstacles')
        self.query_range = rospy.get_param('~query_range', 10.0)  # 查询范围（米）
        self.query_rate = rospy.get_param('~query_rate', 5.0)  # 查询频率（Hz）
        self.frame_id = rospy.get_param('~frame_id', 'map')  # 坐标系名称
        self.current_position = [
            rospy.get_param('~current_x', 0.0),
            rospy.get_param('~current_y', 0.0),
            rospy.get_param('~current_z', 0.0)
        ]
        
        # 可视化发布器
        self.marker_pub = rospy.Publisher(
            '~visualization_markers', 
            MarkerArray, 
            queue_size=10
        )
        
        # 等待服务可用
        rospy.loginfo(f"等待服务 {self.service_name} ...")
        try:
            rospy.wait_for_service(self.service_name, timeout=10.0)
            self.service_proxy = rospy.ServiceProxy(self.service_name, GetDynamicObstacles)
            rospy.loginfo("服务连接成功！")
        except rospy.ROSException:
            rospy.logerr(f"服务 {self.service_name} 不可用，请确保检测器节点已启动")
            raise
        
        # 定时器
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.query_rate), 
            self.timer_callback
        )
        
        rospy.loginfo("可视化测试节点已启动")
    
    def timer_callback(self, event):
        """定时器回调：查询服务并发布可视化"""
        try:
            # 构建请求
            req = GetDynamicObstaclesRequest()
            req.current_position = Point(
                x=self.current_position[0],
                y=self.current_position[1],
                z=self.current_position[2]
            )
            req.range = self.query_range
            
            # 调用服务
            resp = self.service_proxy(req)
            
            # 发布可视化
            self.publish_visualization(resp)
            
            # 打印统计信息
            num_obstacles = len(resp.position)
            if num_obstacles > 0:
                rospy.loginfo(f"检测到 {num_obstacles} 个动态障碍物")
            
        except rospy.ServiceException as e:
            rospy.logwarn(f"服务调用失败: {e}")
    
    def publish_visualization(self, resp):
        """发布可视化标记：只显示预测位置和协方差椭圆"""
        marker_array = MarkerArray()
        timestamp = rospy.Time.now()
        frame_id = self.frame_id
        marker_id = 0
        
        num_obstacles = len(resp.position)
        
        for i in range(num_obstacles):
            vel = resp.velocity[i]
            size = resp.size[i]
            pred_pos = resp.predicted_position[i]
            cov = resp.position_covariance[i]
            
            # 1. 预测位置 - 立方体（红色半透明）
            pred_box = self.create_box_marker(
                marker_id, timestamp, frame_id,
                pred_pos, size,
                r=1.0, g=0.3, b=0.0, a=0.6,
                ns="predicted_position"
            )
            marker_array.markers.append(pred_box)
            marker_id += 1
            
            # 2. 协方差椭圆 - 椭球体（青色半透明）
            cov_ellipse = self.create_covariance_ellipse(
                marker_id, timestamp, frame_id,
                pred_pos, cov,
                r=0.0, g=0.8, b=1.0, a=0.3,
                ns="covariance"
            )
            marker_array.markers.append(cov_ellipse)
            marker_id += 1
            
            # 3. 文本标签 - 显示速度和协方差信息
            speed = np.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
            cov_norm = np.sqrt(cov.x + cov.y + cov.z)
            text_marker = self.create_text_marker(
                marker_id, timestamp, frame_id,
                pred_pos, size.z,
                f"ID:{i} v:{speed:.2f}m/s σ:{cov_norm:.2f}",
                ns="labels"
            )
            marker_array.markers.append(text_marker)
            marker_id += 1
        
        # 清除旧的标记
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.header.frame_id = frame_id
        delete_marker.header.stamp = timestamp
        delete_marker.ns = "delete_all"
        delete_marker.id = 9999
        
        delete_array = MarkerArray()
        delete_array.markers.append(delete_marker)
        self.marker_pub.publish(delete_array)
        
        # 发布新标记
        if marker_array.markers:
            self.marker_pub.publish(marker_array)
    
    def create_box_marker(self, marker_id, timestamp, frame_id, 
                          position, size, r, g, b, a, ns):
        """创建立方体标记"""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = timestamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        marker.pose.position.x = position.x
        marker.pose.position.y = position.y
        marker.pose.position.z = position.z
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = max(size.x, 0.1)
        marker.scale.y = max(size.y, 0.1)
        marker.scale.z = max(size.z, 0.1)
        
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a
        
        marker.lifetime = rospy.Duration(0.5)
        
        return marker
    
    def create_covariance_ellipse(self, marker_id, timestamp, frame_id,
                                  position, covariance, r, g, b, a, ns):
        """创建协方差椭圆标记（表达预测不确定性）"""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = timestamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = position.x
        marker.pose.position.y = position.y
        marker.pose.position.z = position.z
        marker.pose.orientation.w = 1.0
        
        # 使用协方差的对角元素作为椭圆尺寸
        # covariance 是 Vector3，存储的是 [var_x, var_y, var_z]
        # 使用 2*sqrt(variance) 作为约 95% 置信区间
        scale_factor = 2.0
        marker.scale.x = max(scale_factor * np.sqrt(abs(covariance.x)), 0.2)
        marker.scale.y = max(scale_factor * np.sqrt(abs(covariance.y)), 0.2)
        marker.scale.z = max(scale_factor * np.sqrt(abs(covariance.z)), 0.2)
        
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a
        
        marker.lifetime = rospy.Duration(0.5)
        
        return marker
    
    def create_text_marker(self, marker_id, timestamp, frame_id,
                           position, box_height, text, ns):
        """创建文本标记"""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = timestamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # 文本位置（在边界框上方）
        marker.pose.position.x = position.x
        marker.pose.position.y = position.y
        marker.pose.position.z = position.z + box_height / 2.0 + 0.3
        marker.pose.orientation.w = 1.0
        
        marker.scale.z = 0.25  # 文本高度
        
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        marker.text = text
        marker.lifetime = rospy.Duration(0.5)
        
        return marker
    
    def run(self):
        """运行节点"""
        rospy.spin()


def main():
    try:
        tester = ServiceVisualizationTester()
        tester.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"节点异常退出: {e}")


if __name__ == '__main__':
    main()
