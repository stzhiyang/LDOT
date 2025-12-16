#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轨迹预测服务测试与可视化脚本

该脚本用于：
1. 调用 GetPredictedTrajectories 服务获取动态障碍物的长期预测轨迹
2. 可视化预测轨迹线（LINE_STRIP）
3. 可视化每个轨迹点的不确定性椭圆（SPHERE）
"""

import rospy
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from onboard_detector.srv import GetPredictedTrajectories, GetPredictedTrajectoriesRequest


class TrajectoryPredictionTester:
    """轨迹预测服务测试与可视化类"""
    
    def __init__(self):
        rospy.init_node('trajectory_prediction_tester', anonymous=True)
        
        # 参数配置
        self.service_name = rospy.get_param(
            '~service_name', 
            '/onboard_detector/get_predicted_trajectories'
        )
        self.query_range = rospy.get_param('~query_range', 10.0)  # 查询范围（米）
        self.prediction_horizon = rospy.get_param('~prediction_horizon', 1.5)  # 预测时域（秒）
        self.prediction_dt = rospy.get_param('~prediction_dt', 0.1)  # 预测步长（秒）
        self.query_rate = rospy.get_param('~query_rate', 10.0)  # 查询频率（Hz）
        self.frame_id = rospy.get_param('~frame_id', 'map')  # 坐标系名称
        self.current_position = [
            rospy.get_param('~current_x', 0.0),
            rospy.get_param('~current_y', 0.0),
            rospy.get_param('~current_z', 0.0)
        ]
        
        # 可视化参数
        self.trajectory_line_width = rospy.get_param('~trajectory_line_width', 0.05)
        self.covariance_scale = rospy.get_param('~covariance_scale', 1.0)  # 协方差缩放因子（2.0对应约95%置信区间）
        self.show_covariance = rospy.get_param('~show_covariance', True)  # 是否显示协方差椭圆
        self.covariance_skip = rospy.get_param('~covariance_skip', 2)  # 每隔几个点显示一个协方差椭圆
        
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
            self.service_proxy = rospy.ServiceProxy(
                self.service_name, 
                GetPredictedTrajectories
            )
            rospy.loginfo("服务连接成功！")
        except rospy.ROSException:
            rospy.logerr(f"服务 {self.service_name} 不可用，请确保检测器节点已启动")
            raise

        
        # 各元素颜色配置
        self.trajectory_color = ColorRGBA(1.0, 0.5, 0.0, 1.0)  # 轨迹线：橙色
        self.ellipse_color = ColorRGBA(0.2, 0.8, 0.2, 1.0)  # 椭圆：绿色
        self.text_color = ColorRGBA(0.8, 0.6, 1.0, 1.0)  # 文本：浅紫色
        self.endpoint_color = ColorRGBA(1.0, 0.2, 0.2, 1.0)  # 终点：红色
        
        # 定时器
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.query_rate), 
            self.timer_callback
        )
        
        rospy.loginfo("轨迹预测可视化测试节点已启动")
        rospy.loginfo(f"  - 预测时域: {self.prediction_horizon}s")
        rospy.loginfo(f"  - 预测步长: {self.prediction_dt}s")
        rospy.loginfo(f"  - 查询范围: {self.query_range}m")
    
    def timer_callback(self, event):
        """定时器回调：查询服务并发布可视化"""
        try:
            # 构建请求
            req = GetPredictedTrajectoriesRequest()
            req.current_position = Point(
                x=self.current_position[0],
                y=self.current_position[1],
                z=self.current_position[2]
            )
            req.range = self.query_range
            req.prediction_horizon = self.prediction_horizon
            req.prediction_dt = self.prediction_dt
            
            # 调用服务
            resp = self.service_proxy(req)
            
            # 发布可视化
            self.publish_visualization(resp)
            
            # 打印统计信息
            num_obstacles = len(resp.obstacle_types)
            if num_obstacles > 0:
                total_points = sum(resp.trajectory_lengths)
                rospy.loginfo(
                    f"检测到 {num_obstacles} 个动态障碍物，"
                    f"共 {total_points} 个轨迹点"
                )
            
        except rospy.ServiceException as e:
            rospy.logwarn(f"服务调用失败: {e}")
    
    def publish_visualization(self, resp):
        """发布可视化标记"""
        marker_array = MarkerArray()
        timestamp = rospy.Time.now()
        frame_id = self.frame_id
        marker_id = 0
        
        num_obstacles = len(resp.obstacle_types)
        
        # 首先发布删除所有旧标记的命令
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.header.frame_id = frame_id
        delete_marker.header.stamp = timestamp
        marker_array.markers.append(delete_marker)
        
        if num_obstacles == 0:
            self.marker_pub.publish(marker_array)
            return
        
        # 解析扁平化的轨迹数据
        traj_idx = 0  # 轨迹点索引
        
        for i in range(num_obstacles):
            obstacle_type = resp.obstacle_types[i] if i < len(resp.obstacle_types) else "other"
            current_pos = resp.current_positions[i]
            current_vel = resp.current_velocities[i]
            size = resp.sizes[i]
            traj_length = resp.trajectory_lengths[i]
            
            # 提取该障碍物的轨迹点
            traj_positions = resp.trajectory_positions[traj_idx:traj_idx + traj_length]
            traj_velocities = resp.trajectory_velocities[traj_idx:traj_idx + traj_length]
            traj_covariances = resp.position_covariances[traj_idx:traj_idx + traj_length]
            traj_idx += traj_length
            
            # 1. 轨迹线 - LINE_STRIP
            if traj_length > 1:
                trajectory_line = self.create_trajectory_line_marker(
                    marker_id, timestamp, frame_id,
                    traj_positions,
                    ns="trajectory_line"
                )
                marker_array.markers.append(trajectory_line)
                marker_id += 1
            
            # 2. 不确定性椭圆 - SPHERE（每隔几个点显示一个）
            if self.show_covariance:
                for j in range(0, traj_length, self.covariance_skip):
                    pos = traj_positions[j]
                    cov = traj_covariances[j]
                    
                    # 根据时间步调整颜色深度（越远越深）
                    time_ratio = j / max(traj_length - 1, 1)
                    
                    cov_ellipse = self.create_covariance_ellipse_marker(
                        marker_id, timestamp, frame_id,
                        pos, cov, time_ratio=time_ratio,
                        ns="covariance_ellipse"
                    )
                    marker_array.markers.append(cov_ellipse)
                    marker_id += 1
            
            # 3. 文本标签 - 显示障碍物类型和速度
            speed = np.sqrt(
                current_vel.x**2 + current_vel.y**2 + current_vel.z**2
            )
            text_marker = self.create_text_marker(
                marker_id, timestamp, frame_id,
                current_pos, size.z,
                f"{obstacle_type} v:{speed:.2f}m/s pts:{traj_length}",
                ns="labels"
            )
            marker_array.markers.append(text_marker)
            marker_id += 1
            
            # 4. 轨迹终点标记 - 小球
            if traj_length > 0:
                end_pos = traj_positions[-1]
                end_marker = self.create_endpoint_marker(
                    marker_id, timestamp, frame_id,
                    end_pos,
                    ns="trajectory_endpoint"
                )
                marker_array.markers.append(end_marker)
                marker_id += 1
        
        # 发布标记
        self.marker_pub.publish(marker_array)

    
    def create_trajectory_line_marker(self, marker_id, timestamp, frame_id,
                                       positions, ns):
        """创建轨迹线标记（LINE_STRIP类型，显示预测轨迹）"""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = timestamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.trajectory_line_width  # 线宽
        
        # 轨迹线颜色
        marker.color = self.trajectory_color
        for pos in positions:
            p = Point()
            p.x = pos.x
            p.y = pos.y
            p.z = pos.z
            marker.points.append(p)
        
        marker.lifetime = rospy.Duration(0.1)
        
        return marker
    
    def create_covariance_ellipse_marker(self, marker_id, timestamp, frame_id,
                                          position, covariance, time_ratio, ns):
        """创建协方差椭圆标记（SPHERE类型，表示位置不确定性）"""
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
        # 使用 scale_factor * sqrt(variance) 作为椭圆半径
        # scale_factor=2 对应约 95% 置信区间
        scale_factor = self.covariance_scale
        marker.scale.x = max(scale_factor * np.sqrt(abs(covariance.x)), 0.1)
        marker.scale.y = max(scale_factor * np.sqrt(abs(covariance.y)), 0.1)
        marker.scale.z = max(scale_factor * np.sqrt(abs(covariance.z)), 0.1)
        
        # 椭圆颜色，透明度随时间变化
        marker.color.r = self.ellipse_color.r
        marker.color.g = self.ellipse_color.g
        marker.color.b = self.ellipse_color.b
        marker.color.a = 0.4 - 0.2 * time_ratio  # 透明度从0.4渐变到0.2
        
        marker.lifetime = rospy.Duration(0.1)
        
        return marker
    
    def create_text_marker(self, marker_id, timestamp, frame_id,
                           position, box_height, text, ns):
        """创建文本标记（显示障碍物信息）"""
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
        
        # 文本颜色
        marker.color = self.text_color
        
        marker.text = text
        marker.lifetime = rospy.Duration(0.1)
        
        return marker
    
    def create_endpoint_marker(self, marker_id, timestamp, frame_id,
                                position, ns):
        """创建轨迹终点标记（小球，表示轨迹结束位置）"""
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
        
        # 小球尺寸
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        
        # 终点颜色
        marker.color = self.endpoint_color
        
        marker.lifetime = rospy.Duration(0.1)
        
        return marker
    
    def run(self):
        """运行节点"""
        rospy.spin()


def main():
    try:
        tester = TrajectoryPredictionTester()
        tester.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"节点异常退出: {e}")


if __name__ == '__main__':
    main()
