#!/usr/bin/env python3
"""
点云坐标转换节点
功能：将点云数据从机器人坐标系转换到地图坐标系，并发布转换后的点云

订阅话题：
    - /odom: 里程计信息，提供机器人的位置和姿态
    - points_raw: 原始点云数据

发布话题：
    - mapokk: 转换到地图坐标系的点云数据
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class PointTransformNode(Node):
    """点云坐标转换节点类"""
    
    def __init__(self):
        super().__init__("pointcloud_transform_node")
        
        # 声明参数：设置转换后的坐标系，默认为'map'
        self.declare_parameter('frame_id', 'map')

        # 创建里程计订阅器，队列大小为10
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/odom', 
            self.odom_callback, 
            10
            )
        
        # 创建点云订阅器，队列大小为10
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, 
            'points_raw', 
            self.pointcloud_callback,
            10
            )
        
        # 创建点云发布器，发布转换后的点云，队列大小为10
        self.transformed_pointcloud_pub = self.create_publisher(
            PointCloud2, 
            'mapokk', 
            10
        )

        # 初始化变量
        self.odom_data = None          # 存储最新的里程计数据
        self.rotation_matrix = None    # 存储旋转矩阵（4x4齐次变换矩阵）
        self.translation = None        # 存储平移向量

    def odom_callback(self, msg):
        """
        里程计回调函数
        接收里程计数据，提取机器人的位置和姿态，并计算旋转矩阵
        
        Args:
            msg: Odometry消息，包含机器人的位置和姿态信息
        """
        # odom输出信息的样式
        # pose:
        #     pose:
        #         position:
        #         x: 0.0
        #         y: 0.0
        #         z: 0.0
        #         orientation:
        #         x: 0.0
        #         y: 0.0
        #         z: 0.0
        #         w: 1.0
        
        # 提取四元数（表示旋转）
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        
        # 提取位置向量（表示平移）
        translation = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )

        # 检查里程计数据是否发生变化（位置或姿态改变）
        if (quaternion != getattr(self.odom_data, 'pose.pose.orientation', None) or 
            translation != getattr(self.odom_data, 'pose.pose.position', None)):
            
            # 更新里程计数据
            self.odom_data = msg
            
            # 从四元数计算旋转矩阵
            # 四元数到旋转矩阵的转换公式
            qx, qy, qz, qw = quaternion
            sqx, sqy, sqz = qx * qx, qy * qy, qz * qz
            
            # 计算3x3旋转矩阵的元素
            m00, m01, m02 = 1 - 2*(sqy + sqz), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)
            m10, m11, m12 = 2*(qx*qy + qw*qz), 1 - 2*(sqx + sqz), 2*(qy*qz - qw*qx)
            m20, m21, m22 = 2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(sqx + sqy)
            
            # 构建4x4齐次变换矩阵（包含旋转和平移）
            self.rotation_matrix = np.array([
                [m00, m01, m02, translation[0]],
                [m10, m11, m12, translation[1]],
                [m20, m21, m22, translation[2]],
                [0, 0, 0, 1]  # 齐次坐标的最后一行
            ])

    def pointcloud_callback(self, msg):
        """
        点云回调函数
        接收原始点云数据，将其转换到地图坐标系，并发布转换后的点云
        
        Args:
            msg: PointCloud2消息，包含原始点云数据
        """
        # 检查是否已接收到里程计数据和计算出旋转矩阵
        if self.odom_data is None or self.rotation_matrix is None:
            return
        
        # 从PointCloud2消息中读取点云数据（x, y, z坐标）
        # skip_nans=True表示跳过包含NaN值的点
        point_struct = np.array(
            list(
            pc2.read_points(
                msg, 
                field_names=("x", "y", "z"),
                skip_nans=True
                )))
        
        # 如果点云为空，直接返回
        if len(point_struct) == 0:
            return
        
        # 将结构化数组转换为N x 3的numpy数组
        points = np.zeros((len(point_struct), 3), dtype=np.float32)
        points[:, 0] = point_struct['x']
        points[:, 1] = point_struct['y']
        points[:, 2] = point_struct['z']

        # 转换为齐次坐标（添加第四维，值为1）
        # 齐次坐标格式：[x, y, z, 1]
        points_homogeneous = np.hstack([
            points, 
            np.ones(
                (points.shape[0], 1), 
                dtype=np.float32)
            ])

        # 应用齐次变换矩阵进行坐标转换
        # 旋转矩阵 (4x4) @ 点云的转置 (4xN) -> 结果转置后取前三列
        transformed_points = (self.rotation_matrix @ points_homogeneous.T).T[:, :3]

        # 创建新的PointCloud2消息
        header = msg.header
        header.frame_id = self.get_parameter('frame_id').value  # 设置坐标系为'map'
        cloud_msg = pc2.create_cloud_xyz32(header, transformed_points)  # 创建点云消息
        
        # 发布转换后的点云
        self.transformed_pointcloud_pub.publish(cloud_msg)

def main(args = None):
    """
    主函数
    初始化ROS2节点并运行
    
    Args:
        args: 命令行参数
    """
    # 初始化ROS2 Python客户端库
    rclpy.init(args=args)
    
    # 创建点云转换节点
    node = PointTransformNode()
    
    # 运行节点，直到被终止
    rclpy.spin(node)
    
    # 清理并关闭ROS2
    rclpy.shutdown()

if __name__ == '__main__':
    main()
