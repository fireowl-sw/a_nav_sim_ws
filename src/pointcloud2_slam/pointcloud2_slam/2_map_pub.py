#!/usr/bin/env python3
"""
障碍物栅格地图节点
功能：将点云数据转换为栅格地图，并生成多层膨胀的障碍物地图

订阅话题：
    - /mapokk: 转换到地图坐标系的点云数据
    - /odom: 里程计信息，提供机器人的位置

发布话题：
    - combined_grid: 综合栅格地图（包含障碍物及其膨胀区域）

参数说明：
    - grid_width: 地图宽度（米），默认60.0
    - grid_height: 地图高度（米），默认60.0
    - resolution: 地图分辨率（米/格），默认0.1
    - min_height: 点云最小高度（米），默认0.1
    - max_height: 点云最大高度（米），默认1.0
    - obstacle_radius: 障碍物膨胀半径（米），默认0.2

栅格值说明：
    - 100: 障碍物
    - 5: 第一层膨胀区域
    - -8: 第二层膨胀区域
    - -120: 第三层膨胀区域
    - 1: 自由空间
    - -1: 未知区域
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class ObstacleGridNode(Node):
    """障碍物栅格地图节点类"""
    
    def __init__(self):
        super().__init__("obstacle_grid_node")

        # 声明并获取参数
        self.declare_parameter('grid_width', 60.0)  # 地图宽度（米）
        self.declare_parameter('grid_height', 60.0)  # 地图高度（米）
        self.declare_parameter('resolution', 0.1)  # 地图分辨率（米/格）
        self.declare_parameter('min_height', 0.1)  # 点云最小高度（米）- 过滤地面点
        self.declare_parameter('max_height', 1.0)  # 点云最大高度（米）- 过滤过高点
        self.declare_parameter('obstacle_radius', 0.2)  # 障碍物膨胀半径（米）
        
        # 获取参数值
        self.grid_width = self.get_parameter('grid_width').get_parameter_value().double_value
        self.grid_height = self.get_parameter('grid_height').get_parameter_value().double_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value
        self.min_height = self.get_parameter('min_height').get_parameter_value().double_value
        self.max_height = self.get_parameter('max_height').get_parameter_value().double_value
        self.obstacle_radius = self.get_parameter('obstacle_radius').get_parameter_value().double_value        

        # 初始化障碍物集合（存储栅格索引）
        self.obstacles = set()                # 障碍物集合（值=100）
        self.dilated_obstacles_layer1 = set() # 第一层膨胀（值=5）
        self.dilated_obstacles_layer2 = set() # 第二层膨胀（值=-8）
        self.dilated_obstacles_layer3 = set() # 第三层膨胀（值=-120）

        # 初始化OccupancyGrid消息（栅格地图）
        # OccupancyGrid消息结构：
        # ├── Header header
        # │   ├── frame_id       # 坐标系（如 "map"）
        # │   └── stamp          # 时间戳
        # ├── MapMetaData info
        # │   ├── resolution     # 分辨率（米/格）
        # │   ├── width          # 宽度（格子数）
        # │   ├── height         # 高度（格子数）
        # │   └── origin         # 地图原点位置
        # └── int8[] data        # 栅格数据（一维数组）
        self.grid_combined = OccupancyGrid()
        self.grid_combined.header.frame_id = 'map'
        
        # 计算地图的栅格尺寸（单元格数）
        self.grid_combined.info.width = int(self.grid_width / self.resolution)   # 地图宽度（格子数）
        self.grid_combined.info.height = int(self.grid_height / self.resolution) # 地图高度（格子数）
        self.grid_combined.info.resolution = self.resolution
        
        # 初始化栅格数据，全部设为未知区域（-1）
        self.grid_combined.data = [-1] * (self.grid_combined.info.width * self.grid_combined.info.height)
        
        # 创建点云订阅器，订阅转换后的点云数据，队列大小为10
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, 
            '/mapokk', 
            self.pointcloud_callback, 
            10)
        
        # 创建里程计订阅器，订阅机器人的位置信息，队列大小为10
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/odom', 
            self.odom_callback, 
            10)
        
        # 创建栅格地图发布器，发布综合障碍物地图，队列大小为10
        self.grid_combined_pub = self.create_publisher(
            OccupancyGrid, 
            'combined_grid', 
            10)
        
        # 初始化变量
        self.odom_data = None  # 存储最新的里程计数据
        
    def odom_callback(self, msg):
        """
        里程计回调函数
        接收并存储里程计数据，获取机器人的位置
        
        Args:
            msg: Odometry消息，包含机器人的位置和姿态信息
        """
        self.odom_data = msg

    def pointcloud_callback(self, msg):
        """
        点云回调函数
        接收点云数据，将其转换为栅格地图，并进行多层膨胀处理
        
        Args:
            msg: PointCloud2消息，包含点云数据
        """
        # 检查是否已接收到里程计数据
        if self.odom_data is None:
            return
        
        # 获取机器人当前位置（虽然在这个实现中未使用，但保留以备将来扩展）
        origin_x = self.odom_data.pose.pose.position.x
        origin_y = self.odom_data.pose.pose.position.y

        # 读取点云数据（x, y, z坐标），跳过NaN值
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        # 计算障碍物膨胀半径对应的栅格数
        radius_cells = int(self.obstacle_radius / self.resolution)

        # 创建临时集合，存储当前帧新增的障碍物和膨胀区域
        new_obstacles = set()
        new_dilated_obstacles_layer1 = set()
        new_dilated_obstacles_layer2 = set()
        new_dilated_obstacles_layer3 = set()

        # 遍历所有点云数据
        for x, y, z in points:
            # 根据高度过滤点云（保留指定高度范围内的障碍物）
            if self.min_height <= z <= self.max_height:
                # 将点云坐标转换为栅格坐标
                # 坐标系转换：原点从地图中心移到左下角
                center_x = int((x + self.grid_width / 2) / self.resolution)
                center_y = int((y + self.grid_height / 2) / self.resolution)

                # 检查栅格坐标是否在地图范围内
                if 0 <= center_x < self.grid_combined.info.width and 0 <= center_y < self.grid_combined.info.height:
                    # 计算一维数组索引（row-major顺序）
                    index = center_y * self.grid_combined.info.width + center_x
                    new_obstacles.add(index)  # 添加障碍物

                # 对障碍物进行多层膨胀处理
                # enumerate返回(layer_index, dilated_set)，layer_index从0开始
                for layer, dilated_set in enumerate([
                    new_dilated_obstacles_layer1,
                    new_dilated_obstacles_layer2,
                    new_dilated_obstacles_layer3]):
                    # 遍历膨胀区域内的所有栅格
                    for dx in range(-(layer+1)*radius_cells, (layer+1)*radius_cells+1):
                        for dy in range(-(layer+1)*radius_cells, (layer+1)*radius_cells+1):
                            # 检查是否在圆形区域内（x^2 + y^2 <= r^2）
                            if dx**2 + dy**2 <= ((layer+1)*radius_cells)**2:
                                grid_x = center_x + dx
                                grid_y = center_y + dy
                                # 检查膨胀后的栅格是否在地图范围内
                                if 0 <= grid_x < self.grid_combined.info.width and 0 <= grid_y < self.grid_combined.info.height:
                                    index = grid_y * self.grid_combined.info.width + grid_x
                                    dilated_set.add(index)
        
        # 将当前帧的障碍物和膨胀区域添加到全局集合中
        self.obstacles.update(new_obstacles)
        self.dilated_obstacles_layer1.update(new_dilated_obstacles_layer1)
        self.dilated_obstacles_layer2.update(new_dilated_obstacles_layer2)
        self.dilated_obstacles_layer3.update(new_dilated_obstacles_layer3)

        # 更新并发布综合栅格地图
        self.update_combined_grid()
    
    def update_combined_grid(self):
        """
        更新综合栅格地图
        将障碍物及其膨胀区域映射到栅格地图，并发布
        
        栅格值说明：
        - 100: 障碍物（高优先级，不可通行）
        - 5: 第一层膨胀区域（接近障碍物，警告）
        - -8: 第二层膨胀区域（中等距离）
        - -120: 第三层膨胀区域（较远距离，低优先级）
        - 1: 自由空间（可通行）
        """
        # 初始化栅格数据，全部设为自由空间（1）
        self.grid_combined.data = [1] * (self.grid_combined.info.width * self.grid_combined.info.height)
        
        # 标记障碍物（值=100）
        for index in self.obstacles:
            if self.grid_combined.data[index] != 100:
                self.grid_combined.data[index] = 100

        # 标记第一层膨胀区域（值=5）
        # 使用集合差集，确保不覆盖障碍物标记
        for index in self.dilated_obstacles_layer1 - self.obstacles:
            if self.grid_combined.data[index] == 1:
                self.grid_combined.data[index] = 5

        # 标记第二层膨胀区域（值=-8）
        # 确保不覆盖第1层和障碍物标记
        for index in self.dilated_obstacles_layer2 - self.dilated_obstacles_layer1:
            if self.grid_combined.data[index] == 1:
                self.grid_combined.data[index] = -8

        # 标记第三层膨胀区域（值=-120）
        # 确保不覆盖第2层、第1层和障碍物标记
        for index in self.dilated_obstacles_layer3 - self.dilated_obstacles_layer2:
            if self.grid_combined.data[index] == 1:
                self.grid_combined.data[index] = -120

        # 更新消息头信息
        self.grid_combined.header.stamp = self.get_clock().now().to_msg()  # 当前时间戳
        self.grid_combined.header.frame_id = 'map'                         # 坐标系
        
        # 设置地图原点（地图中心对应世界坐标原点）
        self.grid_combined.info.origin.position.x = -self.grid_width/2
        self.grid_combined.info.origin.position.y = -self.grid_height/2
        self.grid_combined.info.origin.position.z = 0.0

        # 发布综合栅格地图
        self.grid_combined_pub.publish(self.grid_combined)

def main(args = None):
    """
    主函数
    初始化ROS2节点并运行
    
    Args:
        args: 命令行参数
    """
    # 初始化ROS2 Python客户端库
    rclpy.init(args=args)
    
    # 创建障碍物栅格地图节点
    node = ObstacleGridNode()
    
    # 运行节点，直到被终止
    rclpy.spin(node)
    
    # 清理并关闭ROS2
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
