# /combined_grid 的数据样式
# header:
#   stamp:
#     sec: 1767772572
#     nanosec: 684721240
#   frame_id: map
# info:
#   map_load_time:
#     sec: 0
#     nanosec: 0
#   resolution: 0.10000000149011612
#   width: 600
#   height: 600
#   origin:
#     position:
#       x: -30.0
#       y: -30.0
#       z: 0.0
#     orientation:
#       x: 0.0
#       y: 0.0
#       z: 0.0
#       w: 1.0
# data:
# - 1
# - 1
# - 1
# ...

from xml.etree.ElementPath import xpath_tokenizer
import rclpy
from rclpy.node import Node
import numpy as np
import heapq
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import math
from rclpy.qos import QoSProfile
import scipy.interpolate as si
from nav_msgs.msg import Odometry
from scipy.interpolate import BSpline
import time

expansion_size = 5  # 扩展障碍物大小，用于成本图中的障碍物膨胀

# 处理成本图数据，扩展障碍物
def costmap(data, width, height, resolution):
    data = np.array(data).reshape(height, width)  # 重塑数据为矩阵
    # 扩展障碍物
    # 使用 NumPy 的广播机制来替代循环
    wall_mask = data == 100
    for i in range(-expansion_size, expansion_size + 1):
        for j in range(-expansion_size, expansion_size + 1):
            if i == 0 and j == 0:
                continue
            shifted_mask = np.roll(wall_mask, (i, j), axis=(0, 1))
            data[shifted_mask] = 100
    data = data * resolution  # 将成本图中的值乘以分辨率
    return data

# 导入numpy库，用于数值计算
# 导入BSpline类，用于创建B样条曲线
def bezier_smoothing(array, num_points):
    # 尝试执行平滑处理，出错时返回原始路径
    try:
        # 转换为numpy数组
        array = np.array(array)
        # 提取x坐标
        x = array[:, 0]
        # 提取y坐标
        y = array[:, 1]
        
        # 计算基于弦长的参数t
        # 计算x方向差分，首位补x[0]
        dx = np.diff(x, prepend=x[0])
        # 计算y方向差分，首位补y[0]
        dy = np.diff(x, prepend=y[0])
        # 弦长
        chord_length = np.sqrt(dx**2 + dy**2)
        # 累积弦长作为参数t
        t = np.concatenate(([0], np.cumsum(chord_length)))
        # 规范化到[0, 1]
        t /= t[-1]
        
        # 贝塞尔曲线的阶数，这里选择三次贝塞尔曲线
        k = num_points - 1
        # 添加重复的节点，确保有足够的节点来定义样条
        t_knots = np.concatenate([0]*k, t, [1]*k)
        # 根据新的节点数组调整x和y的长度
        # 首尾边界扩展k个点 np.pad(数组, (左边填充数, 右边填充数), 填充模式)'edge' 模式：用边缘值填充
        x_padded = np.pad(x, (k,k), 'edge')
        y_padded = np.pad(y, (k,k), 'edge')
        
        # 创建B样条对象 BSpline(节点向量, 控制点, 阶数, 是否外推)
        # 创建x方向的B样条曲线
        spline_x = BSpline(t_knots, x_padded, k, extrapolate=False)
        # 创建y方向的B样条曲线
        spline_y = BSpline(t_knots, y_padded, k, extrapolate=False)

        
        # 基于等间距的t_new重新采样
        # 生成等间距参数点
        t_new = np.linspace(0, 1, num_points)
        # 计算平滑后的x坐标
        x_smoothed = spline_x(t_new)
        # 计算平滑后的y坐标
        y_smoothed = spline_y(t_new)
        
        # 合并为路径点数组
        path = np.column_stack((x_smoothed, y_smoothed))
    # 错误处理：打印错误信息
    except Exception as e:
        # 出错时返回原始路径
        path = array
    # 返回平滑后的路径
    return path

def astar(start, goal, grid):
    # 定义启发式函数，计算两点之间的欧几里得距离
    def heuristic(a, b):
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2) 

    # 获取网格的行数和列数
    rows, cols = grid.shape
    
    # 初始化开放列表
    open_set = []    
    # 将起点加入优先队列，设置初始优先级
    # 组合起来：
    # (5.0, 0, (0, 0))
    #   ↑   ↑   ↑
    #   f   g   节点坐标
    # heapq.heappush的作用：
    # 把 (5.0, 0, (0,0)) 放入 open_set 队列
    # 队列会自动按第一个元素(f值 = 实际代价 + 估算代价)从小到大排序
    # 实际距离 → g值（已付出的代价）
    # 估算距离 → h值（预计还需要的代价）
    # 总距离   → f值 = g + h
    heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start))
    # 初始化父节点字典，用于记录每个节点的来源
    came_from = {}
    # 初始化代价字典，记录从起点到各节点的最小代价
    cost_so_far = {start:0}
    # 初始化已访问集合
    closed_set = set()
    # 当开放列表不为空时持续搜索
    while open_set:
        # 从优先队列中取出代价最小的节点
        _, current_cost, current = heapq.heappop(open_set)
        # 判断当前节点是否为目标
        if current == goal:
            # 构建路径，从目标点开始回溯
            path = [current]
            # 沿着父节点指针回溯到起点
            while current in came_from:
                current = came_from[current]
                path.append(current)
            # 反转路径使其从起点到终点
            path.reverse()
            # 返回找到的路径
            return path
            
        # 检查当前节点是否已被访问过
        if current is closed_set:
            # 跳过已访问的节点
            continue
            
        # 将当前节点标记为已访问
        closed_set.add(current)
        # 遍历8个方向的邻居节点
        for d in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            # 计算邻居节点坐标
            neighbor = (current[0] + d[0], current[1] + d[1])            
            # 检查邻居节点是否在网格范围内且不是障碍物
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor] != 100:
                # 计算到达邻居节点的新代价
                new_cost = cost_so_far[current] + grid[neighbor]
                # 判断是否需要更新路径信息
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:                
                    # 更新到达邻居的最小代价
                    cost_so_far[neighbor] = new_cost                    
                    # 计算邻居的优先级
                    priority = new_cost + heuristic(goal, neighbor)                    
                    # 将邻居加入优先队列
                    heapq.heappush(open_set, (priority, new_cost, neighbor))                    
                    # 记录邻居的父节点
                    came_from[neighbor] = current    
    # 未找到路径，返回空列表
    return []

# 导航控制节点类
class NavigationControl(Node):
    def __init__(self):
        # 初始化ROS 2节点
        super().__init__('Navigation')
        # 创建订阅器订阅地图数据
        self.map_subscription = self.create_subscription(OccupancyGrid, 'combined_grid', self.map_callback, 10)
        # 创建发布者发布路径消息
        self.path_publisher = self.create_publisher(Path, 'path', 10)
        # 创建发布者发布第二条路径消息
        self.path_publisher2 = self.create_publisher(Path, 'path2', 10)
        # 创建订阅器订阅里程计数据
        self.odom_subscriber = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        # 创建订阅器订阅目标位姿 /goal_pose = 用户在RViz中点击地图设置的目标点
        self.pose_subscriber = self.create_subscription(PoseStamped,'/goal_pose',self.goal_callback,10)
        # 初始化机器人位置坐标
        self.x = 0
        self.y = 0
        # 初始化目标位置
        self.goal = None
        # 创建定时器定期发布路径
        self.create_timer(0.1, self.publish_path)
        # 初始化路径变量
        self.path = None
        self.path2 = None

    def goal_callback(self, msg):
        # 从消息中提取目标位置坐标
        self.goal = (msg.pose.position.x,msg.pose.position.y)
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    # 地图数据回调函数
    def map_callback(self, msg):
        # 检查是否设置了目标位置
        if self.goal is None:
            return
        # 计算机器人到目标的距离 hypot是求直角斜边的公式-平方开根号
        distance = abs(math.hypot(self.x - self.goal[0], self.y - self.goal[1]))
        # 判断距离是否大于阈值
        if distance > 0.2:
            # 初始化路径列表
            path = []
            # 获取地图分辨率
            resolution = msg.info.resolution
            # 获取地图原点坐标
            originX = msg.info.origin.position.x 
            originY = msg.info.origin.position.y       
            # 将机器人坐标转换为网格索引 机器人(米) → 网格(第几行第几列)
            column = int((self.x - originX) / resolution) 
            row = int((self.y - originY) / resolution) 
            # 将目标坐标转换为网格索引 网格(第几行第几列) → A*算法用的坐标
            columnH = int((self.goal[0] - originX) / resolution)
            rowH = int((self.goal[1] - originY) / resolution)  
            # 处理成本图数据
            data = costmap(msg.data, msg.info.width, msg.info.height, resolution)  # 处理成本图数据
            # 标记机器人所在位置为可通行区域
            data[row][column] = 1      
            # 根据地图信息设置可通行区域和障碍物区域[ -2 ≤ data ≤ 5 ] → 可通行区域 (设为1)其他→ 障碍物 (设为100)
            data[(data >= -2) & (data <= 5)] = 1
            data[(data < -2) | (data > 5)] = 100 #根据地图信息标记

            # 定义起点和终点
            start = (row, column)
            goal = (rowH, columnH)            
            # 执行A*路径搜索算法
            path = astar(start, goal, data)
            # 将网格坐标转换为实际世界坐标 推导式提取path
            paths = [(p[1] * resolution + originX, p[0] * resolution + originY) for p in path]
            # 保存原始路径
            self.path = paths
            # 对路径进行贝塞尔曲线平滑处理
            self.path2 = bezier_smoothing(paths, len(paths))
        
        # 否则到达目标位置
        else:
            print("reach goal----nav stop")
    
    # 发布路径消息的方法
    def publish_path(self):
        # 检查路径是否存在且不为空
        if self.path is None or len(self.path)==0:
            print('no path')
            return
        # 创建Path消息对象
        path_msg = Path()
        # 设置消息的参考坐标系
        path_msg.header.frame_id = 'map'
        # 遍历路径点列表 # 创建
        for (x, y) in self.path:
            # 位姿消息对象
            pose = PoseStamped()
            # 设置位姿的参考坐标系
            pose.header.frame_id = 'map'
            # 设置位姿的位置坐标
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            # 将位姿添加到路径消息中
            path_msg.poses.append(pose)            
        # 发布原始路径消息
        self.path_publisher.publish(path_msg)
            
        # 创建第二条路径消息对象
        path2_msg = Path()            
        # 设置消息的参考坐标系
        path2_msg.header.frame_id = 'map'
        # 遍历平滑后的路径点列表
        for (x, y) in self.path2:
            # 创建位姿消息对象
            pose2 = PoseStamped()
            # 设置位姿的参考坐标系
            pose2.header.frame_id = 'map'
            # 设置位姿的位置坐标
            pose2.pose.position.x = float(x)
            # 将位姿添加到路径消息中
            pose2.pose.position.y = float(y)
        # 发布平滑后的路径消息
        self.path_publisher2.publish(path2_msg)


def main(args = None):
    rclpy.init(args = args)
    navcontrol = NavigationControl()
    rclpy.spin(navcontrol)
    navcontrol.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()