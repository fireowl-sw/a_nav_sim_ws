import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
import numpy as np
from scipy.spatial import KDTree
from nav_msgs.msg import Path

# 纯追踪控制器
class PurePursuitController:
    # 初始化方法，接收前瞻距离参数
    def __init__(self, lookahead_distance):
        # 存储前瞻距离
        self.lookahead_distance = lookahead_distance

    # 计算转向角方法，接收车辆位姿和路径点
    def calculate_steering_angle(self, vehicle_pose, path_points):
        # 找到离车辆最近的路径点的地址
        # 用KDTree查询离车辆最近的路径点索引
        closest_point_idx = KDTree(path_points[:, :2]).query(vehicle_pose[:2])[1]
        
        # 动态选择最合适的路径点作为目标点
        # 遍历所有路径点
        for i in range(len(path_points)):
            # 计算前瞻点索引，支持循环路径
            lookahead_point_idx = (closest_point_idx + i) % len(path_points)
            # 获取目标路径点
            target_point = path_points[lookahead_point_idx]
            # 计算车辆到目标点的x、y距离
            dx, dy = target_point[0] - vehicle_pose[0], target_point[1] - vehicle_pose[1]
            # 计算车辆到目标点的欧几里得距离
            distance_to_target = math.sqrt(dx**2 + dy**2)
            # 判断距离是否达到前瞻距离
            if distance_to_target >= self.lookahead_distance:
                # 达到前瞻距离，跳出循环
                break
        
        # 计算车辆到目标点的向量
        dx, dy = target_point[0] - vehicle_pose[0], target_point[1] - vehicle_pose[1]
        # 计算目标角度
        # 用atan2计算目标方向角度
        target_angle = math.atan2(dy, dx)
        # 计算转向角
        # 转向角为目标角度减去当前航向角
        steering_angle = target_angle - vehicle_pose[2]
        # 确保转向角在-pi到pi之间
        # 当转向角大于pi时
        while steering_angle > math.pi:
            # 减去2pi进行归一化
            steering_angle -= 2 * math.pi
        # 当转向角小于-pi时
        while steering_angle < -math.pi:
            # 加上2pi进行归一化
            steering_angle += 2 * math.pi
        # 返回转向角和目标点
        return steering_angle, target_point

# ROS 2节点
class PathFollowingNode(Node):
    # 初始化方法
    def __init__(self):
        # 调用父类Node的初始化
        super().__init__('path_following_node')
        # 创建纯追踪控制器
        # 设置前瞻距离为0.5米
        self.pure_pursuit = PurePursuitController(lookahead_distance=0.5)
        # 创建路径点变量
        self.path_points = None
        # 订阅里程计话题
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)
        # 发布速度命令话题
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # 订阅路径话题
        self.path_subscriber = self.create_subscription(Path, '/path', self.path_callback, 10)
        # 变量初始化
        # 初始化里程计数据
        self.current_odom = None
        # 新增的停止标志
        self.stop_flag = False
        # 添加路径是否已接收的标志
        self.path_received = False
        # 准备导航日志信息
        self.get_logger().info('ready---ok---to---nav')

    # 路径回调方法
    def path_callback(self, msg):
        # 从路径消息中提取x、y坐标列表
        self.path_points_list = [[point.pose.position.x, point.pose.position.y] for point in msg.poses]
        # 将列表转换为 numpy 数组
        self.path_points = np.array(self.path_points_list)
        # 断言数组为二维
        assert self.path_points.ndim == 2, "path_points must be a 2D array"
        # 对路径点进行插值
        # 调用插值方法细化路径
        self.path_points = self.interpolate_path(self.path_points)
        # 设置路径接收标志
        self.path_received = True  # 设置路径接收标志
        # 打印路径已接收信息
        print('received path ready to nav-------------')

    # 路径插值方法
    def interpolate_path(self, points, segment_length=0.1):
        # 初始化插值点列表
        interpolated_points = []
        # 遍历每对相邻点
        for i in range(len(points) - 1):
            # 获取起点
            start_point = points[i]
            # 获取终点
            end_point = points[i+1]
            # 计算两点之间的距离
            distance = np.linalg.norm(end_point - start_point)
            # 计算所需点的数量（包括起点）
            num_points = int(distance / segment_length) + 1
            # 生成线性插值点
            t_values = np.linspace(0, 1, num_points)
            # 创建插值参数数组
            # 计算插值线段
            interpolated_segment = start_point + (end_point - start_point)[np.newaxis, :] * t_values[:, np.newaxis]
            # 添加到插值点列表
            interpolated_points.append(interpolated_segment)
        # 将所有插值点合并成一个数组
        # 返回垂直堆叠的插值数组
        return np.vstack(interpolated_points)

    # 四元数转偏航角方法
    def quaternion_to_yaw(self, q):
        # 计算sin(yaw)的2倍值
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        # 计算cos(yaw)的2倍值
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        # 使用atan2计算偏航角
        yaw = math.atan2(siny_cosp, cosy_cosp)
        # 返回偏航角
        return yaw

    # 里程计回调方法
    def odometry_callback(self, msg):
        # 提取当前位置x、y坐标
        self.current_xy = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        # 检查是否接收到路径
        if not self.path_received:
            # 如果还没有接收到路径，则直接返回
            return
        # 存储当前里程计数据
        self.current_odom = msg
        # 提取位置和朝向
        # 构建位姿数组[x, y, yaw]
        pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, self.quaternion_to_yaw(msg.pose.pose.orientation)]
        # 纯追踪控制器计算转向角和目标点
        steering_angle, target_point = self.pure_pursuit.calculate_steering_angle(pose, self.path_points)
        # 计算当前点到终点的欧几里得距离
        distance_to_end = np.linalg.norm(np.array(pose[:2]) - self.path_points[-1])
        # 停止条件
        # 判断是否接近终点
        if distance_to_end < 0.2:  # 0.2m作为接近阈值
            # 设置线速度为0
            speed = 0.0
            # 设置角速度为0
            steering_angle = 0.0
            # 重置路径接收标志
            self.path_received = False
            # 导航成功日志
            self.get_logger().info('Naving node.success..')
        # 其他情况
        else:
            # 根据转向角大小调整速度
            # 使用 sin 函数来调整速度，当转向角接近 0 时，速度接近最大值；当转向角增大时，速度逐渐减小
            # 使用多项式函数调整速度
            # 使用sin函数调整速度，最小0.6
            speed = max(0.6, 1.5 - 1.5*math.sin(0.6*abs(steering_angle)))


        # 发布速度和转向角
        cmd_vel_msg = Twist()
        # 设置线速度
        cmd_vel_msg.linear.x = speed
        # 设置角速度
        cmd_vel_msg.angular.z = steering_angle
        # 发布速度命令
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        # 日志输出导航信息
        self.get_logger().info(f'v: {speed:.2f}, ang: {steering_angle:.2f}, dist_to_end: {distance_to_end:.2f}')


# 主函数入口
def main(args=None):
    # 初始化ROS 2客户端库
    rclpy.init(args=args)
    # 创建路径跟随节点实例
    node = PathFollowingNode()
    # 尝试执行主循环
    try:
        # 让节点持续运行，处理回调
        rclpy.spin(node)
    # 捕获键盘中断异常
    except KeyboardInterrupt:
        # 中断时什么都不做
        pass
    # 无论是否异常都执行清理
    finally:
        # 销毁节点
        node.destroy_node()
        # 关闭ROS 2客户端库
        rclpy.shutdown()

# 判断是否作为主程序运行
if __name__ == '__main__':
    # 调用主函数
    main()
