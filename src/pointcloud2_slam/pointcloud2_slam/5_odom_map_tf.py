import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

# 里程计到地图坐标系变换节点
class OdomToMapTransformer(Node):
    # 初始化方法
    def __init__(self):
        # 调用父类Node的初始化，设置节点名
        super().__init__('odom_to_map_transformer')
        # 创建TF广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # 创建静态TF广播器
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        # 创建里程计订阅者
        self.odom_subscriber = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
    # 里程计回调方法
    def odom_callback(self, odom_msg):
        # 创建变换消息
        # 创建TransformStamped对象
        t = TransformStamped()
        # 设置 header
        # 设置时间戳为当前时间
        t.header.stamp = self.get_clock().now().to_msg()
        # 设置父坐标系为map
        t.header.frame_id = 'map'
        # 设置子坐标系为odom
        t.child_frame_id = 'odom'

        # 设置变换的位置和朝向
        # 设置x平移
        t.transform.translation.x = odom_msg.pose.pose.position.x
        # 设置y平移
        t.transform.translation.y = odom_msg.pose.pose.position.y
        # 设置z平移
        t.transform.translation.z = odom_msg.pose.pose.position.z
        # 设置旋转四元数
        t.transform.rotation = odom_msg.pose.pose.orientation

        # 发布变换
        # 广播TF变换
        self.tf_broadcaster.sendTransform(t)
        
    # 发布静态变换方法
    def publish_static_transform(self):
        # 创建静态变换消息
        # 创建TransformStamped对象
        static_transform = TransformStamped()
        # 设置时间戳为当前时间
        static_transform.header.stamp = self.get_clock().now().to_msg()
        # 设置父坐标系为odom
        static_transform.header.frame_id = 'odom'
        # 设置子坐标系为base_link
        static_transform.child_frame_id = 'base_link'
        # 设置x平移为0
        static_transform.transform.translation.x = 0.0
        # 设置y平移为0
        static_transform.transform.translation.y = 0.0
        # 设置z平移为0
        static_transform.transform.translation.z = 0.0
        # 设置x旋转为0
        static_transform.transform.rotation.x = 0.0
        # 设置y旋转为0
        static_transform.transform.rotation.y = 0.0
        # 设置z旋转为0
        static_transform.transform.rotation.z = 0.0
        # 设置w为1
        # 单位四元数表示没有旋转
        static_transform.transform.rotation.w = 1.0  # 单位四元数表示没有旋转
        # 发布静态变换
        # 广播静态TF变换
        # self.static_tf_broadcaster.sendTransform(static_transform)
        
# 主函数入口
def main(args=None):
    # 初始化ROS 2客户端库
    rclpy.init(args=args)
    # 创建里程计到地图变换节点实例
    node = OdomToMapTransformer()
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