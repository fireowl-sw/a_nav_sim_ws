
# ========================================
# ROS2 Control 控制器参考文档
# ========================================
# 
# 当前项目使用的控制器:
# ========================================
# 
# 1. 关节状态广播器 (JointStateBroadcaster) [已启用]
# 类型: joint_state_broadcaster/JointStateBroadcaster
# 功能: 发布关节状态到/joint_states话题
# 输出话题: /joint_states (sensor_msgs/JointState)
# 用途: RViz可视化、状态监控、其他控制器的输入
# 重要性: ★★★★★ (必需)
# 配置要求: 必须指定所有需要监控的关节名称
#
# 2. 差分驱动控制器 (DiffDriveController) [已启用]
# 类型: diff_drive_controller/DiffDriveController
# 功能: 将Twist命令转换为轮子速度，发布里程计信息
# 输入话题: /cmd_vel (geometry_msgs/Twist)
# 输出话题: /odom (nav_msgs/Odometry)
# TF变换: odom -> base_footprint
# 重要性: ★★★★★ (差分驱动机器人必需)
# 配置要求: 需要准确的轮距和轮径参数
#
# ========================================
# 其他可用控制器 (未启用，供参考扩展)
# ========================================
#
# 3. 力矩控制器 (JointGroupEffortController)
# 类型: effort_controllers/JointGroupEffortController
# 功能: 直接控制关节输出力矩
# 输入话题: /effort_controller/commands (std_msgs/Float64MultiArray)
# 用途: 精确力控、机械臂操作、物理交互
# 适用场景: 机械臂、力控应用、接触任务
# 重要性: ★★☆☆☆ (本项目未使用)
# 配置要求: 需要定义effort命令接口和状态接口
#
# 4. 位置控制器 (JointPositionController)
# 类型: joint_controllers/JointPositionController
# 功能: 控制关节到达指定位置
# 输入话题: /position_controller/commands (std_msgs/Float64MultiArray)
# 用途: 机械臂位置控制、舵机控制、可调结构
# 适用场景: 机械臂、云台、可调机械结构
# 重要性: ★★★☆☆ (按需启用)
# 配置要求: 需要PID参数调优、位置限制设置
#
# 5. 速度控制器 (JointGroupVelocityController)
# 类型: velocity_controllers/JointGroupVelocityController
# 功能: 控制关节以指定速度运动
# 输入话题: /velocity_controller/commands (std_msgs/Float64MultiArray)
# 用途: 轮子速度控制、传送带控制、风扇速度控制
# 适用场景: 需要精确速度控制的场合
# 重要性: ★★★☆☆ (按需启用)
# 配置要求: 需要PID参数、速度限制设置
#
# 6. 轨迹控制器 (JointTrajectoryController)
# 类型: joint_trajectory_controller/JointTrajectoryController
# 功能: 执行预定义的关节轨迹
# 输入话题: /trajectory_controller/command (trajectory_msgs/JointTrajectory)
# 用途: 机械臂复杂运动、舞蹈动作、重复性任务
# 适用场景: 机械臂、多关节机器人、自动化生产线
# 重要性: ★★★★☆ (机械臂必需)
# 配置要求: 需要轨迹规划参数、关节限制、PID参数
#
# 7. IMU传感器广播器 (IMUSensorBroadcaster)
# 类型: imu_sensor_broadcaster/IMUSensorBroadcaster
# 功能: 发布IMU传感器数据
# 输出话题: /imu/data (sensor_msgs/Imu)
# 用途: 姿态估计、运动检测、导航辅助
# 适用场景: 导航系统、姿态控制、运动分析
# 重要性: ★★★★☆ (导航系统常用)
# 配置要求: 需要IMU标定参数、坐标系配置
#
# 8. 力矩传感器广播器 (ForceTorqueSensorBroadcaster)
# 类型: force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster
# 功能: 发布力矩传感器数据
# 输出话题: /ft_sensor_data (geometry_msgs/WrenchStamped)
# 用途: 力控操作、安全监控、负载检测
# 适用场景: 机械臂末端、协作机器人
# 重要性: ★★☆☆☆ (特殊应用需要)
# 配置要求: 需要传感器标定、坐标系变换
#
# 9. 四轮转向控制器 (FourWheelSteeringController)
# 类型: four_wheel_steering_controller/FourWheelSteeringController
# 功能: 控制四个轮子的转向和速度
# 输入话题: /four_ws_steering_controller/cmd_vel (geometry_msgs/TwistStamped)
# 用途: 阿克曼转向、全向移动、特种车辆
# 适用场景: 汽车、叉车、特种移动平台
# 重要性: ★★☆☆☆ (特定平台需要)
# 配置要求: 需要四轮运动学模型、转向角度限制
#
# 10. 机械臂控制器 (JointGroupPositionController)
# 类型: position_controllers/JointGroupPositionController
# 功能: 多关节位置控制，专门为机械臂设计
# 输入话题: /arm_controller/commands (std_msgs/Float64MultiArray)
# 用途: 机械臂末端位置控制、抓取操作
# 适用场景: 6自由度或以上机械臂
# 重要性: ★★★★☆ (机械臂必需)
# 配置要求: 需要正向/逆向运动学、碰撞检测参数

# ========================================
# 控制器使用指南
# ========================================
# 
# 常用命令:
# ========================================
# 
# 查看所有可用控制器:
# ros2 control list_controller_types
# 
# 查看当前加载的控制器状态:
# ros2 control list_controllers
# 
# 启动控制器:
# ros2 control start_controller <controller_name>
# 
# 停止控制器:
# ros2 control stop_controller <controller_name>
# 
# 重新加载控制器库:
# ros2 control reload_controller_libraries
# 
# 查看控制器详细状态:
# ros2 control list_hardware_interfaces
# 
# ========================================
# 常用话题监控:
# ========================================
# 
# 关节状态:
# ros2 topic echo /joint_states
# 
# 里程计信息:
# ros2 topic echo /odom
# 
# 速度命令:
# ros2 topic echo /cmd_vel
# 
# TF变换树:
# ros2 run tf2_tools view_frames
# 
# ========================================
# 调试技巧:
# ========================================
# 
# 1. 检查控制器是否正确加载:
#    ros2 control list_controllers
# 
# 2. 验证硬件接口是否可用:
#    ros2 control list_hardware_interfaces
# 
# 3. 检查话题是否正确发布:
#    ros2 topic list | grep -E "(joint_states|odom|cmd_vel)"
# 
# 4. 手动发送速度命令测试:
#    ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
# 
# 5. 查看控制器参数:
#    ros2 param list /fishbot_diff_drive_controller
# 
# ========================================
# 性能优化建议:
# ========================================
# 
# 1. 更新频率优化:
#    - 控制器管理器: 100Hz (平衡响应性和CPU负载)
#    - 里程计发布: 50Hz (足够导航使用)
# 
# 2. 协方差矩阵调优:
#    - 实测里程计精度，动态调整协方差值
#    - 考虑传感器融合时的重要性权重
# 
# 3. 运动限制优化:
#    - 根据实际应用场景调整速度和加速度限制
#    - 考虑地面摩擦系数和负载情况
# 
# 4. 安全参数设置:
#    - 合理设置命令超时时间
#    - 启用速度限制和监控
# 
# ========================================
# 故障排除:
# ========================================
# 
# 常见问题及解决方案:
# 
# 1. 控制器无法启动:
#    - 检查控制器名称是否正确
#    - 验证硬件接口是否可用
#    - 查看控制器日志: ros2 log info
# 
# 2. 机器人不响应命令:
#    - 检查/cmd_vel话题是否正确发布
#    - 验证控制器是否处于active状态
#    - 检查速度限制是否过于严格
# 
# 3. 里程计跳变:
#    - 调整协方差矩阵参数
#    - 检查轮径和轮距设置
#    - 验证编码器数据质量
# 
# 4. TF变换错误:
#    - 确认enable_odom_tf设置正确
#    - 检查坐标系名称是否一致
#    - 验证URDF中的关节定义
# 
# ========================================
