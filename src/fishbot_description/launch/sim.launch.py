import launch
import launch.event_handlers
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # 获取URDF和World文件路径
    robot_name_in_model = "fishbot"
    urdf_tutorial_path = get_package_share_directory('fishbot_description')
    default_model_path = urdf_tutorial_path + '/urdf/fishbot.urdf.xacro'
    default_world_path = urdf_tutorial_path + '/world/3d.world'
    
    # 声明可从命令行传入的URDF文件参数
    # ros2 launch fishbot_description sim.launch.py model:=/path/to/urdf
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),
        description='URDF 的绝对路径')
    
    # 通过xacro处理URDF文件生成机器人描述
    # xacro /path/to/fishbot.urdf.xacro
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str)
  	
    # 发布机器人TF变换和机器人模型
    # ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /path/to/fishbot.urdf.xacro)"
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # 发布关节状态
    # ros2 run joint_state_publisher joint_state_publisher --ros-args -p robot_description:="$(xacro /path/to/fishbot.urdf.xacro)"
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # 启动rviz2
    # ros2 run rviz2 rviz2
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2'
    )

    # 启动Gazebo仿真环境并加载世界文件
    # ros2 launch gazebo_ros gazebo.launch.py world:=/path/to/area.world verbose:=true
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'gazebo_ros'), '/launch', '/gazebo.launch.py']),
      	launch_arguments=[('world', default_world_path),('verbose','true')]
    )
    
    # 在Gazebo中生成机器人模型
    # ros2 run gazebo_ros spawn_entity.py -topic /robot_description -entity fishbot
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', robot_name_in_model, ])
    
    # 加载关节状态广播控制器
    # ros2 control load_controller --set-state active fishbot_joint_state_broadcaster
    load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'fishbot_joint_state_broadcaster'],
        output = 'screen'
    )

    # 加载差分驱动控制器
    # ros2 control load_controller --set-state active fishbot_diff_drive_controller
    load_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','fishbot_diff_drive_controller'], 
        output = 'screen'
    )
    fake_basel_cmd4 = launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'])
    fake_basel_cmd5 = launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_footprint', 'base_link'])
    fake_basel_cmd6 = launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0.0', '0', '0', '0', 'map', 'odom'])
    
    # 返回所有要执行的动作
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        robot_state_publisher_node,
        # fake_basel_cmd4,
        fake_basel_cmd5,
        fake_basel_cmd6,
        # gazebo可以自己发布joint的tf
        # joint_state_publisher_node,
        # rviz_node,
        launch_gazebo,
        spawn_entity_node,
        
        # 机器人生成完成后加载关节状态控制器
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=spawn_entity_node,
                on_exit=[load_joint_state_controller],
            )
        ),
        # 关节状态控制器加载完成后加载差分驱动控制器
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_diff_drive_controller],
            )
        ),
    ])
