from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    # 声明参数
    lidar_file = LaunchConfiguration('lidar_file')
    nav_file = LaunchConfiguration('nav_file')
    output_dir = LaunchConfiguration('output_dir')
    output_bag = LaunchConfiguration('output_bag')
    
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'lidar_file',
            default_value='/root/nav_ws/data/URG_X_20130903_195003.lms',
            description='激光雷达数据文件路径'
        ),
        DeclareLaunchArgument(
            'nav_file',
            default_value='/root/nav_ws/data/ld.nav',
            description='导航数据文件路径'
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value='/root/nav_ws/output',
            description='输出目录路径'
        ),
        DeclareLaunchArgument(
            'output_bag',
            default_value='/root/nav_ws/output/robot_data.db3',  # 添加.db3扩展名
            description='输出bag文件路径'
        ),
        
        # 步骤1：运行数据转换节点，生成bag文件
        # Node(
        #     package='grid_map_generator',
        #     executable='data_to_rosbag',
        #     name='data_to_rosbag',
        #     output='screen',
        #     arguments=['--lidar_file', lidar_file, 
        #                '--nav_file', nav_file, 
        #                '--output_bag', output_bag]
        # ),
        
        # 步骤2：等待1秒，确保bag文件已经生成
        TimerAction(
            period=1.0,
            actions=[
                # 步骤3：播放bag文件
                ExecuteProcess(
                    cmd=['ros2', 'bag', 'play', output_bag],
                    output='screen'
                ),
            ]
        ),
        
        # 步骤4：启动栅格地图节点
        Node(
            package='grid_map_generator',
            executable='grid_map_rosbag_node',
            name='grid_map_rosbag_node',
            output='screen',
            parameters=[{
                'output_dir': output_dir,
                'grid_size': 0.1
            }]
        )
    ])
