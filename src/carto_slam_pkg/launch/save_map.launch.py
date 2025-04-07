import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('carto_slam_pkg')
    
    # 参数声明
    pbstream_filename = LaunchConfiguration('pbstream_filename')
    map_filestem = LaunchConfiguration('map_filestem')
    
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'pbstream_filename',
            default_value='/root/nav_ws/src/carto_slam_pkg/maps/map.pbstream',
            description='SLAM结果pbstream文件路径'
        ),
        DeclareLaunchArgument(
            'map_filestem',
            default_value=os.path.join(pkg_dir, 'maps', 'map'),
            description='地图文件名前缀'
        ),
        
        # 执行地图转换命令
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'cartographer_ros', 'cartographer_pbstream_to_ros_map',
                '-pbstream_filename', pbstream_filename,
                '-map_filestem', map_filestem,
                '-resolution', '0.05'
            ],
            output='screen'
        )
    ])
