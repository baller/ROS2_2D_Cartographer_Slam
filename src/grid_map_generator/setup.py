from setuptools import setup
import os  # 确保这一行存在
from glob import glob  # 如果使用glob函数，也需要导入

package_name = 'grid_map_generator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         ('share/' + package_name, ['package.xml']),
        # 添加启动文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='1440864812@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'grid_map_generator = grid_map_generator.grid_map_node:main',
             'data_to_rosbag = grid_map_generator.data_to_rosbag:main',
        'grid_map_rosbag_node = grid_map_generator.grid_map_rosbag_node:main',
        'imu_odom_laser_to_rosbag = grid_map_generator.imu_odom_laser_to_rosbag:main',
        ],
    },
)
