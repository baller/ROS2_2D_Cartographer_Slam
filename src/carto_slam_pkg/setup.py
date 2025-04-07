from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'carto_slam_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 添加启动文件
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # 添加配置文件
        (os.path.join('share', package_name, 'config'),
            glob('config/*.lua')),
        # 添加URDF文件
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')),
        # 添加RViz配置
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
        # 地图文件夹
        (os.path.join('share', package_name, 'maps'), []),
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
        ],
    },
)
