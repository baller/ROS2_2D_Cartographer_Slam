#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
import rosbag2_py
import numpy as np
import math
import argparse
import os
import time
import struct

def euler_to_quaternion(roll, pitch, yaw):
    """
    将欧拉角转换为四元数
    
    参数:
    roll (float): 绕x轴的旋转角度（弧度）
    pitch (float): 绕y轴的旋转角度（弧度）
    yaw (float): 绕z轴的旋转角度（弧度）
    
    返回:
    list: 四元数 [x, y, z, w]
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    
    return [qx, qy, qz, qw]


def read_lms_data(filename):
    """读取激光雷达二进制数据文件"""
    with open(filename, 'rb') as f:
        # 读取头部信息
        ang_rng = struct.unpack('f', f.read(4))[0]  # 激光扫描范围
        ang_res = struct.unpack('f', f.read(4))[0]  # 角分辨率
        unit = struct.unpack('f', f.read(4))[0]     # 单位
        
        # 计算每帧数据的点数
        max_dat_len = int(ang_rng / ang_res) + 1
        
        # 读取所有扫描数据
        scans = []
        while True:
            try:
                # 读取时间戳
                milli = struct.unpack('i', f.read(4))[0]
            
                # 读取距离数据
                dat = []
                for _ in range(max_dat_len):
                    val = struct.unpack('H', f.read(2))[0]
                    dat.append(val)
            
                scans.append({'milli': milli, 'dat': dat})
            except Exception as e:
                print(f"Finished reading LMS data: {e}")
                break
        
    return {'ang_rng': ang_rng, 'ang_res': ang_res, 'unit': unit, 'scans': scans}

def read_nav_data(filename):
    """读取车辆轨迹数据"""
    nav_data = []
    with open(filename, 'r') as f:
        # 跳过第一行
        f.readline()
        for line in f:
            if not line.startswith('#') and line.strip():  # 跳过注释行和空行
                data = line.strip().split()
                if len(data) >= 7:  # 确保数据完整
                    nav_data.append({
                        'time': int(data[0]),
                        'ang_x': float(data[1]),  # 横滚角(rad)
                        'ang_y': float(data[2]),  # 俯仰角(rad)
                        'ang_z': float(data[3]),  # 航向角(rad)
                        'shv_x': float(data[4]),  # X位置(m)
                        'shv_y': float(data[5]),  # Y位置(m)
                        'shv_z': float(data[6])   # Z位置(m)
                    })
    return nav_data

def find_closest_nav_data(scan_time, nav_data):
    """找到与扫描时间最接近的车辆位置数据"""
    closest_idx = 0
    min_diff = abs(scan_time - nav_data[0]['time'])
    
    for i, data in enumerate(nav_data):
        diff = abs(scan_time - data['time'])
        if diff < min_diff:
            min_diff = diff
            closest_idx = i
    
    return nav_data[closest_idx]

def main():
    parser = argparse.ArgumentParser(description='Convert lidar and nav data to ROS2 bag')
    parser.add_argument('--lidar_file', default='/root/nav_ws/data/URG_X_20130903_195003.lms',required=False, help='Lidar data file path')
    parser.add_argument('--nav_file',default='/root/nav_ws/data/ld.nav', required=False, help='Navigation data file path')
    parser.add_argument('--output_bag', default='/root/nav_ws/output/robot_data.db3',required=False, help='Output bag file path')
    args = parser.parse_args()
    
    # 初始化ROS2
    rclpy.init()
    
    # 加载数据
    print(f"Loading lidar data from {args.lidar_file}")
    lidar_data = read_lms_data(args.lidar_file)
    print(f"Loaded {len(lidar_data['scans'])} lidar frames")
    print(f"Angle range: {lidar_data['ang_rng']}, Resolution: {lidar_data['ang_res']}, Unit: {lidar_data['unit']}")
    
    print(f"Loading navigation data from {args.nav_file}")
    nav_data = read_nav_data(args.nav_file)
    print(f"Loaded {len(nav_data)} navigation frames")
    
    # 创建bag文件
    writer = rosbag2_py.SequentialWriter()
    
    # 确保输出目录存在
    os.makedirs(os.path.dirname(os.path.abspath(args.output_bag)), exist_ok=True)
    
    # 确保输出文件名以.db3结尾
    output_bag = args.output_bag
    if not output_bag.endswith('.db3'):
        output_bag = output_bag + '.db3'
    
    # 创建存储选项
    storage_options = rosbag2_py._storage.StorageOptions(
        uri=output_bag,
        storage_id='sqlite3'
    )
    
    # 创建转换选项 - 使用正确的参数名称
    converter_options = rosbag2_py._storage.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    writer.open(storage_options, converter_options)
    
    # 创建话题
    topic_info_scan = rosbag2_py._storage.TopicMetadata(
        name='/scan',
        type='sensor_msgs/msg/LaserScan',
        serialization_format='cdr'
    )
    topic_info_odom = rosbag2_py._storage.TopicMetadata(
        name='/odom',
        type='nav_msgs/msg/Odometry',
        serialization_format='cdr'
    )
    topic_info_tf = rosbag2_py._storage.TopicMetadata(
        name='/tf',
        type='tf2_msgs/msg/TFMessage',
        serialization_format='cdr'
    )
    
    writer.create_topic(topic_info_scan)
    writer.create_topic(topic_info_odom)
    writer.create_topic(topic_info_tf)
    
    # 写入数据
    start_time = time.time_ns()
    
    for i, scan in enumerate(lidar_data['scans']):
        # 获取激光扫描数据
        scan_time = scan['milli']
        ranges = scan['dat']
        
        # 找到最接近的导航数据
        closest_nav = find_closest_nav_data(scan_time, nav_data)
        
        # 创建激光雷达消息
        scan_msg = LaserScan()
        scan_msg.header.stamp.sec = int(scan_time / 1000)  # 毫秒转秒
        scan_msg.header.stamp.nanosec = int((scan_time % 1000) * 1000000)  # 余下的毫秒转纳秒
        scan_msg.header.frame_id = 'laser'

        # 设置激光雷达参数 - 根据图片中的信息修改
        # 目前使用的扫描范围是 0° ~ 180°，扫描解析度是 0.5°
        scan_msg.angle_min = 0.0  # 起始角度 0°
        scan_msg.angle_max = math.radians(180.0)  # 结束角度 180°
        scan_msg.angle_increment = math.radians(0.5)  # 角度增量 0.5°
        scan_msg.time_increment = 1.0 / (25.0 * 360)  # 基于25Hz的扫描频率计算
        scan_msg.scan_time = 1.0 / 25.0  # 扫描时间 (25Hz)
        scan_msg.range_min = 0.1
        scan_msg.range_max = 100.0  # 根据Hokuyo UTM/UXM型号的实际最大量程设置
        
        # 将距离值转换为米
        scan_msg.ranges = [float(r) / lidar_data['unit'] for r in ranges]
        
        # 创建里程计消息
        odom_msg = Odometry()
        odom_msg.header.stamp = scan_msg.header.stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # 设置位置
        odom_msg.pose.pose.position.x = closest_nav['shv_x']
        odom_msg.pose.pose.position.y = closest_nav['shv_y']
        odom_msg.pose.pose.position.z = closest_nav['shv_z']
        
        # 设置方向（四元数）
        q = euler_to_quaternion(closest_nav['ang_x'], closest_nav['ang_y'], closest_nav['ang_z'])
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        
        # 设置速度（假设速度信息不可用）
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        
        # 创建 TF 消息
        tf_msg = TFMessage()
        
        # 添加 odom -> base_link 变换
        transform1 = TransformStamped()
        transform1.header.stamp = scan_msg.header.stamp
        transform1.header.frame_id = 'odom'
        transform1.child_frame_id = 'base_link'
        
        # 设置位置
        transform1.transform.translation.x = closest_nav['shv_x']
        transform1.transform.translation.y = closest_nav['shv_y']
        transform1.transform.translation.z = closest_nav['shv_z']
        
        # 设置方向（四元数）
        transform1.transform.rotation.x = q[0]
        transform1.transform.rotation.y = q[1]
        transform1.transform.rotation.z = q[2]
        transform1.transform.rotation.w = q[3]
        
        # 添加 base_link -> laser 变换
        transform2 = TransformStamped()
        transform2.header.stamp = scan_msg.header.stamp
        transform2.header.frame_id = 'base_link'
        transform2.child_frame_id = 'laser'
        
        # 设置激光雷达相对于车辆的位置（通常激光雷达安装在车辆前部）
        transform2.transform.translation.x = 0.0  # 假设激光雷达在车辆前方 0 米处
        transform2.transform.translation.y = 0.0
        transform2.transform.translation.z = 0.0  # 假设激光雷达高于车辆 0 米
        
        # 设置激光雷达相对于车辆的方向（通常与车辆方向一致）
        transform2.transform.rotation.x = 0.0
        transform2.transform.rotation.y = 0.0
        transform2.transform.rotation.z = 0.0
        transform2.transform.rotation.w = 1.0
        
        # 将两个变换添加到 TF 消息中
        tf_msg.transforms = [transform1, transform2]
        
        # 写入bag
        timestamp_ns = start_time + i * 100_000_000  # 每帧间隔100ms
        
        writer.write(
            '/scan',
            serialize_message(scan_msg),
            timestamp_ns
        )
        
        writer.write(
            '/odom',
            serialize_message(odom_msg),
            timestamp_ns
        )
        
        writer.write(
            '/tf',
            serialize_message(tf_msg),
            timestamp_ns
        )
        
        if i % 100 == 0:
            print(f"Processed {i}/{len(lidar_data['scans'])} frames")
        
        
    
    print(f"Finished writing frames to {output_bag}")
    
    # 关闭bag
    del writer
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
