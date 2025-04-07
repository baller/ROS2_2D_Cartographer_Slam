#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Vector3
from tf2_msgs.msg import TFMessage
import rosbag2_py
import numpy as np
import math
import argparse
import os
import time
import struct
from collections import defaultdict

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

def read_imu_data(filename):
    """读取惯性导航(IMU)数据文件"""
    imu_data = []
    with open(filename, 'r') as f:
        for line in f:
            if line.startswith('IMU') and len(line.strip().split()) >= 6:
                data = line.strip().split()
                # IMU 71406999 0 0 -1.64934 3.71048 -138.031
                # 将朝向角度从度转换为弧度
                yaw_deg = float(data[6])
                yaw_rad = math.radians(yaw_deg)
                
                # 提取横滚角和俯仰角，并转换为弧度
                roll_deg = float(data[4])
                pitch_deg = float(data[5])
                roll_rad = math.radians(roll_deg)
                pitch_rad = math.radians(pitch_deg)
                
                imu_data.append({
                    'timestamp': int(data[1]),  # 毫秒时间戳
                    'frame_rate': float(data[2]),
                    'valid': int(data[3]) >= 180,  # 数据>=180表示有效
                    'roll': roll_rad,
                    'pitch': pitch_rad,
                    'yaw': yaw_rad
                })
    return imu_data

def read_encoder_data(filename):
    """读取编码器数据文件"""
    encoder_data = []
    with open(filename, 'r') as f:
        for line in f:
            if line.startswith('E') and len(line.strip().split()) >= 4:
                data = line.strip().split()
                # E 71403642 1 13832
                encoder_data.append({
                    'timestamp': int(data[1]),  # 毫秒时间戳
                    'pulses': int(data[3])
                })
    return encoder_data

def process_encoder_data(encoder_data, imu_data):
    """处理编码器数据以计算位置"""
    # 提取时间戳和计数
    wheel_millis = np.array([data['timestamp'] for data in encoder_data])
    wheel_counts = np.array([data['pulses'] for data in encoder_data])
    
    # 提取IMU时间戳和偏航角
    imu_millis = np.array([data['timestamp'] for data in imu_data if data['valid']])
    imu_yaws = np.array([data['yaw'] for data in imu_data if data['valid']])
    #！！！！！！！！！重中之重！！！！！！！！
    imu_yaws =  -(imu_yaws-imu_yaws[0])  # 偏航角修正

    # 计算轮式里程计的差分，即每次更新的计数变化量
    delta_counts = np.diff(wheel_counts, prepend=wheel_counts[0])
    # 对计数变化量进行修正，如果计数减少，则加上30000，以处理计数溢出问题
    delta_counts = np.where(delta_counts < 0, delta_counts + 30000, delta_counts)
    # 将计数变化量转换为实际的距离变化量，单位为米
    delta_s = delta_counts * 0.003846154  # 单位：米
    # 根据IMU的时间戳和偏航角，插值计算出与轮式里程计时间戳对应的偏航角
    yaws = np.zeros_like(wheel_millis, dtype=float)
    for i, wm in enumerate(wheel_millis):
        # 找到imu_millis中与当前wheel_millis最接近的索引
        closest_idx = np.argmin(np.abs(imu_millis - wm))
        # 使用该索引获取对应的偏航角
        yaws[i] = imu_yaws[closest_idx]
    
    
    # 初始化位置
    x = np.zeros_like(wheel_millis, dtype=float)
    y = np.zeros_like(wheel_millis, dtype=float)
    
    # 设置初始位置为(0, 0)
    x[0] = 0
    y[0] = 0
    
    # 逐步计算位置 - 现在使用旋转后的偏航角
    for i in range(1, len(wheel_millis)):
        # 使用当前的偏航角计算位移
        dx = delta_s[i] * np.cos(yaws[i])
        dy = delta_s[i] * np.sin(yaws[i])
        
        # 更新位置
        x[i] = x[i-1] + dx
        y[i] = y[i-1] + dy
    
    
    position_data = []
    for i in range(len(encoder_data)):
        position_data.append({
            'timestamp': encoder_data[i]['timestamp'],
            'x': -y[i] if i < len(y) else 0.0,#注意要反转xy轴，并对x取反，以匹配激光雷达坐标系
            'y': x[i] if i < len(x) else 0.0,
            'yaw': yaws[i] if i < len(yaws) else 0.0,
            'distance': delta_s[i] if i < len(delta_s) else 0.0,
        })
    
    return position_data

def find_closest_position_data(scan_time, nav_data, last_idx=0):
    """
    使用缓存的上次查找位置，找到与扫描时间最接近的车辆位置数据
    
    参数:
    scan_time: 目标扫描时间戳
    nav_data: 导航数据列表
    last_idx: 上次查找到的索引位置
    
    返回:
    (与目标时间戳最接近的导航数据, 找到的索引)
    """
    if not nav_data:
        raise ValueError("导航数据为空")
    
    # 从上次找到的位置开始搜索
    start_idx = min(last_idx, len(nav_data) - 1)
    
    # 如果当前时间戳比上次找到的位置的时间戳小，需要向前搜索
    if scan_time < nav_data[start_idx]['timestamp']:
        # 向前搜索
        closest_idx = start_idx
        min_diff = abs(scan_time - nav_data[start_idx]['timestamp'])
        
        for i in range(start_idx - 1, -1, -1):
            diff = abs(scan_time - nav_data[i]['timestamp'])
            if diff < min_diff:
                min_diff = diff
                closest_idx = i
            # 如果时间戳开始增大，说明已经越过了最接近的点
            elif nav_data[i]['timestamp'] < scan_time:
                break
    else:
        # 向后搜索
        closest_idx = start_idx
        min_diff = abs(scan_time - nav_data[start_idx]['timestamp'])
        
        for i in range(start_idx + 1, len(nav_data)):
            diff = abs(scan_time - nav_data[i]['timestamp'])
            if diff < min_diff:
                min_diff = diff
                closest_idx = i
            # 如果时间戳开始减小，说明已经越过了最接近的点
            elif nav_data[i]['timestamp'] > scan_time:
                break
    
    return nav_data[closest_idx], closest_idx


def find_closest_state(timestamp, imu_data, position_data, imu_index_cache=None, pos_index_cache=None):
    """快速查找最接近给定时间戳的IMU和位置数据"""
    # 使用缓存的上一次索引作为起点，加速查找过程
    i_start = imu_index_cache if imu_index_cache is not None else 0
    p_start = pos_index_cache if pos_index_cache is not None else 0
    
    # 查找最近的IMU数据
    closest_imu_idx = i_start
    min_diff_imu = abs(timestamp - imu_data[i_start]['timestamp'])
    
    for i in range(i_start, len(imu_data)):
        diff = timestamp - imu_data[i]['timestamp']
        abs_diff = abs(diff)
        
        if diff > 0 and abs_diff < min_diff_imu:
            min_diff_imu = abs_diff
            closest_imu_idx = i
        elif diff < 0:
            # 如果已经超过了目标时间戳，就停止搜索
            break
    
    # 查找最近的位置数据
    closest_pos_idx = p_start
    min_diff_pos = abs(timestamp - position_data[p_start]['timestamp'])
    
    for i in range(p_start, len(position_data)):
        diff = timestamp - position_data[i]['timestamp']
        abs_diff = abs(diff)
        
        if diff > 0 and abs_diff < min_diff_pos:
            min_diff_pos = abs_diff
            closest_pos_idx = i
        elif diff < 0:
            # 如果已经超过了目标时间戳，就停止搜索
            break
    
    # 获取最接近的数据
    imu = imu_data[closest_imu_idx]
    pos = position_data[closest_pos_idx]
    
    return {
        'imu': imu,
        'pos': pos,
        'imu_idx': closest_imu_idx,
        'pos_idx': closest_pos_idx
    }

def create_tf_message(timestamp_ms, x, y, q, ros_time_sec, ros_time_nanosec):
    """创建TF消息，包含所有必要的变换"""
    tf_msg = TFMessage()
    
    # 添加 odom -> base_link 变换
    transform1 = TransformStamped()
    transform1.header.stamp.sec = ros_time_sec
    transform1.header.stamp.nanosec = ros_time_nanosec
    transform1.header.frame_id = 'odom'
    transform1.child_frame_id = 'base_link'
    
    # 设置位置
    transform1.transform.translation.x = x
    transform1.transform.translation.y = y
    transform1.transform.translation.z = 0.0  # 假设在平面上移动
    
    # 设置方向（四元数）
    transform1.transform.rotation.x = q[0]
    transform1.transform.rotation.y = q[1]
    transform1.transform.rotation.z = q[2]
    transform1.transform.rotation.w = q[3]
    
    # 添加 base_link -> laser 变换
    transform2 = TransformStamped()
    transform2.header.stamp.sec = ros_time_sec
    transform2.header.stamp.nanosec = ros_time_nanosec
    transform2.header.frame_id = 'base_link'
    transform2.child_frame_id = 'laser'
    
    # 设置激光雷达相对于车辆的位置
    transform2.transform.translation.x = 0.1  # 假设激光雷达在车辆前方 0.1 米处
    transform2.transform.translation.y = 0.0
    transform2.transform.translation.z = 0.05  # 假设激光雷达高于车辆基座 0.05 米
    
    # 设置激光雷达相对于车辆的方向
    transform2.transform.rotation.x = 0.0
    transform2.transform.rotation.y = 0.0
    transform2.transform.rotation.z = 0.0
    transform2.transform.rotation.w = 1.0
    
    # 添加 base_link -> imu 变换
    transform3 = TransformStamped()
    transform3.header.stamp.sec = ros_time_sec
    transform3.header.stamp.nanosec = ros_time_nanosec
    transform3.header.frame_id = 'base_link'
    transform3.child_frame_id = 'imu'
    
    # 设置IMU相对于车辆的位置
    transform3.transform.translation.x = 0.0
    transform3.transform.translation.y = 0.0
    transform3.transform.translation.z = 0.0
    
    # IMU与车辆朝向一致
    transform3.transform.rotation.x = 0.0
    transform3.transform.rotation.y = 0.0
    transform3.transform.rotation.z = 0.0
    transform3.transform.rotation.w = 1.0
    
    # 将三个变换添加到 TF 消息中
    tf_msg.transforms = [transform1, transform2, transform3]
    
    return tf_msg

def main():
    parser = argparse.ArgumentParser(description='Convert lidar, IMU and encoder data to ROS2 bag')
    parser.add_argument('--lidar_file', default='/root/nav_ws/data/URG_X_20130903_195003.lms', required=False, help='Lidar data file path')
    parser.add_argument('--imu_file', default='/root/nav_ws/data/InterSense_X_20130903_195003.txt', required=False, help='IMU data file path')
    parser.add_argument('--encoder_file', default='/root/nav_ws/data/COMPort_X_20130903_195003.txt', required=False, help='Encoder data file path')
    parser.add_argument('--output_bag', default='/root/nav_ws/output/fusion_robot_data.db3', required=False, help='Output bag file path')
    args = parser.parse_args()
    
    # 初始化ROS2
    rclpy.init()
    
    # 加载数据
    print(f"Loading lidar data from {args.lidar_file}")
    lidar_data = read_lms_data(args.lidar_file)
    print(f"Loaded {len(lidar_data['scans'])} lidar frames")
    print(f"Angle range: {lidar_data['ang_rng']}, Resolution: {lidar_data['ang_res']}, Unit: {lidar_data['unit']}")
    
    print(f"Loading IMU data from {args.imu_file}")
    imu_data = read_imu_data(args.imu_file)
    print(f"Loaded {len(imu_data)} IMU frames")
    
    print(f"Loading encoder data from {args.encoder_file}")
    encoder_data = read_encoder_data(args.encoder_file)
    print(f"Loaded {len(encoder_data)} encoder frames")
    
    # 处理编码器数据，计算位置
    print("Processing encoder data for odometry...")
    # position_data = process_encoder_data(encoder_data)
    
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
    
    # 创建转换选项
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
    topic_info_imu = rosbag2_py._storage.TopicMetadata(
        name='/imu',
        type='sensor_msgs/msg/Imu',
        serialization_format='cdr'
    )
    topic_info_tf = rosbag2_py._storage.TopicMetadata(
        name='/tf',
        type='tf2_msgs/msg/TFMessage',
        serialization_format='cdr'
    )
    
    writer.create_topic(topic_info_scan)
    writer.create_topic(topic_info_odom)
    writer.create_topic(topic_info_imu)
    writer.create_topic(topic_info_tf)
    
    # 写入数据
    start_time = time.time_ns()
    
    print("处理激光雷达数据...")
    last_imu_idx = 0
    last_pos_idx = 0
    global_x = 0
    global_y = 0
    nav_data = process_encoder_data(encoder_data, imu_data)
    # 按原始时间戳处理激光雷达数据
    last_pos_idx=0
    for i, scan in enumerate(lidar_data['scans']):
        if i <100:
            continue
        timestamp_ms = scan['milli']
        ros_time_sec = int(timestamp_ms / 1000)
        ros_time_nanosec = int((timestamp_ms % 1000) * 1000000)
        timestamp_ns = start_time + (timestamp_ms - lidar_data['scans'][0]['milli']) * 1000000
        
        # 找到最接近的导航数据
        closest_nav = find_closest_position_data(timestamp_ms, nav_data, last_pos_idx)
        last_pos_idx = closest_nav[1]
        closest_nav = closest_nav[0]

        # 创建并写入激光雷达消息
        scan_msg = LaserScan()
        scan_msg.header.stamp.sec = ros_time_sec
        scan_msg.header.stamp.nanosec = ros_time_nanosec
        scan_msg.header.frame_id = 'laser'
        
        # 设置激光雷达参数
        scan_msg.angle_min = 0.0  # 起始角度 0°
        scan_msg.angle_max = math.radians(180.0)  # 结束角度
        scan_msg.angle_increment =  math.radians(0.5)  # 角度增量
        scan_msg.time_increment = 1.0 / (25.0 * 360)  # 基于25Hz的扫描频率计算
        scan_msg.scan_time = 1.0 / 25.0  # 扫描时间 (25Hz)
        scan_msg.range_min = 0.1
        scan_msg.range_max = 100.0  # 根据激光雷达的实际最大量程设置
        
        # 将距离值转换为米
        scan_msg.ranges = [float(r) / lidar_data['unit'] for r in scan['dat']]
        
        # 写入激光雷达消息
        writer.write('/scan', serialize_message(scan_msg), timestamp_ns)

        # 创建里程计消息
        odom_msg = Odometry()
        odom_msg.header.stamp = scan_msg.header.stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # 设置位置
        odom_msg.pose.pose.position.x = closest_nav['x']
        odom_msg.pose.pose.position.y = closest_nav['y']
        odom_msg.pose.pose.position.z = 0.0
        
        # 设置方向（四元数）
        q = euler_to_quaternion(0.0, 0.0, closest_nav['yaw'])
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        
        # 设置速度（假设速度信息不可用）
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        # 写入里程计消息
        writer.write('/odom', serialize_message(odom_msg), timestamp_ns)

        

        # 创建IMU消息
        imu_msg = Imu()
        imu_msg.header.stamp.sec = ros_time_sec
        imu_msg.header.stamp.nanosec = ros_time_nanosec
        imu_msg.header.frame_id = 'imu'
        
        # 方向四元数
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]
        
        
        
        # 写入IMU消息
        writer.write('/imu', serialize_message(imu_msg), timestamp_ns)
        # 创建 TF 消息
        tf_msg = TFMessage()
        
        # 添加 odom -> base_link 变换
        transform1 = TransformStamped()
        transform1.header.stamp = scan_msg.header.stamp
        transform1.header.frame_id = 'odom'
        transform1.child_frame_id = 'base_link'
        
        # 设置位置
        transform1.transform.translation.x = closest_nav['x']
        transform1.transform.translation.y = closest_nav['y']
        transform1.transform.translation.z = 0.0
        
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
        
        # 添加 base_link -> imu 变换
        transform3 = TransformStamped()
        transform3.header.stamp = scan_msg.header.stamp
        transform3.header.frame_id = 'base_link'
        transform3.child_frame_id = 'imu'
        
        # 设置imu相对于车辆的位置（通常imu安装在车辆前部）
        transform3.transform.translation.x = 0.0  # 假设imu在车辆前方 0 米处
        transform3.transform.translation.y = 0.0
        transform3.transform.translation.z = 0.0  # 假设imu高于车辆 0 米
        
        # 设置imu相对于车辆的方向（通常与车辆方向一致）
        transform3.transform.rotation.x = q[0]
        transform3.transform.rotation.y = q[1]
        transform3.transform.rotation.z = q[2]
        transform3.transform.rotation.w = q[3]
        # 将两个变换添加到 TF 消息中
        tf_msg.transforms = [transform1, transform2, transform3]
        writer.write('/tf', serialize_message(tf_msg), timestamp_ns)
        
        if i % 500 == 0:
            print(f"已处理 {i}/{len(lidar_data['scans'])} 激光雷达帧")
    
    
    
    print(f"完成写入所有数据到 {output_bag}")
    
    # 关闭bag
    del writer
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
