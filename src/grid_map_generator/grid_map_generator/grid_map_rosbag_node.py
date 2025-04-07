#!/usr/bin/env python3
from collections import deque
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from tf2_msgs.msg import TFMessage
import numpy as np
import matplotlib.pyplot as plt
import os
import math
from datetime import datetime
import tf2_ros
import tf_transformations
import threading
import time

class GridMapRosbagNode(Node):
    def __init__(self):
        super().__init__('grid_map_rosbag_node')
        
        # 参数声明
        self.declare_parameter('grid_size', 0.1)
        self.declare_parameter('save_images', True)
        self.declare_parameter('process_interval', 5.0)  # 新增处理间隔参数
        
        self.grid_size = self.get_parameter('grid_size').value
        self.save_images = self.get_parameter('save_images').value
        self.process_interval = self.get_parameter('process_interval').value
        
        # 数据存储队列
        self.scan_queue = deque()
        self.odom_queue = deque()
        
        # 创建发布器
        self.map_publisher = self.create_publisher(OccupancyGrid, 'grid_map', 10)
        self.path_publisher = self.create_publisher(Path, 'vehicle_path', 10)
        self.points_publisher = self.create_publisher(MarkerArray, 'lidar_points', 10)
        
        # 创建订阅器
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # 初始化变量
        self.all_global_points = []
        self.vehicle_path = []
        self.processing = False
        
        # 设置处理定时器（使用参数化的间隔）
        self.create_timer(self.process_interval, self.process_data)
        self.get_logger().info(f"定时器已启动，处理间隔: {self.process_interval}秒")

    def scan_callback(self, msg):
        """存储扫描数据并记录时间戳"""
        self.scan_queue.append({
            'stamp': msg.header.stamp,
            'ranges': msg.ranges,
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'range_max': msg.range_max
        })
        self.get_logger().debug(f"收到激光数据，队列长度: {len(self.scan_queue)}", throttle_duration_sec=5)
    def odom_callback(self, msg):
        """存储里程计数据并记录时间戳"""
        self.odom_queue.append({
            'stamp': msg.header.stamp,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'orientation': msg.pose.pose.orientation
        })
        # 更新车辆轨迹
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.vehicle_path.append(pose)
        self.get_logger().debug(f"收到里程计数据，队列长度: {len(self.odom_queue)}", throttle_duration_sec=5)
    def find_closest_odom(self, scan_stamp):
        """查找最近的odom数据"""
        closest = None
        min_diff = float('inf')
        for odom in self.odom_queue:
            diff = abs(odom['stamp'].sec - scan_stamp.sec) + \
                   abs(odom['stamp'].nanosec - scan_stamp.nanosec) * 1e-9
            if diff < min_diff:
                min_diff = diff
                closest = odom
        return closest
    def process_data(self):
        """定时触发的数据处理流程"""
        if self.processing:
            self.get_logger().warning("已有处理正在进行，跳过本次触发")
            return
            
        if not self.scan_queue:
            self.get_logger().warning("扫描数据队列为空，跳过处理")
            return
            
        self.processing = True
        self.get_logger().info("="*50)
        self.get_logger().info(f"开始处理数据，扫描队列: {len(self.scan_queue)}, 里程计队列: {len(self.odom_queue)}")
        
        try:
            processed_count = 0
            while self.scan_queue:
                scan = self.scan_queue.popleft()  # 从队列头部取数据
                odom = self.find_closest_odom(scan['stamp'])
                
                if not odom:
                    self.get_logger().warning(f"未找到匹配的里程计数据，丢弃扫描数据（时间戳: {scan['stamp']}）")
                    continue
                
                # 四元数转欧拉角
                q = odom['orientation']
                roll, pitch, yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
                
                # 处理每个激光点
                for i, distance in enumerate(scan['ranges']):
                    if 0.1 < distance < scan['range_max']:
                        angle = scan['angle_min'] + i * scan['angle_increment']
                        x_local = distance * math.cos(angle)
                        y_local = distance * math.sin(angle)
                        
                        # 全局坐标转换
                        x_global = x_local * math.cos(yaw) - y_local * math.sin(yaw) + odom['x']
                        y_global = x_local * math.sin(yaw) + y_local * math.cos(yaw) + odom['y']
                        self.all_global_points.append((x_global, y_global))
                
                processed_count += 1
                if processed_count % 100 == 0:
                    self.get_logger().info(f"已处理 {processed_count} 帧扫描数据")
            
            # 生成并发布地图
            if self.all_global_points:
                success = self.create_grid_map()
                if success:
                    self.publish_grid_map()
                    self.publish_vehicle_path()
                    self.publish_lidar_points()
                    if self.save_images:
                        threading.Thread(target=self.save_visualization).start()
            else:
                self.get_logger().warning("未处理任何有效激光点")
            
        except Exception as e:
            self.get_logger().error(f"数据处理失败: {str(e)}")
        finally:
            self.processing = False
            self.get_logger().info(f"数据处理完成，共处理 {processed_count} 帧数据")
            self.get_logger().info("="*50)
    def euler_from_quaternion(self, x, y, z, w):
        """四元数转欧拉角"""
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        
        t2 = 2.0 * (w * y - z * x)
        pitch = math.asin(t2) if abs(t2) < 1.0 else math.copysign(math.pi/2, t2)
        
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        
        return roll, pitch, yaw
    def create_grid_map(self):
        """从全局点创建栅格地图"""
        if not self.all_global_points:
            self.get_logger().warning("没有激光点数据，无法创建栅格地图")
            return False
            
        # 根据实际数据点的分布确定合适的范围
        x_coords, y_coords = zip(*self.all_global_points)
        x_min, x_max = min(x_coords), max(x_coords)
        y_min, y_max = min(y_coords), max(y_coords)
        
        # 给范围添加一些余量
        margin = 5.0  # 米，增加边界余量
        self.x_range = (x_min - margin, x_max + margin)
        self.y_range = (y_min - margin, y_max + margin)
        
        # 计算栅格数量
        x_cells = max(1, int((self.x_range[1] - self.x_range[0]) / self.grid_size))
        y_cells = max(1, int((self.y_range[1] - self.y_range[0]) / self.grid_size))
        
        self.get_logger().info(f"创建栅格地图，范围: X={self.x_range}, Y={self.y_range}, 大小: {x_cells}x{y_cells}")
        
        # 初始化栅格地图（0表示未知，正值表示占用可能性）
        vote_map = np.zeros((x_cells, y_cells))
        
        # 对每个点进行投票
        points_processed = 0
        for x, y in self.all_global_points:
            # 将全局坐标转换为栅格索引
            if self.x_range[0] <= x < self.x_range[1] and self.y_range[0] <= y < self.y_range[1]:
                i = min(x_cells - 1, max(0, int((x - self.x_range[0]) / self.grid_size)))
                j = min(y_cells - 1, max(0, int((y - self.y_range[0]) / self.grid_size)))
                vote_map[i, j] += 1
                points_processed += 1
        
        # 二值化栅格地图（大于阈值的认为是占用）
        threshold = 1  # 可以根据实际情况调整
        binary_map = vote_map > threshold
        
        self.vote_map = vote_map
        self.binary_map = binary_map
        
        self.get_logger().info(f"栅格地图创建完成，大小: {binary_map.shape}")
        self.get_logger().info(f"处理了 {points_processed}/{len(self.all_global_points)} 个点")
        self.get_logger().info(f"投票地图非零元素数量: {np.count_nonzero(vote_map)}")
        self.get_logger().info(f"投票地图最大值: {np.max(vote_map) if vote_map.size > 0 else 0}")
        
        return True
    
    def update_and_publish_map(self):
        """更新栅格地图并发布"""
        # if not self.tf_available:
        #     self.get_logger().warning("TF变换尚未可用，等待变换可用后再更新地图...")
        #     return
            
        if len(self.all_global_points) > 0:
            self.get_logger().info(f"处理 {len(self.all_global_points)} 个激光点创建栅格地图...")
            
            if self.create_grid_map():
                self.publish_grid_map()
                self.publish_vehicle_path()
                self.publish_lidar_points()
                
                # 保存可视化结果
                if self.save_images:
                    threading.Thread(target=self.save_visualization).start()
            
            self.get_logger().info("栅格地图更新完成")
        else:
            self.get_logger().warning("没有收到足够的激光点数据，等待更多数据...")
    
    def publish_grid_map(self):
        """发布OccupancyGrid消息"""
        if self.binary_map is None:
            return
            
        # 创建栅格地图消息
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = "odom"
        
        # 设置地图元数据
        grid_msg.info.resolution = self.grid_size
        grid_msg.info.width = self.binary_map.shape[0]
        grid_msg.info.height = self.binary_map.shape[1]
        
        # 设置地图原点
        grid_msg.info.origin.position.x = self.x_range[0]
        grid_msg.info.origin.position.y = self.y_range[0]
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        
        # 将二值地图转换为占用栅格数据
        # 在OccupancyGrid中，0-100表示占用概率，-1表示未知
        grid_data = np.ones(self.binary_map.shape, dtype=np.int8) * -1  # 默认为未知
        grid_data[self.binary_map] = 100  # 占用的栅格设置为100
        grid_data[~self.binary_map & (self.vote_map > 0)] = 0  # 空闲的栅格设置为0
        
        # 扁平化数组并转换为Python列表
        grid_msg.data = grid_data.T.flatten().tolist()  # 注意转置以匹配ROS坐标系
        
        # 发布栅格地图
        self.map_publisher.publish(grid_msg)
        self.get_logger().info(f"已发布栅格地图 ({grid_msg.info.width}x{grid_msg.info.height})")
    
    def publish_vehicle_path(self):
        """发布车辆轨迹作为Path消息"""
        if not self.vehicle_path:
            return
            
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "odom"
        path_msg.poses = self.vehicle_path
        
        self.path_publisher.publish(path_msg)
        self.get_logger().info(f"已发布车辆轨迹 ({len(self.vehicle_path)} 点)")
    
    def publish_lidar_points(self):
        """发布激光点云作为MarkerArray"""
        if not self.all_global_points:
            return
            
        # 由于点云可能很大，限制发布的点数量
        
        step = 1
        
        marker_array = MarkerArray()
        
        # 创建点云标记
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lidar_points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.7
        
        for i in range(0, len(self.all_global_points), step):
            x, y = self.all_global_points[i]
            point = Point()
            point.x = float(x)
            point.y = float(y)
            point.z = 0.1  # 稍微抬高以便于可视化
            
            marker.points.append(point)
        
        marker_array.markers.append(marker)
        self.points_publisher.publish(marker_array)
        self.get_logger().info(f"已发布激光点云 ({len(marker.points)} 点)")
    
    def save_visualization(self):
        """保存可视化结果为图像文件"""
        try:
            # 设置中文字体支持
            plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
            plt.rcParams['axes.unicode_minus'] = False
            
            # 创建输出目录
            output_dir = os.path.join(os.getcwd(), 'grid_map_output')
            os.makedirs(output_dir, exist_ok=True)
            
            # 生成带时间戳的文件名前缀
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            prefix = f"grid_map"
            
            # 创建图形1: 栅格地图与车辆轨迹
            plt.figure(figsize=(12, 10))
            
            # 绘制车辆轨迹
            if self.vehicle_path:
                path_x = [pose.pose.position.x for pose in self.vehicle_path]
                path_y = [pose.pose.position.y for pose in self.vehicle_path]
                plt.plot(path_x, path_y, 'r-', linewidth=2, label='Vehical Path')
            
            # 绘制投票地图作为热图背景
            if self.vote_map is not None and self.vote_map.size > 0 and np.max(self.vote_map) > 0:
                im = plt.imshow(self.vote_map.T, cmap='jet', origin='lower', alpha=0.7,
                              extent=[self.x_range[0], self.x_range[1], self.y_range[0], self.y_range[1]])
                cbar = plt.colorbar(im)
                cbar.set_label('vote count')
            else:
                self.get_logger().warning("警告: 投票地图为空或全为零")
            
            # 绘制二值地图边界
            if self.binary_map is not None and self.binary_map.size > 0 and np.any(self.binary_map):
                plt.contour(self.binary_map.T, levels=[0.5], colors='black', linewidths=0.5,
                          extent=[self.x_range[0], self.x_range[1], self.y_range[0], self.y_range[1]])
            
            plt.title('map and vehicle path')
            plt.xlabel('X (meter)')
            plt.ylabel('Y (meter)')
            plt.legend()
            plt.grid(True)
            combined_map_path = os.path.join(output_dir, f"{prefix}_with_trajectory.png")
            plt.savefig(combined_map_path, dpi=300)
            plt.close()
            
            # 创建图形2: 单独显示二值栅格地图
            if self.binary_map is not None:
                plt.figure(figsize=(12, 10))
                plt.imshow(self.binary_map.T, cmap='gray_r', origin='lower',
                          extent=[self.x_range[0], self.x_range[1], self.y_range[0], self.y_range[1]])
                plt.title('binary occupancy grid map')
                plt.xlabel('X (meter)')
                plt.ylabel('Y (meter)')
                binary_map_path = os.path.join(output_dir, f"{prefix}_binary.png")
                plt.savefig(binary_map_path, dpi=300)
                plt.close()
            
            self.get_logger().info("栅格地图可视化结果已保存")
            self.get_logger().info(f"保存的文件：\n- {combined_map_path}\n- {binary_map_path}")
            
        except Exception as e:
            self.get_logger().error(f"可视化保存过程中出现错误: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    node = GridMapRosbagNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
